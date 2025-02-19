#include "oml_mrtu.h"
#include <iostream>
#include <cstring>
#include <errno.h>

//=================================================================
// CommPC implementation
CommPC::CommPC(const std::string &port, int baudrate)
{
    // Create a new RTU context.
    ctx = modbus_new_rtu(port.c_str(), baudrate, 'E', 8, 1);
    if (ctx == nullptr)
    {
        std::cerr << "Unable to allocate libmodbus context" << std::endl;
        // In a production system, you might throw an exception.
    }
    if (modbus_connect(ctx) == -1)
    {
        std::cerr << "Connection failed: " << modbus_strerror(errno) << std::endl;
        modbus_free(ctx);
        ctx = nullptr;
    }
}

CommPC::~CommPC()
{
    if (ctx)
    {
        modbus_close(ctx);
        modbus_free(ctx);
    }
}

std::vector<int> CommPC::modbusWriteReg(int add, int idx, const std::vector<int> &val)
{
    std::vector<int> ret;

    if (modbus_set_slave(ctx, add) == -1)
    {
        std::cerr << "Set slave failed: " << modbus_strerror(errno) << std::endl;
        ret.push_back(-2);
        ret.push_back(1);
        return ret;
    }

    int nb = static_cast<int>(val.size());

    // Convert std::vector<int> to std::vector<uint16_t>
    std::vector<uint16_t> val_16(val.begin(), val.end());

    int rc = modbus_write_registers(ctx, idx, nb, val_16.data());
    if (rc == -1)
    {
        std::cerr << "Write registers failed: " << modbus_strerror(errno) << std::endl;
        ret.push_back(-2);
        ret.push_back(1);
        return ret;
    }

    ret.push_back(0);
    return ret;
}

std::vector<int> CommPC::modbusWriteRegWide(int add, int idx, const std::vector<int> &val)
{
    // Split each 32-bit value into two 16-bit registers.
    std::vector<int> list16bit_val;
    list16bit_val.reserve(val.size() * 2);
    for (auto v : val)
    {
        int high = (v >> 16) & 0xffff;
        int low = v & 0xffff;
        list16bit_val.push_back(high);
        list16bit_val.push_back(low);
    }
    return modbusWriteReg(add, idx, list16bit_val);
}

std::vector<int> CommPC::modbusReadReg(int add, int idx, int length)
{
    std::vector<int> ret;
    std::vector<uint16_t> regs(length);
    if (modbus_set_slave(ctx, add) == -1)
    {
        std::cerr << "Set slave failed: " << modbus_strerror(errno) << std::endl;
        ret.push_back(-2);
        ret.push_back(1);
        return ret;
    }
    int rc = modbus_read_registers(ctx, idx, length, regs.data());
    if (rc == -1)
    {
        std::cerr << "Read registers failed: " << modbus_strerror(errno) << std::endl;
        ret.push_back(-2);
        ret.push_back(1);
        return ret;
    }
    ret.push_back(0);
    for (int i = 0; i < length; ++i)
    {
        ret.push_back(regs[i]);
    }
    return ret;
}

std::vector<int> CommPC::modbusReadRegWide(int add, int idx, int length)
{
    // Read 2 registers per 32-bit value.
    std::vector<int> ret;
    int count = length * 2;
    std::vector<uint16_t> regs(count);
    if (modbus_set_slave(ctx, add) == -1)
    {
        std::cerr << "Set slave failed: " << modbus_strerror(errno) << std::endl;
        ret.push_back(-2);
        ret.push_back(1);
        return ret;
    }
    int rc = modbus_read_registers(ctx, idx, count, regs.data());
    if (rc == -1)
    {
        std::cerr << "Read wide registers failed: " << modbus_strerror(errno) << std::endl;
        ret.push_back(-2);
        ret.push_back(1);
        return ret;
    }
    ret.push_back(0);
    for (int i = 0; i < length; ++i)
    {
        int value = (regs[2 * i] << 16) | regs[2 * i + 1];
        ret.push_back(value);
    }
    return ret;
}

//=================================================================
// MathConv implementation
MathConv::MathConv() {}

int MathConv::unsigned_to_signed_32bit_bitwise(uint32_t value)
{
    // If the highest bit is set, interpret as negative.
    if (value & 0x80000000)
        return -static_cast<int>((~value + 1) & 0xffffffff);
    else
        return static_cast<int>(value);
}

uint32_t MathConv::to_2s_complement(int decimal_num, int bit_length)
{
    if (decimal_num < 0)
    {
        uint32_t abs_num = static_cast<uint32_t>(-decimal_num);
        return ((1u << bit_length) - abs_num) & ((1u << bit_length) - 1);
    }
    return static_cast<uint32_t>(decimal_num);
}

//=================================================================
// ModbusOM implementation
ModbusOM::ModbusOM(CommPC *comm, int serverAddress) : comm(comm), serverAddress(serverAddress)
{
    // Read drive command from register 0x007d (assume length = 1).
    std::vector<int> res = comm->modbusReadReg(serverAddress, 0x007d, 1);
    if (!res.empty() && res[0] == 0 && res.size() > 1)
    {
        driveCmd = res[1];
    }
    else
    {
        driveCmd = 0;
    }
    driverOutput = 0;
    OpeCurrent = 1000; // default current (1: 0.1% steps)
    rateAcc = 1000000; // default acceleration rate
    rateDec = 1000000; // default deceleration rate
}

ModbusOM::~ModbusOM() {}

std::vector<int> ModbusOM::freeDrive(bool sw)
{
    if (sw)
        return comm->modbusWriteReg(serverAddress, 0x007d, {driveCmd | 0x0040});
    else
        return comm->modbusWriteReg(serverAddress, 0x007d, {driveCmd & 0xffbf});
}

std::vector<int> ModbusOM::isReady()
{
    driverOutput = 0;
    std::vector<int> dout = readDriverOutput();
    if (dout.size() > 1)
        driverOutput = dout[1];

    std::vector<int> ret = {0, 0};
    // If bit 0x0020 is set then ready.
    if ((driverOutput & 0x0020) == 0x0020)
        ret[1] = 1; // true (ready)
    return ret;
}

std::vector<int> ModbusOM::ppreset()
{
    std::vector<int> res = comm->modbusWriteReg(serverAddress, 0x018A, {0, 1});
    if (!res.empty() && res[0] != 0)
        return res;
    res = comm->modbusWriteReg(serverAddress, 0x018A, {0, 0});
    return res;
}

std::vector<int> ModbusOM::readAlarm()
{
    return comm->modbusReadRegWide(serverAddress, 0x0080, 1);
}

std::vector<int> ModbusOM::readDriverOutput()
{
    return comm->modbusReadRegWide(serverAddress, 0x007e, 1);
}

std::vector<int> ModbusOM::requestConfiguration()
{
    comm->modbusWriteReg(serverAddress, 0x018C, {0, 1});
    return comm->modbusWriteReg(serverAddress, 0x018C, {0, 0});
}

std::vector<int> ModbusOM::resetAlarm()
{
    std::vector<int> res = comm->modbusWriteReg(serverAddress, 0x0180, {0, 1});
    if (!res.empty() && res[0] != 0)
        return res;
    res = comm->modbusWriteReg(serverAddress, 0x0180, {0, 0});
    // Restore the drive command.
    comm->modbusWriteReg(serverAddress, 0x007d, {driveCmd});
    return res;
}

std::vector<int> ModbusOM::startContinuous(int speed, int direction)
{
    MathConv math;
    uint32_t speed_tc = math.to_2s_complement(speed, 32);
    int cmd = 0;
    if (direction == 0)
        cmd = 0x4000;
    else if (direction == 1)
        cmd = 0x8000;
    else
        return {-2, 0};

    std::vector<int> res = comm->modbusWriteRegWide(serverAddress, 0x1806, {rateAcc, rateDec, OpeCurrent});
    if (!res.empty() && res[0] != 0)
        return res;
    res = comm->modbusWriteRegWide(serverAddress, 0x1804, {static_cast<int>(speed_tc)});
    if (!res.empty() && res[0] != 0)
        return res;
    res = comm->modbusWriteReg(serverAddress, 0x007d, {driveCmd | cmd});
    return res;
}

std::vector<int> ModbusOM::startPosition(int position, int speed, int OpeType)
{
    if (OpeType < 1 || OpeType > 2)
        return {-2, 0};
    return startDirectData(position, speed, OpeType);
}

std::vector<int> ModbusOM::startDirectData(int position, int speed, int OpeType,
                                           int OpeDataNo, int Trigger, int Memory)
{
    // Validate parameters
    if (position < -(1 << 31) || position > ((1 << 31) - 1))
        return {-2, 0};
    if (speed < -4000000 || speed > 4000000)
        return {-2, 0};
    if (OpeDataNo < 0 || OpeDataNo > 255)
        return {-2, 0};
    if (Memory < 0 || Memory > 1)
        return {-2, 0};

    MathConv math;
    uint32_t pos_tc = math.to_2s_complement(position, 32);
    uint32_t speed_tc = math.to_2s_complement(speed, 32);
    uint32_t trigger_tc = math.to_2s_complement(Trigger, 32);
    uint32_t opedata_tc = math.to_2s_complement(OpeDataNo, 32);
    uint32_t memory_tc = math.to_2s_complement(Memory, 32);

    std::vector<int> directDriveData = {
        static_cast<int>(opedata_tc),
        OpeType,
        static_cast<int>(pos_tc),
        static_cast<int>(speed_tc),
        rateAcc,
        rateDec,
        OpeCurrent,
        static_cast<int>(trigger_tc),
        static_cast<int>(memory_tc)};

    return comm->modbusWriteRegWide(serverAddress, 0x0058, directDriveData);
}

std::vector<int> ModbusOM::stop()
{
    std::vector<int> res = comm->modbusWriteReg(serverAddress, 0x007d, {driveCmd | 0x0020});
    if (!res.empty() && res[0] != 0)
        return res;
    res = comm->modbusWriteReg(serverAddress, 0x007d, {driveCmd});
    return res;
}

std::vector<int> ModbusOM::writeParamAcc(int time, int speed)
{
    int preAcc = rateAcc;
    rateAcc = static_cast<int>(1000.0 * speed / time);
    if (rateAcc > 0 && rateAcc <= 1000000000)
        return {0};
    else
    {
        rateAcc = preAcc;
        return {-2, 0};
    }
}

std::vector<int> ModbusOM::writeParamDec(int time, int speed)
{
    int preDec = rateDec;
    rateDec = static_cast<int>(1000.0 * speed / time);
    if (rateDec > 0 && rateDec <= 1000000000)
        return {0};
    else
    {
        rateDec = preDec;
        return {-2, 0};
    }
}

std::vector<int> ModbusOM::writeParamCurrent(int current)
{
    int preCurrent = OpeCurrent;
    if (current >= 0 && current <= 1000)
    {
        OpeCurrent = current;
        return {0};
    }
    else
    {
        OpeCurrent = preCurrent;
        return {-2, 0};
    }
}

//=================================================================
// ModbusAZ implementation
ModbusAZ::ModbusAZ(CommPC *comm, int serverAddress) : ModbusOM(comm, serverAddress)
{
    rateAcc = 1000000;
    rateDec = 1000000;
}

std::vector<int> ModbusAZ::readPosition()
{
    std::vector<int> posVal = comm->modbusReadRegWide(serverAddress, 0x00cc, 1);
    if (posVal.empty() || posVal[0] != 0)
        return posVal;
    MathConv math;
    int signedPos = math.unsigned_to_signed_32bit_bitwise(static_cast<uint32_t>(posVal[1]));
    return {0, signedPos};
}

std::vector<int> ModbusAZ::readSpeed(int unit)
{
    int registerAddress = 0;
    if (unit == 0)
        registerAddress = 0x00d0; // Hz
    else if (unit == 1)
        registerAddress = 0x00ce; // r/min
    else
        return {-2, 0};

    std::vector<int> speedVal = comm->modbusReadRegWide(serverAddress, registerAddress, 1);
    if (speedVal.empty() || speedVal[0] != 0)
        return speedVal;
    MathConv math;
    int signedSpeed = math.unsigned_to_signed_32bit_bitwise(static_cast<uint32_t>(speedVal[1]));
    return {0, signedSpeed};
}

std::vector<int> ModbusAZ::startDirectData(int position, int speed, int OpeType, int OpeDataNo, int Trigger, int Memory)
{
    if (OpeType >= 0 && OpeType <= 22)
    {
        if ((OpeType >= 4 && OpeType <= 6) || OpeType == 19)
            return {-2, 0};
    }
    else
    {
        return {-2, 0};
    }
    return ModbusOM::startDirectData(position, speed, OpeType, OpeDataNo, Trigger, Memory);
}

std::vector<int> ModbusAZ::startZhome()
{
    std::vector<int> res = comm->modbusWriteReg(serverAddress, 0x007d, {driveCmd | 0x0010});
    if (!res.empty() && res[0] != 0)
        return res;
    res = comm->modbusWriteReg(serverAddress, 0x007d, {driveCmd});
    return res;
}

//=================================================================
// ModbusCVD implementation
ModbusCVD::ModbusCVD(CommPC *comm, int serverAddress) : ModbusOM(comm, serverAddress)
{
    rateAcc = 30000;
    rateDec = 30000;
}

std::vector<int> ModbusCVD::readPosition()
{
    std::vector<int> posVal = comm->modbusReadRegWide(serverAddress, 0x00cc, 1);
    if (posVal.empty() || posVal[0] != 0)
        return posVal;
    MathConv math;
    int signedPos = math.unsigned_to_signed_32bit_bitwise(static_cast<uint32_t>(posVal[1]));
    return {0, signedPos};
}

std::vector<int> ModbusCVD::readSpeed(int unit)
{
    int registerAddress = 0;
    if (unit == 0)
        registerAddress = 0x00d0;
    else if (unit == 1)
        registerAddress = 0x00ce;
    else
        return {-2, 0};

    std::vector<int> speedVal = comm->modbusReadRegWide(serverAddress, registerAddress, 1);
    if (speedVal.empty() || speedVal[0] != 0)
        return speedVal;
    MathConv math;
    int signedSpeed = math.unsigned_to_signed_32bit_bitwise(static_cast<uint32_t>(speedVal[1]));
    return {0, signedSpeed};
}

std::vector<int> ModbusCVD::startDirectData(int position, int speed, int OpeType, int OpeDataNo, int Trigger, int Memory)
{
    if (OpeType < 0 || OpeType > 2)
        return {-2, 0};
    return ModbusOM::startDirectData(position, speed, OpeType, OpeDataNo, Trigger, Memory);
}

std::vector<int> ModbusCVD::startHome(int mode, int direction, int offset)
{
    MathConv math;
    uint32_t mode_tc = math.to_2s_complement(mode, 16);
    uint32_t direction_tc = math.to_2s_complement(direction, 16);
    // Check offset range for 32-bit signed value.
    if (offset < -((1 << 31) - 1) || offset > ((1 << 31) - 1))
        return {-2, 0};
    uint32_t offset_tc = math.to_2s_complement(offset, 32);

    std::vector<int> res = comm->modbusWriteReg(serverAddress, 0x02c1, {static_cast<int>(mode_tc)});
    if (!res.empty() && res[0] != 0)
        return res;
    res = comm->modbusWriteReg(serverAddress, 0x02c3, {static_cast<int>(direction_tc)});
    if (!res.empty() && res[0] != 0)
        return res;
    res = comm->modbusWriteRegWide(serverAddress, 0x02d0, {static_cast<int>(offset_tc)});
    if (!res.empty() && res[0] != 0)
        return res;
    comm->modbusWriteReg(serverAddress, 0x007d, {driveCmd | 0x0010});
    res = comm->modbusWriteReg(serverAddress, 0x007d, {driveCmd});
    return res;
}

//=================================================================
// ModbusBLVR implementation
ModbusBLVR::ModbusBLVR(CommPC *comm, int serverAddress) : ModbusOM(comm, serverAddress)
{
    rateAcc = 30000;
    rateDec = 30000;
}

std::vector<int> ModbusBLVR::freeDrive(bool sw)
{
    if (sw)
        return comm->modbusWriteReg(serverAddress, 0x007d, {driveCmd | 0x0040});
    else
        return comm->modbusWriteReg(serverAddress, 0x007d, {driveCmd & 0xffbf});
}
