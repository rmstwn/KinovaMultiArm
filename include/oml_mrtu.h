#ifndef OML_MRTU_H
#define OML_MRTU_H

#include <vector>
#include <string>
#include <cstdint>
#include <modbus.h>

//-----------------------------------------------------------------
// CommPC class
// A wrapper around libmodbus for Modbus RTU communication.
class CommPC {
public:
    /**
     * @brief Construct a new CommPC object.
     * 
     * @param port Serial port (e.g. "COM3" on Windows or "/dev/ttyUSB0" on Linux)
     * @param baudrate Baud rate (default 115200)
     */
    CommPC(const std::string &port, int baudrate = 115200);

    ~CommPC();

    /**
     * @brief Write 16-bit registers (Modbus function code 0x10)
     * 
     * @param add Slave address (Modbus ID)
     * @param idx Start register address
     * @param val Vector of 16-bit values to write
     * @return std::vector<int>  Returns {0} on success or negative error codes.
     */
    std::vector<int> modbusWriteReg(int add, int idx, const std::vector<int>& val);

    /**
     * @brief Write 32-bit values as two 16-bit registers each.
     * 
     * @param add Slave address
     * @param idx Start register address
     * @param val Vector of 32-bit values to write
     * @return std::vector<int> Returns {0} on success or negative error codes.
     */
    std::vector<int> modbusWriteRegWide(int add, int idx, const std::vector<int>& val);

    /**
     * @brief Read 16-bit registers.
     * 
     * @param add Slave address
     * @param idx Start register address
     * @param length Number of registers to read
     * @return std::vector<int> Returns a vector with first element 0 on success then the values.
     */
    std::vector<int> modbusReadReg(int add, int idx, int length);

    /**
     * @brief Read 32-bit values (read 2 registers per value).
     * 
     * @param add Slave address
     * @param idx Start register address
     * @param length Number of 32-bit values (i.e. 2×length registers)
     * @return std::vector<int> Returns a vector with first element 0 on success then the values.
     */
    std::vector<int> modbusReadRegWide(int add, int idx, int length);

private:
    modbus_t* ctx;
};

//-----------------------------------------------------------------
// MathConv class
// Provides two’s complement conversion utilities.
class MathConv {
public:
    MathConv();
    /**
     * @brief Convert an unsigned 32-bit value to a signed 32-bit integer.
     */
    int unsigned_to_signed_32bit_bitwise(uint32_t value);

    /**
     * @brief Convert a signed decimal number to its two's complement representation.
     * 
     * @param decimal_num A signed integer.
     * @param bit_length Bit width of the target representation.
     * @return uint32_t Two's complement representation.
     */
    uint32_t to_2s_complement(int decimal_num, int bit_length);
};

//-----------------------------------------------------------------
// Base class for Modbus motion (OrientalMotor)
class ModbusOM {
public:
    /**
     * @brief Construct a new ModbusOM object.
     * 
     * @param comm Pointer to a CommPC object.
     * @param serverAddress Modbus slave address.
     */
    ModbusOM(CommPC* comm, int serverAddress);
    virtual ~ModbusOM();

    virtual std::vector<int> freeDrive(bool sw = false);
    virtual std::vector<int> isReady();
    virtual std::vector<int> ppreset();
    virtual std::vector<int> readAlarm();
    virtual std::vector<int> readDriverOutput();
    virtual std::vector<int> requestConfiguration();
    virtual std::vector<int> resetAlarm();
    virtual std::vector<int> startContinuous(int speed, int direction = 0);
    virtual std::vector<int> startPosition(int position, int speed, int OpeType = 1);
    virtual std::vector<int> startDirectData(int position, int speed, int OpeType = 2,
                                               int OpeDataNo = 0, int Trigger = 1,
                                               int Memory = 0);
    virtual std::vector<int> stop();
    virtual std::vector<int> writeParamAcc(int time, int speed = 16667);
    virtual std::vector<int> writeParamDec(int time, int speed = 16667);
    virtual std::vector<int> writeParamCurrent(int current);

protected:
    CommPC* comm;
    int serverAddress;
    int driveCmd;
    int driverOutput;
    int OpeCurrent;
    int rateAcc;
    int rateDec;
};

//-----------------------------------------------------------------
// Derived class for AZ series
class ModbusAZ : public ModbusOM {
public:
    ModbusAZ(CommPC* comm, int serverAddress);
    virtual std::vector<int> readPosition();
    virtual std::vector<int> readSpeed(int unit = 0);
    virtual std::vector<int> startDirectData(int position, int speed, int OpeType = 2,
                                               int OpeDataNo = 0, int Trigger = 1,
                                               int Memory = 0) override;
    virtual std::vector<int> startZhome();
};

//-----------------------------------------------------------------
// Derived class for CVD series
class ModbusCVD : public ModbusOM {
public:
    ModbusCVD(CommPC* comm, int serverAddress);
    virtual std::vector<int> readPosition();
    virtual std::vector<int> readSpeed(int unit = 0);
    virtual std::vector<int> startDirectData(int position, int speed, int OpeType = 2,
                                               int OpeDataNo = 0, int Trigger = 1,
                                               int Memory = 0) override;
    virtual std::vector<int> startHome(int mode = 0, int direction = 0, int offset = 0);
};

//-----------------------------------------------------------------
// Derived class for BLV-R series
class ModbusBLVR : public ModbusOM {
public:
    ModbusBLVR(CommPC* comm, int serverAddress);
    virtual std::vector<int> freeDrive(bool sw = false) override;
};

#endif // OML_MRTU_H
