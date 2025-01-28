/*
Author(s): Muhammad Ramadhan Hadi Setyawan
Institute: Takesue Laboratory, Tokyo Metropolitan University
Description: 

Copyright (c) 2025

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef KINOVA_MANAGER_HPP
#define KINOVA_MANAGER_HPP
#include <robot_manager.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <state_specification.hpp>
#include <urdf/model.h>
#include <constants.hpp>
#include <memory>
#include <iostream>
#include <utility> 
#include <sstream>
#include <fstream>
#include <chrono>
#include <thread> // std::this_thread::sleep_for
#include <time.h>
#include <cmath>
#include <string>
#include <vector>
#include <math.h>
#include <boost/assign/list_of.hpp>
#include <stdlib.h> /* abs */
#include <unistd.h>
#include <Eigen/Dense>// Eigen
#include <model_prediction.hpp>
#include <fd_solver_rne.hpp>

#include <KDetailedException.h>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>

#include <google/protobuf/util/json_util.h>
