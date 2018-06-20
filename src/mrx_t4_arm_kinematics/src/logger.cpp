/*
 * logger.cpp
 *
 *  Created on: 25-May-2018
 *      Author: YiKangJ
 */

#include <mrx_t4_arm_kinematics/logger.h>

using namespace mrx_t4_arm_kinematics;

Logger Logger::null;


Logger::Logger()
{
}


Logger::~Logger()
{
}


void Logger::write(const std::string &msg, const char *file, int line)
{
}
