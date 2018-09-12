/**
 *@file logger.cpp
 *
 *@author YikangJ
 *
 *@date 07/20 2018
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
