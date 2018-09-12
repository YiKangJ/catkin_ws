/*
 * @file logger.h
 *
 * @author YikangJ
 * @date 07/20 2018
 */

#ifndef INCLUDE_LOGGER_H_
#define INCLUDE_LOGGER_H_

#include <string>

namespace mrx_t4_arm_kinematics
{

/// @brief 日志类
class Logger
{
    public:

        Logger();

        virtual ~Logger();

        /**
         * 输出日志信息。默认是一个虚函数。要实现该函数功能应该在子
         * 类中重写。
         *
         * @param msg 要写入日志文件的内容
         *
         * @param file 要写入日志的源文件
         *
         * @param line 指定从该行开始写入日志
         */

        virtual void write(const std::string &msg, const char *file, int line);


    public:
        /**
         * 一个可以用作默认参数的空对象
         */
        static Logger null;
};

}



#endif /* INCLUDE_LOGGER_H_ */
