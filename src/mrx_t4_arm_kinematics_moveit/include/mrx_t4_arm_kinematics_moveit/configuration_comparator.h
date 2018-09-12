#ifndef CONFIGURATION_COMPARATOR_H
#define CONFIGURATION_COMPARATOR_H

#include <vector>

/**
 * 关节序列比较器
 * 将求解值与参考值进行比较
 *
 * 将所有求解的关节序列集合按照与参考序列的临近关系从小到达排序，最终最近的元素就是离参考序列最近的解。
 */

template <typename T>
class ConfigurationComparator
{
        public:
                /**
                 *
                 * @param reference 参考关节序列
                 */

                ConfigurationComparator(const std::vector<T> &reference);

                virtual ~ConfigurationComparator();

                /**
                 * 根据参考序列来比较两输入序列与参考序列的距离大小。所谓距
                 * 离，指的是相对应两个序列元素的平方差和。
                 *
                 * @param a 第一个关节序列
                 *
                 * @param b 第二个关节序列
                 *
                 * @return 当第一个关节序列距离参考序列更近时返回true，否则
                 * 返回false。
                 */
                bool operator()(const std::vector<T> &a, const std::vector<T> &b);
        
        private:
                /**
                 * 提供给构造函数的参考序列
                 */
                std::vector<T> reference_;
};


#include "configuration_comparator.inl"

#endif
