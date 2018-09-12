/**
 *@file inverse_kinematics.h
 *
 *@brief MRX_T4 机械臂求解器核心
 *
 *@author YiKangJ
 *
 *@version 1.0
 *
 *@date 07/20 2018
 */

#ifndef INCLUDE_INVERSE_KINEMATICS_H_
#define INCLUDE_INVERSE_KINEMATICS_H_

#include <vector>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <mrx_t4_arm_kinematics/logger.h>

///@brief 求解器核心命名空间
namespace mrx_t4_arm_kinematics
{

///@brief 求解器核心类
///
/// 包括求逆解、关节数据类型转换、日志输出等模块 
class InverseKinematics
{
public:
    /**
      * @param min_angles 最小关节角度限制，以弧度为单位。
      *
      * @param max_angles 最大关节角度限制，以弧度为单位。
      *
      * @param logger 日志输出器，用来输出内部日志。默认为空对象即不输出日志。
      */
     InverseKinematics(
         const std::vector<double> &min_angles,
         const std::vector<double> &max_angles,
         Logger &logger = Logger::null);

     virtual ~InverseKinematics();

     /**
      * @brief 计算逆解函数，从笛卡尔空间到关节空间求解。
      *
      * @param q_init 求解时输入的初始关节角度序列，通常用于数值求解器中，
      * 在解析求解器中可以传入当前的关节角度序列，所以求解器可以通过对比当
      * 前位置返回多解情况中最接近当前位置的解。由于目前的位置对比函数在求
      * 解器插件类中实现了，故在此并没有使用该参数。输入角度单位为弧度。
      *
      * @param p_in 要求解的笛卡尔空间位姿。KDL::Frame类型（位姿矩阵前3行,
      * 即3×4矩阵），详细解释可参考官方KDL文档。其中，位置向量单位为米。
      *
      * @param q_out 所有有效逆解序列的集合。单位为弧度。
      *
      * @return 如果找到可行解，则返回大于0的值，否则返回小于0的值。
      
      */

     int CartToJnt(const KDL::JntArray &q_init,
         const KDL::Frame &p_in,
         std::vector<KDL::JntArray> &q_out);

private:

     /**
      *@brief 判断返回值是否有效，即求解器返回的解不为初始的INVALID值则返
      * 回了有效解。
      *
      * @param solution 输入的单个关节解序列。
      *
      * @return 解有效返回true否则返回false。
      *
      */

     bool isSolutionValid(const KDL::JntArray &solution) const;

     /**
      * IK求解函数，由于MRX_T4 是四自由度机械臂，故其只能到达三维空间的部
      * 分位姿，而大部分任意位姿的解是无效的。所以为了避免过多的求解失败而
      * 导致无法使用，我们对末端的方位进行了固定限制，这样便可以以这个末端
      * 固定姿态到达工作空间内任意位置。而求解器只需提取输入的（x,z,z）三
      * 个位置便可以求解，只获取坐标生成位姿的函数见文档calFixPosMatr()函
      * 数。函数核心算法参见 四自由度串联机器人V0.6.docx(周家兴)。
      *
      * @param frame 输入笛卡尔空间目标位姿矩阵(KDL::Frame类型)。
      *
      * @return 含有两个元素的解集，若只有一个解，则两个解元素相同，若误解，解元素值均为INVALID。
      */
     std::vector<KDL::JntArray> ik(const KDL::Frame &frame);


private:
     /**
      * 最小关节角度限制序列
      */
     std::vector<double> min_angles_;

     /**
      * 最大关节角度限制序列
      */
     std::vector<double> max_angles_;

     /**
      * 日志发布器
      */
     Logger &logger_;

}; // class end

/**
  * 通过限定的末端姿态与提取输入的(x,y,z)位置信息进行有效位姿求解。
  *
  * @param pos_d 位置(x,y,z)列表
  * @param pos_ox Frame位姿矩阵中的第一列
  * @param pos_oy Frame位姿矩阵中的第二列
  * @param pos_oz Frame位姿矩阵中的第三列
  * 
  * @return 若成功生成位姿矩阵则返回值为1，若末端点不在工作区间内，则返回0
  * 若末端点在xoy投影与原点距离过小则返回-1。具体可以参见 四自由度串联机器
  * 人V6.0中calFixPosMatr()函数。
  */
int calFixPosMatr(KDL::Vector pos_d, KDL::Vector &pos_ox, KDL::Vector &pos_oy, KDL::Vector &pos_oz);

/**
  * 检查目标角度是否合法，即是否处于最小关节角度限制与最大关节角度限制区间
  * @param target 待检测目标值
  * @param min_angle 区间下限
  * @param max_angle 区间上限
  */
bool checkLim(double target, double min_angle, double max_angle);
/**
  * 在误差范围内判别目标函数值是否在区间内
  * @param target 待判断目标值及修正结果
  * @param lim 限制范围
  * @param error 计算误差容许值
  * 
  * @return 若目标值在区内返回true，否则返回false
    * 
    * @return 若目标值在区内返回true，否则返回false。
  */
bool judgeNum(double &target, std::vector<double> lim, double error);

/**
  * 限制角度范围为[-π,π]的标准化过程
  * @param target 待标准化目标角度值
  * 
  * @return 标准化的角度值
  */
double standard(double target);

/**
  * 判别有效解（位置）
  * @param pos 待判别关节角度序列
  * @param min_angles 最小关节角度限制序列
  * @param max_angles 最大关节角度限制序列
  *
  * @return 若位置有效返回true，否则返回false
  */
bool judgePos(std::vector<double> pos, std::vector<double> min_angles, std::vector<double> max_angles);
} // namespace end

#endif /* INCLUDE_INVERSE_KINEMATICS_H_ */
