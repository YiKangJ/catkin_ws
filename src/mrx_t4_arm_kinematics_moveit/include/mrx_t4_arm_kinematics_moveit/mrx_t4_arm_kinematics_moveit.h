/**
  * @file mrx_t4_arm_kinematics_moveit.h
  * @brief MRX_T4 机械臂求解器插件
  * 
  * 求解器插件对象由Moveit!生成并调用相应函数。
  */
#ifndef MRX_TR4_ARM_KINEMATICS_MOVEIT_H
#define MRX_TR4_ARM_KINEMATICS_MOVEIT_H

#include <boost/shared_ptr.hpp>

#include <moveit/kinematics_base/kinematics_base.h>
#include <urdf/model.h>
#include <mrx_t4_arm_kinematics/inverse_kinematics.h>

#include <mrx_t4_arm_kinematics_moveit/kinematics_logger.h>

/// @brief 求解器插件命名空间
namespace mrx_t4_arm_kinematics_moveit
{
/**
  * 求解器插件类，继承自kinematics::KinematicsBase
  */
class KinematicsPlugin : public kinematics::KinematicsBase
{
    public:
        /**
         * Ctor.
         */
        KinematicsPlugin();

        /**
         * Dtor.
         */
        virtual ~KinematicsPlugin();

        /**
         * 求解器插件初始化，由Moveit!调用
         * @param robot_description 机器人描述，即URDF文件
         * @param group_name 规划组
         * @param base_frame 基坐标系
         * @param tip_frame 末端坐标系
         * @param search_discretization 搜索离散因子
         */
        bool initialize(const std::string &robot_description,
                const std::string &group_name,
                const std::string &base_frame,
                const std::string &tip_frame,
                double search_discretization);

        /**
         * 调用求解器核心的求逆解函数进行求解
         *
         * @param ik_pose 待求解末端位姿
         * @param ik_seed_state 机器人当前位姿，作为参考序列
         * @param solution 返回唯一最近有效解
         * @param error_code 错误代码
         * @param options 其它选项
         */
        bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                const std::vector<double> &ik_seed_state,
                std::vector<double> &solution,
                moveit_msgs::MoveItErrorCodes &error_code,
                const kinematics::KinematicsQueryOptions &options
                        = kinematics::KinematicsQueryOptions()) const;

        /**
         * 调用求解器核心求逆解函数，与getPositionIK一样的作用。
         * 按照github的issue说法，getPosition与searchPositionIK都是用来求
         * 逆解的，KDL是将getPositionIK参数传递给了searchPositionIK，而
         * ikfast则是将searchPositionIK传递给了getPositionIK（该插件是这么
         * 做的），故本文档中说moveit是直接或者间接调用getPositionIK函数。
         * 另外，还有个searchPositionIK独有的特点：The search was intended
         * to allow a search through the redundancy for a solution while 
         * the getPositionIK is not supposed to that.
         * @param ik_pose 同getPositionIK
         * @param ik_seed_state 同getPositionIK
         * @param timeout 超时时间
         * @param solution 同getPositionIK
         * @param error_code 同getPositionIK
         * @param options 同getPositionIK
         */
        bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                const std::vector<double> &ik_seed_state,
                double timeout,
                std::vector<double> &solution,
                moveit_msgs::MoveItErrorCodes &error_code,
                const kinematics::KinematicsQueryOptions &options
                        = kinematics::KinematicsQueryOptions()) const;

        /**
         * 参考getPositionIK
         */
        bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                const std::vector<double> &ik_seed_state,
                double timeout,
                const std::vector<double> &consistency_limits,
                std::vector<double> &solution,
                moveit_msgs::MoveItErrorCodes &error_code,
                const kinematics::KinematicsQueryOptions &options
                        = kinematics::KinematicsQueryOptions()) const;

        /**
         * 参考getPositionIK
         */
        bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                const std::vector<double> &ik_seed_state,
                double timeout,
                std::vector<double> &solution,
                const IKCallbackFn &solution_callback,
                moveit_msgs::MoveItErrorCodes &error_code,
                const kinematics::KinematicsQueryOptions &options
                        = kinematics::KinematicsQueryOptions()) const;

        /**
         * 参考getPositionIK
         */
        bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                const std::vector<double> &ik_seed_state,
                double timeout,
                const std::vector<double> &consistency_limits,
                std::vector<double> &solution,
                const IKCallbackFn &solution_callback,
                moveit_msgs::MoveItErrorCodes &error_code,
                const kinematics::KinematicsQueryOptions &options
                        = kinematics::KinematicsQueryOptions()) const;

        /**
         * 求正解函数，此处未实现，求正解无需求解器 
         */
        bool getPositionFK(const std::vector<std::string> &link_names,
                const std::vector<double> &joint_angles,
                std::vector<geometry_msgs::Pose> &poses) const;

        /**
         * 获取关节名（URDF中的joint名） 
         */
        const std::vector<std::string> &getJointNames() const;

        /**
         * 获取杆件名（URDF中的link名）
         */
        const std::vector<std::string> &getLinkNames() const;


    private:
        /**
         * Copy ctor.
         */
        KinematicsPlugin(const KinematicsPlugin &other);

        /**
         * 操作符定义
         */
        KinematicsPlugin &operator=(const KinematicsPlugin &other);

        /**
         * 提取给定URDF模型的关节名，关节限制，坐标系等信息。
         *
         * @param robot_model 机器人URDF文件
         *
         * @param base_frame 运动链基坐标系（在setup_assistant中已定义）
         *
         * @param tip_frame 运动链末端坐标系（在setup_assistant中已定义）
         *
         * @param joint_names 提取到的关节名序列
         *
         * @param link_names 杆件名序列
         *
         * @param lower_limits URDF中定义的最小关节角度限制序列
         *
         * @param upper_limits 最大关节角度限制
         *
         * @return 提取信息成功返回true，否则返回false
         */
        bool extractKinematicData(const urdf::Model &robot_model,
                const std::string &base_frame,
                const std::string &tip_frame,
                std::vector<std::string> &joint_names,
                std::vector<std::string> &link_names,
                std::vector<double> &lower_limits,
                std::vector<double> &upper_limits) const;

        /**
         * 将关节序列向量std::vector<double>类型转换到KDL::JntArray数据类
         * 型。相反的转换参见{configurationKdlToStd}函数。
         *
         * @param v 输入关节序列向量
         *
         * @return KDL::Array表示
         */
        KDL::JntArray configurationStdToKdl(const std::vector<double> &v) const;

        /**
         * 将KDL::JntArray数据类型转换为std::vector<double> 关节向量 
         * 相反的实现见{configurationStdToKdl}函数
         *
         * @param jnt 输入JntArray
         *
         * @return 输出关节序列vector
         */
        std::vector<double> configurationKdlToStd(const KDL::JntArray &jnt)
                const;


    private:
        /**
         * 逆解函数所能求解关节个数，MRX为4自由度
         */
        const std::size_t NUM_JOINTS;

        /**
         * 关节名序列
         */
        std::vector<std::string> joint_names_;

        /**
         * 在运动链基件杆与末端杆件之间的杆件名序列
         */
        std::vector<std::string> link_names_;

        /**
         * 求解器对象
         */
        boost::shared_ptr<mrx_t4_arm_kinematics::InverseKinematics> ik_;

        /**
         * 运动学求解过程中的日志生成器
         */
        KinematicsLogger kinematics_logger_;
};

}

#endif
