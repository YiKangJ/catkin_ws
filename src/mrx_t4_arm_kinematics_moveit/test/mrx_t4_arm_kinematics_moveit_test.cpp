#include <moveit/kinematics_base/kinematics_base.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <gtest/gtest.h>


const std::string PLUGIN = "mrx_t4_arm_kinematics_moveit/KinematicsPlugin";
typedef boost::shared_ptr<kinematics::KinematicsBase> KinematicsBasePtr;

/*
TEST(mrx_t4_arm_kinematics_moveit, wrong_arguments_fail_init)
{
    pluginlib::ClassLoader<kinematics::KinematicsBase> loader(
            "moveit_core", "kinematics::KinematicsBase");
    KinematicsBasePtr kinematics = loader.createInstance(PLUGIN);

    EXPECT_FALSE(kinematics->initialize("", "", "", "", 0.0));
}

TEST(mrx_t4_arm_kinematics_moveit, init_sets_values)
{
    pluginlib::ClassLoader<kinematics::KinematicsBase> loader(
            "moveit_core", "kinematics::KinematicsBase");
    KinematicsBasePtr kinematics = loader.createInstance(PLUGIN);

    EXPECT_TRUE(kinematics->initialize("/robot_description", "arm",
            "base_link", "vt_hand", 0.0));

    EXPECT_EQ("arm", kinematics->getGroupName());
    EXPECT_EQ("base_link", kinematics->getBaseFrame());
    EXPECT_EQ("vt_hand", kinematics->getTipFrame());
    EXPECT_NEAR(0.0, kinematics->getSearchDiscretization(), 0.001);
}


TEST(mrx_t4_arm_kinematics_moveit, wrong_kinematics_fails_init)
{
    pluginlib::ClassLoader<kinematics::KinematicsBase> loader(
            "moveit_core", "kinematics::KinematicsBase");
    KinematicsBasePtr kinematics = loader.createInstance(PLUGIN);

    EXPECT_FALSE(kinematics->initialize("/kong", "", "base_link",
            "vt_hand", 0.0));
}


TEST(mrx_t4_arm_kinematics_moveit, wrong_base_and_tip_frame_fails_init)
{
    pluginlib::ClassLoader<kinematics::KinematicsBase> loader(
            "moveit_core", "kinematics::KinematicsBase");
    KinematicsBasePtr kinematics = loader.createInstance(PLUGIN);

    EXPECT_FALSE(kinematics->initialize("/kong", "arm", "base_link",
            "vt_hand", 0.0));
}


TEST(mrx_t4_arm_kinematics_moveit, joint_and_link_names_not_empty_after_init)
{
    pluginlib::ClassLoader<kinematics::KinematicsBase> loader(
            "moveit_core", "kinematics::KinematicsBase");
    KinematicsBasePtr kinematics = loader.createInstance(PLUGIN);

    EXPECT_TRUE(kinematics->getJointNames().empty());
    EXPECT_TRUE(kinematics->getLinkNames().empty());

    EXPECT_TRUE(kinematics->initialize("/robot_description", "arm",
            "base_link", "vt_hand", 0.0));

    EXPECT_EQ(4, kinematics->getJointNames().size());
    EXPECT_EQ(6, kinematics->getLinkNames().size());

    
    EXPECT_EQ("base_to_armA", kinematics->getJointNames()[0]);
    EXPECT_EQ("armA_to_armB", kinematics->getJointNames()[1]);
    EXPECT_EQ("armB_to_armC", kinematics->getJointNames()[2]);
    EXPECT_EQ("armC_to_armD", kinematics->getJointNames()[3]);

    EXPECT_EQ("base", kinematics->getLinkNames()[0]);
    EXPECT_EQ("armA", kinematics->getLinkNames()[1]);
    EXPECT_EQ("armB", kinematics->getLinkNames()[2]);
    EXPECT_EQ("armC", kinematics->getLinkNames()[3]);
    EXPECT_EQ("armD", kinematics->getLinkNames()[4]);
    EXPECT_EQ("vt_hand", kinematics->getLinkNames()[5]);
    
}


TEST(mrx_t4_arm_kinematics_moveit, inverse_kinematics_fails_before_init)
{
    pluginlib::ClassLoader<kinematics::KinematicsBase> loader(
            "moveit_core", "kinematics::KinematicsBase");
    KinematicsBasePtr kinematics = loader.createInstance(PLUGIN);

    geometry_msgs::Pose pose;
    std::vector<double> seed;
    std::vector<double> solution;
    moveit_msgs::MoveItErrorCodes error_code;

    EXPECT_FALSE(kinematics->getPositionIK(pose, seed, solution, error_code));
}*/





TEST(mrx_t4_arm_kinematics_moveit, get_position_ik_fails_on_wrong_seed)
{
    pluginlib::ClassLoader<kinematics::KinematicsBase> loader(
            "moveit_core", "kinematics::KinematicsBase");
    KinematicsBasePtr kinematics = loader.createInstance(PLUGIN);
    kinematics->initialize("/robot_description", "arm", "base_link",
            "arm_link_5", 0.0);

    geometry_msgs::Pose pose;
    std::vector<double> seed(4,0);
    std::vector<double> solution;
    moveit_msgs::MoveItErrorCodes error_code;

    pose.position.x = 0.057;
    pose.position.y = 0.0;
    pose.position.z = 0.535;

    EXPECT_FALSE(kinematics->getPositionIK(pose, seed, solution, error_code));
}

TEST(mrx_t4_arm_kinematics_moveit, find_solution_for_candle_configuration)
{
    pluginlib::ClassLoader<kinematics::KinematicsBase> loader(
            "moveit_core", "kinematics::KinematicsBase");
    KinematicsBasePtr kinematics = loader.createInstance(PLUGIN);
    kinematics->initialize("/robot_description", "arm", "base_link","vt_hand", 0.0);

    geometry_msgs::Pose pose;
    const std::vector<double> seed(4, 0.0);
    std::vector<double> solution;
    moveit_msgs::MoveItErrorCodes error_code;

    pose.position.x = -0.345;
    pose.position.y = 0.34;
    pose.position.z = 0.4;
    pose.orientation.x = 0.707104;
    pose.orientation.y = -4.19318e-6;
    pose.orientation.z = -9.11915e-7;
    pose.orientation.w = 0.707109;
   /* 
    pose.position.x = 0.655;
    pose.position.y = 0;
    pose.position.z = 0.257;
    pose.orientation.x = 0.70710678;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 0.70710678;
    */
	const geometry_msgs::Pose pose_ = pose; 
    const kinematics::KinematicsQueryOptions options = kinematics::KinematicsQueryOptions();

    EXPECT_TRUE(kinematics->getPositionIK(pose_, seed, solution, error_code));
 
    EXPECT_NEAR( 2.3635, solution[0], 0.001);
    EXPECT_NEAR( 1.1614, solution[1], 0.001);
    EXPECT_NEAR( -1.5337, solution[2], 0.001);
    EXPECT_NEAR( 0.3722, solution[3], 0.001);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mrx_t4_arm_kinematics_moveit_test");

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
