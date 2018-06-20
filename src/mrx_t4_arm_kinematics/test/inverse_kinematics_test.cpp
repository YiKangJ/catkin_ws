#include <moveit/kinematics_base/kinematics_base.h>
#include <gtest/gtest.h>
#include <mrx_t4_arm_kinematics/inverse_kinematics.h>

using namespace mrx_t4_arm_kinematics;

TEST(inverse_kinematics, carttojnt_test)
{
    
    KDL::JntArray q_init(4);
    for (int i=0;i<4;i++)
    {
        q_init(i) = 0;
    }

    KDL::Frame p_in;
    p_in.p.x(0.655);
    p_in.p.y(0);
    p_in.p.z(0.257);
    const KDL::Vector ux(1, 0 ,0);
    const KDL::Vector uy(1, 0 ,0);
    const KDL::Vector uz(1, 0 ,0);

    p_in.M = KDL::Rotation(ux, uy, uz);

    const KDL::JntArray q_init_= q_init;
    const KDL::Frame p_in_ = p_in;
    std::vector<KDL::JntArray> q_out;
    InvesreKinematics ik;
    EXPECT_TRUE(ik.CartToJnt(q_init_, p_in_, q_out));
    EXPECT_EQ(0,q_out_[0](0));
    EXPECT_EQ(0,q_out_[0](1));
    EXPECT_EQ(0,q_out_[0](2));
    EXPECT_EQ(0,q_out_[0](3));
}

