#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "action_tutorial/DoDishesAction.h"

typedef actionlib::SimpleActionServer<action_tutorial::DoDishesAction> Server;

void execute(const action_tutorial::DoDishesGoalConstPtr &goal, Server *as
)
{
    ros::Rate r(1);
    action_tutorial::DoDishesFeedback feedback;

    ROS_INFO("Dishwasher %d is working.", goal->dishwasher_id);

    for(int i=1;i<=10;i++)
    {
        feedback.percent_complete = i * 10;
        as->publishFeedback(feedback);
        r.sleep();
    }

    ROS_INFO("Disheswasher %d finish working.", goal->dishwasher_id);
    as->setSucceeded();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "do_dishes_server");
    ros::NodeHandle n;
    
    Server server(n, "do_dishes", boost::bind(&execute, _1, &server), false);
    server.start();
    ros::spin();

    return 0;
}
