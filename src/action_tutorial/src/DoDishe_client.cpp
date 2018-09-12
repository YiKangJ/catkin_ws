#include <actionlib/client/simple_action_client.h>
#include "action_tutorial/DoDishesAction.h"

typedef actionlib::SimpleActionClient<action_tutorial::DoDishesAction> Client;

void doneCb(const actionlib::SimpleClientGoalState &state,
            const action_tutorial::DoDishesResultConstPtr &result)
{
    ROS_INFO("yay! The dishes are now clean");
    ros::shutdown();
}

void activeCb()
{
    ROS_INFO("Goal just went active");
}

void feedbackCb(const action_tutorial::DoDishesFeedbackConstPtr &feedback)
{
    ROS_INFO(" percent_complete : %f ", feedback->percent_complete);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "do_dishes_client");

    Client client("do_dishes", true);
    
    ROS_INFO("Waiting for action server to start.");
    client.waitForServer();
    ROS_INFO("Action srver started, sending goal.");

    action_tutorial::DoDishesGoal goal;
    goal.dishwasher_id = 1;

    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    ros::spin();

    return 0;
}
