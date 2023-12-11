// This receives the final pose from the user (client)

#include"ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "controller/final_poseActionAction.h"
#include <stdlib.h>

typedef actionlib::SimpleActionClient<controller::final_poseActionAction> actionClient;

void feedbackCallback(const controller::final_poseActionFeedbackConstPtr& feedback)
{
	ROS_INFO("[%f,\t %f,\t %f]", feedback->current_x, feedback->current_y, feedback->current_theta);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller");
	/*for (int i = 1; i<4; i++)
	{
		ROS_INFO("argv %d: %s", i, argv[i]);
	}*/
	
	if (argc!=4){ROS_ERROR("WRONG NUMBER OF PARAMETERS"); return 0;}
	
	actionlib::SimpleActionClient<controller::final_poseActionAction> ac("final_poseAction",true);
	ROS_INFO("Waiting for action server to start");
	ac.waitForServer();
	ROS_INFO("Action server started,sending goal.");
	controller::final_poseActionGoal goal;
	goal.pose.x = atof(argv[1]);
	goal.pose.y = atof(argv[2]);
	goal.pose.theta = atof(argv[3]);
	controller::final_poseActionFeedback feedback;
	ac.sendGoal(goal, actionClient::SimpleDoneCallback(), actionClient::SimpleActiveCallback(), boost::bind(&feedbackCallback, _1));
	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
	if(finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
	{
		ROS_INFO("Action did not finish before the time out.");
	}
}
