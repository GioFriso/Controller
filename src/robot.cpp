//This receives a call from the controller (server) and computes the position of the obstacles in order to achieve the final pose avoiding obstacles
//It gives the feedback to the controller
// After the task finishes it gives the position of the obstacles to the server

#include"ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "controller/final_poseActionAction.h"
#include <sstream>
#include<string>

class final_poseActionAction{
	protected:
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<controller::final_poseActionAction> as_;
		std::string action_name_;
		controller::final_poseActionFeedback feedback_;
		controller::final_poseActionResult result_;

	public:
		final_poseActionAction(std::string name) : as_(nh_,name,boost::bind(&final_poseActionAction::executeCB,this,_1),false),action_name_(name){
		as_.start();
	}

	void executeCB(const controller::final_poseActionGoalConstPtr &goal){
		ros::Rate r(1);//1 message per second, i guess
		bool success = true;
		for(int i=0;i<1;i++){
			if(as_.isPreemptRequested() || !ros::ok()){
				ROS_INFO("%s: Preempted",action_name_.c_str());
				as_.setPreempted();
				success = false;
			}
			feedback_.current_x = goal -> pose.x;
			feedback_.current_y = goal -> pose.y;
			feedback_.current_theta = goal -> pose.theta;
			as_.publishFeedback(feedback_);
			r.sleep();
		}
		if(success){
			result_.goal_reached = true;
			ROS_INFO("%s, succedeed",action_name_.c_str());
			as_.setSucceeded(result_);
		}
	}
	~final_poseActionAction(void){}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robot");
	/*for (int i = 1; i<4; i++)
	{
		ROS_INFO("argv %d: %s", i, argv[i]);
	}*/
	ros::init(argc, argv, "Robot_doing_things");
	final_poseActionAction final_pose("final_poseAction");
	ros::spin();
	return 0;
}
