/*
TODO:
-SEND A SERVICE TO THE HUMAN NODE WHICH TELLS YOU THE OBJECT YOU HAVE TO PICK
-CALL THE SERVER FROM AS1
-DEFINE ROUTINES FOR EACH SINGLE OBJECT SUCH THAT YOU CAN HAVE A PERFECT NAVVIGATION
-COMUNICATE YOUR STATUS TO NODE B
-RECEIVE INFORMATIONS FROM NODE C
-HOPE THAT NAVIGATION FROM AS 1 WWORKS PROPERLY BITCH
*/
#include"ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "controller/final_poseActionAction.h"
#include <stdlib.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "tiago_iaslab_simulation/Objs.h"

typedef actionlib::SimpleActionClient<controller::final_poseActionAction> actionClient;

void feedbackCallback(const controller::final_poseActionFeedbackConstPtr& feedback)
{
	//ROS_INFO("TIAGO %s AND  IT IS CURRENTLY HERE: [%f,%f]",feedback->status.c_str(), feedback->current_x, feedback->current_y);
}

//CONSTANTS DEFINITION
const float tiago_starting_x = -6.58;
const float tiago_starting_y = 1.37;
const float mark1_x = 1.93;
const float mark1_y = 1.27;
const float mark1_theta = -91.0;//IT ODES NOTHING APPARENTLY :(
const float markBlue_x = 1.41;
const float markBlue_y = -0.85;
const float markBlue_theta = -91.0;
const float markRed_x = 0.87;
const float markRed_y = -0.85;
const float markRed_theta = -91.0;
const float markGreen_x = 1.12;
const float markGreen_y = -2.75;
const float markGreen_theta = -91.0;
//IF IT IS THE FIRST OBJECT THAT TIAGO AS TO PICK, THE ROUTINE IS A BIT DIFFERENT, AT LEAST IN THE INITIAL PART
void firstRoutine(actionlib::SimpleActionClient<controller::final_poseActionAction>* ac, int id){
    ROS_INFO("Waiting for action server to start");
	ac->waitForServer();
	ROS_INFO("Action server started,sending goal.");
	controller::final_poseActionGoal goal;
    controller::final_poseActionFeedback feedback;
    goal.pose.x = mark1_x-tiago_starting_x;//EVERYTHING IS GONNA BE A CONSTANT, IN A BIT THO
    goal.pose.y = mark1_y-tiago_starting_y;
    goal.pose.theta = mark1_theta;                 //YET TO BE DEFINED, BUT I WAS THINKING MAYBE -PI/2
    ac->sendGoal(goal, actionClient::SimpleDoneCallback(), actionClient::SimpleActiveCallback(), boost::bind(&feedbackCallback, _1));
    bool finished_before_timeout = ac->waitForResult(ros::Duration(60.0));
    if(finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac->getState();
        //ROS_INFO("Action finished: %s",state.toString().c_str());
        ROS_INFO("TIAGO REACHED MARK 1");
    }
    else
    {
        ROS_INFO("Action did not finish before the time out.");
    }
    ros::Rate wait_for_plan(0.20);
    wait_for_plan.sleep();
    switch (id)
    {
    case 1:
        goal.pose.x = markBlue_x-tiago_starting_x;
        goal.pose.y = markBlue_y-tiago_starting_y;
        goal.pose.theta = markBlue_theta;
        ac->sendGoal(goal, actionClient::SimpleDoneCallback(), actionClient::SimpleActiveCallback(), boost::bind(&feedbackCallback, _1));
        finished_before_timeout = ac->waitForResult(ros::Duration(60.0));
        if(finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac->getState();
            //ROS_INFO("Action finished: %s",state.toString().c_str());
            ROS_INFO("TIAGO REACHED MARK BLUE");
        }
        else
        {
            ROS_INFO("Action did not finish before the time out.");
        }
        break;
    case 2:
        goal.pose.x = markGreen_x-tiago_starting_x;
        goal.pose.y = markGreen_y-tiago_starting_y;
        goal.pose.theta = markGreen_theta;
        ac->sendGoal(goal, actionClient::SimpleDoneCallback(), actionClient::SimpleActiveCallback(), boost::bind(&feedbackCallback, _1));
        finished_before_timeout = ac->waitForResult(ros::Duration(60.0));
        if(finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac->getState();
            //ROS_INFO("Action finished: %s",state.toString().c_str());
            ROS_INFO("TIAGO REACHED MARK GREEN");
        }
        else
        {
            ROS_INFO("Action did not finish before the time out.");
        }
        break;
    case 3:
        goal.pose.x = markRed_x-tiago_starting_x;
        goal.pose.y = markRed_y-tiago_starting_y;
        goal.pose.theta = markRed_theta;
        ac->sendGoal(goal, actionClient::SimpleDoneCallback(), actionClient::SimpleActiveCallback(), boost::bind(&feedbackCallback, _1));
        finished_before_timeout = ac->waitForResult(ros::Duration(60.0));
        if(finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac->getState();
            //ROS_INFO("Action finished: %s",state.toString().c_str());
            ROS_INFO("TIAGO REACHED MARK RED");
        }
        else
        {
            ROS_INFO("Action did not finish before the time out.");
        }
        break;
    }
}


void blueRoutine(bool isFirst){
    actionlib::SimpleActionClient<controller::final_poseActionAction> ac("final_poseAction",true);
	/*ROS_INFO("Waiting for action server to start");
	ac.waitForServer();
	ROS_INFO("Action server started,sending goal.");
	controller::final_poseActionGoal goal;
    controller::final_poseActionFeedback feedback;*/
    if(isFirst){
        firstRoutine(&ac,1);
        //DO THE FIRST STEPS OF THE ROUTINE, FROM THE CORRIDOR TO THE FIRST LANDMARK, THEN TO THE BLUE OBJECT, THEN TO THE RECOVERY
    }
    else{
        //THE NAVIGGATION GOES FROM THE RECOVERY
    }
    ROS_INFO("DOING THE BLUE ROUTINE");
}
void redRoutine(bool isFirst){
    actionlib::SimpleActionClient<controller::final_poseActionAction> ac("final_poseAction",true);
    if(isFirst){
        firstRoutine(&ac,3);
        //DO THE FIRST STEPS OF THE ROUTINE, FROM THE CORRIDOR TO THE FIRST LANDMARK, THEN TO THE BLUE OBJECT, THEN TO THE RECOVERY
    }
    else{
        //THE NAVIGGATION GOES FROM THE RECOVERY
    }
    ROS_INFO("DOING THE RED ROUTINE");
}
void greenroutine(bool isFirst){
    actionlib::SimpleActionClient<controller::final_poseActionAction> ac("final_poseAction",true);
    if(isFirst){
        firstRoutine(&ac,2);
        //DO THE FIRST STEPS OF THE ROUTINE, FROM THE CORRIDOR TO THE FIRST LANDMARK, THEN TO THE BLUE OBJECT, THEN TO THE RECOVERY
    }
    else{
        //THE NAVIGGATION GOES FROM THE RECOVERY
    }
    ROS_INFO("DOING THE GREEN ROUTINE");
}


//THIS FUNCTION IS TO CHECK WHICH ROUTINE TO CALL SINCE WE HAVE TO IMPLEMENT AN AD HOC ROUTINE PER EACH OBJECT
//IT IS GONNA CALL A SPECIFIC ROUTINE ACCORDING TO THE OBJECT'S ID AND IF IT IS THE FIRST OBJECT TO COLLECT OR NO
void chooseRoutine(int id, bool isFirst){
    if(id==1){
        //DO ROUTINE 1
        ROS_INFO("GONNA DO ROUTINE 1");
        blueRoutine(isFirst);
    }
    else if(id==2){
        //DO ROUTINE 2
        ROS_INFO("GONNA DO ROUTINE 2");
        greenroutine(isFirst);
    }
    else{
        //DO ROUTINE 3
        ROS_INFO("GONNA DO ROUTINE 3");
        redRoutine(isFirst);
    }
}

int main(int argc, char** argv){
    ros::init(argc,argv,"node_a");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.33);
    //FLAG THAT WILL TELL ME IF THE IT IS THE FIRST OBJECT TO PICK OR NO
    bool isFirst = true;
    //HERE I'M CREATING THE SERVICE AND I'M SENDINGG IT TO THE HUMAN NODE
    ros::ServiceClient humanClient = n.serviceClient<tiago_iaslab_simulation::Objs>("/human_objects_srv");
    tiago_iaslab_simulation::Objs humanService;
    //LET'S GO FOR NOW WITH JUST THE FIRST BOOL EQUAL TO TRUE
    humanService.request.ready = true;
    humanService.request.all_objs = false;
    //SPACE FOR THE EXTRA POINTS GOES HERE, MAYBE

    //CHECKS ON THE SERVICE STATUS
    while(ros::ok()){
        if(humanClient.call(humanService)){
            //DO YOUR STAFF
            int temp_id = humanService.response.ids[0];
            ROS_INFO("IDs: [%d]",temp_id);
            chooseRoutine(temp_id,isFirst);
        }
        else{
            ROS_ERROR("SOMETHING WENT WRONG WHILE CONTACTING THE HUMAN_NODE...");
        }
        break;
        ros::spinOnce();
        loop_rate.sleep();
        //isFirst = false;
    }
    return 0;
}