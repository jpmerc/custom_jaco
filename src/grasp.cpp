#include <ros/ros.h>
// PCL specific includes

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <jaco_msgs/SetFingersPositionAction.h>
#include <jaco_msgs/ArmPoseAction.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

geometry_msgs::PoseStamped arm_pose;

boost::mutex mtx_;

bool can_update = true;
bool end_program = false;

void grasp();
void move_up();

void arm_position_callback (const geometry_msgs::PoseStampedConstPtr& input_pose){
    mtx_.lock();
    arm_pose = *input_pose;
    mtx_.unlock();
}

void thread_function(){
    if(ros::ok()){
        //grasp();
        move_up();
        end_program = true;
    }
}


void grasp(){
    actionlib::SimpleActionClient<jaco_msgs::SetFingersPositionAction> action_client("jaco/finger_joint_angles",true);
    action_client.waitForServer();
    jaco_msgs::SetFingersPositionGoal fingers = jaco_msgs::SetFingersPositionGoal();
    fingers.fingers.Finger_1 = 60;
    fingers.fingers.Finger_2 = 60;
    fingers.fingers.Finger_3 = 60;
    action_client.sendGoal(fingers);
    action_client.waitForResult(ros::Duration(5));
    std::cout << "Grasp completed" << std::endl;
}


void move_up(){
    actionlib::SimpleActionClient<jaco_msgs::ArmPoseAction> action_client("/jaco/arm_pose",true);
    action_client.waitForServer();
    jaco_msgs::ArmPoseGoal pose_goal = jaco_msgs::ArmPoseGoal();

    mtx_.lock();
    //pose_goal.pose.header.frame_id = "/jaco_api_origin";
    //    pose_goal.pose.pose.position.x = -0.27;
    //    pose_goal.pose.pose.position.y = 0.41;
    //    pose_goal.pose.pose.position.z = 0.6;
    //    pose_goal.pose.pose.orientation.x = 0.95;
    //    pose_goal.pose.pose.orientation.y = -0.02;
    //    pose_goal.pose.pose.orientation.z = 0.3;
    //    pose_goal.pose.pose.orientation.w = -0.09;
    pose_goal.pose = arm_pose;
    pose_goal.pose.header.frame_id = "/jaco_api_origin";
    pose_goal.pose.pose.position.z += 0.1;

    mtx_.unlock();
    action_client.sendGoal(pose_goal);
    action_client.waitForResult(); //unlimited time for displacement at the moment
    std::cout << "Finished moving up" << std::endl;
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "grasp");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    ros::Subscriber sub = n.subscribe ("/jaco/tool_position", 1, arm_position_callback);
    boost::thread thread_(thread_function);

    ros::Rate r(30);
    while(ros::ok() && !end_program){
        ros::spinOnce();
        r.sleep();
    }
    thread_.join();

    return 0;
}
