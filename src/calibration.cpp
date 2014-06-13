#include <ros/ros.h>
#include <angles/angles.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


using namespace std;


void calibrate();
void printPose(string str, tf::Transform &in_pose);


void calibrate(){

    // Listens to tf camera_link -> AR_OBJECT launched by ar_kinect
    tf::TransformListener listener_cam_ar;
    tf::StampedTransform artag_kinect_referential;
    listener_cam_ar.waitForTransform("camera_link","AR_OBJECT",ros::Time(0),ros::Duration(3.0));
    listener_cam_ar.lookupTransform("camera_link","AR_OBJECT",ros::Time(0),artag_kinect_referential);

    //original_ar_object_pose.rotate(tf::Vector3(0,1,0),angles::from_degrees(-15));


    // Transforms AR_OBJECT to the same orientation of camera_link frame to facilitate the calculations afterwards
    artag_kinect_referential *= tf::Transform(tf::createQuaternionFromRPY(angles::from_degrees(-90),angles::from_degrees(90),angles::from_degrees(0)));


    // Save for later
    tf::Pose initial_pose_ar = artag_kinect_referential;


    // Publish tf to visualize the results in rviz
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(artag_kinect_referential, ros::Time::now(), "camera_link", "AR_OBJECT_REORIENTED"));
    // printPose("TF", tf_to_same_orientation);


    //Add the position difference of the marker to jaco_joint_6
    tf::TransformListener listener;
    tf::StampedTransform artag_jaco_referential;
    listener.waitForTransform("arm_base","ARtag",ros::Time(0),ros::Duration(3.0));
    listener.lookupTransform("arm_base","ARtag",ros::Time(0),artag_jaco_referential);

    // ARtag with good orientation
    tf::Transform artag_same_orientation = tf::Transform(tf::createQuaternionFromRPY(angles::from_degrees(-90),angles::from_degrees(0),angles::from_degrees(-90)));
    artag_jaco_referential *= artag_same_orientation;
    static tf::TransformBroadcaster br5;
    br5.sendTransform(tf::StampedTransform(artag_jaco_referential, ros::Time::now(), "arm_base", "ARtag_REORIENTED"));


    // ARtag offset (from static_tf in launch file) (adjusted manually)
    // Could be removed if the ARtag position in the real world was measure precisely with respect to the joints
    tf::Transform t = tf::Transform(tf::createQuaternionFromRPY(angles::from_degrees(0),angles::from_degrees(-11),angles::from_degrees(2)),tf::Vector3(-0.004,-0.045,-0.015));
    static tf::TransformBroadcaster br3;
    br3.sendTransform(tf::StampedTransform(t, ros::Time::now(), "ARtag_REORIENTED", "ARtag_OFFSET"));
    artag_jaco_referential *= t;


    //Calculate the inverse transform to get the difference between arm_base and camera_link
    //tf::Transform trans = artag_jaco_referential * artag_kinect_referential.inverse();
    tf::Vector3 translation = artag_jaco_referential.getOrigin() - artag_kinect_referential.getOrigin();
    tf::Transform trans = tf::Transform(artag_kinect_referential.getBasis().transposeTimes(artag_jaco_referential.getBasis()), translation);


    //Rotate a point to get the good Z value depending on the tilt angle (pitch) of the camera
    double roll,pitch,yaw;
    trans.getBasis().getRPY(roll,pitch,yaw);
    tf::Pose kin = tf::Transform(tf::createQuaternionFromRPY(0,-pitch,0),tf::Vector3(0,0,0));
    tf::Transform tilt_correction = kin.inverseTimes(initial_pose_ar);
    artag_kinect_referential.getOrigin().setZ(tilt_correction.getOrigin().getZ());

    // repeat, but neccessary due to tilt correction
    translation = artag_jaco_referential.getOrigin() - artag_kinect_referential.getOrigin();
    trans.setOrigin(translation);

    //printPose("Transformation",trans);

    // Publish tf to visualize the results in rviz
    static tf::TransformBroadcaster br2;
    br.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "arm_base", "camera_link_calculated"));

    printPose("JACO",artag_jaco_referential);
    printPose("Kinect",artag_kinect_referential);
    printPose("Tranform", trans);

}



void printPose(string str, tf::Transform &in_pose){
    cout << str << " : " << endl;
    cout << "x: " << in_pose.getOrigin().getX() << endl;
    cout << "y: " << in_pose.getOrigin().getY() << endl;
    cout << "z: " << in_pose.getOrigin().getZ() << endl;
    cout << "rotx: " << in_pose.getRotation().getX() << endl;
    cout << "roty: " << in_pose.getRotation().getY() << endl;
    cout << "rotz: " << in_pose.getRotation().getZ() << endl;
    cout << "rotw: " << in_pose.getRotation().getW() << endl;

    double roll,pitch,yaw;
    in_pose.getBasis().getRPY(roll,pitch,yaw);

    cout << "Yaw: "   << angles::to_degrees(yaw)   << " (" << yaw   << ")"   << endl;
    cout << "Pitch: " << angles::to_degrees(pitch) << " (" << pitch << ")"   << endl;
    cout << "Roll: "  << angles::to_degrees(roll)  << " (" << roll  << ")"   << endl;

    cout << endl;

}




int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "calibration");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    ros::Rate r(5);
    while(ros::ok()){
        calibrate();
    }


    return 0;
}
