#include <ros/ros.h>
#include <tf/tf.h>
#include <interactive_markers/interactive_marker_server.h>
using namespace visualization_msgs;

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM(feedback->marker_name << " is now at \n"
      << "Position:\n"
      << "   x:" << feedback->pose.position.x << "\n" 
      << "   y:" << feedback->pose.position.y << "\n" 
      << "   z:" << feedback->pose.position.z << "\n"
      << "Orientation:\n"
      << "   w:" << feedback->pose.orientation.w << "\n" 
      << "   x:" << feedback->pose.orientation.x << "\n" 
      << "   y:" << feedback->pose.orientation.y << "\n" 
      << "   z:" << feedback->pose.orientation.z << "\n");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ee_goal_marker");

    ros::NodeHandle nh("~");
    if (!nh.hasParam("world_frame"))
    {
        ROS_FATAL("Missing node parameter world_frame. Finishing node");
        return -1;
    }

    std::string world_frame;
    nh.getParam("world_frame", world_frame);
    interactive_markers::InteractiveMarkerServer server("ee_goal_marker_server");
    InteractiveMarker int_marker;
    int_marker.header.frame_id = world_frame;
    int_marker.header.stamp = ros::Time::now();
    int_marker.name = "ee_goal_marker";
    int_marker.description = "ee 6-DOF interactive control";

    Marker box_marker;
    box_marker.type = Marker::CUBE;
    box_marker.scale.x = 0.1;
    box_marker.scale.y = 0.1;
    box_marker.scale.z = 0.1;
    box_marker.color.r = 0.5;
    box_marker.color.g = 0.5;
    box_marker.color.b = 0.5;
    box_marker.color.a = 1.0;

    // non-interactive control which contains the box
    InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back(box_marker);

    // add the control to the interactive marker
    int_marker.controls.push_back(box_control);
    int_marker.scale = 0.3;
 
    // Create the controls to translate and rotate the box
    InteractiveMarkerControl trans_control;
    trans_control.name = "MOVE_3D";
    trans_control.orientation_mode = InteractiveMarkerControl::FIXED;
    trans_control.interaction_mode = InteractiveMarkerControl::MOVE_3D;

    InteractiveMarkerControl rotate_control;
    rotate_control.name = "ROTATE_3D";
    rotate_control.interaction_mode = InteractiveMarkerControl::ROTATE_3D;

    //Add controls for each degree of freedom
    tf::Quaternion orien(1.0, 0.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, trans_control.orientation);
    trans_control.name = "move_x";
    trans_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(trans_control);
    tf::quaternionTFToMsg(orien, rotate_control.orientation);
    rotate_control.name = "rotate_x";    
    rotate_control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;    
    int_marker.controls.push_back(rotate_control);
    
    orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, trans_control.orientation);
    trans_control.name = "move_y";
    trans_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(trans_control);
    tf::quaternionTFToMsg(orien, rotate_control.orientation);
    rotate_control.name = "rotate_y";    
    rotate_control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;    
    int_marker.controls.push_back(rotate_control);

    orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, trans_control.orientation);
    trans_control.name = "move_z";
    trans_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(trans_control);
    tf::quaternionTFToMsg(orien, rotate_control.orientation);
    rotate_control.name = "rotate_z";    
    rotate_control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;    
    int_marker.controls.push_back(rotate_control);

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker);
    server.setCallback("ee_goal_marker", &processFeedback);

    // commit changes and send to all clients
    server.applyChanges();

    ros::spin();
}