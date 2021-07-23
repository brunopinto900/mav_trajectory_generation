#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <ros/spinner.h>
#include <core/State.h>
#include <core/Trajectory.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <math.h>

mav_trajectory_generation::Vertex::Vector vertices;
const int dimension = 4;
const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);
ros::Subscriber gate_poses_sub;
ros::Publisher path_viz_pub;
ros::Publisher trajectory_pub,debug_traj_pub; 

double normalize_angle(double angle) {
  while(angle > M_PI)
        angle -= 2.0 * M_PI;
  while(angle < -M_PI)
        angle += 2.0 * M_PI;

    return angle;
}

double getYaw(double qx, double qy, double qz, double qw )
{
    return normalize_angle( std::atan2( 2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz) ) );
}

void gate_poses_callback(geometry_msgs::PoseArray gate_poses) 
{
    double qx,qy,qz,qw;
    Eigen::Vector4d initial_pose; initial_pose(0) = 6;
    initial_pose(1) = 81; initial_pose(2) = -43; initial_pose(3) = -0.349;

    start.makeStartOrEnd(initial_pose, derivative_to_optimize);
    vertices.push_back(start);

    Eigen::Vector4d second_gate;
    for(int i = 0; i < ( gate_poses.poses.size() - 1 ); i++)
    {
        second_gate(0) = gate_poses.poses[i].position.x;
        second_gate(1) = gate_poses.poses[i].position.y; 
        second_gate(2) = gate_poses.poses[i].position.z;
        qx = gate_poses.poses[i].orientation.x;
        qy = gate_poses.poses[i].orientation.y;
        qz = gate_poses.poses[i].orientation.z;
        qw = gate_poses.poses[i].orientation.w;

        second_gate(3) = getYaw(qx,qy,qz,qw);
        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, second_gate);
        vertices.push_back(middle);
    }
    int last_idx =  gate_poses.poses.size() - 1;
    Eigen::Vector4d last_gate; last_gate(0) = gate_poses.poses[last_idx].position.x;
    last_gate(1) = gate_poses.poses[last_idx].position.y; last_gate(2) = gate_poses.poses[last_idx].position.z;
    qx = gate_poses.poses[last_idx].orientation.x;
    qy = gate_poses.poses[last_idx].orientation.y;
    qz = gate_poses.poses[last_idx].orientation.z;
    qw = gate_poses.poses[last_idx].orientation.w;

    last_gate(3) = getYaw(qx,qy,qz,qw);
    end.makeStartOrEnd(last_gate, derivative_to_optimize);
    vertices.push_back(end);

    //std::cout << "Vertices created\n";
    std::vector<double> segment_times;
    const double v_max = 4.0;
    const double a_max = 16.0;
    segment_times = estimateSegmentTimes(vertices, v_max, a_max);

    const int N = 10;
    mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    opt.solveLinear();
    mav_trajectory_generation::Trajectory mav_trajectory;
    opt.getTrajectory(&mav_trajectory);
    //std::cout << "Optimization solved\n";

    const double min_time = mav_trajectory.getMinTime();
    const double max_time = mav_trajectory.getMaxTime();
    double time_iterator = min_time;
    double dt = 0.01;
    int position = mav_trajectory_generation::derivative_order::POSITION;
    int velocity = mav_trajectory_generation::derivative_order::VELOCITY;
    int acceleration = mav_trajectory_generation::derivative_order::ACCELERATION;

    geometry_msgs::PoseStamped pose;
    Eigen::VectorXd sample;
    nav_msgs::Path path;
    geometry_msgs::PoseStamped debug_traj;

    core::Trajectory trajectory;
    trajectory.start_time = min_time;
    trajectory.end_time = max_time;
    trajectory.dt = dt;

    core::State state;

    //std::cout << "Evaluating trajectories\n";
    for(time_iterator; time_iterator <= max_time; time_iterator+=dt) 
    {
      
        sample = mav_trajectory.evaluate(time_iterator, position);
        pose.header.frame_id = "world_ned";
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = sample(0); pose.pose.position.y = sample(1); pose.pose.position.z = sample(2);
        
        
        debug_traj.header.frame_id = "world_ned";
        debug_traj.header.stamp = pose.header.stamp;
        debug_traj.pose.position.x = pose.pose.position.x;
        debug_traj.pose.position.y = pose.pose.position.y;
        debug_traj.pose.position.z = pose.pose.position.z;
        

        // Positions
        state.position.x = sample(0); state.position.y = sample(1); state.position.z = sample(2);
        state.yaw = atan2(sample(3),1);

        
      
        //Velocites
        sample = mav_trajectory.evaluate(time_iterator, velocity);
        state.velocity.x = sample(0); state.velocity.y = sample(1); state.velocity.z = sample(2);

        //double yaw = atan2(state.velocity.y,state.velocity.x);
        state.yaw = atan2(state.velocity.y,state.velocity.x);
        tf2::Quaternion q;
        q.setRPY( 0, 0, state.yaw );  // Create this quaternion from roll/pitch/yaw (in radian
        q.normalize();

        geometry_msgs::Quaternion quat_msg;
        quat_msg = tf2::toMsg(q);
        pose.pose.orientation = quat_msg;

        debug_traj.pose.orientation = quat_msg;
        debug_traj_pub.publish(debug_traj);
        path.poses.push_back(pose);
 
       
        // Accelerations
        sample = mav_trajectory.evaluate(time_iterator, acceleration);
        state.acceleration.x = sample(0); state.acceleration.y = sample(1); state.acceleration.z = sample(2);
      
        state.time_instant = time_iterator;
        trajectory.state.push_back(state);

        
    }
    
    path.header.frame_id = "world_ned";
    path.header.stamp = ros::Time::now();
    path_viz_pub.publish(path);
    
    trajectory.header.frame_id = "world_ned";
    trajectory.header.stamp = ros::Time::now();
    
   
    trajectory_pub.publish(trajectory);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "minimum_snap");
    ros::NodeHandle nh;
    gate_poses_sub = nh.subscribe("/gates_ground_truth", 100, gate_poses_callback);
    path_viz_pub = nh.advertise<nav_msgs::Path>("/minimum_snap_path",100);
    trajectory_pub = nh.advertise<core::Trajectory>("/trajectory",100);
    debug_traj_pub = nh.advertise<geometry_msgs::PoseStamped>("/debug_trajectory",100);

    ros::spin();
    
    return 0;
}