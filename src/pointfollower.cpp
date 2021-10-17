#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <vector>

#define _USE_MATH_DEFINES
#define MAX_LIN_VEL 1
#define MIN_LIN_VEL 0
#define MAX_ANG_VEL 1
#define MIN_ANG_VEL -1
#define MAX_DIST 0.5
#define MIN_ANG_DIFF 5

using namespace std;

vector<geometry_msgs::PoseStamped> planned_poses;

geometry_msgs::Twist cmd_vel_mapping(double dist, double max_dist, double target_angle, double min_ang_diff) {
	double vx = 0; // linear velocity
	double wz = 0; // angular velocity

	if (dist > max_dist)
		vx = MAX_LIN_VEL;
	else if (max_dist * 0.5 <= dist && dist <= max_dist)
		vx = round(MAX_LIN_VEL * 0.6666);
	else if (0 < dist && dist < max_dist * 0.5)
		vx = round(MAX_LIN_VEL * 0.3333);
	
	// if target angle is positive: turn left
	// if target angle is negative: turn right
	if (90 < target_angle && target_angle < 180)
        wz = MIN_ANG_VEL;
	else if (min_ang_diff < target_angle && target_angle < 90)
		wz = MIN_ANG_VEL * 0.5;
	else if (fabs(target_angle) < min_ang_diff)
		wz = 0;
	else if (-90 < target_angle && target_angle < -min_ang_diff)
		wz = MAX_ANG_VEL * 0.5;
	else if (-180 < target_angle && target_angle < -90)
		wz = MAX_ANG_VEL;

	geometry_msgs::Vector3 lin;
    lin.x = vx;
    geometry_msgs::Vector3 ang;
    ang.z = wz;
    geometry_msgs::Twist result;
    result.linear = lin;
    result.angular = ang;

    return result;
}

void pathCallback(const nav_msgs::Path::ConstPtr& path) {
    planned_poses = path->poses;
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "pointfollower");
    ros::NodeHandle nh;
    ros::NodeHandle nh_prv("~");

    ros::Subscriber plan_sub = nh.subscribe("global_plan", 1000, pathCallback);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
    tf::TransformListener listener;

    int wp_idx = 0;
    int n_waypoint = planned_poses.size();
    int close_cnt = 0;

    ros::Rate r(10);

    while (ros::ok()) {

        tf::StampedTransform transform;
        try {
            listener.lookupTransform("/odom", "/map", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        tf::Vector3 current_pos = transform.getOrigin();
        tf::Quaternion current_dir = transform.getRotation();

        geometry_msgs::Point target_pos = planned_poses[wp_idx].pose.position;
        tf::Vector3 target_pos_v(target_pos.x, target_pos.y, target_pos.z);
        geometry_msgs::Quaternion target_dir = planned_poses[wp_idx].pose.orientation;
        tf::Quaternion target_dir_q(target_dir.x, target_dir.y, target_dir.z, target_dir.w);

        tf::Vector3 v1;
        v1 = tf::quatRotate(current_dir, v1);
        tf::Vector3 v2;
        v2 = tf::quatRotate(target_dir_q, v2);

        double angle = v1.angle(v2) * 180 / M_PI;
        double dist = current_pos.distance(target_pos_v);

        geometry_msgs::Twist cmd_vel = cmd_vel_mapping(dist, MAX_DIST, angle, MIN_ANG_DIFF);

        cmd_vel_pub.publish(cmd_vel);

        if (wp_idx < n_waypoint)
			if (dist < MAX_DIST)
				close_cnt++;
		else {
			geometry_msgs::Twist stop_vel;
            geometry_msgs::Vector3 lin;
            geometry_msgs::Vector3 ang;
			stop_vel.linear = lin;
			stop_vel.angular = ang;
			cmd_vel_pub.publish(stop_vel);
			// stop ship exception throw
        }

		if (close_cnt > 1) {
            if (wp_idx < n_waypoint - 1) {
				ROS_INFO_STREAM("reached at a waypoint. move to next waypoint");
				ros::Duration(1).sleep();
				close_cnt = 0;
				wp_idx += 1;
            }
			else if (wp_idx == n_waypoint - 1) {
				ROS_INFO_STREAM("reached final waypoint. hopping tour finished");
			    geometry_msgs::Twist stop_vel;
                geometry_msgs::Vector3 lin;
                geometry_msgs::Vector3 ang;
			    stop_vel.linear = lin;
			    stop_vel.angular = ang;
			    cmd_vel_pub.publish(stop_vel);
				ros::Duration(600).sleep();
            }

        }

        r.sleep();
    }
}
