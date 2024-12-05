/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/rc_channels.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <std_msgs/msg/int32.hpp>

#include <nav_msgs/msg/path.hpp>

#include <radar_cable_follower_msgs/msg/tracked_powerlines.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include <stdint.h>
#include <math.h>  
#include <limits>
#include <mutex>
#include <chrono>
#include <iostream>
#include <vector>


#include "geometry.h"

// PCL includes
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/angles.h>
#include <pcl/filters/extract_indices.h>


#define PI 3.14159265
#define NAN_ std::numeric_limits<double>::quiet_NaN()

using namespace std::chrono;
using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node {
public:
	OffboardControl() : Node("offboard_control") {
		_offboard_control_mode_publisher =
			this->create_publisher<px4_msgs::msg::OffboardControlMode>("fmu/offboard_control_mode/in", 10);
		_trajectory_setpoint_publisher =
			this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("fmu/trajectory_setpoint/in", 10);
		_vehicle_command_publisher =
			this->create_publisher<px4_msgs::msg::VehicleCommand>("fmu/vehicle_command/in", 10);
		_marker_pub = 
			this->create_publisher<visualization_msgs::msg::MarkerArray>("/markers", 10);

		this->declare_parameter<float>("yaw_frac", 0.25);
		this->declare_parameter<float>("pos_frac", 0.5);

		this->declare_parameter<int>("launch_with_debug", 1);
		this->get_parameter("launch_with_debug", _launch_with_debug);

		this->declare_parameter<float>("take_off_to_height", 0.0);
		this->get_parameter("take_off_to_height", _takeoff_height);

		this->declare_parameter<float>("max_decelration", 1.5);
		this->get_parameter("max_decelration", _max_decelration);

		this->declare_parameter<float>("max_x_vel", 10);
		this->get_parameter("max_x_vel", _max_x_vel);

		this->declare_parameter<float>("max_y_vel", 10);
		this->get_parameter("max_y_vel", _max_y_vel);

		this->declare_parameter<float>("max_z_vel", 3);
		this->get_parameter("max_z_vel", _max_z_vel);

		this->declare_parameter<float>("max_yaw_rate", 1.570796); // rad/s
		this->get_parameter("max_yaw_rate", _max_yaw_rate);

		this->declare_parameter<float>("caution_sphere_radius", 5.0); //1.5); // meters
		this->get_parameter("caution_sphere_radius", _caution_sphere_radius);

		this->declare_parameter<float>("safety_sphere_radius", 1.5); // 0.75 meters
		this->get_parameter("safety_sphere_radius", _safety_sphere_radius);

		this->declare_parameter<float>("min_input_velocity", 0.5); // 
		this->get_parameter("min_input_velocity", _min_input_velocity);


		_vehicle_odometry_subscriber = create_subscription<px4_msgs::msg::VehicleOdometry>(
			"/fmu/vehicle_odometry/out", 10,
			[this](px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg) {
              _x_vel = msg->vx; // Negated to get NED
			  _y_vel = -msg->vy;
			  _z_vel = -msg->vz;
			});


		_manual_control_input_sub = create_subscription<px4_msgs::msg::ManualControlSetpoint>(
			"/fmu/manual_control_setpoint/out", 10,
			[this](px4_msgs::msg::ManualControlSetpoint::ConstSharedPtr msg) {
              _input_x_vel = msg->x; 
			  _input_y_vel = -msg->y; // Negated to get NED
			  _input_z_vel = (msg->z*2.0)-1.0;
			  _input_yaw_rate = -msg->r; // Negated to get NED
			});


		// VehicleStatus: https://github.com/PX4/px4_msgs/blob/master/msg/VehicleStatus.msg
		_vehicle_status_sub = create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/vehicle_status/out", 10,
            [this](px4_msgs::msg::VehicleStatus::ConstSharedPtr msg) {
              _arming_state = msg->arming_state;
              _nav_state = msg->nav_state;
			});


		_rc_channels_sub = this->create_subscription<px4_msgs::msg::RcChannels>(
			"/fmu/rc_channels/out",	10,
            [this](px4_msgs::msg::RcChannels::ConstSharedPtr msg) {
              _rc_misc_state = msg->channels[7];
			  _rc_height_state = msg->channels[6];
			  
			});


		_combined_pcl_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			"/combined_pcl", 10,
			std::bind(&OffboardControl::pclmsg_to_pcl, this, std::placeholders::_1));



		_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/debug_pose", 10);
		_manual_path_pub = this->create_publisher<nav_msgs::msg::Path>("/manual_path", 10);
		_offboard_path_pub = this->create_publisher<nav_msgs::msg::Path>("/offboard_path", 10);


		tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
		transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


		// get common timestamp
		_timesync_sub =	this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					_timestamp.store(msg->timestamp);
				});


		timer_ = this->create_wall_timer(100ms, 
				std::bind(&OffboardControl::flight_state_machine, this));

		_path_timer = this->create_wall_timer(500ms, 
				std::bind(&OffboardControl::publish_path, this));

		_timer_markers = this->create_wall_timer(50ms, std::bind(&OffboardControl::publish_markers, this));

		pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		this->_combined_points = tmp_cloud;
		
	}


	~OffboardControl() {
		RCLCPP_INFO(this->get_logger(),  "Shutting down offboard control, landing..");
		publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND); 
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
		
	}

	void arm() const;
	void disarm() const;
	


private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr _path_timer;
	rclcpp::TimerBase::SharedPtr _timer_markers;

	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr _offboard_control_mode_publisher;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _trajectory_setpoint_publisher;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_publisher;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pose_pub;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _manual_path_pub;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _offboard_path_pub;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _marker_pub;

	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr _timesync_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _vehicle_status_sub;
	rclcpp::Subscription<px4_msgs::msg::RcChannels>::SharedPtr _rc_channels_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _vehicle_odometry_subscriber;
	rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr _manual_control_input_sub;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _combined_pcl_sub;

	pcl::PointCloud<pcl::PointXYZ>::Ptr _combined_points;

	std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	std::atomic<uint64_t> _timestamp;   //!< common synced timestamped
	int _nav_state, _old_nav_state = 0;
	int _arming_state;
	geometry_msgs::msg::PoseArray::SharedPtr _powerline_array_msg;
	int _counter = 0;
	float _follow_speed = 0;
	int _selected_ID = -1;
	int _launch_with_debug;
	float _takeoff_height;
	float _max_decelration;
	float _max_x_vel;
	float _max_y_vel;
	float _max_z_vel;
	float _max_yaw_rate;
	float _input_x_vel = 0.0;
	float _input_y_vel = 0.0;
	float _input_z_vel = 0.0;
	float _input_yaw_rate;
	float _caution_sphere_radius;
	float _safety_sphere_radius;
	bool _in_caution_sphere = 0;
	float _min_input_velocity;

	bool _new_takeoff = true;
	float _rc_misc_state = -1;
	float _prev_rc_misc_state = -2;

	float _rc_height_state = -1;
	float _prev_rc_height_state = -1;

    bool _printed_offboard = false;

	bool _in_offboard = false;

	float _x_vel = 0.0; // NED or NWU?
	float _y_vel = 0.0;
	float _z_vel = 0.0;

	std::mutex _drone_pose_mutex;
	std::mutex _powerline_mutex;

	quat_t _input_velocity_orientation_drone_RP; // input velocity quat in drone frame without drone pitch+roll
	pose_t _drone_pose; // in world coordinates North-West-Up
	pose_t _alignment_pose;

	float _hover_height = 2;

	void publish_path();
	void flight_state_machine();
	void update_drone_pose();
	void publish_offboard_control_mode() const;
	void publish_takeoff_setpoint();
	void publish_tracking_setpoint();
	void publish_hold_setpoint() const;
	void publish_setpoint(px4_msgs::msg::TrajectorySetpoint msg) const;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,
				     float param2 = 0.0) const;
	void collision_checker(std::vector<size_t> * combined_points_idx_in_caution_sphere, std::vector<size_t> * caution_distances, 
										std::vector<size_t> * combined_points_idx_in_safety_sphere, std::vector<size_t> * safety_distances, float * smallest_dist);
	vector_t speed_limiter();
	void publish_markers(); 
	void input_to_output_setpoint(); // translate sticks to velocity in offboard
	void read_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, 
										pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	void pclmsg_to_pcl(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};


void OffboardControl::pclmsg_to_pcl(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

	_combined_points->clear();
	OffboardControl::read_pointcloud(msg, _combined_points);

}


void OffboardControl::read_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, 
										pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

	// read PointCloud2 msg data
	int pcl_size = msg->width;
	uint8_t *ptr = msg->data.data();
	const uint32_t POINT_STEP = 12;

	for (size_t i = 0; i < (size_t)pcl_size; i++) 
	{
		pcl::PointXYZ point(
				(float)(*(reinterpret_cast<float*>(ptr + 0))),
				(float)(*(reinterpret_cast<float*>(ptr + 4))),
				(float)(*(reinterpret_cast<float*>(ptr + 8)))
			);

			cloud->push_back(point);

		ptr += POINT_STEP;
	}
}   



void OffboardControl::collision_checker(std::vector<size_t> * combined_points_idx_in_caution_sphere, std::vector<size_t> * caution_distances, 
										std::vector<size_t> * combined_points_idx_in_safety_sphere, std::vector<size_t> * safety_distances, float * smallest_dist) {

	// Find distances to points within caution sphere

	float point_dist = 0.0;

	_in_caution_sphere = 0;

	for (size_t i = 0; i < (size_t)_combined_points->size(); i++)
	{
		point_dist = sqrt( pow(_combined_points->at(i).x, 2) + pow(_combined_points->at(i).y, 2) + pow(_combined_points->at(i).z, 2) );

		if (point_dist < _caution_sphere_radius)
		{
			_in_caution_sphere = 1;
			combined_points_idx_in_caution_sphere->push_back(i);
			caution_distances->push_back(point_dist);

			if (point_dist < _safety_sphere_radius)
			{
				combined_points_idx_in_safety_sphere->push_back(i);
				safety_distances->push_back(point_dist);
			}

			if (point_dist < *smallest_dist)
			{
				*smallest_dist = point_dist;
			}
		}		
	}	
}



vector_t OffboardControl::speed_limiter() {

	// Assume all obstacles static in world frame
	// Predict relative motion based on drone velocity (and in future radial velocities)
	// If obstacle moves too fast towards drone (based on max drone decelaration and safety bubble size)
	//	-> reduce speed to avoid entering safety bubble
	// if obstacle inside caution sphere, progressively limit speed until 0 when obstacle touches surface of safety bubble

	std::vector<size_t> points_idx_in_caution_sphere = {};
	std::vector<size_t> distances_in_caution_sphere = {};

	std::vector<size_t> points_idx_in_safety_sphere = {};
	std::vector<size_t> distances_in_safety_sphere = {};

	float smallest_dist = 999999.9;

	collision_checker(&points_idx_in_caution_sphere, &distances_in_caution_sphere, &points_idx_in_safety_sphere, &distances_in_safety_sphere, &smallest_dist);

	float input_velocity_vector_scalar = sqrt(pow((_max_x_vel*_input_x_vel),2)+pow((_max_y_vel*_input_y_vel),2)+pow((_max_z_vel*_input_z_vel),2));


	vector_t input_velocity_vector(
		_input_x_vel,
		-_input_y_vel,
		_input_z_vel
	);

	// reduce input velocity vector magnitude inversely proportional to safety sphere proximity to obstacle
	if (smallest_dist < _caution_sphere_radius)
	{
		float closeness = smallest_dist / (_caution_sphere_radius-_safety_sphere_radius); 
		input_velocity_vector = input_velocity_vector * _min_input_velocity + input_velocity_vector * (1-_min_input_velocity) * closeness;
	}
	
	// scale vector by original vector magnitude
	input_velocity_vector = input_velocity_vector * input_velocity_vector_scalar;

	if (!_in_caution_sphere) 
	{
		return input_velocity_vector;
	}


	////////// -------- caution bubble -------- //////////

	vector_t caution_rejection_vector_sum(
		0.0,
		0.0,
		0.0
	);

	vector_t caution_rejection_vector(
		0.0,
		0.0,
		0.0
	);

	for (size_t i = 0; i < points_idx_in_caution_sphere.size(); i++)
	{
		vector_t obst_vect(
			_combined_points->at(points_idx_in_caution_sphere.at(i)).x, 
			_combined_points->at(points_idx_in_caution_sphere.at(i)).y,
			_combined_points->at(points_idx_in_caution_sphere.at(i)).z
		);

		vector_t alt_input_velocity_vector(
			_input_x_vel,
			_input_y_vel,
			_input_z_vel
		);

		// only calculate rejection if input vel has component in obstacle direction
		//	i.e. between parellel and orthogonal (dotprod < 0), but not antiparallel
		if (obst_vect.dot(alt_input_velocity_vector) > 0)
		{	

			// project input vel vector onto obstacle vector
			vector_t in_vel_proj_on_obst_vect = -projectVectorOnVector(alt_input_velocity_vector, obst_vect); 

			// make rejection stronger when deeper in caution sphere
			// float rejection_strength = 1.0 - ( ((distances_in_caution_sphere.at(i)-_safety_sphere_radius) - (_caution_sphere_radius-_safety_sphere_radius)));  // scale with input_velocity_vector_scalar?
			float _cautiousness = 1.5 * input_velocity_vector_scalar;
			float caution_truncate = distances_in_caution_sphere.at(i)-_safety_sphere_radius;

			// make sure does not go negative when accidentally inside safety sphere
			if ( caution_truncate < 0.0)
			{
				caution_truncate = 0.0;
			}
			
			float rejection_strength = _cautiousness * (1.0 - ( caution_truncate / (_caution_sphere_radius-_safety_sphere_radius)) );  // scale with input_velocity_vector_scalar?

			caution_rejection_vector = in_vel_proj_on_obst_vect * rejection_strength;

			caution_rejection_vector_sum += caution_rejection_vector;

		}

		caution_rejection_vector_sum(1) = -caution_rejection_vector_sum(1);

	}



	////////// -------- safety bubble -------- //////////

	// for each obstacle in safety sphere
	// - calculate rejection_strength
	// - create unit vector drone_obst and multiply with rejection_strength
	// sum up rejection_strengths into 1 vector

	vector_t safety_rejection_vector_sum(
		0.0,
		0.0,
		0.0
	);

	for (size_t i = 0; i < points_idx_in_safety_sphere.size(); i++)
	{
		vector_t obst_vect(
			_combined_points->at(points_idx_in_safety_sphere.at(i)).x, 
			_combined_points->at(points_idx_in_safety_sphere.at(i)).y,
			_combined_points->at(points_idx_in_safety_sphere.at(i)).z
		);

		obst_vect.normalize();

		vector_t obst_rejection_vect = obst_vect;

		safety_rejection_vector_sum += -obst_vect;
	}

	safety_rejection_vector_sum = safety_rejection_vector_sum  * input_velocity_vector_scalar;

	vector_t unit_x(
		1.0,
		0.0,
		0.0
	);

	// quat_t rejection_quat = findRotation(unit_x, safety_rejection_vector_sum);

	// auto pose_msg = geometry_msgs::msg::PoseStamped();
	// pose_msg.header.stamp = this->now();
	// pose_msg.header.frame_id = "drone_yaw_only";
	// pose_msg.pose.orientation.x = rejection_quat(0);
	// pose_msg.pose.orientation.y = rejection_quat(1);
	// pose_msg.pose.orientation.z = rejection_quat(2);
	// pose_msg.pose.orientation.w = rejection_quat(3);
	// pose_msg.pose.position.x = 0.0;
	// pose_msg.pose.position.y = 0.0;
	// pose_msg.pose.position.z = 0.0;

	// _pose_pub->publish(pose_msg);
	


	/////////////// need some negative tangential stuff & velocity prediction
	/////////////// gradually reduce overall speed deeper in caution sphere - allow higher speeds towards front



	// return (input_velocity_vector);
	// return (input_velocity_vector+caution_rejection_vector_sum);
	// return (input_velocity_vector+safety_rejection_vector_sum);
	return (input_velocity_vector+safety_rejection_vector_sum+caution_rejection_vector_sum);
}

void OffboardControl::input_to_output_setpoint() {

	vector_t input_velocity_vector(
		_input_x_vel,
		-_input_y_vel,
		_input_z_vel
	);

	float drone_yaw_test = quatToEul(_drone_pose.quaternion)(2);

	// find drone yaw and subtract from world arrow yaw
	orientation_t drone_yaw(
		0.0, 
		0.0, 
		-drone_yaw_test
	);

	quat_t drone_yaw_quat = eulToQuat(drone_yaw);

	rotation_matrix_t yaw_rot_mat = quatToMat(drone_yaw_quat);

	// vector_t rotated_input_velocity = rotateVector(yaw_rot_mat, input_velocity_vector);

	// rotated_input_velocity(0) = rotated_input_velocity(0) * _max_x_vel;
	// rotated_input_velocity(1) = rotated_input_velocity(1) * _max_y_vel;
	// rotated_input_velocity(2) = rotated_input_velocity(2) * _max_z_vel;


	/////////// new
	vector_t limited_velocity = OffboardControl::speed_limiter();	
	vector_t rotated_input_velocity = rotateVector(yaw_rot_mat, limited_velocity);
	


	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.timestamp = _timestamp.load();
	msg.x = NAN; //_drone_pose.position(0); //NAN 
	msg.y = NAN; //-_drone_pose.position(1); //NAN; 
	msg.z = NAN; //-_drone_pose.position(2); //NAN; 
	msg.yaw = NAN; //-drone_yaw_test; //NAN 
	msg.yawspeed = -_input_yaw_rate;
	msg.vx = rotated_input_velocity(0); 
	msg.vy = rotated_input_velocity(1); 
	msg.vz = -rotated_input_velocity(2); 

	OffboardControl::publish_setpoint(msg);

}


void OffboardControl::publish_markers() {

	visualization_msgs::msg::MarkerArray marker_array;

	///////////////////////////////////////////////////////////
	// input velocity arrow
	// visualization_msgs::msg::Marker input_velocity_marker;
	// input_velocity_marker.header = std_msgs::msg::Header();
	// input_velocity_marker.header.stamp = this->now();
	// input_velocity_marker.header.frame_id = "world";

	// input_velocity_marker.ns = "input_velocity_arrow";
	// input_velocity_marker.id = 1;

	// input_velocity_marker.type = visualization_msgs::msg::Marker::ARROW;

	// input_velocity_marker.action = visualization_msgs::msg::Marker::ADD;

	// vector_t unit_x_vector(
	// 	1.0,
	// 	0.0,
	// 	0.0
	// );

	// vector_t input_velocity_vector(
	// 	_input_x_vel,
	// 	_input_y_vel,
	// 	_input_z_vel
	// );

	// quat_t arrow_rotation = findRotation(unit_x_vector, input_velocity_vector);

	// float drone_yaw_test = quatToEul(_drone_pose.quaternion)(2);

	// // find drone yaw and subtract from world arrow yaw
	// orientation_t drone_yaw(
	// 	0.0, 
	// 	0.0, 
	// 	drone_yaw_test
	// );

	// quat_t drone_yaw_quat = (eulToQuat(drone_yaw));

	// quat_t arrow_orientation_drone_yaw = quatMultiply(drone_yaw_quat, arrow_rotation);

	// arrow_rotation = arrow_orientation_drone_yaw;


	// input_velocity_marker.pose.orientation.x = arrow_rotation(0);
	// input_velocity_marker.pose.orientation.y = arrow_rotation(1);
	// input_velocity_marker.pose.orientation.z = arrow_rotation(2);
	// input_velocity_marker.pose.orientation.w = arrow_rotation(3);
	// input_velocity_marker.pose.position.x = _drone_pose.position(0);
	// input_velocity_marker.pose.position.y = _drone_pose.position(1);
	// input_velocity_marker.pose.position.z = _drone_pose.position(2); 

	// // Set the scale of the marker -- 1x1x1 here means 1m on a side
	// input_velocity_marker.scale.x = sqrt(pow((_max_x_vel*_input_x_vel),2)+pow((_max_y_vel*_input_y_vel),2)+pow((_max_z_vel*_input_z_vel),2));
	// input_velocity_marker.scale.y = input_velocity_marker.scale.x / 5;
	// input_velocity_marker.scale.z = input_velocity_marker.scale.x / 5;
	// // Set the color -- be sure to set alpha to something non-zero!
	// input_velocity_marker.color.r = 0.0f;
	// input_velocity_marker.color.g = 1.0f;
	// input_velocity_marker.color.b = 0.0f;
	// input_velocity_marker.color.a = 0.6;

	// input_velocity_marker.lifetime = rclcpp::Duration::from_seconds(0);

	// marker_array.markers.push_back(input_velocity_marker);
	visualization_msgs::msg::Marker input_velocity_marker;
	input_velocity_marker.header = std_msgs::msg::Header();
	input_velocity_marker.header.stamp = this->now();
	input_velocity_marker.header.frame_id = "drone_yaw_only";

	input_velocity_marker.ns = "input_velocity_arrow";
	input_velocity_marker.id = 1;

	input_velocity_marker.type = visualization_msgs::msg::Marker::ARROW;

	input_velocity_marker.action = visualization_msgs::msg::Marker::ADD;

	vector_t unit_x_vector(
		1.0,
		0.0,
		0.0
	);

	vector_t input_velocity_vector(
		_input_x_vel,
		_input_y_vel,
		_input_z_vel
	);

	quat_t arrow_rotation = findRotation(unit_x_vector, input_velocity_vector);

	// float drone_roll_test = quatToEul(_drone_pose.quaternion)(0);
	// float drone_pitch_test = quatToEul(_drone_pose.quaternion)(1);

	// // find drone yaw and subtract from world arrow yaw
	// orientation_t drone_RP(
	// 	-drone_roll_test, 
	// 	-drone_pitch_test, 
	// 	0.0
	// );

	// quat_t drone_RP_quat = (eulToQuat(drone_RP));

	// _input_velocity_orientation_drone_RP = quatMultiply(drone_RP_quat, arrow_rotation);

	// arrow_rotation = _input_velocity_orientation_drone_RP;


	input_velocity_marker.pose.orientation.x = arrow_rotation(0);
	input_velocity_marker.pose.orientation.y = arrow_rotation(1);
	input_velocity_marker.pose.orientation.z = arrow_rotation(2);
	input_velocity_marker.pose.orientation.w = arrow_rotation(3);
	input_velocity_marker.pose.position.x = 0.0; //_drone_pose.position(0);
	input_velocity_marker.pose.position.y = 0.0; //_drone_pose.position(1);
	input_velocity_marker.pose.position.z = 0.0; //_drone_pose.position(2); 

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	input_velocity_marker.scale.x = sqrt(pow((_max_x_vel*_input_x_vel),2)+pow((_max_y_vel*_input_y_vel),2)+pow((_max_z_vel*_input_z_vel),2));
	input_velocity_marker.scale.y = input_velocity_marker.scale.x / 5;
	input_velocity_marker.scale.z = input_velocity_marker.scale.x / 5;
	// Set the color -- be sure to set alpha to something non-zero!
	input_velocity_marker.color.r = 0.0f;
	input_velocity_marker.color.g = 1.0f;
	input_velocity_marker.color.b = 0.0f;
	input_velocity_marker.color.a = 0.6;

	input_velocity_marker.lifetime = rclcpp::Duration::from_seconds(0);

	marker_array.markers.push_back(input_velocity_marker);




	///////////////////////////////////////////////////////////
	// drone velocity arrow
	visualization_msgs::msg::Marker drone_velocity_marker;
	drone_velocity_marker.header = std_msgs::msg::Header();
	drone_velocity_marker.header.stamp = this->now();
	drone_velocity_marker.header.frame_id = "world";

	drone_velocity_marker.ns = "velocity_arrow";
	drone_velocity_marker.id = 0;

	drone_velocity_marker.type = visualization_msgs::msg::Marker::ARROW;

	drone_velocity_marker.action = visualization_msgs::msg::Marker::ADD;

	vector_t drone_velocity_vector(
		_x_vel,
		_y_vel,
		_z_vel
	);

	quat_t drone_arrow_rotation = findRotation(unit_x_vector, drone_velocity_vector);

	drone_velocity_marker.pose.orientation.x = drone_arrow_rotation(0);
	drone_velocity_marker.pose.orientation.y = drone_arrow_rotation(1);
	drone_velocity_marker.pose.orientation.z = drone_arrow_rotation(2);
	drone_velocity_marker.pose.orientation.w = drone_arrow_rotation(3);
	drone_velocity_marker.pose.position.x = _drone_pose.position(0); //0; 
	drone_velocity_marker.pose.position.y = _drone_pose.position(1); //0;
	drone_velocity_marker.pose.position.z = _drone_pose.position(2); //0; 

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	drone_velocity_marker.scale.x = sqrt(pow(this->_x_vel,2)+pow(this->_y_vel,2)+pow(this->_z_vel,2));
	drone_velocity_marker.scale.y = drone_velocity_marker.scale.x / 5;
	drone_velocity_marker.scale.z = drone_velocity_marker.scale.x / 5;
	// Set the color -- be sure to set alpha to something non-zero!
	drone_velocity_marker.color.r = 1.0f;
	drone_velocity_marker.color.g = 0.0f;
	drone_velocity_marker.color.b = 0.0f;
	drone_velocity_marker.color.a = 0.6;

	drone_velocity_marker.lifetime = rclcpp::Duration::from_seconds(0);

	marker_array.markers.push_back(drone_velocity_marker);




	///////////////////////////////////////////////////////////
	// safety bubble sphere
	visualization_msgs::msg::Marker safety_bubble_marker;
	safety_bubble_marker.header = std_msgs::msg::Header();
	safety_bubble_marker.header.stamp = this->now();
	safety_bubble_marker.header.frame_id = "world";
	safety_bubble_marker.ns = "safety_bubble";
	safety_bubble_marker.id = 2;
	safety_bubble_marker.type = visualization_msgs::msg::Marker::SPHERE;
	safety_bubble_marker.action = visualization_msgs::msg::Marker::ADD;

	safety_bubble_marker.pose.orientation.x = _drone_pose.quaternion(0);
	safety_bubble_marker.pose.orientation.y = _drone_pose.quaternion(1);
	safety_bubble_marker.pose.orientation.z = _drone_pose.quaternion(2);
	safety_bubble_marker.pose.orientation.w = _drone_pose.quaternion(3);
	safety_bubble_marker.pose.position.x = _drone_pose.position(0); 
	safety_bubble_marker.pose.position.y = _drone_pose.position(1);
	safety_bubble_marker.pose.position.z = _drone_pose.position(2); 

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	safety_bubble_marker.scale.x = _safety_sphere_radius * 2; /// make parameter
	safety_bubble_marker.scale.y = safety_bubble_marker.scale.x;
	safety_bubble_marker.scale.z = safety_bubble_marker.scale.x;
	// Set the color -- be sure to set alpha to something non-zero!
	safety_bubble_marker.color.r = 1.0f;
	safety_bubble_marker.color.g = 0.0f;
	safety_bubble_marker.color.b = 0.0f;
	safety_bubble_marker.color.a = 0.25;

	safety_bubble_marker.lifetime = rclcpp::Duration::from_seconds(0);

	marker_array.markers.push_back(safety_bubble_marker);




	///////////////////////////////////////////////////////////
	// caution bubble sphere
	visualization_msgs::msg::Marker caution_sphere_marker;
	caution_sphere_marker.header = std_msgs::msg::Header();
	caution_sphere_marker.header.stamp = this->now();
	caution_sphere_marker.header.frame_id = "world";
	caution_sphere_marker.ns = "caution_sphere";
	caution_sphere_marker.id = 2;
	caution_sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
	caution_sphere_marker.action = visualization_msgs::msg::Marker::ADD;

	caution_sphere_marker.pose.orientation.x = _drone_pose.quaternion(0);
	caution_sphere_marker.pose.orientation.y = _drone_pose.quaternion(1);
	caution_sphere_marker.pose.orientation.z = _drone_pose.quaternion(2);
	caution_sphere_marker.pose.orientation.w = _drone_pose.quaternion(3);
	caution_sphere_marker.pose.position.x = _drone_pose.position(0); 
	caution_sphere_marker.pose.position.y = _drone_pose.position(1);
	caution_sphere_marker.pose.position.z = _drone_pose.position(2); 

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	caution_sphere_marker.scale.x = _caution_sphere_radius * 2; /// make parameter
	caution_sphere_marker.scale.y = caution_sphere_marker.scale.x;
	caution_sphere_marker.scale.z = caution_sphere_marker.scale.x;
	// Set the color -- be sure to set alpha to something non-zero!
	caution_sphere_marker.color.r = (float)_in_caution_sphere;
	caution_sphere_marker.color.g = (float)(!_in_caution_sphere);
	caution_sphere_marker.color.b = 0.0f;
	caution_sphere_marker.color.a = 0.15;

	caution_sphere_marker.lifetime = rclcpp::Duration::from_seconds(0);

	marker_array.markers.push_back(caution_sphere_marker);




	this->_marker_pub->publish(marker_array);
}


void OffboardControl::publish_path() {
	// limit length
	// offboard and manual paths

	static auto manual_path_msg = nav_msgs::msg::Path();
	manual_path_msg.header.stamp = this->now();
	manual_path_msg.header.frame_id = "world";

	static auto offboard_path_msg = nav_msgs::msg::Path();
	offboard_path_msg.header.stamp = this->now();
	offboard_path_msg.header.frame_id = "world";

	auto pose_msg = geometry_msgs::msg::PoseStamped();
	pose_msg.header.stamp = this->now();
	pose_msg.header.frame_id = "world";

	_drone_pose_mutex.lock(); {

		pose_msg.pose.position.x = _drone_pose.position(0);
		pose_msg.pose.position.y = _drone_pose.position(1);
		pose_msg.pose.position.z = _drone_pose.position(2);

		pose_msg.pose.orientation.x = _drone_pose.quaternion(0);
		pose_msg.pose.orientation.y = _drone_pose.quaternion(1);
		pose_msg.pose.orientation.z = _drone_pose.quaternion(2);
		pose_msg.pose.orientation.w = _drone_pose.quaternion(3);

	} _drone_pose_mutex.unlock();


	if (_in_offboard)
	{
		float dist = 99999.9;

		if (offboard_path_msg.poses.size() > 0 )
		{		
			float x_diff = offboard_path_msg.poses.back().pose.position.x - pose_msg.pose.position.x;
			float y_diff = offboard_path_msg.poses.back().pose.position.y - pose_msg.pose.position.y;
			float z_diff = offboard_path_msg.poses.back().pose.position.z - pose_msg.pose.position.z;
			dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2) + pow(z_diff, 2));
		}		

		if(dist > 1.0)
		{
			offboard_path_msg.poses.push_back(pose_msg);
			_offboard_path_pub->publish(offboard_path_msg);
		}
		manual_path_msg.poses.clear();
	}
	else
	{
		float dist = 99999.9;

		if (manual_path_msg.poses.size() > 0 )
		{	
			float x_diff = manual_path_msg.poses.back().pose.position.x - pose_msg.pose.position.x;
			float y_diff = manual_path_msg.poses.back().pose.position.y - pose_msg.pose.position.y;
			float z_diff = manual_path_msg.poses.back().pose.position.z - pose_msg.pose.position.z;
			dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2) + pow(z_diff, 2));
		}	

		if(dist > 1.0)
		{
			manual_path_msg.poses.push_back(pose_msg);
			_manual_path_pub->publish(manual_path_msg);
		}
		offboard_path_msg.poses.clear();
	}
		
}


void OffboardControl::flight_state_machine() {

	OffboardControl::update_drone_pose();

	// If drone not armed (from external controller) and put in offboard mode, do nothing
	if (_nav_state != 14) 
	{
		if (_old_nav_state != _nav_state && _nav_state != 14 && _launch_with_debug > 0)
		{				
			RCLCPP_INFO(this->get_logger(), "nav_state: %d", _nav_state);
			RCLCPP_INFO(this->get_logger(), "\n \nWaiting for offboard mode\n");
		}

		if (_old_nav_state != _nav_state && _nav_state == 14 && _launch_with_debug > 0)
		{				
			RCLCPP_INFO(this->get_logger(), "nav_state: %d", _nav_state);
			RCLCPP_INFO(this->get_logger(), "\n \nOffboard mode enabled\n");
		}

		publish_hold_setpoint();
		_in_offboard = false;
		_new_takeoff = true;
		_old_nav_state = _nav_state;
		_counter = 0;
		return;
	}

	_in_offboard = true;

	if (!_printed_offboard)
	{
		RCLCPP_INFO(this->get_logger(), "\n \nEntering offboard control \n");
		_printed_offboard = true;
		this->arm();
	}

	this->get_parameter("take_off_to_height", _takeoff_height);
	if(_takeoff_height > 1){
		static bool takeoff_print = false;
		if(takeoff_print == false){
			takeoff_print = true;
			RCLCPP_INFO(this->get_logger(), "\n \nTaking off to %f meters\n", _takeoff_height);
		}
		publish_takeoff_setpoint();
		return;
	}

	else if(_counter < 1000000000){
		if(_counter == 10 && _launch_with_debug > 0){
			RCLCPP_INFO(this->get_logger(), "\n \nManual offboard mode \n");
		}
		// publish_tracking_setpoint();
		input_to_output_setpoint();

	}

	_counter++;

}


void OffboardControl::update_drone_pose() {

	geometry_msgs::msg::TransformStamped t;

	try {
		if (tf_buffer_->canTransform("world", "drone", tf2::TimePointZero))	{
			t = tf_buffer_->lookupTransform("world","drone",tf2::TimePointZero);
		}
		else {
			RCLCPP_INFO(this->get_logger(), "Can not transform");
			return;
		}
	} catch (const tf2::TransformException & ex) {
		RCLCPP_INFO(this->get_logger(), "Could not transform: %s", ex.what());
		return;
	}

	_drone_pose_mutex.lock(); {

		_drone_pose.position(0) = t.transform.translation.x;
		_drone_pose.position(1) = t.transform.translation.y;
		_drone_pose.position(2) = t.transform.translation.z;
		
		_drone_pose.quaternion(0) = t.transform.rotation.x;
		_drone_pose.quaternion(1) = t.transform.rotation.y;
		_drone_pose.quaternion(2) = t.transform.rotation.z;
		_drone_pose.quaternion(3) = t.transform.rotation.w;

	} _drone_pose_mutex.unlock();

}


/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm() const {
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send\n");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm() const {
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send\n");
}


/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode() const {
	px4_msgs::msg::OffboardControlMode msg{};
	msg.timestamp = _timestamp.load();
	msg.position = true;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;	
	_offboard_control_mode_publisher->publish(msg);
}


/**
 * @brief Publish a trajectory setpoint
 *        Should go to align with cable of choice there
 */
void OffboardControl::publish_tracking_setpoint() {

	if (_alignment_pose.position(0) == 0 && 
		_alignment_pose.position(1) == 0 && 
		_alignment_pose.position(2) == 0)
	{
		RCLCPP_INFO(this->get_logger(), "Nothing to align with - holding position");
		OffboardControl::publish_hold_setpoint();
		return;
	}
	

	float pos_frac;
	this->get_parameter("pos_frac", pos_frac);

	float yaw_frac;
	this->get_parameter("yaw_frac", yaw_frac);
	static float tmp_follow_speed;
	this->get_parameter("powerline_following_speed", tmp_follow_speed);

	orientation_t target_yaw_eul;

	pose_eul_t publish_pose;

	_powerline_mutex.lock(); {
	_drone_pose_mutex.lock(); {
		
		// calculate fractional yaw positions (basic porportional control)
		target_yaw_eul = quatToEul(_alignment_pose.quaternion);

		float cur_yaw = quatToEul(_drone_pose.quaternion)(2); 
		float target_yaw = target_yaw_eul(2);
		
		if ( abs(cur_yaw - target_yaw) <= (float)M_PI )
		{
			target_yaw = cur_yaw + (target_yaw - cur_yaw)*yaw_frac;			
		}
		else 
		{
			float diff = (2*M_PI - target_yaw) + cur_yaw;

			target_yaw = cur_yaw - diff*yaw_frac;			
		}


		publish_pose.position(0) = _drone_pose.position(0) + (_alignment_pose.position(0) - _drone_pose.position(0))*pos_frac;
		publish_pose.position(1) = _drone_pose.position(1) + (_alignment_pose.position(1) - _drone_pose.position(1))*pos_frac;
		publish_pose.position(2) = _drone_pose.position(2) + (_alignment_pose.position(2) - _drone_pose.position(2))*pos_frac;		

		publish_pose.orientation(0) = 0.0;
		publish_pose.orientation(1) = 0.0;
		publish_pose.orientation(2) = target_yaw;


	} _drone_pose_mutex.unlock();
	} _powerline_mutex.unlock();

	point_t unit_x(
		1.0 * tmp_follow_speed,
		0.0,
		0.0
	);


	publish_pose = pose_NWU_to_NED(publish_pose);
	
	// rotate unit x (1,0,0) velocity to align with powerline direction
	rotation_matrix_t rot_mat = quatToMat(_alignment_pose.quaternion);
	point_t unit_velocity = rotateVector(rot_mat, unit_x);

	// rotate powerline direction velocity from NWU to NED frame
	static rotation_matrix_t R_NWU_to_NED = eulToR(orientation_t(-M_PI, 0, 0));
	unit_velocity = rotateVector(R_NWU_to_NED, unit_velocity);

	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.timestamp = _timestamp.load();
	msg.x = publish_pose.position(0); // in meters NED
	msg.y = publish_pose.position(1); // in meters NED
	msg.z = publish_pose.position(2); // in meters NED
	msg.yaw = publish_pose.orientation(2); // rotation around z NED in radians
	msg.vx = unit_velocity(0); // m/s NED
	msg.vy = unit_velocity(1); // m/s NED
	msg.vz = unit_velocity(2); // m/s NED

	// RCLCPP_INFO(this->get_logger(), "Xv:%f Yv:%f Zv:%f", msg.velocity[0], msg.velocity[1], msg.velocity[2]);

	OffboardControl::publish_setpoint(msg);
}


/**
 * @brief Publish a trajectory setpoint
 *        Drone should take off to _takeoff_height
 */
void OffboardControl::publish_takeoff_setpoint() {

	static pose_eul_t NWU_to_NED_pose;	

	if (_new_takeoff == true)
	{	
		// freeze takeoff setpoint
		_new_takeoff = false;
		NWU_to_NED_pose.position = _drone_pose.position; 
		NWU_to_NED_pose.orientation = quatToEul(_drone_pose.quaternion);
		NWU_to_NED_pose = pose_NWU_to_NED(NWU_to_NED_pose);
	}

	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.timestamp = _timestamp.load();
	msg.x = NWU_to_NED_pose.position(0); 		// in meters NED
	msg.y = NWU_to_NED_pose.position(1);
	msg.z = - _takeoff_height;
	// YAW is cropped to 0-PI for some reason, uncrop to 0-2PI based on if ROLL is 0 or PI
	msg.yaw = (float)NWU_to_NED_pose.orientation(2);

	OffboardControl::publish_setpoint(msg);
}


/**
 * @brief Publish a trajectory setpoint
 *        Drone should holds it pose
 */
void OffboardControl::publish_hold_setpoint() const {

	pose_eul_t NWU_to_NED_pose;
	NWU_to_NED_pose.position = _drone_pose.position; 
	NWU_to_NED_pose.orientation = quatToEul(_drone_pose.quaternion);	

	NWU_to_NED_pose = pose_NWU_to_NED(NWU_to_NED_pose);

	px4_msgs::msg::TrajectorySetpoint msg{}; // in meters NED
	msg.timestamp = _timestamp.load();
	msg.x = NWU_to_NED_pose.position(0); 		
	msg.y = NWU_to_NED_pose.position(1);
	msg.z = NWU_to_NED_pose.position(2);
	//YAW is cropped to 0-PI for some reason, uncrop to 0-2PI based on if ROLL is 0 or PI
	msg.yaw = (float)NWU_to_NED_pose.orientation(2);// + (float)NWU_to_NED_pose.orientation(0);

	// RCLCPP_INFO(this->get_logger(), "DRONE EUL:\n R:%f P:%f Y:%f ", NWU_to_NED_pose.orientation(0), NWU_to_NED_pose.orientation(1), NWU_to_NED_pose.orientation(2));

	OffboardControl::publish_setpoint(msg);
}


/**
 * @brief Publish a trajectory setpoint
 * and pose message
 */
void OffboardControl::publish_setpoint(px4_msgs::msg::TrajectorySetpoint msg) const {

	publish_offboard_control_mode();

	orientation_t eul (
		0.0,
		0.0,
		-msg.yaw // NED to NWU
	);

	quat_t quat = eulToQuat(eul);

	_trajectory_setpoint_publisher->publish(msg);
}


/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1,
					      float param2) const {
	px4_msgs::msg::VehicleCommand msg{};
	msg.timestamp = _timestamp.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	_vehicle_command_publisher->publish(msg);
}


int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
