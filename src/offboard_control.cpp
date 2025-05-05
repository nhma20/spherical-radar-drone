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
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <nav_msgs/msg/path.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <stdint.h>
#include <math.h>  
#include <limits>
#include <mutex>
#include <chrono>
#include <iostream>
#include <vector>
#include <algorithm>


#include "geometry.h"

// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/angles.h>


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

		this->declare_parameter<float>("max_decelration", 10.0); // m/s²
		this->get_parameter("max_decelration", _max_decelration);

		this->declare_parameter<float>("max_x_vel", 10);
		this->get_parameter("max_x_vel", _max_x_vel);

		this->declare_parameter<float>("max_y_vel", 10);
		this->get_parameter("max_y_vel", _max_y_vel);

		this->declare_parameter<float>("max_z_vel", 3);
		this->get_parameter("max_z_vel", _max_z_vel);

		this->declare_parameter<float>("max_yaw_rate", 1.570796); // rad/s
		this->get_parameter("max_yaw_rate", _max_yaw_rate);

		this->declare_parameter<float>("caution_sphere_radius", 2.5); //1.5); // meters
		this->get_parameter("caution_sphere_radius", _caution_sphere_radius);

		this->declare_parameter<float>("caution_sphere_max_speed", 1.5); //1.5); // m/s
		this->get_parameter("caution_sphere_max_speed", _caution_sphere_max_speed);

		this->declare_parameter<float>("safety_sphere_radius", 0.75); // 0.75 meters
		this->get_parameter("safety_sphere_radius", _safety_sphere_radius);

		this->declare_parameter<float>("min_input_velocity", 0.5); // 
		this->get_parameter("min_input_velocity", _min_input_velocity);

		this->declare_parameter<float>("caution_rejection_scalar", 0.1); // 
		this->get_parameter("caution_rejection_scalar", _caution_rejection_scalar);

		this->declare_parameter<float>("safety_rejection_scalar", 1.0); // 
		this->get_parameter("safety_rejection_scalar", _safety_rejection_scalar);

		this->declare_parameter<float>("tangential_rejection_scalar", 0.25); // 
		this->get_parameter("tangential_rejection_scalar", _tangential_rejection_scalar);

		this->declare_parameter<float>("look_ahead_cone_length_width_ratio", 2.0); // 
		this->get_parameter("look_ahead_cone_length_width_ratio", _look_ahead_cone_length_to_width_ratio);

		this->declare_parameter<float>("braking_safety_factor", 1.5); // 
		this->get_parameter("braking_safety_factor", _braking_safety_factor);

		this->declare_parameter<float>("emergency_brake_speed_limit", 2.00); //1.25 
		this->get_parameter("emergency_brake_speed_limit", _emergency_brake_speed_limit);

		this->declare_parameter<float>("front_horizontal_fov_deg", 30); // 
		this->get_parameter("front_horizontal_fov_deg", _front_horizontal_fov_deg);
		
		this->declare_parameter<float>("front_vertical_fov_deg", 110); // 
		this->get_parameter("front_vertical_fov_deg", _front_vertical_fov_deg);

		this->declare_parameter<float>("front_detection_range", 16); // 
		this->get_parameter("front_detection_range", _front_detection_range);

		this->declare_parameter<float>("side_detection_range", 8); // 
		this->get_parameter("side_detection_range", _side_detection_range);
		


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
			  _input_y_vel = -msg->y; // Negated to get NWU
			  _input_z_vel = (msg->z*2.0)-1.0;
			  _input_yaw_rate = -msg->r; // Negated to get NWU
			});


		// VehicleStatus: https://github.com/PX4/px4_msgs/blob/master/msg/VehicleStatus.msg
		_vehicle_status_sub = create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/vehicle_status/out", 10,
            [this](px4_msgs::msg::VehicleStatus::ConstSharedPtr msg) {
              _arming_state = msg->arming_state;
              _nav_state = msg->nav_state;
			});


		_combined_pcl_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			"/combined_pcl", 10,
			std::bind(&OffboardControl::pclmsg_to_pcl, this, std::placeholders::_1));


		_out_vel_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/out_vel_pose", 10);
		_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/debug_pose", 10);
		_manual_path_pub = this->create_publisher<nav_msgs::msg::Path>("/manual_path", 10);
		_offboard_path_pub = this->create_publisher<nav_msgs::msg::Path>("/offboard_path", 10);
		_point_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/debug_point", 10);


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

		_max_allowed_front_speed = sqrt( 2 * _max_decelration *  ( _front_detection_range / _braking_safety_factor ) );
		_max_allowed_side_speed = sqrt( 2 * _max_decelration *  ( _side_detection_range / _braking_safety_factor ) );


		while(true) {

			static int _t_tries = 0;

			try {

				geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform("world","drone",tf2::TimePointZero);

				RCLCPP_INFO(this->get_logger(), "Found transform world->drone");
				break;

			} catch(tf2::TransformException & ex) {

				RCLCPP_WARN(this->get_logger(), "Could not get transform world->drone, trying again...");
				
				_t_tries++;

				std::this_thread::sleep_for(std::chrono::milliseconds(500));

				if( _t_tries > 10) {
					RCLCPP_FATAL(this->get_logger(), "Failed to get transform world->drone after 10 tries.");
					throw std::exception();
				}
			}

		}
		
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
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _out_vel_pose_pub;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _manual_path_pub;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _offboard_path_pub;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _marker_pub;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr _point_pub;

	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr _timesync_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _vehicle_status_sub;
	rclcpp::Subscription<px4_msgs::msg::RcChannels>::SharedPtr _rc_channels_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _vehicle_odometry_subscriber;
	rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr _manual_control_input_sub;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _combined_pcl_sub;

	pcl::PointCloud<pcl::PointXYZ>::Ptr _combined_points;

	// std::unique_ptr<tf2_ros::TransformBroadcaster> _drone_velocity_frame_tf_broadcaster;
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
	float _tangential_rejection_scalar;
	float _safety_rejection_scalar;
	float _caution_rejection_scalar;
	float _look_ahead_cone_length = 0.0;
	float _look_ahead_cone_length_to_width_ratio;
	float _braking_safety_factor;
	float _emergency_brake_speed_limit;
	float _front_vertical_fov_deg;
	float _front_horizontal_fov_deg;
	float _side_detection_range;
	float _front_detection_range;
	float _max_allowed_front_speed;
	float _max_allowed_side_speed;
	float _caution_sphere_max_speed;

	bool _new_takeoff = true;

    bool _printed_offboard = false;

	bool _in_offboard = false;

	float _x_vel = 0.0; // NED or NWU?
	float _y_vel = 0.0;
	float _z_vel = 0.0;

	std::mutex _drone_pose_mutex;
	std::mutex _powerline_mutex;

	quat_t _input_velocity_orientation_drone_RP; // input velocity quat in drone frame without drone pitch+roll
	pose_t _drone_pose; // in world coordinates North-West-Up
	pose_t _drone_pose_yaw_only; // in world coordinates North-West-Up
	pose_t _alignment_pose;
	vector_t _out_vel_vector;
	vector_t _obst_tangent_vector;

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
	float smallest_distance = 9999.0;

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
		}	

		if (point_dist < smallest_distance)
		{
			smallest_distance = point_dist;
		}	
	}	

	*smallest_dist = smallest_distance;
}



vector_t OffboardControl::speed_limiter() {

	// adjust velocity of drone based on obstacles and input velocity
	// Assume all obstacles static in world frame

	std::vector<size_t> points_idx_in_caution_sphere = {};
	std::vector<size_t> distances_in_caution_sphere = {};

	std::vector<size_t> points_idx_in_safety_sphere = {};
	std::vector<size_t> distances_in_safety_sphere = {};

	float smallest_dist = 999999.9;

	collision_checker(&points_idx_in_caution_sphere, &distances_in_caution_sphere, &points_idx_in_safety_sphere, &distances_in_safety_sphere, &smallest_dist);

	vector_t input_velocity_vector(
		_input_x_vel * _max_x_vel,
		_input_y_vel * _max_y_vel, //- 
		_input_z_vel * _max_z_vel
	);

	float input_velocity_vector_scalar = input_velocity_vector.stableNorm();

	
	vector_t drone_velocity(
		_x_vel,
		_y_vel,
		_z_vel
	);

	float drone_yaw_f = quatToEul(_drone_pose_yaw_only.quaternion)(2);

	// find drone yaw and subtract from world yaw
	orientation_t drone_yaw(
		0.0, 
		0.0,
		-drone_yaw_f
	); 

	rotation_matrix_t yaw_rotation = quatToMat(eulToQuat(drone_yaw));

	// get drone velocity in local drone frame (no roll and pitch)
	vector_t drone_velocity_drone_frame = rotateVector(yaw_rotation, drone_velocity);



	////////// -------- caution bubble -------- //////////
	// if obstacle inside caution sphere, progressively limit speed until 0 when obstacle touches surface of safety bubble
	// only reduce input velocity component in direction of obstacle


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
		if (obst_vect.dot(alt_input_velocity_vector) > 0.0)
		{	

			// project input vel vector onto obstacle vector
			vector_t in_vel_proj_on_obst_vect = -projectVectorOnVector(alt_input_velocity_vector, obst_vect); 

			// make rejection stronger when deeper in caution sphere
			// float rejection_strength = 1.0 - ( ((distances_in_caution_sphere.at(i)-_safety_sphere_radius) - (_caution_sphere_radius-_safety_sphere_radius)));  // scale with input_velocity_vector_scalar?
			float _cautiousness = _caution_rejection_scalar * input_velocity_vector_scalar;
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
			-_combined_points->at(points_idx_in_safety_sphere.at(i)).z
		);

		float factor = 1 - (_safety_sphere_radius / obst_vect.stableNorm());
		// RCLCPP_INFO(this->get_logger(),  "factor: %f", factor);

		obst_vect.normalize();

		safety_rejection_vector_sum += -obst_vect * factor;
	}

	float timidness = _safety_rejection_scalar;
	safety_rejection_vector_sum = safety_rejection_vector_sum  * timidness; // * input_velocity_vector_scalar;

	// RCLCPP_INFO(this->get_logger(),  "Safety rejection strength %f", safety_rejection_vector_sum.stableNorm());
	// RCLCPP_INFO(this->get_logger(),  "X: %f", safety_rejection_vector_sum(0));
	// RCLCPP_INFO(this->get_logger(),  "Y: %f", safety_rejection_vector_sum(1));
	// RCLCPP_INFO(this->get_logger(),  "Z: %f", safety_rejection_vector_sum(2));

	if (safety_rejection_vector_sum.stableNorm() > 0.01)
	{
		return (safety_rejection_vector_sum);
	}





	////////// -------- tangential rejection -------- //////////
	// calculate tangential rejection to guide drone around obstacles

	vector_t tangential_rejection_vector_sum(
		0.0,
		0.0,
		0.0
	);

	this->_obst_tangent_vector = tangential_rejection_vector_sum;

	vector_t unit_z(
		0.0,
		0.0,
		1.0
	);

	for (size_t i = 0; i < points_idx_in_caution_sphere.size(); i++)
	{
		vector_t obst_vect(
			_combined_points->at(points_idx_in_caution_sphere.at(i)).x, 
			_combined_points->at(points_idx_in_caution_sphere.at(i)).y,
			_combined_points->at(points_idx_in_caution_sphere.at(i)).z
		);

		obst_vect.normalize();



		// only calculate rejection if input vel has component in obstacle direction
		//	i.e. between parellel and orthogonal (dotprod < 0), but not antiparallel

		vector_t alt_input_velocity_vector(
			_input_x_vel,
			_input_y_vel,
			_input_z_vel
		);

		// 1 if fully parallel, 0 if perpendicular, -1 if opposite directions
		float normalized_obst_dot_input = obst_vect.dot(alt_input_velocity_vector.normalized());

		// only consider if obstacle vector somewhat parallel to input velocity vector
		if (normalized_obst_dot_input > 0)
		{	

			vector_t obst_z_crossprod = obst_vect.cross(unit_z);
			vector_t tangent;

			vector_t obst_tangent = obst_z_crossprod.cross(obst_vect);

			// find if pos or neg tangent is most parallel to input vel
			if (obst_tangent.dot(alt_input_velocity_vector) > 0)
			{
				tangent = obst_tangent.normalized();
			}
			else {
				tangent = -obst_tangent.normalized();
			}

			// increase tangent response the more parallel obst vect and input vect are
			tangent = tangent * (normalized_obst_dot_input + 0.15) * 2.5; 

			float _cautiousness = _tangential_rejection_scalar * input_velocity_vector_scalar;
			float caution_truncate = distances_in_caution_sphere.at(i)-_safety_sphere_radius;

			// make sure does not go negative when accidentally inside safety sphere
			if ( caution_truncate < 0.0)
			{
				caution_truncate = 0.0;
			}

			float tangential_strength = _cautiousness * (1.0 - ( caution_truncate / (_caution_sphere_radius-_safety_sphere_radius)) );;
			
			tangential_rejection_vector_sum += tangent * tangential_strength;

		}


		// do same for drone velocity vector (in addition to input velocity vector)

		// only calculate rejection if drone vel has component in obstacle direction
		//	i.e. between parellel and orthogonal (dotprod < 0), but not antiparallel

		// 1 if fully parallel, 0 if perpendicular, -1 if opposite directions
		float normalized_obst_dot_drone_vel = obst_vect.dot(drone_velocity_drone_frame.normalized());
		if (normalized_obst_dot_drone_vel > 0)
		{	

			vector_t obst_z_crossprod = obst_vect.cross(unit_z);
			vector_t tangent;

			vector_t obst_tangent = obst_z_crossprod.cross(obst_vect);

			// find if pos or neg tangent is most parallel to vel
			if (obst_tangent.dot(drone_velocity_drone_frame) > 0)
			{
				tangent = obst_tangent.normalized();
			}
			else {
				tangent = -obst_tangent.normalized();
			}

			tangent = tangent * (normalized_obst_dot_drone_vel + 0.25) * 2.5;

			float _cautiousness = _tangential_rejection_scalar * drone_velocity_drone_frame.stableNorm();
			float caution_truncate = distances_in_caution_sphere.at(i)-_safety_sphere_radius;

			// make sure does not go negative when accidentally inside safety sphere
			if ( caution_truncate < 0.0)
			{
				caution_truncate = 0.0;
			}

			float tangential_strength = _cautiousness * (1.0 - ( caution_truncate / (_caution_sphere_radius-_safety_sphere_radius)) );;
			
			tangential_rejection_vector_sum += tangent * tangential_strength;

		}

		// RCLCPP_INFO(this->get_logger(),  "Drone Vel Output Tangent X: %f\t, Y: %f\t, Z: %f", tangential_rejection_vector_sum(0), tangential_rejection_vector_sum(1), tangential_rejection_vector_sum(2));
		// RCLCPP_INFO(this->get_logger(),  "\n");
		this->_obst_tangent_vector = tangential_rejection_vector_sum;
	}



	////////// -------- velocity prediction rejection -------- //////////
	// Predict relative motion based on drone velocity (and in future radial velocities)
	// If obstacle moves too fast towards drone (based on max drone decelaration and perception horizon)
	//	-> reduce speed to avoid entering safety bubble

	// get angle between obst vector and velocity vector - if inside cone and close enough wrt. speed, apply rejection
	static const float max_angle = atan( 1.0 / (_look_ahead_cone_length_to_width_ratio*2) );// rad 

	float drone_vel_magnitude = drone_velocity_drone_frame.stableNorm(); // sqrt(pow(this->_x_vel,2)+pow(this->_y_vel,2)+pow(this->_z_vel,2));

	vector_t cone_rejection(
		0.0,
		0.0,
		0.0
	);

	static bool emergency_brake = false;
	float time_to_stop = drone_vel_magnitude / _max_decelration;
	_look_ahead_cone_length = drone_vel_magnitude * time_to_stop * _braking_safety_factor;
	vector_t emergency_trigger_point;

	static vector_t emergency_tangent;

	// only check velocity when over certain speed
	if ( drone_vel_magnitude > _emergency_brake_speed_limit && emergency_brake == false ) //////////// @@@@@@@
	{	
		for (size_t i = 0; i < (size_t)_combined_points->size(); i++)
		{
						
			vector_t point_i(
				_combined_points->at(i).x,
				_combined_points->at(i).y,
				_combined_points->at(i).z
			);

			emergency_trigger_point = point_i;
			
			float time_to_obst = (point_i.stableNorm() - _safety_sphere_radius) / drone_vel_magnitude; // dist approximately just x, save computation

			// if distance is within safety margin AND obstacle along positive direction of drone velocity
			// add cross product check here to easily verify if inside cone??

			float obst_vel_angle = acos( drone_velocity_drone_frame.dot( point_i ) / (drone_velocity_drone_frame.stableNorm() * point_i.stableNorm()) );

			if ( time_to_obst < ( time_to_stop * _braking_safety_factor ) && obst_vel_angle < max_angle)
			{
				cone_rejection = -drone_velocity_drone_frame.normalized() * 2.0;
				// cone_rejection(2) = -cone_rejection(2); // negation really needed??
				emergency_brake = true;
				// RCLCPP_INFO(this->get_logger(),  "Look ahead cone length: %f:", _look_ahead_cone_length);
				// RCLCPP_INFO(this->get_logger(),  "Dist to obst: %f VS Drone velocity: %f", point_i.stableNorm(), drone_vel_magnitude);
				// RCLCPP_INFO(this->get_logger(),  "Time to obst: %f VS Time to stop: %f", time_to_obst, ( time_to_stop * _braking_safety_factor ));
				// RCLCPP_INFO(this->get_logger(),  "Obst vel angle: %f VS Max angle: %f", obst_vel_angle, max_angle);
				RCLCPP_INFO(this->get_logger(),  "We need to brake now!");


				// calculate emergency avoidance tangent
				float normalized_obst_dot_drone_vel = emergency_trigger_point.dot(drone_velocity_drone_frame.normalized());
				if (normalized_obst_dot_drone_vel > 0.0)
				{	

					vector_t obst_z_crossprod = emergency_trigger_point.cross(unit_z);
					vector_t tangent;

					vector_t obst_tangent = obst_z_crossprod.cross(emergency_trigger_point);

					// find if pos or neg tangent is most parallel to vel
					if (obst_tangent.dot(drone_velocity_drone_frame) > 0)
					{
						tangent = obst_tangent.normalized();
					}
					else {
						tangent = -obst_tangent.normalized();
					}

					emergency_tangent = tangent * drone_vel_magnitude * normalized_obst_dot_drone_vel * 0.067;

				}


				auto point_msg = geometry_msgs::msg::PointStamped();
				point_msg.header.stamp = this->now();
				point_msg.header.frame_id = "drone_yaw_only";
				point_msg.point.x = point_i(0);
				point_msg.point.y = point_i(1);
				point_msg.point.z = point_i(2);  

				_point_pub->publish(point_msg);


				break;			
			}		
		}
	}

	static int brake_ctr = 0;

	if (emergency_brake == true && drone_vel_magnitude > _emergency_brake_speed_limit)
	{
		brake_ctr++; 

		emergency_tangent = emergency_tangent * std::max(0.0f,  (1.0f - 0.067f * (float)brake_ctr));
	

		RCLCPP_INFO(this->get_logger(),  "Emergency braking %d", brake_ctr);

		RCLCPP_INFO(this->get_logger(),  "Emergency tangent magnitude: %f", emergency_tangent.stableNorm());

		this->_obst_tangent_vector = emergency_tangent;

		this->_out_vel_vector = cone_rejection+safety_rejection_vector_sum+tangential_rejection_vector_sum+emergency_tangent;

		return (this->_out_vel_vector);
		// return (cone_rejection);
	}
	else
	{
		emergency_brake = false;
		emergency_tangent.setZero();

		brake_ctr = 0;
	}






	////////// -------- perception horizon speed limitation -------- //////////
	// make sure maximum speed reflects maximum power line detection distance
	// atan2(x,y) < 15deg and atan2(x,z) < 55deg

	float xy_angle = atan2(input_velocity_vector(1), input_velocity_vector(0)) * 57.296; // degrees
	float xz_angle = atan2(input_velocity_vector(2), input_velocity_vector(0)) * 57.296; // degrees

	// v^2 = 2ad

	// limit front speed depending on radar FOV, detection range, and max deceleration

	if ( (abs(xy_angle) < _front_horizontal_fov_deg/2) && (abs(xz_angle) < _front_vertical_fov_deg/2))
	{
		if (input_velocity_vector_scalar > _max_allowed_front_speed)
		{
			float input_overextension_correction = input_velocity_vector_scalar / _max_allowed_front_speed;

			input_velocity_vector = input_velocity_vector / input_overextension_correction;
		}
		// RCLCPP_INFO(this->get_logger(),  "FRONT cur: %f, max: %f", input_velocity_vector_scalar, _max_allowed_front_speed);
		
	}
	else // limit side speed depending on radar FOV, detection range, and max deceleration
	{
		if ( input_velocity_vector_scalar > _max_allowed_side_speed)
		{
			float input_overextension_correction = input_velocity_vector_scalar / _max_allowed_side_speed;

			input_velocity_vector = input_velocity_vector / input_overextension_correction;
		}
		// RCLCPP_INFO(this->get_logger(),  "SIDE cur: %f, max: %f", input_velocity_vector_scalar, _max_allowed_side_speed);
	}



	// sum input velocity and all safety adjustments
	vector_t rejection_sum = input_velocity_vector + safety_rejection_vector_sum + caution_rejection_vector_sum + tangential_rejection_vector_sum;


	// find "most dangerous" object (distance to drone as well as angle wrt. velocity)
	// calculate speed correction based on angle and distance (reduce the more directly towards object)
	float reject_ratio = 1.0;
	float reject_ratio_min = 1.0;

	for (size_t i = 0; i < _combined_points->size(); i++)
	{
		vector_t obst_vect(
			_combined_points->at(i).x, 
			_combined_points->at(i).y,
			_combined_points->at(i).z
		);

		float obst_vect_l2norm = obst_vect.stableNorm();
		if (obst_vect_l2norm < _side_detection_range)
		{

			// only calculate rejection if input vel has component in obstacle direction
			//	i.e. between parellel and orthogonal (dotprod < 0), but not antiparallel
			// more parallel: dotp>0, orthogonal: dotp=0, more antiparallel: dotp<0
			float dotp = obst_vect.dot(rejection_sum);
			if (dotp > 0.0)
			{	
				// define rejection ratio based on angle to object (smaller angle more harsh) and distance to object (smaller distance more harsh)
				//  						cos(angle) - smaller angle, closer to 1						smaller distance, closer to 1
				reject_ratio = 1.0 - ( ( dotp / (rejection_sum.stableNorm() + obst_vect_l2norm )) * ( 1.0 - (obst_vect_l2norm / _side_detection_range)));

				if (reject_ratio < reject_ratio_min)
				{
					reject_ratio_min = reject_ratio;
				}
			}
		}
	}

	if (reject_ratio_min < 0.0)
	{
		reject_ratio_min = 0.0;
	}
	if (reject_ratio_min > 1.0)
	{
		reject_ratio_min = 1.0;
	}
	
	// find out how much faster than _caution_sphere_max_speed the drone is trying to go, reduce that amount by reject_ratio_min
	// reject_ratio_min will only be <1 if there is an obstacle to slow down for
	float rejection_sum_l2norm = rejection_sum.stableNorm();
	if (rejection_sum_l2norm > _caution_sphere_max_speed)
	{
		vector_t limited_out_vel = rejection_sum * (_caution_sphere_max_speed / rejection_sum_l2norm);
		vector_t reduced_overspeed = ( rejection_sum - limited_out_vel ) * reject_ratio_min;
		rejection_sum = limited_out_vel + reduced_overspeed;
	}

	_out_vel_vector = rejection_sum;
	 
	return rejection_sum;
}

void OffboardControl::input_to_output_setpoint() {

	// // static pose_t last_given_pose;
	// // static pose_t latched_drone_pose; 
	// // static bool latched_pose = false;
	// pose_t pose_to_publish;

	// // // if no input velocity and no drone velocity, control for last received position
	// // if (rotated_input_velocity.norm() < 0.01)
	// // {
		
	// // 	if (latched_pose == false)
	// // 	{
	// // 		latched_drone_pose = last_given_pose;// _drone_pose;
	// // 	}

	// // 	latched_pose = true;

	// // 	pose_to_publish = latched_drone_pose;

	// // }
	// // else 
	// // {
	// // 	latched_pose = false;

	// // 	pose_to_publish = _drone_pose;
	// // }
	
	// pose_to_publish = _drone_pose;

	// get local->world rotation (inverse of world->local)
	quat_t local_to_world_quat = quatInv(_drone_pose_yaw_only.quaternion);

	// get corresponding rotation matrix
	rotation_matrix_t local_to_world_rotation = quatToMat(local_to_world_quat);

	// calculate desired output velocity based on input velocity and nearby obstacles
	vector_t limited_velocity = OffboardControl::speed_limiter();

	// rotate desired output velocity with local->world rotation matrix
	vector_t rotated_limited_velocity = rotateVector(local_to_world_rotation, limited_velocity);

	// translate desired output velocity fron NWU to NED
	vector_t rotated_limited_velocity_NED = vector_NWU_to_NED(rotated_limited_velocity);

	// get drone position in NED
	vector_t drone_position_NED = vector_NWU_to_NED(_drone_pose.position);

	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.timestamp = _timestamp.load();

	// negate desired yaw rate to translate from NWU to NED
	float NED_yaw_speed = -_input_yaw_rate; 

	// translate yaw from NWU to NED
	static float NED_yaw = quaternionToYaw( quat_NWU_to_NED( _drone_pose_yaw_only.quaternion )) + NED_yaw_speed;

	// only change yaw when yaw input > 0
	if (NED_yaw_speed != 0.0)
	{
		NED_yaw = quaternionToYaw( quat_NWU_to_NED( _drone_pose_yaw_only.quaternion )) + NED_yaw_speed;
	}

	msg.x = drone_position_NED(0) + rotated_limited_velocity_NED(0);
	msg.y = drone_position_NED(1) + rotated_limited_velocity_NED(1);
	msg.z = drone_position_NED(2) + rotated_limited_velocity_NED(2);
	msg.yaw = NED_yaw;
	msg.yawspeed = NED_yaw_speed;
	msg.vx = rotated_limited_velocity_NED(0); 
	msg.vy = rotated_limited_velocity_NED(1); 
	msg.vz = rotated_limited_velocity_NED(2); 

	OffboardControl::publish_setpoint(msg);
}


void OffboardControl::publish_markers() {

	visualization_msgs::msg::MarkerArray marker_array;

	///////////////////////////////////////////////////////////
	// input velocity arrow
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
		_input_x_vel * _max_x_vel,
		_input_y_vel * _max_y_vel,
		_input_z_vel * _max_z_vel
	);

	quat_t arrow_rotation = findRotation(unit_x_vector, input_velocity_vector);

	input_velocity_marker.pose.orientation.x = arrow_rotation(0);
	input_velocity_marker.pose.orientation.y = arrow_rotation(1);
	input_velocity_marker.pose.orientation.z = arrow_rotation(2);
	input_velocity_marker.pose.orientation.w = arrow_rotation(3);
	input_velocity_marker.pose.position.x = 0.0; //_drone_pose.position(0);
	input_velocity_marker.pose.position.y = 0.0; //_drone_pose.position(1);
	input_velocity_marker.pose.position.z = 0.0; //_drone_pose.position(2); 

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	input_velocity_marker.scale.x = input_velocity_vector.stableNorm(); //sqrt(pow((_max_x_vel*_input_x_vel),2)+pow((_max_y_vel*_input_y_vel),2)+pow((_max_z_vel*_input_z_vel),2));
	input_velocity_marker.scale.y = input_velocity_marker.scale.x / 5;
	input_velocity_marker.scale.z = input_velocity_marker.scale.x / 5;
	// Set the color -- be sure to set alpha to something non-zero!
	input_velocity_marker.color.r = 0.0f;
	input_velocity_marker.color.g = 1.0f;
	input_velocity_marker.color.b = 0.0f;
	input_velocity_marker.color.a = 0.25;

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
	drone_velocity_marker.scale.x = drone_velocity_vector.stableNorm(); //(pow(this->_x_vel,2)+pow(this->_y_vel,2)+pow(this->_z_vel,2));
	drone_velocity_marker.scale.y = drone_velocity_marker.scale.x / 5;
	drone_velocity_marker.scale.z = drone_velocity_marker.scale.x / 5;
	// Set the color -- be sure to set alpha to something non-zero!
	drone_velocity_marker.color.r = 1.0f;
	drone_velocity_marker.color.g = 0.0f;
	drone_velocity_marker.color.b = 0.0f;
	drone_velocity_marker.color.a = 0.4;

	drone_velocity_marker.lifetime = rclcpp::Duration::from_seconds(0);

	marker_array.markers.push_back(drone_velocity_marker);




	
	///////////////////////////////////////////////////////////
	// output velocity arrow
	visualization_msgs::msg::Marker out_velocity_marker;
	out_velocity_marker.header = std_msgs::msg::Header();
	out_velocity_marker.header.stamp = this->now();
	out_velocity_marker.header.frame_id = "drone_yaw_only";

	out_velocity_marker.ns = "out_vel_arrow";
	out_velocity_marker.id = 40;

	out_velocity_marker.type = visualization_msgs::msg::Marker::ARROW;

	out_velocity_marker.action = visualization_msgs::msg::Marker::ADD;

	vector_t temp_out_vector(
		_out_vel_vector(0),
		_out_vel_vector(1),
		_out_vel_vector(2)
	);

	quat_t out_arrow_rotation = findRotation(unit_x_vector, temp_out_vector);

	out_velocity_marker.pose.orientation.x = out_arrow_rotation(0);
	out_velocity_marker.pose.orientation.y = out_arrow_rotation(1);
	out_velocity_marker.pose.orientation.z = out_arrow_rotation(2);
	out_velocity_marker.pose.orientation.w = out_arrow_rotation(3);
	out_velocity_marker.pose.position.x = 0.0; //0; 
	out_velocity_marker.pose.position.y = 0.0; //0;
	out_velocity_marker.pose.position.z = 0.0; //0; 

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	out_velocity_marker.scale.x = _out_vel_vector.stableNorm();
	out_velocity_marker.scale.y = out_velocity_marker.scale.x / 5;
	out_velocity_marker.scale.z = out_velocity_marker.scale.x / 5;
	// Set the color -- be sure to set alpha to something non-zero!
	out_velocity_marker.color.r = 1.0f;
	out_velocity_marker.color.g = 1.0f;
	out_velocity_marker.color.b = 0.0f;
	out_velocity_marker.color.a = 0.4;

	out_velocity_marker.lifetime = rclcpp::Duration::from_seconds(0);

	marker_array.markers.push_back(out_velocity_marker);





	///////////////////////////////////////////////////////////
	// obstacle tangent arrow
	visualization_msgs::msg::Marker obst_tangent_marker;
	obst_tangent_marker.header = std_msgs::msg::Header();
	obst_tangent_marker.header.stamp = this->now();
	obst_tangent_marker.header.frame_id = "drone_yaw_only";
	obst_tangent_marker.ns = "obst_tangent_arrow";
	obst_tangent_marker.id = 45;
	obst_tangent_marker.type = visualization_msgs::msg::Marker::ARROW;
	obst_tangent_marker.action = visualization_msgs::msg::Marker::ADD;

	quat_t obst_tangent_arrow_rotation = findRotation(unit_x_vector, _obst_tangent_vector);

	obst_tangent_marker.pose.orientation.x = obst_tangent_arrow_rotation(0);
	obst_tangent_marker.pose.orientation.y = obst_tangent_arrow_rotation(1);
	obst_tangent_marker.pose.orientation.z = obst_tangent_arrow_rotation(2);
	obst_tangent_marker.pose.orientation.w = obst_tangent_arrow_rotation(3);
	obst_tangent_marker.pose.position.x = 0.0; //_drone_pose.position(0);
	obst_tangent_marker.pose.position.y = 0.0; //_drone_pose.position(1);
	obst_tangent_marker.pose.position.z = 0.0; //_drone_pose.position(2); 

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	obst_tangent_marker.scale.x = _obst_tangent_vector.stableNorm();
	obst_tangent_marker.scale.y = obst_tangent_marker.scale.x / 5;
	obst_tangent_marker.scale.z = obst_tangent_marker.scale.x / 5;
	// Set the color -- be sure to set alpha to something non-zero!
	obst_tangent_marker.color.r = 1.0f;
	obst_tangent_marker.color.g = 0.0f;
	obst_tangent_marker.color.b = 1.0f;
	obst_tangent_marker.color.a = 0.4;

	obst_tangent_marker.lifetime = rclcpp::Duration::from_seconds(0);

	marker_array.markers.push_back(obst_tangent_marker);



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




	///////////////////////////////////////////////////////////
	// look-ahead volume
	visualization_msgs::msg::Marker look_ahead_marker;
	look_ahead_marker.header = std_msgs::msg::Header();
	look_ahead_marker.header.stamp = this->now();
	look_ahead_marker.header.frame_id = "world";
	look_ahead_marker.ns = "look_ahead";
	look_ahead_marker.id = 20;
	look_ahead_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
	std::string home_dir = std::getenv("HOME");
	std::string mesh_path = std::string("file://") + home_dir + std::string("/look_ahead_cone.stl");
	look_ahead_marker.mesh_resource = mesh_path;
	look_ahead_marker.action = visualization_msgs::msg::Marker::ADD;

	look_ahead_marker.pose.orientation.x = drone_arrow_rotation(0);
	look_ahead_marker.pose.orientation.y = drone_arrow_rotation(1);
	look_ahead_marker.pose.orientation.z = drone_arrow_rotation(2);
	look_ahead_marker.pose.orientation.w = drone_arrow_rotation(3);
	look_ahead_marker.pose.position.x = _drone_pose.position(0); //0; 
	look_ahead_marker.pose.position.y = _drone_pose.position(1); //0;
	look_ahead_marker.pose.position.z = _drone_pose.position(2); //0; 

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	look_ahead_marker.scale.x = _look_ahead_cone_length * 2; // times 2 because marker scale also extends into negative axis?
	look_ahead_marker.scale.y = look_ahead_marker.scale.x / _look_ahead_cone_length_to_width_ratio;
	look_ahead_marker.scale.z = look_ahead_marker.scale.x / _look_ahead_cone_length_to_width_ratio;
	// Set the color -- be sure to set alpha to something non-zero!
	look_ahead_marker.color.r = 0.9f;
	look_ahead_marker.color.g = 0.9f;
	look_ahead_marker.color.b = 0.9f;
	look_ahead_marker.color.a = 0.75f;

	look_ahead_marker.lifetime = rclcpp::Duration::from_seconds(0);

	marker_array.markers.push_back(look_ahead_marker);
	




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


	_drone_pose.position(0) = t.transform.translation.x;
	_drone_pose.position(1) = t.transform.translation.y;
	_drone_pose.position(2) = t.transform.translation.z;
	
	_drone_pose.quaternion(0) = t.transform.rotation.x;
	_drone_pose.quaternion(1) = t.transform.rotation.y;
	_drone_pose.quaternion(2) = t.transform.rotation.z;
	_drone_pose.quaternion(3) = t.transform.rotation.w;




	try {
		if (tf_buffer_->canTransform("world", "drone_yaw_only", tf2::TimePointZero))	{
			t = tf_buffer_->lookupTransform("world","drone_yaw_only",tf2::TimePointZero);
		}
		else {
			RCLCPP_INFO(this->get_logger(), "Can not transform");
			return;
		}
	} catch (const tf2::TransformException & ex) {
		RCLCPP_INFO(this->get_logger(), "Could not transform: %s", ex.what());
		return;
	}


	_drone_pose_yaw_only.position(0) = t.transform.translation.x;
	_drone_pose_yaw_only.position(1) = t.transform.translation.y;
	_drone_pose_yaw_only.position(2) = t.transform.translation.z;
	
	_drone_pose_yaw_only.quaternion(0) = t.transform.rotation.x;
	_drone_pose_yaw_only.quaternion(1) = t.transform.rotation.y;
	_drone_pose_yaw_only.quaternion(2) = t.transform.rotation.z;
	_drone_pose_yaw_only.quaternion(3) = t.transform.rotation.w;

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


	publish_pose = pose_eul_NWU_to_NED(publish_pose);
	
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
		NWU_to_NED_pose = pose_eul_NWU_to_NED(NWU_to_NED_pose);
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

	NWU_to_NED_pose = pose_eul_NWU_to_NED(NWU_to_NED_pose);

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
