#include <rclcpp/rclcpp.hpp>
#include <toggle_radar_msgs/msg/toggle_radar.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <px4_msgs/msg/vehicle_odometry.hpp>


#include <algorithm>
#include <cstdlib>
#include <stdlib.h> 
#include <iostream>
#include <chrono>
#include <ctime>    
#include <math.h>  
#include <limits>
#include <vector>
#include <string>

using namespace std::chrono_literals;

//creates a RadarToggler class that subclasses the generic rclcpp::Node base class.
class RadarToggler : public rclcpp::Node
{

//Creates a function for when messages are to be sent. 
//Messages are sent based on a timed callback.
	public:
		RadarToggler() : Node("radar_toggler") {

			this->declare_parameter<float>("min_vel", 0.25); // minimum velocity below which radars are cycled every 1s
			this->get_parameter("min_vel", _min_vel);


			toggle_publisher_ = this->create_publisher<toggle_radar_msgs::msg::ToggleRadar>("/radar_toggle", 10);

			velocity_vector_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/velocity_vector_pose", 10);

			odometry_subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/vehicle_odometry/out", 10,
            std::bind(&RadarToggler::odometryCallback, this, std::placeholders::_1));

		}

		~RadarToggler() {
			RCLCPP_INFO(this->get_logger(),  "Shutting down lidar_to_mmwave_converter..");
		}


	private:
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<toggle_radar_msgs::msg::ToggleRadar>::SharedPtr toggle_publisher_;
		rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr velocity_vector_pub;
		rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscription_;
		
		void odometryCallback(const std::shared_ptr<px4_msgs::msg::VehicleOdometry> msg);

		std::string _input_topic;
		std::string _output_topic;
		int _frame_idx;
		float _min_vel;
		std::array<bool, 6> _toggle_array;

		rclcpp::Time last_pub_{0, 0, RCL_ROS_TIME};
    	const rclcpp::Duration min_period_{rclcpp::Duration::from_seconds(0.1)};  // 10 Hz

};


void RadarToggler::odometryCallback(const std::shared_ptr<px4_msgs::msg::VehicleOdometry> msg){

	rclcpp::Time now = this->get_clock()->now();

	if (now - last_pub_ < min_period_) { // only update transforms at 1/min_period_ Hz
		return;
	}
	last_pub_ = now;

	// get drone velocity in drone frame

    // 1. Velocity in NED (world) frame
    tf2::Vector3 vel_ned(msg->vx, msg->vy, msg->vz);

    // 2. Orientation of drone in world frame
    tf2::Quaternion q_enu(msg->q[1], msg->q[2], msg->q[3], msg->q[0]); // PX4 msg format is wxyz, change to xyzw - why is q[3] q[0] q[1] q[2] used in frame_broadcaster?

    // 3. Inverse rotation: world -> body frame
    tf2::Quaternion q_inv = q_enu.inverse();

    // 4. Rotate velocity vector into body frame
    tf2::Vector3 vel_body = tf2::quatRotate(q_inv, vel_ned);

    // 5. Normalize to get unit vector, transform NED -> ENU (negate y and z)
    tf2::Vector3 unit_vec = vel_body.normalized();
	tf2::Vector3 velocity_body_ENU( unit_vec.x(), -unit_vec.y(), -unit_vec.z() );

	geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = rclcpp::Clock().now();
    pose.header.frame_id = "drone";

    // 1. Set position
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;

    // 2. Get normalized direction
    tf2::Vector3 dir = velocity_body_ENU;

    // 3. Define default forward direction
    tf2::Vector3 forward(1, 0, 0);

    // 4. Compute rotation from forward → dir
    tf2::Quaternion q;
    if ((dir - forward).length2() < 1e-6) {
        q.setRPY(0, 0, 0); // already aligned
    } else if ((dir + forward).length2() < 1e-6) {
        q.setRPY(M_PI, 0, 0); // opposite
    } else {
        tf2::Vector3 axis = forward.cross(dir).normalized();
        double angle = std::acos(forward.dot(dir));
        q.setRotation(axis, angle);
    }

    // 5. Set quaternion
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

	// velocity_vector_pub->publish(pose);

	std::fill(std::begin(_toggle_array), std::end(_toggle_array), false); // set _toggle_array all false

	if ( (pow(msg->vx,2) + pow(msg->vy,2) + pow(msg->vz,2)) > pow(_min_vel,2) )
	{
		// Case 1: drone velocity >_min_vel
		// Turn on radar in the same direction as velocity, turn off others

		// express Quaternion in spherical (elevation and azimuth)
		double x = dir.x();
		double y = dir.y();
		double z = dir.z();

		// Azimuth: angle in XY plane from +X (forward)
		double azimuth_rad = std::atan2(y, x);

		// Elevation: angle above XY plane
		double elevation_rad = std::atan2(z, std::sqrt(x*x + y*y));

		// Convert to degrees
		double azimuth_deg = azimuth_rad * 180.0 / M_PI;
		double elevation_deg = elevation_rad * 180.0 / M_PI;

		// if elevation > 45deg = use top
		if (elevation_deg > 45)
		{
			_toggle_array[2] = true; //{0, 0, 1, 0, 0, 0};
		}
		
		// elif elevation < -45deg = use bot
		else if (elevation_deg < -45)
		{
			_toggle_array[3] = true; //{0, 0, 0, 1, 0, 0};
		}

		// elif azim {-30, 30} = use front
		else if (azimuth_deg > -30 && azimuth_deg < 30)
		{
			_toggle_array[0] = true; //{1, 0, 0, 0, 0, 0};
		}

		// elif azim {30, 130} = use left
		else if (azimuth_deg > 30 && azimuth_deg < 130)
		{
			_toggle_array[5] = true; //{0, 0, 0, 0, 0, 1};
		}

		// elif azim {-30, -130} = use right
		else if (azimuth_deg < -30 && azimuth_deg > -130)
		{
			_toggle_array[4] = true; //{0, 0, 0, 0, 1, 0};
		}

		// else use rear
		else {
			_toggle_array[1] = true; //{0, 1, 0, 0, 0, 0};
		}
	
	}

	else {
		// Case 2: drone velocity <_min vel
		// Cycle through all radars every 1s

		static float cycle_index = 0.0;

		switch ((int)std::floor(cycle_index)) {
			case 0:
				_toggle_array[0] = true;
				_toggle_array[1] = true;
				break;
			case 1:
				_toggle_array[2] = true;
				_toggle_array[3] = true;
				break;
			case 2:
				_toggle_array[4] = true;
				_toggle_array[5] = true;
				break;
		}

		cycle_index = cycle_index + 0.51;
		if( cycle_index > 3.0 && ((int)std::floor(cycle_index)) % 3  == 0)
			cycle_index = 0.0;

	}

	toggle_radar_msgs::msg::ToggleRadar toggle_msg;

	toggle_msg.radar_toggle_array = _toggle_array;

	toggle_publisher_->publish(toggle_msg);

	RCLCPP_WARN(this->get_logger(), "Current: %d %d %d %d %d %d ", _toggle_array[0],_toggle_array[1],_toggle_array[2],_toggle_array[3],_toggle_array[4],_toggle_array[5]);

}

	
			
int main(int argc, char *argv[])
{
	std::cout << "Starting lidar_to_mmwave_converter node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RadarToggler>());

	rclcpp::shutdown();
	return 0;
}
