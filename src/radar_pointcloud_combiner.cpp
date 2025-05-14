// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include "geometry.h"

 // MISC includes
#include <cstdlib>
#include <stdlib.h> 
#include <chrono>
#include <math.h> 
#include <cmath> 
#include <vector>
#include <deque>
#include <string>
#include <numeric>

// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/angles.h>
#include <pcl/filters/extract_indices.h>



#define DEG_PER_RAD 57.2957795
#define RAD_PER_DEG 0.01745329
#define PI 3.14159265

using namespace std::chrono_literals;

//creates a RadarPCLFilter class that subclasses the generic rclcpp::Node base class.
class RadarPCLFilter : public rclcpp::Node
{
	public:
		RadarPCLFilter() : Node("radar_pcl_combiner_node") {
			
			// Params			
			this->declare_parameter<int>("pointcloud_update_rate", 25); // Hz
			this->get_parameter("pointcloud_update_rate", _pointcloud_update_rate);

			this->declare_parameter<int>("pointcloud_clear_rate", 1); // Hz
			this->get_parameter("pointcloud_clear_rate", _pointcloud_clear_rate);

			this->declare_parameter<int>("snr_cutoff", 10); // dB
			this->get_parameter("snr_cutoff", _snr_cutoff);

			this->declare_parameter<float>("doppler_margin", 2.0); // ms
			this->get_parameter("doppler_margin", _doppler_margin);
			
			


			front_pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			"/front_pcl",	10,
			std::bind(&RadarPCLFilter::add_front_radar_pointcloud, this, std::placeholders::_1));

			rear_pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			"/rear_pcl",	10,
			std::bind(&RadarPCLFilter::add_rear_radar_pointcloud, this, std::placeholders::_1));

			top_pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			"/top_pcl",	10,
			std::bind(&RadarPCLFilter::add_top_radar_pointcloud, this, std::placeholders::_1));

			bot_pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			"/bot_pcl",	10,
			std::bind(&RadarPCLFilter::add_bot_radar_pointcloud, this, std::placeholders::_1));

			right_pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			"/right_pcl",	10,
			std::bind(&RadarPCLFilter::add_right_radar_pointcloud, this, std::placeholders::_1));

			left_pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			"/left_pcl",	10,
			std::bind(&RadarPCLFilter::add_left_radar_pointcloud, this, std::placeholders::_1));

			combined_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/combined_pcl", 10);

			tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
			transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

			_timer_pcl = this->create_wall_timer(
				std::chrono::milliseconds(static_cast<int>((1.0 / _pointcloud_update_rate) * 1000)),
				std::bind(&RadarPCLFilter::publish_combined_pointcloud, this));

			_timer_clear_pcl = this->create_wall_timer(
				std::chrono::milliseconds(static_cast<int>((1.0 / _pointcloud_clear_rate) * 1000)),
				std::bind(&RadarPCLFilter::check_pointclouds, this));

			_vehicle_odometry_subscriber = create_subscription<px4_msgs::msg::VehicleOdometry>(
				"/fmu/vehicle_odometry/out", 10,
				[this](px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg) {
					_x_vel = msg->vx; 
					_y_vel = -msg->vy; // Negated to get NWU
					_z_vel = -msg->vz; // Negated to get NWU
				});
		
			while(true) {

				static int _t_tries = 0;

				try {

					front_to_drone_tf = tf_buffer_->lookupTransform("drone", "front_frame", tf2::TimePointZero);
					rear_to_drone_tf = tf_buffer_->lookupTransform("drone", "rear_frame", tf2::TimePointZero);
					top_to_drone_tf = tf_buffer_->lookupTransform("drone", "top_frame", tf2::TimePointZero);
					bot_to_drone_tf = tf_buffer_->lookupTransform("drone", "bot_frame", tf2::TimePointZero);
					right_to_drone_tf = tf_buffer_->lookupTransform("drone", "right_frame", tf2::TimePointZero);
					left_to_drone_tf = tf_buffer_->lookupTransform("drone", "left_frame", tf2::TimePointZero);
					drone_to_drone_yaw_tf = tf_buffer_->lookupTransform("drone", "drone_yaw_only", tf2::TimePointZero);


					RCLCPP_INFO(this->get_logger(), "Found all transforms");
					break;

				} catch(tf2::TransformException & ex) {

					RCLCPP_INFO(this->get_logger(), "Could not get all transforms, trying again...");
					
					_t_tries++;

					std::this_thread::sleep_for(std::chrono::milliseconds(500));

					if( _t_tries > 10) {
						RCLCPP_FATAL(this->get_logger(), "Failed to get all transforms after 10 tries.");
						throw std::exception();
					}
				}

			}


			pcl::PointCloud<pcl::PointXYZ>::Ptr tmp1_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr tmp2_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr tmp3_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr tmp4_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr tmp5_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr tmp6_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			this->front_cloud = tmp1_cloud;
			this->rear_cloud = tmp2_cloud;
			this->top_cloud = tmp3_cloud;
			this->bot_cloud = tmp4_cloud;
			this->right_cloud = tmp5_cloud;
			this->left_cloud = tmp6_cloud;

			pcl::PointCloud<pcl::PointXYZ>::Ptr tmp7_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			this->combined_cloud = tmp7_cloud;


			_combined_pcl_msg = sensor_msgs::msg::PointCloud2();
			_combined_pcl_msg.header = std_msgs::msg::Header();
			const std::string frame_id = "drone_yaw_only";
			_combined_pcl_msg.header.frame_id = frame_id;
			_combined_pcl_msg.fields.resize(3);
			_combined_pcl_msg.fields[0].name = 'x';
			_combined_pcl_msg.fields[0].offset = 0;
			_combined_pcl_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
			_combined_pcl_msg.fields[0].count = 1;
			_combined_pcl_msg.fields[1].name = 'y';
			_combined_pcl_msg.fields[1].offset = 4;
			_combined_pcl_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
			_combined_pcl_msg.fields[1].count = 1;
			_combined_pcl_msg.fields[2].name = 'z';
			_combined_pcl_msg.fields[2].offset = 8;
			_combined_pcl_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
			_combined_pcl_msg.fields[2].count = 1;
			const uint32_t POINT_STEP = 12;
			_combined_pcl_msg.point_step = POINT_STEP; // size (bytes) of 1 point (float32 * dimensions (3 when xyz))
			_combined_pcl_msg.height = 1;  // because unordered cloud
			_combined_pcl_msg.is_dense = false; // there may be invalid points


		}

		~RadarPCLFilter() {
			RCLCPP_INFO(this->get_logger(),  "Shutting down radar_pointcloud_combiner node..");
		}

	private:
		std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  		std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

		rclcpp::TimerBase::SharedPtr _timer_pcl;
		rclcpp::TimerBase::SharedPtr _timer_clear_pcl;

		rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _vehicle_odometry_subscriber;

		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr combined_pointcloud_pub;

		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr front_pcl_subscription_;
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr rear_pcl_subscription_;
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr top_pcl_subscription_;
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr bot_pcl_subscription_;
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr right_pcl_subscription_;
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr left_pcl_subscription_;


		

		void add_new_radar_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

		void add_front_radar_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
		void add_rear_radar_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
		void add_top_radar_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
		void add_bot_radar_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
		void add_right_radar_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
		void add_left_radar_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
		void check_pointclouds();
		void update_drone_pose();
		homog_transform_t get_radar_to_drone_tf(geometry_msgs::msg::TransformStamped tf);
		void publish_combined_pointcloud();

		void create_pointcloud_msg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, sensor_msgs::msg::PointCloud2 * pcl_msg);

		void read_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, 
										pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		void read_pointcloud_and_sideinfo(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
											pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
											std::vector<float> &velocities,
											std::vector<float> &snrs,
											std::vector<float> &noises);

		void doppler_filter_points(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
											std::vector<float> &velocities);

		pcl::PointCloud<pcl::PointXYZ>::Ptr front_cloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr rear_cloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr top_cloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr bot_cloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr right_cloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr left_cloud;

		pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud;

		geometry_msgs::msg::TransformStamped front_to_drone_tf;
		geometry_msgs::msg::TransformStamped rear_to_drone_tf;
		geometry_msgs::msg::TransformStamped top_to_drone_tf;
		geometry_msgs::msg::TransformStamped bot_to_drone_tf;
		geometry_msgs::msg::TransformStamped right_to_drone_tf;
		geometry_msgs::msg::TransformStamped left_to_drone_tf;
		geometry_msgs::msg::TransformStamped drone_to_drone_yaw_tf;

		int _pointcloud_update_rate;
		int _pointcloud_clear_rate;
		float _doppler_margin; // m/s
		int _snr_cutoff;

		int top_cnt = 0;
		int bot_cnt = 0;
		int front_cnt = 0;
		int rear_cnt = 0;
		int right_cnt = 0;
		int left_cnt = 0;

		float _x_vel = 0.0; // NWU
		float _y_vel = 0.0; // NWU
		float _z_vel = 0.0; // NWU

		sensor_msgs::msg::PointCloud2 _combined_pcl_msg; 

		pose_t _drone_pose_yaw_only; // in world coordinates North-West-Up
};


void RadarPCLFilter::check_pointclouds() {
	// periodically check if radar data is still receieved, clear if stale data
	std::string str_missing_pcl = "";
	
	if (front_cnt < 1)
	{
		str_missing_pcl += "FRONT ";
		this->front_cloud->clear();
	}
	front_cnt = 0;
	
	if (rear_cnt < 1)
	{
		str_missing_pcl += "REAR ";
		this->rear_cloud->clear();
	}
	rear_cnt = 0;

	if (top_cnt < 1)
	{
		str_missing_pcl += "TOP ";
		this->top_cloud->clear();
	}
	top_cnt = 0;

	if (bot_cnt < 1)
	{
		str_missing_pcl += "BOT ";
		this->bot_cloud->clear();
	}
	bot_cnt = 0;
	
	if (right_cnt < 1)
	{
		str_missing_pcl += "RIGHT ";
		this->right_cloud->clear();
	}
	right_cnt = 0;

	if (left_cnt < 1)
	{
		str_missing_pcl += "LEFT";
		this->left_cloud->clear();
	}
	left_cnt = 0;

	if (str_missing_pcl != "")
	{
		RCLCPP_WARN(this->get_logger(),  "Missing radar data: %s%s%s", "\033[33m", str_missing_pcl.c_str(), "\033[0m"); // YELLOW and RESET
	}
	
}


void RadarPCLFilter::publish_combined_pointcloud() {

	this->combined_cloud->clear();
	// RCLCPP_INFO(this->get_logger(), "Size %d", combined_cloud->size());
	*combined_cloud += *front_cloud; 
	// RCLCPP_INFO(this->get_logger(), "Size %d", front_cloud->size());
	*combined_cloud += *rear_cloud;
	// RCLCPP_INFO(this->get_logger(), "Size %d", rear_cloud->size());
	*combined_cloud += *top_cloud;
	// RCLCPP_INFO(this->get_logger(), "Size %d", top_cloud->size());
	*combined_cloud += *bot_cloud;
	// RCLCPP_INFO(this->get_logger(), "Size %d", bot_cloud->size());
	*combined_cloud += *right_cloud;
	// RCLCPP_INFO(this->get_logger(), "Size %d", right_cloud->size());
	*combined_cloud += *left_cloud;
	// RCLCPP_INFO(this->get_logger(), "Size %d", left_cloud->size());


	drone_to_drone_yaw_tf = tf_buffer_->lookupTransform("drone_yaw_only", "drone", tf2::TimePointZero);

	homog_transform_t radar_to_drone = RadarPCLFilter::get_radar_to_drone_tf(drone_to_drone_yaw_tf);

	pcl::transformPointCloud (*combined_cloud, *combined_cloud, radar_to_drone);

	RadarPCLFilter::create_pointcloud_msg(combined_cloud, &_combined_pcl_msg);

	if (_combined_pcl_msg.width < 1)
	{
		return;
	}
		
	combined_pointcloud_pub->publish(_combined_pcl_msg); 
}



void RadarPCLFilter::add_front_radar_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

	this->front_cnt++;

	this->front_cloud->clear();

	if (msg->width < 1)
	{
		return;
	}

	static std::vector<float> velocities, snrs, noises; 

	RadarPCLFilter::read_pointcloud_and_sideinfo(msg, front_cloud, velocities, snrs, noises);

	// make transform front->drone
	static const homog_transform_t radar_to_drone = RadarPCLFilter::get_radar_to_drone_tf(this->front_to_drone_tf);

	// transform points in pointcloud
	pcl::transformPointCloud (*front_cloud, *front_cloud, radar_to_drone);

	RadarPCLFilter::doppler_filter_points(front_cloud, velocities);
}


void RadarPCLFilter::add_rear_radar_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

	this->rear_cnt++;

	this->rear_cloud->clear();

	if (msg->width < 1)
	{
		return;
	}

	static std::vector<float> velocities, snrs, noises; 

	RadarPCLFilter::read_pointcloud_and_sideinfo(msg, rear_cloud, velocities, snrs, noises);

	// make transform front->drone
	static const homog_transform_t radar_to_drone = RadarPCLFilter::get_radar_to_drone_tf(this->rear_to_drone_tf);

	// transform points in pointcloud
	pcl::transformPointCloud (*rear_cloud, *rear_cloud, radar_to_drone);

	RadarPCLFilter::doppler_filter_points(rear_cloud, velocities);
}


void RadarPCLFilter::add_top_radar_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

	this->top_cnt++;

	this->top_cloud->clear();

	if (msg->width < 1)
	{
		return;
	}

	static std::vector<float> velocities, snrs, noises; 

	RadarPCLFilter::read_pointcloud_and_sideinfo(msg, top_cloud, velocities, snrs, noises);

	// make transform front->drone
	static const homog_transform_t radar_to_drone = RadarPCLFilter::get_radar_to_drone_tf(this->top_to_drone_tf);

	// transform points in pointcloud
	pcl::transformPointCloud (*top_cloud, *top_cloud, radar_to_drone);

	RadarPCLFilter::doppler_filter_points(top_cloud, velocities);
}


void RadarPCLFilter::add_bot_radar_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

	this->bot_cnt++;

	this->bot_cloud->clear();

	if (msg->width < 1)
	{
		return;
	}

	static std::vector<float> velocities, snrs, noises; 

	RadarPCLFilter::read_pointcloud_and_sideinfo(msg, bot_cloud, velocities, snrs, noises);

	// make transform front->drone
	static const homog_transform_t radar_to_drone = RadarPCLFilter::get_radar_to_drone_tf(this->bot_to_drone_tf);

	// transform points in pointcloud
	pcl::transformPointCloud (*bot_cloud, *bot_cloud, radar_to_drone);

	RadarPCLFilter::doppler_filter_points(bot_cloud, velocities);
}


void RadarPCLFilter::add_right_radar_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

	this->right_cnt++;

	this->right_cloud->clear();

	if (msg->width < 1)
	{
		return;
	}

	static std::vector<float> velocities, snrs, noises; 

	RadarPCLFilter::read_pointcloud_and_sideinfo(msg, right_cloud, velocities, snrs, noises);

	// make transform front->drone
	static const homog_transform_t radar_to_drone = RadarPCLFilter::get_radar_to_drone_tf(this->right_to_drone_tf);

	// transform points in pointcloud
	pcl::transformPointCloud (*right_cloud, *right_cloud, radar_to_drone);

	RadarPCLFilter::doppler_filter_points(right_cloud, velocities);
}


void RadarPCLFilter::add_left_radar_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

	this->left_cnt++;

	this->left_cloud->clear();

	if (msg->width < 1)
	{
		return;
	}

	static std::vector<float> velocities, snrs, noises; 

	RadarPCLFilter::read_pointcloud_and_sideinfo(msg, left_cloud, velocities, snrs, noises);

	// make transform front->drone
	static const homog_transform_t radar_to_drone = RadarPCLFilter::get_radar_to_drone_tf(this->left_to_drone_tf);

	// transform points in pointcloud
	pcl::transformPointCloud (*left_cloud, *left_cloud, radar_to_drone);

	RadarPCLFilter::doppler_filter_points(left_cloud, velocities);
	
}


homog_transform_t RadarPCLFilter::get_radar_to_drone_tf(geometry_msgs::msg::TransformStamped tf) {

		// make transform radar->drone
		vector_t _t_xyz(
			tf.transform.translation.x,
			tf.transform.translation.y,
			tf.transform.translation.z
		);
	
		quat_t _t_rot(
			tf.transform.rotation.x,
			tf.transform.rotation.y,
			tf.transform.rotation.z,
			tf.transform.rotation.w
		);

		homog_transform_t radar_to_drone = getTransformMatrix(_t_xyz, _t_rot);

		return radar_to_drone;

}


// Remove points if their radial doppler velocity cannot be explained by ego velocity
void RadarPCLFilter::doppler_filter_points(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
											std::vector<float> &velocities) {

	RadarPCLFilter::update_drone_pose(); 

	float yaw = quatToEul(_drone_pose_yaw_only.quaternion)(2);

	// rotation about Z
    float c = std::cos(-yaw), s = std::sin(-yaw);
	
	vector_t drone_velocity_drone_frame{
        _x_vel * c - _y_vel * s,
        _x_vel * s + _y_vel * c,
        _z_vel
    };

	// // find drone yaw and subtract from world yaw
	// orientation_t drone_yaw(
	// 	0.0, 
	// 	0.0,
	// 	-drone_yaw_f
	// ); 

	// rotation_matrix_t yaw_rotation = quatToMat(eulToQuat(drone_yaw));

	// vector_t drone_velocity(
	// 	_x_vel,
	// 	_y_vel,
	// 	_z_vel
	// );

	// // get drone velocity in local drone frame (no roll and pitch)
	// vector_t drone_velocity_drone_frame = rotateVector(yaw_rotation, drone_velocity);

	// check if each point's radial dopper velocity is within expected region wrt. drone velocty - else remove
	pcl::PointIndices::Ptr indices(new pcl::PointIndices);
	for (size_t i = 0; i < velocities.size(); i++)
	{
		vector_t obst_vect(
			cloud->points[i].x, 
			cloud->points[i].y, 
			cloud->points[i].z
		);

		// 1) form the line‐of‐sight unit vector:
		vector_t obst_vect_normalized = obst_vect.normalized();

		// 2) signed radial velocity = dot(drone_velocity, los)
		float ego_radial_velocity = obst_vect_normalized.dot(drone_velocity_drone_frame);

		// 3) compare that signed value to your measured Doppler (also signed!)
		if ( abs(ego_radial_velocity - velocities.at(i)) > _doppler_margin )
		{
			indices->indices.push_back(static_cast<int>(i));
		}
		
	}

	// Set up the extractor
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.setNegative(true);    // remove, not keep

    // Perform filtering in place
    extract.filter(*cloud);

}


void RadarPCLFilter::create_pointcloud_msg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, sensor_msgs::msg::PointCloud2 * pcl_msg) {

  // create PointCloud2 msg
	//https://github.com/ros-drivers/velodyne/blob/master/velodyne_laserscan/tests/system.cpp
	int pcl_size = cloud->size();
	const uint32_t POINT_STEP = 12;

	pcl_msg->header.stamp = this->now();

	if(pcl_size > 0){
		pcl_msg->data.resize(std::max((size_t)1, (size_t)pcl_size) * POINT_STEP, 0x00);
	} else {
        return;
    }

	pcl_msg->row_step = pcl_msg->data.size();//pcl_msg->point_step * pcl_msg->width; // only 1 row because unordered
	pcl_msg->width = pcl_msg->row_step / POINT_STEP; // number of points in cloud

	// fill PointCloud2 msg data
	uint8_t *ptr = pcl_msg->data.data();

	for (size_t i = 0; i < (size_t)pcl_size; i++)
	{
		pcl::PointXYZ point = (*cloud)[i];

        *(reinterpret_cast<float*>(ptr + 0)) = point.x;
        *(reinterpret_cast<float*>(ptr + 4)) = point.y;
        *(reinterpret_cast<float*>(ptr + 8)) = point.z;
        ptr += POINT_STEP;
	}
	
}
									
// read point cloud information
void RadarPCLFilter::read_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, 
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

// read point cloud information, including radial doppler velocity (m/s), SNR (dB), and noise (dB) information
void RadarPCLFilter::read_pointcloud_and_sideinfo(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    std::vector<float> &velocities,
    std::vector<float> &snrs,
    std::vector<float> &noises)
{
    // 1) Discover field offsets
    int off_x     = -1, off_y     = -1, off_z     = -1;
    int off_vel   = -1, off_snr   = -1, off_noise = -1;
    for (const auto &f : msg->fields) {
        if      (f.name == "x")     off_x     = f.offset;
        else if (f.name == "y")     off_y     = f.offset;
        else if (f.name == "z")     off_z     = f.offset;
        else if (f.name == "vel")   off_vel   = f.offset;
        else if (f.name == "snr")   off_snr   = f.offset;
        else if (f.name == "noise") off_noise = f.offset;
    }

    // 2) Prepare containers
    size_t count = msg->width * msg->height;
    cloud->clear();
    cloud->reserve(count);

    velocities.clear();  if (off_vel   >= 0) velocities.reserve(count);
    snrs.clear();        if (off_snr   >= 0) snrs.reserve(count);
    noises.clear();      if (off_noise >= 0) noises.reserve(count);

    const uint8_t *ptr     = msg->data.data();
    uint32_t      step     = msg->point_step;

    // 3) Iterate points
    for (size_t i = 0; i < count; ++i, ptr += step) {
        // XYZ
        float x = *reinterpret_cast<const float*>(ptr + off_x);
        float y = *reinterpret_cast<const float*>(ptr + off_y);
        float z = *reinterpret_cast<const float*>(ptr + off_z);
        cloud->push_back({x,y,z});

        // Optional extras
        if (off_vel >= 0) {
            float v = *reinterpret_cast<const float*>(ptr + off_vel);
            velocities.push_back(v);
        }
        if (off_snr >= 0) {
            float s = *reinterpret_cast<const float*>(ptr + off_snr);
            snrs.push_back(s);
        }
        if (off_noise >= 0) {
            float n = *reinterpret_cast<const float*>(ptr + off_noise);
            noises.push_back(n);
        }
    }
}


void RadarPCLFilter::update_drone_pose() {

	geometry_msgs::msg::TransformStamped t;

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

	_drone_pose_yaw_only.quaternion(2) = t.transform.rotation.z; // only need this value for calculations

}

	
			
int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RadarPCLFilter>());

	rclcpp::shutdown();
	return 0;
}
