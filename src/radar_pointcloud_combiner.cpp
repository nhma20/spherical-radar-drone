// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <sensor_msgs/msg/image.hpp>
#include "geometry.h"

// Debug
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

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
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/angles.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/segmentation/sac_segmentation.h>



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
			this->declare_parameter<int>("pointcloud_update_rate", 20); // Hz
			this->get_parameter("pointcloud_update_rate", _pointcloud_update_rate);

			this->declare_parameter<int>("pointcloud_clear_rate", 5); // Hz
			this->get_parameter("pointcloud_clear_rate", _pointcloud_clear_rate);
			
			


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
				std::bind(&RadarPCLFilter::clear_pointclouds, this));


		
			while(true) {

				static int _t_tries = 0;

				try {

					// front_to_drone_tf = tf_buffer_->lookupTransform("front_frame", "drone", tf2::TimePointZero);
					front_to_drone_tf = tf_buffer_->lookupTransform("drone", "front_frame", tf2::TimePointZero);
					rear_to_drone_tf = tf_buffer_->lookupTransform("drone", "rear_frame", tf2::TimePointZero);
					top_to_drone_tf = tf_buffer_->lookupTransform("drone", "top_frame", tf2::TimePointZero);
					bot_to_drone_tf = tf_buffer_->lookupTransform("drone", "bot_frame", tf2::TimePointZero);
					right_to_drone_tf = tf_buffer_->lookupTransform("drone", "right_frame", tf2::TimePointZero);
					left_to_drone_tf = tf_buffer_->lookupTransform("drone", "left_frame", tf2::TimePointZero);

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
		}

		~RadarPCLFilter() {
			RCLCPP_INFO(this->get_logger(),  "Shutting down radar_pointcloud_combiner node..");
		}

	private:
		std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  		std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

		rclcpp::TimerBase::SharedPtr _timer_pcl;
		rclcpp::TimerBase::SharedPtr _timer_clear_pcl;

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
		void clear_pointclouds();

		void publish_combined_pointcloud();

		void create_pointcloud_msg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, sensor_msgs::msg::PointCloud2 * pcl_msg, std::string frame_id);

		void read_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, 
										pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

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

		int _pointcloud_update_rate;
		int _pointcloud_clear_rate;
};


void RadarPCLFilter::clear_pointclouds() {
	// clear all clouds periodically to avoid stale points when no new radar clouds received
	front_cloud->clear();
	rear_cloud->clear();
	top_cloud->clear();
	bot_cloud->clear(); 		
	right_cloud->clear();
	left_cloud->clear();
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

	// RCLCPP_INFO(this->get_logger(), "End\n");

	auto pcl_msg = sensor_msgs::msg::PointCloud2();
	std::string frame_id = "drone";
	RadarPCLFilter::create_pointcloud_msg(combined_cloud, &pcl_msg, frame_id);

	if (pcl_msg.width < 1)
	{
		return;
	}
		
	combined_pointcloud_pub->publish(pcl_msg); 
}



void RadarPCLFilter::add_front_radar_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

	if (msg->width < 1)
	{
		return;
	}

	homog_transform_t radar_to_drone;

	vector_t _t_xyz;
	quat_t _t_rot;

	// make transform front->drone
	_t_xyz(0) = this->front_to_drone_tf.transform.translation.x;
	_t_xyz(1) = this->front_to_drone_tf.transform.translation.y;
	_t_xyz(2) = this->front_to_drone_tf.transform.translation.z;

	_t_rot(0) = this->front_to_drone_tf.transform.rotation.x;
	_t_rot(1) = this->front_to_drone_tf.transform.rotation.y;
	_t_rot(2) = this->front_to_drone_tf.transform.rotation.z;
	_t_rot(3) = this->front_to_drone_tf.transform.rotation.w;

	radar_to_drone = getTransformMatrix(_t_xyz, _t_rot);


	// transform points in pointcloud

	this->front_cloud->clear();

	RadarPCLFilter::read_pointcloud(msg, front_cloud);

	pcl::transformPointCloud (*front_cloud, *front_cloud, radar_to_drone);
}


void RadarPCLFilter::add_rear_radar_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

	if (msg->width < 1)
	{
		return;
	}

	homog_transform_t radar_to_drone;

	vector_t _t_xyz;
	quat_t _t_rot;

	// make transform front->drone
	_t_xyz(0) = this->rear_to_drone_tf.transform.translation.x;
	_t_xyz(1) = this->rear_to_drone_tf.transform.translation.y;
	_t_xyz(2) = this->rear_to_drone_tf.transform.translation.z;

	_t_rot(0) = this->rear_to_drone_tf.transform.rotation.x;
	_t_rot(1) = this->rear_to_drone_tf.transform.rotation.y;
	_t_rot(2) = this->rear_to_drone_tf.transform.rotation.z;
	_t_rot(3) = this->rear_to_drone_tf.transform.rotation.w;

	radar_to_drone = getTransformMatrix(_t_xyz, _t_rot);


	// transform points in pointcloud

	this->rear_cloud->clear();

	RadarPCLFilter::read_pointcloud(msg, rear_cloud);

	pcl::transformPointCloud (*rear_cloud, *rear_cloud, radar_to_drone);
}


void RadarPCLFilter::add_top_radar_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

	if (msg->width < 1)
	{
		return;
	}

	homog_transform_t radar_to_drone;

	vector_t _t_xyz;
	quat_t _t_rot;

	// make transform front->drone
	_t_xyz(0) = this->top_to_drone_tf.transform.translation.x;
	_t_xyz(1) = this->top_to_drone_tf.transform.translation.y;
	_t_xyz(2) = this->top_to_drone_tf.transform.translation.z;

	_t_rot(0) = this->top_to_drone_tf.transform.rotation.x;
	_t_rot(1) = this->top_to_drone_tf.transform.rotation.y;
	_t_rot(2) = this->top_to_drone_tf.transform.rotation.z;
	_t_rot(3) = this->top_to_drone_tf.transform.rotation.w;

	radar_to_drone = getTransformMatrix(_t_xyz, _t_rot);


	// transform points in pointcloud

	this->top_cloud->clear();

	RadarPCLFilter::read_pointcloud(msg, top_cloud);

	pcl::transformPointCloud (*top_cloud, *top_cloud, radar_to_drone);
}


void RadarPCLFilter::add_bot_radar_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

	if (msg->width < 1)
	{
		return;
	}

	homog_transform_t radar_to_drone;

	vector_t _t_xyz;
	quat_t _t_rot;

	// make transform front->drone
	_t_xyz(0) = this->bot_to_drone_tf.transform.translation.x;
	_t_xyz(1) = this->bot_to_drone_tf.transform.translation.y;
	_t_xyz(2) = this->bot_to_drone_tf.transform.translation.z;

	_t_rot(0) = this->bot_to_drone_tf.transform.rotation.x;
	_t_rot(1) = this->bot_to_drone_tf.transform.rotation.y;
	_t_rot(2) = this->bot_to_drone_tf.transform.rotation.z;
	_t_rot(3) = this->bot_to_drone_tf.transform.rotation.w;

	radar_to_drone = getTransformMatrix(_t_xyz, _t_rot);


	// transform points in pointcloud

	this->bot_cloud->clear();

	RadarPCLFilter::read_pointcloud(msg, bot_cloud);

	pcl::transformPointCloud (*bot_cloud, *bot_cloud, radar_to_drone);
}


void RadarPCLFilter::add_right_radar_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

	if (msg->width < 1)
	{
		return;
	}

	homog_transform_t radar_to_drone;

	vector_t _t_xyz;
	quat_t _t_rot;

	// make transform front->drone
	_t_xyz(0) = this->right_to_drone_tf.transform.translation.x;
	_t_xyz(1) = this->right_to_drone_tf.transform.translation.y;
	_t_xyz(2) = this->right_to_drone_tf.transform.translation.z;

	_t_rot(0) = this->right_to_drone_tf.transform.rotation.x;
	_t_rot(1) = this->right_to_drone_tf.transform.rotation.y;
	_t_rot(2) = this->right_to_drone_tf.transform.rotation.z;
	_t_rot(3) = this->right_to_drone_tf.transform.rotation.w;

	radar_to_drone = getTransformMatrix(_t_xyz, _t_rot);


	// transform points in pointcloud

	this->right_cloud->clear();

	RadarPCLFilter::read_pointcloud(msg, right_cloud);

	pcl::transformPointCloud (*right_cloud, *right_cloud, radar_to_drone);
}


void RadarPCLFilter::add_left_radar_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

	if (msg->width < 1)
	{
		return;
	}

	homog_transform_t radar_to_drone;

	vector_t _t_xyz;
	quat_t _t_rot;

	// make transform front->drone
	_t_xyz(0) = this->left_to_drone_tf.transform.translation.x;
	_t_xyz(1) = this->left_to_drone_tf.transform.translation.y;
	_t_xyz(2) = this->left_to_drone_tf.transform.translation.z;

	_t_rot(0) = this->left_to_drone_tf.transform.rotation.x;
	_t_rot(1) = this->left_to_drone_tf.transform.rotation.y;
	_t_rot(2) = this->left_to_drone_tf.transform.rotation.z;
	_t_rot(3) = this->left_to_drone_tf.transform.rotation.w;

	radar_to_drone = getTransformMatrix(_t_xyz, _t_rot);


	// transform points in pointcloud

	this->left_cloud->clear();

	RadarPCLFilter::read_pointcloud(msg, left_cloud);

	pcl::transformPointCloud (*left_cloud, *left_cloud, radar_to_drone);
}


void RadarPCLFilter::create_pointcloud_msg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, sensor_msgs::msg::PointCloud2 * pcl_msg, std::string frame_id) {

  // create PointCloud2 msg
	//https://github.com/ros-drivers/velodyne/blob/master/velodyne_laserscan/tests/system.cpp
	int pcl_size = cloud->size();
	auto pcl2_msg = sensor_msgs::msg::PointCloud2();
	const uint32_t POINT_STEP = 12;

	pcl_msg->header = std_msgs::msg::Header();
	pcl_msg->header.stamp = this->now();
	pcl_msg->header.frame_id = frame_id;
	pcl_msg->fields.resize(3);
	pcl_msg->fields[0].name = 'x';
	pcl_msg->fields[0].offset = 0;
	pcl_msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
	pcl_msg->fields[0].count = 1;
	pcl_msg->fields[1].name = 'y';
	pcl_msg->fields[1].offset = 4;
	pcl_msg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
	pcl_msg->fields[1].count = 1;
	pcl_msg->fields[2].name = 'z';
	pcl_msg->fields[2].offset = 8;
	pcl_msg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
	pcl_msg->fields[2].count = 1;

	if(pcl_size > 0){
		pcl_msg->data.resize(std::max((size_t)1, (size_t)pcl_size) * POINT_STEP, 0x00);
	} else {
        return;
    }

	pcl_msg->point_step = POINT_STEP; // size (bytes) of 1 point (float32 * dimensions (3 when xyz))
	pcl_msg->row_step = pcl_msg->data.size();//pcl_msg->point_step * pcl_msg->width; // only 1 row because unordered
	pcl_msg->height = 1;  // because unordered cloud
	pcl_msg->width = pcl_msg->row_step / POINT_STEP; // number of points in cloud
	pcl_msg->is_dense = false; // there may be invalid points

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


	
			
int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RadarPCLFilter>());

	rclcpp::shutdown();
	return 0;
}
