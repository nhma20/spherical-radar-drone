// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

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
#include <pcl/kdtree/kdtree_flann.h>


#define DEG_PER_RAD 57.2957795
#define RAD_PER_DEG 0.01745329
#define PI 3.14159265

using namespace std::chrono_literals;

//creates a RadarPCLFilter class that subclasses the generic rclcpp::Node base class.
class RadarPCLFilter : public rclcpp::Node
{
	public:
		RadarPCLFilter() : Node("radar_pcl_combiner_node") {

			_combined_pcl_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			"/combined_pcl", 10,
			std::bind(&RadarPCLFilter::get_angle, this, std::placeholders::_1));

			test_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/test_pcl", 10);

			_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/yaw_pose", 10);

			tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
			transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

		
			while(true) {

				static int _t_tries = 0;

				try {

					drone_to_drone_yaw_tf = tf_buffer_->lookupTransform("drone", "drone_yaw_only", tf2::TimePointZero);
					tf_buffer_->canTransform("world", "drone_yaw_only", tf2::TimePointZero);

					RCLCPP_INFO(this->get_logger(), "Found all transforms");
					break;


				} catch (const tf2::TransformException & ex) {

					RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
					
					_t_tries++;

					std::this_thread::sleep_for(std::chrono::milliseconds(500));

					if( _t_tries > 10) {
						RCLCPP_FATAL(this->get_logger(), "Failed to get all transforms after 10 tries.");
						throw std::exception();
					}
				}

			}

		}

		~RadarPCLFilter() {
			RCLCPP_INFO(this->get_logger(),  "Shutting down radar_pointcloud_combiner node..");
		}

	private:
		std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  		std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

		rclcpp::TimerBase::SharedPtr _timer_pcl;
		rclcpp::TimerBase::SharedPtr _timer_clear_pcl;

		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _combined_pcl_sub;

		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr test_pointcloud_pub;

		rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pose_pub;

		void get_angle(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

		void update_drone_pose();
		homog_transform_t get_radar_to_drone_tf(geometry_msgs::msg::TransformStamped tf);
		homog_transform_t get_world_to_frame_tf_at_time(std::string frame_id, rclcpp::Time timestamp);
		void publish_combined_pointcloud();

		void create_pointcloud_msg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, sensor_msgs::msg::PointCloud2 * pcl_msg);

		void read_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, 
										pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);


		pcl::PointCloud<pcl::PointXYZ>::Ptr _combined_points; // in drone_yaw_only frame

		geometry_msgs::msg::TransformStamped front_to_drone_tf;
		geometry_msgs::msg::TransformStamped rear_to_drone_tf;
		geometry_msgs::msg::TransformStamped top_to_drone_tf;
		geometry_msgs::msg::TransformStamped bot_to_drone_tf;
		geometry_msgs::msg::TransformStamped right_to_drone_tf;
		geometry_msgs::msg::TransformStamped left_to_drone_tf;
		geometry_msgs::msg::TransformStamped drone_to_drone_yaw_tf;

		geometry_msgs::msg::Quaternion _yaw_quat;

		sensor_msgs::msg::PointCloud2 _combined_pcl_msg; 

		float _drone_yaw;

		bool _first = true;
		float _zero_yaw = 0.123;
		float _current_yaw;

};


void RadarPCLFilter::get_angle(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

	update_drone_pose();

	if (_first==true)
	{
		_first = false;
		
		// latch drone yaw and keep as 0
		_zero_yaw = _drone_yaw;
	}
	
	// get current yaw
	_current_yaw = _drone_yaw;

	// subtract yaws
	float normalized_yaw = _zero_yaw - _current_yaw;

	RCLCPP_INFO(this->get_logger(), "\n");
	RCLCPP_INFO(this->get_logger(), "Zero yaw: %f", _zero_yaw);
	RCLCPP_INFO(this->get_logger(), "Current yaw: %f", _current_yaw);
	RCLCPP_INFO(this->get_logger(), "Normalized yaw: %f", normalized_yaw);
	
	// publish yaw pose
	auto pose_msg = geometry_msgs::msg::PoseStamped(); 
	pose_msg.header = std_msgs::msg::Header();
	const std::string frame_id = "world"; //drone_yaw_only
	pose_msg.header.frame_id = frame_id;
	pose_msg.header.stamp = this->now();
	pose_msg.pose.orientation.x = _yaw_quat.x;
	pose_msg.pose.orientation.y = _yaw_quat.y;
	pose_msg.pose.orientation.z = _yaw_quat.z;
	pose_msg.pose.orientation.w = _yaw_quat.w;
	pose_msg.pose.position.x = 0.0;
	pose_msg.pose.position.y = 0.0;
	pose_msg.pose.position.z = 0.0;

	_pose_pub->publish(pose_msg);




	// read pcl msg
	pcl::PointCloud<pcl::PointXYZ>::Ptr incoming_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	RadarPCLFilter::read_pointcloud(msg, incoming_cloud); // in drone_yaw_only frame

	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);

	static auto prev_pt = pcl::PointXYZ{0.0f, 0.0f, 0.0f};

	std::vector<int> idx;
	std::vector<float> mag;
	std::vector<float> dist_to_prev;

	// remove all points below and above 1m and horizontally closer than 1m and further than 3m
	for (size_t i = 0; i < (size_t)incoming_cloud->size(); i++)
	{
		const auto& pt = incoming_cloud->points[i];

		// above and below
		if (pt.z < 1.0 && pt.z > -1.0)
		{
			// closer and further
			float hypot = std::hypot(pt.x, pt.y); //sqrt( (pt.x*pt.x) + (pt.y*pt.y) ) ;
			float dist_to_prev_pt = std::hypot((pt.x-prev_pt.x), (pt.y-prev_pt.y)); //sqrt( ((pt.x-prev_pt.x)*(pt.x-prev_pt.x)) + ((pt.y-prev_pt.y)*(pt.y-prev_pt.y)) ) ;
			if ( hypot < 3.5 && hypot > 1.5 )
			{
				idx.push_back(i);
				mag.push_back(hypot);
				dist_to_prev.push_back(dist_to_prev_pt);
				// filtered_cloud->push_back(pt);
				// RCLCPP_INFO(this->get_logger(), "Found point!");
			}
		}	
	}	

	int smallest_idx;
	float smallest_hypot = 9999.9;
	float smallest_dist_to_prev = 999.9;
	float smallest_dist_idx;

	if (!idx.empty())
	{
		
		for (size_t i = 0; i < idx.size(); i++)
		{
			if (mag.at(i) < smallest_hypot)
			{
				smallest_hypot = mag.at(i);
				smallest_idx = idx.at(i);
			}

			if (dist_to_prev.at(i) < smallest_dist_to_prev)
			{
				smallest_dist_to_prev = dist_to_prev.at(i);
				smallest_dist_idx = idx.at(i);
			}
			
		}
		
		// filtered_cloud->push_back(incoming_cloud->points[smallest_idx]);
		filtered_cloud->push_back(incoming_cloud->points[smallest_dist_idx]);
		prev_pt = filtered_cloud->points[0];
	}




	sensor_msgs::msg::PointCloud2 filtered_pcl_msg = sensor_msgs::msg::PointCloud2();
	filtered_pcl_msg.header = std_msgs::msg::Header();
	const std::string pcl_frame_id = "world"; //drone_yaw_only
	filtered_pcl_msg.header.frame_id = pcl_frame_id;
	filtered_pcl_msg.fields.resize(3);
	filtered_pcl_msg.fields[0].name = 'x';
	filtered_pcl_msg.fields[0].offset = 0;
	filtered_pcl_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
	filtered_pcl_msg.fields[0].count = 1;
	filtered_pcl_msg.fields[1].name = 'y';
	filtered_pcl_msg.fields[1].offset = 4;
	filtered_pcl_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
	filtered_pcl_msg.fields[1].count = 1;
	filtered_pcl_msg.fields[2].name = 'z';
	filtered_pcl_msg.fields[2].offset = 8;
	filtered_pcl_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
	filtered_pcl_msg.fields[2].count = 1;
	const uint32_t POINT_STEP = 12;
	filtered_pcl_msg.point_step = POINT_STEP; // size (bytes) of 1 point (float32 * dimensions (3 when xyz))
	filtered_pcl_msg.height = 1;  // because unordered cloud
	filtered_pcl_msg.is_dense = false; // there may be invalid points

	RadarPCLFilter::create_pointcloud_msg(filtered_cloud, &filtered_pcl_msg);

	// if (filtered_pcl_msg.width < 1)
	// {
	// 	return;
	// }
	
	test_pointcloud_pub->publish(filtered_pcl_msg); 

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
		pcl_msg->width = 0;
		pcl_msg->data.clear();
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


void RadarPCLFilter::update_drone_pose() {

	geometry_msgs::msg::TransformStamped t;

	try {
		if (tf_buffer_->canTransform("world", "drone_yaw_only", tf2::TimePointZero))	{
			t = tf_buffer_->lookupTransform("world","drone_yaw_only",tf2::TimePointZero);
		}
		else {
			RCLCPP_WARN(this->get_logger(), "Can not transform world->drone_yaw_only");
			return;
		}
	} catch (const tf2::TransformException & ex) {
		RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
		return;
	}

	// Extract quaternion
    const auto & q = t.transform.rotation;
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);

	_yaw_quat = q;

    // Convert to roll/pitch/yaw
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

	_drone_yaw = (float)yaw;
}

	
			
int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RadarPCLFilter>());

	rclcpp::shutdown();
	return 0;
}
