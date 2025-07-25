#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <toggle_radar_msgs/msg/toggle_radar.hpp>

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

//creates a LidarToMmwave class that subclasses the generic rclcpp::Node base class.
class LidarToMmwave : public rclcpp::Node
{

//Creates a function for when messages are to be sent. 
//Messages are sent based on a timed callback.
	public:
		LidarToMmwave() : Node("lidar_to_mmwave_converter") {
			this->declare_parameter<std::string>("input_topic", "/dist_sensor/laser_scan");
			this->get_parameter("input_topic", _input_topic);

			this->declare_parameter<std::string>("output_topic", "/iwr6843_pcl");
			this->get_parameter("output_topic", _output_topic);

			this->declare_parameter<std::string>("frame_id", "iwr6843_frame");
			this->get_parameter("frame_id", _frame_id);

			if (_frame_id == "front_frame")
				_frame_idx = 0;
			else if (_frame_id == "rear_frame")
				_frame_idx = 1;
			else if (_frame_id == "top_frame")
				_frame_idx = 2;
			else if (_frame_id == "bot_frame")
				_frame_idx = 3;
			else if (_frame_id == "right_frame")
				_frame_idx = 4;
			else if (_frame_id == "left_frame")
				_frame_idx = 5;
			else
				RCLCPP_WARN(this->get_logger(),  "%s matches no frame to toggle", _frame_id);

			this->declare_parameter<float>("min_dist", 0.3);
			this->get_parameter("min_dist", _min_dist);

			this->declare_parameter<float>("max_dist", 15.0);
			this->get_parameter("max_dist", _max_dist);

			lidar_to_mmwave_pcl_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(_output_topic, 10);

			subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
			_input_topic,	10,
			std::bind(&LidarToMmwave::lidar_to_mmwave_pcl, this, std::placeholders::_1));

			toggle_subscription_ = this->create_subscription<toggle_radar_msgs::msg::ToggleRadar>(
			"/radar_toggle",	10,
			std::bind(&LidarToMmwave::toggle_callback, this, std::placeholders::_1));
		}

		~LidarToMmwave() {
			RCLCPP_INFO(this->get_logger(),  "Shutting down lidar_to_mmwave_converter..");
		}


	private:
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_to_mmwave_pcl_publisher_;
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
		rclcpp::Subscription<toggle_radar_msgs::msg::ToggleRadar>::SharedPtr toggle_subscription_;

		std::vector<float> objects_dist;
		std::vector<float> objects_angl;
		void lidar_to_mmwave_pcl(const sensor_msgs::msg::LaserScan::SharedPtr _msg);
		void toggle_callback(const toggle_radar_msgs::msg::ToggleRadar::SharedPtr _msg);

		std::string _input_topic;
		std::string _output_topic;
		std::string _frame_id;
		int _frame_idx;
		bool _enable_data = true; // on by default
		float _min_dist;
		float _max_dist;

		rclcpp::Time last_pub_{0, 0, RCL_ROS_TIME};
    	const rclcpp::Duration min_period_{rclcpp::Duration::from_seconds(0.1)};  // 10 Hz

};


void LidarToMmwave::toggle_callback(const toggle_radar_msgs::msg::ToggleRadar::SharedPtr _msg){

	if(  _msg->radar_toggle_array[_frame_idx] == true )
		_enable_data = true;

	if( _msg->radar_toggle_array[_frame_idx] == false )
		_enable_data = false;

}



// converts lidar data to pointcloud of detected objects to simulate sparse mmwave
void LidarToMmwave::lidar_to_mmwave_pcl(const sensor_msgs::msg::LaserScan::SharedPtr _msg){

	if( _enable_data == false )
		return;

	rclcpp::Time now = this->get_clock()->now();

	// if (now - last_pub_ < min_period_) { // only update transforms at 1/min_period_ Hz
	// 	return;
	// }
	// last_pub_ = now;

	auto pcl2_msg = sensor_msgs::msg::PointCloud2();
	pcl2_msg.header = std_msgs::msg::Header();

	rclcpp::Duration offset(0, 10 * 1000 * 1000); // artificial 10 ms delay in timestamp (sec = 0, nanosec = 10 000 000)

	pcl2_msg.header.stamp = now - offset;

	// RCLCPP_INFO(this->get_logger(),  "Got laser scan");

	float angle_increment = _msg->angle_increment;
	float angle_min = _msg->angle_min;
	float angle_max = _msg->angle_max;
	float range_max = _msg->range_max;
	float range_min = _msg->range_min;
	float total_angle = angle_max - angle_min;
	int num_of_rays = round(total_angle / angle_increment); // 1000
	
	// get shortest dist index
	int shortestDistIdx = 0;
	for(int i = 0; i < num_of_rays; i++){
		if(_msg->ranges[shortestDistIdx] > _msg->ranges[i]){
			shortestDistIdx = i;
		}
	}

	// group objects in lidar fov to simulate mmwave readings
	std::vector<float> object_center_angls;
	std::vector<float> object_center_dists;
	int grouped_previous = 0;
	float group_dist = 0;
	float group_angl = 0;
	for(int i = 0; i < num_of_rays-1; i++){
		if(_msg->ranges[i] > range_min && _msg->ranges[i] < range_max){
			//std::cout << "Object detected, range: " << _msg->ranges[i] << std::endl;
			// object_center_dists.push_back( _msg->ranges[i] );
			// object_center_angls.push_back( float(i)*angle_increment - angle_max );
			// continue; ////////////////////////////////////////////////// "PERFECT DATA" FOR TESTING, REMOVE ////////////////////////////////////////////////////////////////
			if(grouped_previous == 0){	
				//std::cout << "First beam of object" << std::endl;
				group_dist += _msg->ranges[i];
				group_angl += float(i)*angle_increment - angle_max;
			}
			// Group object if current and next ray almost same distance
			if( abs(_msg->ranges[i+1] - _msg->ranges[i]) < 0.1 ){
				//std::cout << "Object more than one beam wide" << std::endl;
				group_dist += _msg->ranges[i+1];
				group_angl += float(i+1)*angle_increment - angle_max;
				grouped_previous++;
			}
			else{
				// add random dropout of points (~95% detection rate)
				float dist = group_dist/(grouped_previous+1);
				if ( ((rand() % 100) + 1) < 95 && dist > _min_dist && dist < _max_dist){
					//std::cout << "End of object detected, pushing" << std::endl;
					object_center_dists.push_back( dist );
					object_center_angls.push_back( group_angl/(grouped_previous+1) );
				}
				grouped_previous = 0;
				group_dist = 0;
				group_angl = 0;
			}
		}
	}
	this->objects_angl = object_center_angls;
	this->objects_dist = object_center_dists;
	
	// angle compared to straight up from drone
	// float shortestDistIdxAngle = float(shortestDistIdx)*angle_increment - angle_max; 


	// convert from "spherical" to cartesian
	std::vector<float> pcl_x;
	std::vector<float> pcl_y;
	std::vector<float> pcl_z;

	// set random seed once
	static bool seeded = false;
	if (seeded == false)
	{
		srand (1);
	}
	seeded = true;
	// generate noise
	float amplitude = 0.025; //////////////////////////////////// REDUCE FOR TESTING, MAKE 0.025 /////////////////////////////////////////////////////
	float noise;
	// convert to xyz (including noise)
	for(size_t i = 0; i<objects_dist.size(); i++){
		noise = -amplitude + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(amplitude-(-amplitude))));
		pcl_x.push_back( sin(object_center_angls.at(i)) * object_center_dists.at(i) + noise*object_center_dists.at(i));
		// noise = -amplitude + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(amplitude-(-amplitude))));
		// pcl_y.push_back( sin(			0			) * object_center_dists.at(i) 	+ noise*object_center_dists.at(i));
		// noise = -amplitude + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(amplitude-(-amplitude))));
		// pcl_z.push_back( cos(object_center_angls.at(i)) * object_center_dists.at(i) + noise*object_center_dists.at(i));
		noise = -amplitude + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(amplitude-(-amplitude))));
		pcl_z.push_back( sin(			0			) * object_center_dists.at(i) 	+ noise*object_center_dists.at(i));
		noise = -amplitude + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(amplitude-(-amplitude))));
		pcl_y.push_back( cos(object_center_angls.at(i)) * object_center_dists.at(i) + noise*object_center_dists.at(i));
	}

	// create PointCloud2 msg
	//https://github.com/ros-drivers/velodyne/blob/master/velodyne_laserscan/tests/system.cpp
	
	pcl2_msg.header.frame_id = _frame_id;
	pcl2_msg.fields.resize(3);
	pcl2_msg.fields[0].name = 'x';
	pcl2_msg.fields[0].offset = 0;
	pcl2_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
	pcl2_msg.fields[0].count = 1;
	pcl2_msg.fields[1].name = 'y';
	pcl2_msg.fields[1].offset = 4;
	pcl2_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
	pcl2_msg.fields[1].count = 1;
	pcl2_msg.fields[2].name = 'z';
	pcl2_msg.fields[2].offset = 8;
	pcl2_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
	pcl2_msg.fields[2].count = 1;
	const uint32_t POINT_STEP = 12;
	if(objects_dist.size() > 0){
		pcl2_msg.data.resize(std::max((size_t)1, objects_dist.size()) * POINT_STEP, 0x00);
	}
	pcl2_msg.point_step = POINT_STEP; // size (bytes) of 1 point (float32 * dimensions (3 when xyz))
	pcl2_msg.row_step = pcl2_msg.data.size();//pcl2_msg.point_step * pcl2_msg.width; // only 1 row because unordered
	pcl2_msg.height = 1;  // because unordered cloud
	pcl2_msg.width = pcl2_msg.row_step / POINT_STEP; // number of points in cloud
	pcl2_msg.is_dense = false; // there may be invalid points

	// fill PointCloud2 msg data
	if(objects_dist.size() > 0){
		uint8_t *ptr = pcl2_msg.data.data();
		for (size_t i = 0; i < objects_dist.size(); i++)
		{
			*(reinterpret_cast<float*>(ptr + 0)) = pcl_x.at(i);
			*(reinterpret_cast<float*>(ptr + 4)) = pcl_y.at(i);
			*(reinterpret_cast<float*>(ptr + 8)) = pcl_z.at(i);
			ptr += POINT_STEP;
		}
	}
	// publish PointCloud2 msg
	// RCLCPP_INFO(this->get_logger(),  "Publishing radar pcl");
	this->lidar_to_mmwave_pcl_publisher_->publish(pcl2_msg);
}


	
			
int main(int argc, char *argv[])
{
	std::cout << "Starting lidar_to_mmwave_converter node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LidarToMmwave>());

	rclcpp::shutdown();
	return 0;
}
