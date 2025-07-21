/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <memory>
#include <string>

// #include <eigen3/Eigen/Core>
// #include <eigen3/Eigen/Geometry>
#include "geometry.h"


/*****************************************************************************/
// Defines
/*****************************************************************************/
typedef Eigen::Vector3f orientation_t;

typedef Eigen::Vector4f quat_t;

typedef Eigen::Matrix3f rotation_matrix_t;

typedef Eigen::Vector3f point_t;
/*****************************************************************************/
// Class
/*****************************************************************************/

class DroneFrameBroadcasterNode : public rclcpp::Node {
public:
explicit
    DroneFrameBroadcasterNode(const std::string & node_name="drone_frame_broadcaster", const std::string & node_namespace="/drone_frame_broadcaster")
            : Node(node_name, node_namespace) {

        // Params
        this->declare_parameter<std::string>("drone_yaw_frame_id", "drone_yaw_only");
        this->declare_parameter<std::string>("drone_frame_id", "drone");
        this->declare_parameter<std::string>("world_frame_id", "world");

        this->get_parameter("drone_yaw_frame_id", drone_yaw_frame_id_);
        this->get_parameter("drone_frame_id", drone_frame_id_);
        this->get_parameter("world_frame_id", world_frame_id_);


        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        no_yaw_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        _drone_velocity_frame_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        std::ostringstream stream;

        subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/vehicle_odometry/out", 10,
            std::bind(&DroneFrameBroadcasterNode::odometryCallback, this, std::placeholders::_1));

        // Roll PI to get odo location in world frame
        R_NED_to_body_frame = eulToR(orientation_t(M_PI, 0, 0));

        _t_yaw.header.frame_id = world_frame_id_;
        _t_yaw.child_frame_id = drone_yaw_frame_id_;

        _t.header.frame_id = world_frame_id_;
        _t.child_frame_id = drone_frame_id_;

        _t_vel.header.frame_id = world_frame_id_;
        _t_vel.child_frame_id = "drone_velocity";

    }

private:
    void odometryCallback(const std::shared_ptr<px4_msgs::msg::VehicleOdometry> msg) {

        rclcpp::Time now = this->get_clock()->now();

        // if (now - last_pub_ < min_period_) { // only update transforms at 1/min_period_ Hz
        //     return;
        // }
        // last_pub_ = now;
        
        // corresponding tf variables
        _t.header.stamp = now;


        point_t position(
            msg->x,
            msg->y, 
            msg->z
        );

        position = R_NED_to_body_frame * position;


        quat_t quat( // msg format is wxyz, so change to xyzw (THIS MUST BE WRONG?)
            msg->q[3],
            msg->q[0],
            msg->q[1],
            msg->q[2]
        );

        

        // Roll PI and Yaw PI to orient odo frame with world frame
        quat = quatMultiply(quat, _RollYaw_PI_quat);

        _t.transform.translation.x = position(0);
        _t.transform.translation.y = position(1);
        _t.transform.translation.z = position(2);
  
        _t.transform.rotation.x = quat(0); // <- how can this ever be x ? according to quat declaration above it must be either z or w ?
        _t.transform.rotation.y = quat(1);
        _t.transform.rotation.z = quat(2);
        _t.transform.rotation.w = quat(3);

        // Send the transformation
        tf_broadcaster_->sendTransform(_t);







        //////// world to drone (no roll+pitch) transform

        // corresponding tf variables
        _t_yaw.header.stamp = now;
        
        float drone_yaw = quaternionToYaw(quat);

        // RCLCPP_INFO(this->get_logger(),  "Yaw: %f", drone_yaw);

        orientation_t eul_yaw_only(
            0.0,
            0.0,
            drone_yaw
        );

        quat_t yaw_quat = eulToQuat(eul_yaw_only);

        // Roll PI and Yaw PI to orient odo frame with world frame
        quat = yaw_quat;//quatMultiply(yaw_quat, RollYaw_PI_quat);

        _t_yaw.transform.translation.x = position(0);
        _t_yaw.transform.translation.y = position(1);
        _t_yaw.transform.translation.z = position(2);
  
        _t_yaw.transform.rotation.x = quat(0);
        _t_yaw.transform.rotation.y = quat(1);
        _t_yaw.transform.rotation.z = quat(2);
        _t_yaw.transform.rotation.w = quat(3);

        no_yaw_tf_broadcaster_->sendTransform(_t_yaw);







        // drone velocity frame
        vector_t drone_velocity(
            msg->vx,
            -msg->vy,
            -msg->vz
        );



        quat_t world_to_velocity_quat = findRotation(_unit_x_vector, drone_velocity);

        // corresponding tf variables
        _t_vel.header.stamp = now;

        _t_vel.transform.translation.x = position(0);
        _t_vel.transform.translation.y = position(1);
        _t_vel.transform.translation.z = position(2);

        _t_vel.transform.rotation.x = world_to_velocity_quat(0);
        _t_vel.transform.rotation.y = world_to_velocity_quat(1);
        _t_vel.transform.rotation.z = world_to_velocity_quat(2);
        _t_vel.transform.rotation.w = world_to_velocity_quat(3);

        // Send the transformation
        _drone_velocity_frame_tf_broadcaster->sendTransform(_t_vel);

    }

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> no_yaw_tf_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> _drone_velocity_frame_tf_broadcaster;

    rotation_matrix_t R_NED_to_body_frame;

    std::string drone_yaw_frame_id_;
    std::string drone_frame_id_;
    std::string world_frame_id_;

    geometry_msgs::msg::TransformStamped _t_yaw;
    geometry_msgs::msg::TransformStamped _t;  
    geometry_msgs::msg::TransformStamped _t_vel;

    rclcpp::Time last_pub_{0, 0, RCL_ROS_TIME};
    const rclcpp::Duration min_period_{rclcpp::Duration::from_seconds(0.02)};  // 50 Hz

    quat_t const _RollYaw_PI_quat{ 0.0f, -1.0f, 0.0f, 0.0f };
    vector_t const _unit_x_vector{1.0f, 0.0f, 0.0f};


};



/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneFrameBroadcasterNode>());
    rclcpp::shutdown();
    return 0;
}
