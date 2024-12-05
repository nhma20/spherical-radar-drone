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

        std::ostringstream stream;

        subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/vehicle_odometry/out", 10,
            std::bind(&DroneFrameBroadcasterNode::odometryCallback, this, std::placeholders::_1));

        // Roll PI to get odo location in world frame
        R_NED_to_body_frame = eulToR(orientation_t(M_PI, 0, 0));
    }

private:
    void odometryCallback(const std::shared_ptr<px4_msgs::msg::VehicleOdometry> msg) {

        rclcpp::Time now = this->get_clock()->now();
        geometry_msgs::msg::TransformStamped t;

        // Read message content and assign it to
        // corresponding tf variables
        t.header.stamp = now;
        t.header.frame_id = world_frame_id_;
        t.child_frame_id = drone_frame_id_;

        point_t position(
            msg->x,
            msg->y, 
            msg->z
        );

        position = R_NED_to_body_frame * position;


        quat_t quat( // msg format is wxyz, so change to xyzw
            msg->q[3],
            msg->q[0],
            msg->q[1],
            msg->q[2]
        );

        quat_t RollYaw_PI_quat (
            0.0,
            -1.0,
            0.0,
            0.0
        );

        // Roll PI and Yaw PI to orient odo frame with world frame
        quat = quatMultiply(quat, RollYaw_PI_quat);

        t.transform.translation.x = position(0);
        t.transform.translation.y = position(1);
        t.transform.translation.z = position(2);
  
        t.transform.rotation.x = quat(0);
        t.transform.rotation.y = quat(1);
        t.transform.rotation.z = quat(2);
        t.transform.rotation.w = quat(3);

        // Send the transformation
        tf_broadcaster_->sendTransform(t);







        //////// world to drone (no roll+pitch) transform
        geometry_msgs::msg::TransformStamped t_yaw;

        // Read message content and assign it to
        // corresponding tf variables
        t_yaw.header.stamp = now;
        t_yaw.header.frame_id = world_frame_id_;
        t_yaw.child_frame_id = drone_yaw_frame_id_;

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

        t_yaw.transform.translation.x = position(0);
        t_yaw.transform.translation.y = position(1);
        t_yaw.transform.translation.z = position(2);
  
        t_yaw.transform.rotation.x = quat(0);
        t_yaw.transform.rotation.y = quat(1);
        t_yaw.transform.rotation.z = quat(2);
        t_yaw.transform.rotation.w = quat(3);

        no_yaw_tf_broadcaster_->sendTransform(t_yaw);

    }

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> no_yaw_tf_broadcaster_;

    rotation_matrix_t R_NED_to_body_frame;

    std::string drone_yaw_frame_id_;
    std::string drone_frame_id_;
    std::string world_frame_id_;

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
