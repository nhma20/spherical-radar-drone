/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "geometry.h"
#include <iostream>

/*****************************************************************************/
// Function implementations
/*****************************************************************************/


vector_t projectVectorOnVector(vector_t v1, vector_t v2) {

    vector_t out = ( (v1.dot(v2)) / v2.dot(v2) ) * v2 ;

    return out;

}


rotation_matrix_t eulToR(orientation_t eul) {

    Eigen::Quaternionf q; 

    q = Eigen::AngleAxisf(eul[0], Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(eul[1], Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(eul[2], Eigen::Vector3f::UnitZ());

    Eigen::Matrix3f rotationMatrix = q.normalized().toRotationMatrix();

    return rotationMatrix;

}

vector_t rotateVector(rotation_matrix_t R, vector_t v) {

    vector_t ret_vec = R*v;

    return ret_vec;
}


float quaternionToYaw(const quat_t quat) {

    // Eigen::Quaternionf quaternion;
    // quaternion.x() = quat(0);
    // quaternion.y() = quat(1);
    // quaternion.z() = quat(2);
    // quaternion.w() = quat(3);

    // // Normalize the quaternion to ensure it's a unit quaternion
    // Eigen::Quaternionf normalized_quaternion = quaternion.normalized();

    // // Convert the quaternion to Euler angles
    // Eigen::Vector3f euler_angles = normalized_quaternion.toRotationMatrix().eulerAngles(2, 1, 0);

    // // The yaw angle is the first element in the eulerAngles vector (rotation around the z-axis)
    // float yaw = euler_angles[0];

    return quatToEul(quat)(2); //yaw;
}


vector_t quaternionToDirection(const quat_t quaternion) {

    Eigen::Quaternionf eq;
    eq.x() = quaternion(0);
    eq.y() = quaternion(1);
    eq.z() = quaternion(2);
    eq.w() = quaternion(3);

    vector_t unit_x(
        1,
        0,
        0
    );

    rotation_matrix_t rot_mat = quatToMat(quaternion);

    vector_t direction = rotateVector(rot_mat, unit_x);

    // // Normalize the quaternion to ensure it's a unit quaternion
    // Eigen::Quaternionf normalized_quaternion = eq.normalized();

    // // Apply the quaternion to the unit vector along the x-axis (1, 0, 0)
    // Eigen::Vector3f direction = normalized_quaternion * Eigen::Vector3f::UnitX();
    return direction;
}


quat_t findRotation(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2) {
    // Normalize the input vectors
    Eigen::Vector3f v1_normalized = v1.normalized();
    Eigen::Vector3f v2_normalized = v2.normalized();

    // Calculate the cross product and dot product
    Eigen::Vector3f cross_product = v1_normalized.cross(v2_normalized);
    float dot_product = v1_normalized.dot(v2_normalized);

    // Calculate the quaternion components
    float w = std::sqrt((v1_normalized.squaredNorm() * v2_normalized.squaredNorm())) + dot_product;
    Eigen::Quaternionf quaternion(w, cross_product.x(), cross_product.y(), cross_product.z());

    // Normalize the quaternion
    quaternion.normalize();

    quat_t quat;
    quat(0) = quaternion.x();
    quat(1) = quaternion.y();
    quat(2) = quaternion.z();
    quat(3) = quaternion.w();

    return quat;
}

point_t projectPointOnPlane(point_t point, plane_t plane) {
//     line_t l = {
//         .p = point,
//         .v = plane.normal
//     };

//     float t = - plane.normal.dot(l.p) / plane.normal.dot(plane.normal);

//     point_t proj_point = l.p + (point_t)(t*l.v);

//     return proj_point;

    vector_t diff = point - plane.p;

    float dist = diff.dot(plane.normal);

    point_t proj_point = point - (point_t)(dist*plane.normal);

    return proj_point;
}

orientation_t quatToEul(quat_t quat) {

    // Eigen::Quaternionf q;
    // q.x() = quat[0];
    // q.y() = quat[1];
    // q.z() = quat[2];
    // q.w() = quat[3];

    // orientation_t eul  = q.toRotationMatrix().eulerAngles(0, 1, 2);

    // thanks wikipedia
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_angles_conversion
    orientation_t eul;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (quat[3] * quat[0] + quat[1] * quat[2]);
    double cosr_cosp = 1 - 2 * (quat[0] * quat[0] + quat[1] * quat[1]);
    eul(0) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (quat[3] * quat[1] - quat[2] * quat[0]);
    if (std::abs(sinp) >= 1)
        eul(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        eul(1) = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (quat[3] * quat[2] + quat[0] * quat[1]);
    double cosy_cosp = 1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2]);
    eul(2) = std::atan2(siny_cosp, cosy_cosp);


    return eul;

}

quat_t quatInv(quat_t quat) {

    quat_t ret_quat(quat[0], -quat[1], -quat[2], -quat[3]);

    return ret_quat;

}

quat_t quatMultiply(quat_t quat1, quat_t quat2) {

    quat_t ret_quat(
        // quat1[0]*quat2[0] - quat1[1]*quat2[1] - quat1[2]*quat2[2] - quat1[3]*quat2[3],
        // quat1[0]*quat2[1] + quat1[1]*quat2[0] + quat1[2]*quat2[3] - quat1[3]*quat2[2],
        // quat1[0]*quat2[2] - quat1[1]*quat2[3] + quat1[2]*quat2[0] + quat1[3]*quat2[1],
        // quat1[0]*quat2[3] + quat1[1]*quat2[2] - quat1[2]*quat2[1] + quat1[3]*quat2[0]
         quat1[0] * quat2[3] + quat1[1] * quat2[2] - quat1[2] * quat2[1] + quat1[3] * quat2[0],
        -quat1[0] * quat2[2] + quat1[1] * quat2[3] + quat1[2] * quat2[0] + quat1[3] * quat2[1],
         quat1[0] * quat2[1] - quat1[1] * quat2[0] + quat1[2] * quat2[3] + quat1[3] * quat2[2],
        -quat1[0] * quat2[0] - quat1[1] * quat2[1] - quat1[2] * quat2[2] + quat1[3] * quat2[3]
    );

    return ret_quat;

}

rotation_matrix_t quatToMat(quat_t quat) {

    Eigen::Quaternionf q;
    q.x() = quat[0];
    q.y() = quat[1];
    q.z() = quat[2];
    q.w() = quat[3];
    Eigen::Matrix3f test_ER = q.normalized().toRotationMatrix(); //.cast<float>()

    return test_ER;

}

quat_t matToQuat(rotation_matrix_t R) {

    Eigen::Quaternionf q(R);

    quat_t quat(q.x(), q.y(), q.z(), q.w());

    return quat;

}

quat_t eulToQuat(orientation_t eul) {

    Eigen::Quaternionf q; 

    q = Eigen::AngleAxisf(eul[0], Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(eul[1], Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(eul[2], Eigen::Vector3f::UnitZ());


    quat_t quat;
    quat(0) = q.x();
    quat(1) = q.y();
    quat(2) = q.z();
    quat(3) = q.w();


    // Abbreviations for the various angular functions
    // double cr = cos(eul(0) * 0.5);
    // double sr = sin(eul(0) * 0.5);
    // double cp = cos(eul(1) * 0.5);
    // double sp = sin(eul(1) * 0.5);
    // double cy = cos(eul(2) * 0.5);
    // double sy = sin(eul(2) * 0.5);

    // quat_t quat;
    // quat(0) = sr * cp * cy - cr * sp * sy;
    // quat(1) = cr * sp * cy + sr * cp * sy;
    // quat(2) = cr * cp * sy - sr * sp * cy;
    // quat(3) = cr * cp * cy + sr * sp * sy;

    return quat;
}

transform_t getTransformMatrix(vector_t vec, quat_t quat) {

    Eigen::Matrix4f Trans; // Transformation Matrix

    Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    
    Trans.block<3,3>(0,0) = quatToMat(quat); // Copy rotation matrix into Trans
    
    Trans.block<3,1>(0,3) = vec; // Copy translation matrix into Trans

    return Trans;
}

plane_t create_plane(quat_t powerline_direction, point_t drone_xyz) {

    vector_t unit_x(1, 0, 0);

    orientation_t eul = quatToEul(powerline_direction);

    //orientation_t rotation(0, -eul[1], direction_tmp);

    vector_t plane_normal = rotateVector(eulToR(eul), unit_x);

	plane_t projection_plane;

    projection_plane.p = drone_xyz;
    projection_plane.normal = plane_normal;

	return projection_plane;
}


vector_t vector_NWU_to_NED(vector_t NWU_position) {

    // static rotation_matrix_t R_NWU_to_NED = eulToR(orientation_t(-M_PI, 0, 0));

    // vector_t NED_position;

    // NED_position = R_NWU_to_NED * NWU_position;

    // return NED_position;

    vector_t NED_position;

    NED_position(0) = NWU_position(0); 
    NED_position(1) = -NWU_position(1);
    NED_position(2) = -NWU_position(2);

    return NED_position;
}


orientation_t orientation_NWU_to_NED(orientation_t NWU_orientation) {

    quat_t NWU_quat = eulToQuat(NWU_orientation);

    quat_t NED_quat = quat_NWU_to_NED(NWU_quat);

    orientation_t NED_orientation = quatToEul(NED_quat);

    return NED_orientation;
}


quat_t quat_NWU_to_NED(quat_t NWU_quat) {

    quat_t NED_quat;
    NED_quat(0) =  NWU_quat(0);
    NED_quat(1) = -NWU_quat(1);
    NED_quat(2) = -NWU_quat(2);
    NED_quat(3) =  NWU_quat(3);
    return NED_quat;
}


pose_t pose_quat_NWU_to_NED(pose_t NWU_pose) {

    pose_t NED_pose;

    NED_pose.position = vector_NWU_to_NED(NWU_pose.position);

    NED_pose.quaternion = quat_NWU_to_NED(NWU_pose.quaternion);

    return NED_pose;
}


pose_eul_t pose_eul_NWU_to_NED(pose_eul_t NWU_pose) {

    // static rotation_matrix_t R_NWU_to_NED = eulToR(orientation_t(-M_PI, 0, 0));

    pose_eul_t NED_pose;

    // NED_pose.position = R_NWU_to_NED * NWU_pose.position;

    NED_pose.position = vector_NWU_to_NED(NWU_pose.position);

    // NED_pose.orientation(0) = NWU_pose.orientation(0); 
    // NED_pose.orientation(1) = -NWU_pose.orientation(1);
    // NED_pose.orientation(2) = -NWU_pose.orientation(2);

    NED_pose.orientation = orientation_NWU_to_NED(NWU_pose.orientation);

    return NED_pose;
}
