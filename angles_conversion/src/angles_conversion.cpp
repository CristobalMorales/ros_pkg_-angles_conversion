#include "ros/ros.h"
#include <cmath>
#include "angles_conversion_srv/quat2zyx.h"
#include "angles_conversion_srv/quat2rodrigues.h"
#include "angles_conversion_srv/rotmat2quat.h"
#define _USE_MATH_DEFINES

bool convert_quat2zyx(angles_conversion_srv::quat2zyx::Request &req, angles_conversion_srv::quat2zyx::Response &res)
{
    /**
     * @brief Convert from quaternions to z, y and x Euler rotation representation in this order
     * 
     * @param req The Request data that contains the quaternion data (q.x, q.y, q.z, q.w)
     * @param res The response that contains the Euler angles roll, pitch, yaw (x, y, z) in radians
     */
    double qx = req.q.x;
    double qy = req.q.y;
    double qz = req.q.z;
    double qw = req.q.w;

    double num = 2*(qw*qx + qy*qz);
    double den = 1 - 2*(qx*qx + qy*qy);
    res.x.data = std::atan2(num, den);

    double arg_y = 2*(qw*qy - qz*qx);
    if (std::abs(arg_y) >= 1)
        res.y.data = std::copysign(M_PI/2, arg_y);
    else
        res.y.data = std::asin(arg_y);

    num = 2*(qw*qz + qx*qy);
    den = 1 - 2*(qy*qy + qz*qz);
    res.z.data = std::atan2(num, den);
}
bool convert_quat2rodrigues(angles_conversion_srv::quat2rodrigues::Request &req, angles_conversion_srv::quat2rodrigues::Response &res)
{
    /**
     * @brief Convert from quaternions to Rodrigues representation
     * 
     * @param req The Request data that contains the quaternion data (q.x, q.y, q.z, q.w)
     * @param res The response that contains the rodrigues angles for rotation (x, y and z) in radians
     */
    double qx = req.q.x;
    double qy = req.q.y;
    double qz = req.q.z;
    double qw = req.q.w;

    double half_angle = std::acos(qw);
    double norm = std::sin(half_angle);
    res.x.data = (qx/norm)*(2*half_angle);
    res.y.data = (qy/norm)*(2*half_angle);
    res.z.data = (qz/norm)*(2*half_angle);
}
bool convert_rotmat2quat(angles_conversion_srv::rotmat2quat::Request &req, angles_conversion_srv::rotmat2quat::Response &res)
{
    /**
     * @brief Convert from Matriz rotation to quaternion representation
     * 
     * @param req The Request data that contains 3 std_msgs::Float32Multiarray components (r1, r2 and r3)
     * these vectors represent the column-wise vectors of the rotation matrix
     * @param res The response that contains the quaternion data (q.x, q.y, q.z, q.w)
     */
    float* x_vect = &(req.r1.data[0]);
    float* y_vect = &(req.r2.data[0]);
    float* z_vect = &(req.r3.data[0]);

    float qw = std::sqrt(1.0 + x_vect[0] + y_vect[1] + z_vect[2])/2;
    float qx = (y_vect[2] - z_vect[1])/(4*qw);
    float qy = (z_vect[0] - x_vect[2])/(4*qw);
    float qz = (x_vect[1] - y_vect[0])/(4*qw);

    res.q.w = qw;
    res.q.x = qx;
    res.q.y = qy;
    res.q.z = qz;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rotation_converter");
    ros::NodeHandle nh;

    ros::ServiceServer quat2zyx_server = nh.advertiseService("quat2zyx", convert_quat2zyx);
    ros::ServiceServer quat2rodrigues_server = nh.advertiseService("quat2rodrigues", convert_quat2rodrigues);
    ros::ServiceServer rotmat2quat_server = nh.advertiseService("rotmat2quat", convert_rotmat2quat);

    ros::spin();
    return 5;
}