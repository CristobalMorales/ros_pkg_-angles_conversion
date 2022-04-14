#include "ros/ros.h"
#include <cmath>
#include "angles_conversion_srv/quat2zyx.h"
#include "angles_conversion_srv/quat2rodrigues.h"
#include "angles_conversion_srv/rotmat2quat.h"
#define _USE_MATH_DEFINES

bool convert_quat2zyx(cw1q4_srv::quat2zyx::Request &req, cw1q4_srv::quat2zyx::Response &res)
{
    double qx = req.q.x;
    double qy = req.q.y;
    double qz = req.q.z;
    double qw = req.q.w;

    double num = 2*(qx*qx + qy*qz);
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
bool convert_quat2rodrigues(cw1q4_srv::quat2rodrigues::Request &req, cw1q4_srv::quat2rodrigues::Response &res)
{
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
bool convert_rotmat2quat(cw1q4_srv::rotmat2quat::Request &req, cw1q4_srv::rotmat2quat::Response &res)
{
    double* x_vect = &req.r1.data[0];
    double* y_vect = &req.r2.data[0];
    double* z_vect = &req.r3.data[0];

    double angle = std::acos((x_vect[1] + y_vect[2] + z_vect[3] - 1)/2);
    double den = 1/(2*std::sin(angle));
    double u_x = den*(z_vect[2] - y_vect[3]);
    double u_y = den*(x_vect[3] - z_vect[1]);
    double u_z = den*(y_vect[1] - x_vect[2]);

    res.q.w = std::cos(angle/2);
    res.q.x = std::sin(angle/2)*u_x;
    res.q.y = std::sin(angle/2)*u_y;
    res.q.z = std::sin(angle/2)*u_z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rotation_converter");
    ros::NodeHandle nh;

    //TODO: Define three services
    ros::ServiceServer quat2zyx_server = nh.advertiseService("quat2zyx", convert_quat2zyx);
    ros::ServiceServer quat2rodrigues_server = nh.advertiseService("quat2rodrigues", convert_quat2rodrigues);
    ros::ServiceServer rotmat2quat_server = nh.advertiseService("rotmat2quat", convert_rotmat2quat);

    ros::spin();
    return 5;
}