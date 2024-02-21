
#include <iostream>
#include <map>
#include <math.h>
#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>

#include <fstream>

#include "followMpc.h"
#include <Eigen/Core>
#include <Eigen/QR>
#include <queue>

using namespace std;
using namespace Eigen;

/********************/
/* CLASS DEFINITION */
/********************/
class MPCNode
{
    public:
        MPCNode();
        int get_thread_numbers();
        
    private:
        ros::NodeHandle _nh;
        ros::Subscriber _sub_odom, _sub_target, _sub_marker;
        ros::Publisher _pub_twist, _pub_mpctraj, _pub_target_prediction, _pub_follow_path, _pub_bazier_path, _pub_desire_pose, _pub_desirePointPath;
        ros::Timer _timer1;
        tf::TransformListener _tf_listener;
        ros::Time tracking_stime;
        ros::Time tracking_etime;
        ros::Time tracking_time;
        int tracking_time_sec;
        int tracking_time_nsec;
        
        //time flag
        bool start_timef = false;
        bool end_timef = false;

        geometry_msgs::Point _goal_pos;
        nav_msgs::Odometry _odom;
        nav_msgs::Path _odom_path, _mpc_traj, _hist_path, _desired_pose_path;
        geometry_msgs::Twist _twist_msg;
        geometry_msgs::PoseStamped _target_pose, _pre_target_pose;
        visualization_msgs::Marker _marker_pose, _pre_marker_pose;
        ros::Time _prev_time = ros::Time::now();

        string _odom_topic, _target_topic, _marker_topic;
        string _map_frame, _odom_frame, _car_frame;

        MPC _mpc;   // MPC类实例化
        map<string, double> _mpc_params;

        // 声明MPC求解参数
        double _mpc_steps, _w_ex, _w_etheta, _w_vel, 
               _w_angvel, _w_angvel_d, _w_vel_d, _max_angvel, _bound_value,
               _distance, _alpha;

        // 声明控制回环变量
        double _vR, _wR, _thetaR, _vF, _wF, _aF, _thetaF, _xE, _yE, _xR, _yR, _aR, _prev_vx, _prev_vy, _l;
        double _dt, _w, _speed, _pre_speed, _pre_w, _max_speed, _distance_in_queue;
        int _controller_freq, _downSampling, _thread_numbers;
        bool _pub_twist_flag, _debug_info;
        bool _target_received;
        double polyeval(Eigen::VectorXd coeffs, double x);
        Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

        // 声明成员函数
        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void followControlLoopCB(const ros::TimerEvent&);
        void targetCB(const geometry_msgs::PoseStamped::ConstPtr &targetMsg);
        void markerCB(const visualization_msgs::Marker::ConstPtr &targetMsg);
        void pubTargetPrediction(const VectorXd& target_params);
        void pubFollowPath(double x, double y, double theta);
        std::vector<std::vector<double>> bezier_curve(double start_x, double start_y, double start_theta, double end_x, double end_y, double end_theta, int num_points);
        double calculate_heading_error(const std::vector<std::vector<double>>& curve, double start_theta, double end_theta, double l);
        void publishBezierCurve(const std::vector<std::vector<double>>& bezierCurve);
        void publishDesirePose(double x_des, double y_des, double theta_des);
        void publishMPCTraj();
}; // end of class

MPCNode::MPCNode()
{
    //Private parameters handler
    ros::NodeHandle pn("~");

    // 加载控制回环参数
    pn.param("thread_numbers", _thread_numbers, 2); // number of threads for this ROS node
    pn.param("pub_twist_cmd", _pub_twist_flag, true);
    pn.param("debug_info", _debug_info, true);
    pn.param("max_speed", _max_speed, 1.0); // unit: m/s
    pn.param("controller_freq", _controller_freq, 10);
    _dt = double(1.0/_controller_freq); // time step duration dt in s 
    pn.param("l_hand", _l, 0.5);  //手端模型
    pn.param("distance_in_queue", _distance_in_queue, 2.0);

    // 加载求解MPC参数
    pn.param("target_distance", _distance,  1.0);
    pn.param("target_alpha",    _alpha,     0.0);
    pn.param("mpc_steps",       _mpc_steps, 20.0);
    pn.param("mpc_w_ex",        _w_ex,      10000.0);
    pn.param("mpc_w_etheta",    _w_etheta,  10.0);
    pn.param("mpc_w_vel",       _w_vel,     1000.0);
    pn.param("mpc_w_vel_d",     _w_vel_d,   1000.0);
    pn.param("mpc_w_angvel",    _w_angvel,  10.0);
    pn.param("mpc_w_angvel_d",  _w_angvel_d,   1000.0);
    pn.param("mpc_max_angvel",  _max_angvel,   1.5); // Maximal angvel radian (~30 deg)
    pn.param("mpc_bound_value", _bound_value,  1.0e6);

    // 话题&坐标系参数
    pn.param<std::string>("target_topic", _target_topic, "/target_point/pose" );
    pn.param<std::string>("odom_topic", _odom_topic, "/odom_gazebo" );
    pn.param<std::string>("marker_topic", _marker_topic, "/visualization_marker" );
    pn.param<std::string>("map_frame", _map_frame, "map" ); //*****for mpc, "odom"
    pn.param<std::string>("global_frame", _odom_frame, "odom");
    pn.param<std::string>("car_frame", _car_frame, "base_footprint" );

    // 发布和订阅
    _sub_odom   = _nh.subscribe( _odom_topic, 1, &MPCNode::odomCB, this);
    _sub_target = _nh.subscribe( _target_topic, 1, &MPCNode::targetCB, this);
    _sub_marker = _nh.subscribe( _marker_topic, 1, &MPCNode::markerCB, this);

    _pub_mpctraj   = _nh.advertise<nav_msgs::Path>("/mpc_trajectory", 1);
    if(_pub_twist_flag)
        _pub_twist = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    _pub_target_prediction = _nh.advertise<visualization_msgs::Marker>("/target_prediction", 1);
    _pub_follow_path = _nh.advertise<nav_msgs::Path>("/followCar_path", 1);
    _pub_bazier_path = _nh.advertise<nav_msgs::Path>("/bazier_path",1);
    _pub_desire_pose = _nh.advertise<geometry_msgs::PoseStamped>("/desire_pose", 1);
    _pub_desirePointPath = _nh.advertise<nav_msgs::Path>("/desire_pose_path", 1);

    ros::Duration(0.5).sleep(); //等待接受目标信息
    _timer1 = _nh.createTimer(ros::Duration((1.0)/_controller_freq), &MPCNode::followControlLoopCB, this); // 10Hz
    
    // 初始化变量
    _target_received = false;
    _w = 0.0;
    _speed = 0.0;
    _xR = 0.0;
    _yR = 0.0;
    _thetaR = 0.0;
    _vR = 0.0;
    _wR = 0.0;
    _prev_vx = 0.0;
    _prev_vx = 0.0;
    _pre_speed = 0.0;
    _pre_w     = 0.0;
    _twist_msg = geometry_msgs::Twist();
    _mpc_traj = nav_msgs::Path();

    // MPC目标函数参数初始化
    _mpc_params["DT"] = _dt;
    _mpc_params["STEPS"]    = _mpc_steps;
    _mpc_params["W_EX"]     = _w_ex;
    _mpc_params["W_ETHETA"] = _w_etheta;
    _mpc_params["W_V"]      = _w_vel;
    _mpc_params["W_ANGVEL"] = _w_angvel;
    _mpc_params["W_DANGVEL"]= _w_angvel_d;
    _mpc_params["W_DVEL"]     = _w_vel_d;
    _mpc_params["ANGVEL"]   = _max_angvel;
    _mpc_params["BOUND"]    = _bound_value;
    _mpc_params["MAXVEL"]   = _max_speed;
    _mpc_params["L"]   = _l;
    _mpc.LoadParams(_mpc_params);
}

// Public: return _thread_numbers
int MPCNode::get_thread_numbers()
{
    return _thread_numbers;
}

// CallBack: Update odometry
void MPCNode::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg) {
    _odom = *odomMsg;
}

void MPCNode::targetCB(const geometry_msgs::PoseStamped::ConstPtr& targetMsg) 
{
    _pre_target_pose = _target_pose;
    _target_pose = *targetMsg;
    _target_received = true;
    // 获得目标车姿态
    _xR = _target_pose.pose.position.x;
    _yR = _target_pose.pose.position.y;
    ros::Time current_time = ros::Time::now();

    // 计算目标速度，角速度，角度
    if (!_pre_target_pose.header.frame_id.empty()) {
        
        double delta_time = (current_time - _prev_time).toSec();

        // thetaR
        tf2::Quaternion quaternion(
            targetMsg->pose.orientation.x, 
            targetMsg->pose.orientation.y, 
            targetMsg->pose.orientation.z, 
            targetMsg->pose.orientation.w);
        tf2::Matrix3x3 rotation_matrix(quaternion);
        // 获取欧拉角
        double roll, pitch, yaw;
        rotation_matrix.getRPY(roll, pitch, yaw);
        // _thetaR = yaw + M_PI;   // -pi ~ pi
        _thetaR = yaw;

        // 计算位置差异
        double delta_x = targetMsg->pose.position.x - _pre_target_pose.pose.position.x;
        double delta_y = targetMsg->pose.position.y - _pre_target_pose.pose.position.y;
        tf2::Quaternion quaternion_diff = tf2::Quaternion(targetMsg->pose.orientation.x, targetMsg->pose.orientation.y, targetMsg->pose.orientation.z, targetMsg->pose.orientation.w) *
                                        tf2::Quaternion(_pre_target_pose.pose.orientation.x, _pre_target_pose.pose.orientation.y, _pre_target_pose.pose.orientation.z, _pre_target_pose.pose.orientation.w).inverse();
        tf2::Vector3 rotation_vector = quaternion_diff.getAxis() * quaternion_diff.getAngle();

        // 计算速度
        const double vx = delta_x / delta_time;
        const double vy = delta_y / delta_time;
        _vR = sqrt(vx * vx + vy * vy);
        _wR = rotation_vector.z() / delta_time;

        // 计算加速度
        const double ax = (vx - _prev_vx) / delta_time;
        const double ay = (vy - _prev_vy) / delta_time;
        // _aR = sqrt(ax * ax + ay * ay);
        _aR = ax;

        // 更新前一次位姿 时间 速度
        _prev_vx = vx;
        _prev_vy = vy;
        _pre_target_pose = _target_pose;
        _prev_time = current_time;

    }
}

void MPCNode::markerCB(const visualization_msgs::Marker::ConstPtr &targetMsg) {
}

void MPCNode::pubTargetPrediction(const VectorXd& target_params) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = _nh.getNamespace();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    // 设置标记的尺寸
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    // 设置标记的颜色
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    // 设置标记的姿态和位置
    marker.pose.position.x = target_params[0];
    marker.pose.position.y = target_params[1];
    marker.pose.position.z = 0.0;
    tf::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, target_params[2]);  // 使用欧拉角形式设置旋转矩阵
    marker.pose.orientation.x = quaternion.x();
    marker.pose.orientation.y = quaternion.y();
    marker.pose.orientation.z = quaternion.z();
    marker.pose.orientation.w = quaternion.w();
    // 设置标记的生存时间
    marker.lifetime = ros::Duration(1);
    _pub_target_prediction.publish(marker);
}

void MPCNode::pubFollowPath(double x, double y, double theta) {
    // 创建并填充当前车辆位置的消息
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = _odom_frame; // 设置坐标系为 "map"

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0; // 设置位置z

    // 计算车辆的朝向
    tf2::Quaternion quat;
    quat.setRPY(0, 0, theta); // 设置车辆的欧拉角
    pose.pose.orientation.x = quat.x();
    pose.pose.orientation.y = quat.y();
    pose.pose.orientation.z = quat.z();
    pose.pose.orientation.w = quat.w();

    // 将当前位置信息添加到历史路径中
    _hist_path.poses.push_back(pose);

    // 发布历史路径消息
    _hist_path.header.stamp = ros::Time::now();
    _hist_path.header.frame_id = _odom_frame; // 设置坐标系为 "map"
    _pub_follow_path.publish(_hist_path);
}

std::vector<std::vector<double>> MPCNode::bezier_curve(double start_x, double start_y, double start_theta, double end_x, double end_y, double end_theta, int num_points) {
    double distance = std::sqrt(std::pow(end_x - start_x, 2) + std::pow(end_y - start_y, 2));
    double start_dx = distance * std::cos(start_theta);
    double start_dy = distance * std::sin(start_theta);
    double end_dx = distance * std::cos(end_theta);
    double end_dy = distance * std::sin(end_theta);

    double control_x1 = start_x + start_dx / 3;
    double control_y1 = start_y + start_dy / 3;
    double control_x2 = end_x - end_dx / 3;
    double control_y2 = end_y - end_dy / 3;
    // double start_slope = tan(start_theta);
    // double end_slope   = tan(end_theta);
    // double control_x1 = start_x + (end_x - start_x) / 3;
    // double control_y1 = start_y + start_slope * (control_x1 - start_x);
    // double control_x2 = end_x - (end_x - start_x) / 3;
    // double control_y2 = end_y - end_slope * (end_x - control_x2);


    std::vector<double> t(num_points);
    for (int i = 0; i < num_points; ++i) {
        t[i] = static_cast<double>(i) / (num_points - 1);
    }

    std::vector<std::vector<double>> curve(num_points, std::vector<double>(2));
    for (int i = 0; i < num_points; ++i) {
        double t_ = t[i];
        double one_minus_t = 1 - t_;
        double one_minus_t_cubed = std::pow(one_minus_t, 3);
        double t_cubed = std::pow(t_, 3);

        curve[i][0] = one_minus_t_cubed * start_x + 3 * one_minus_t * one_minus_t * t_ * control_x1 + 3 * one_minus_t * t_ * t_ * control_x2 + t_cubed * end_x;
        curve[i][1] = one_minus_t_cubed * start_y + 3 * one_minus_t * one_minus_t * t_ * control_y1 + 3 * one_minus_t * t_ * t_ * control_y2 + t_cubed * end_y;
    }

    return curve;
}

double MPCNode::calculate_heading_error(const std::vector<std::vector<double>>& curve, double start_theta, double end_theta, double l) {
    std::vector<double> segment_lengths;
    segment_lengths.reserve(curve.size() - 1);
    for (size_t i = 1; i < curve.size(); ++i) {
        double dx = curve[i][0] - curve[i - 1][0];
        double dy = curve[i][1] - curve[i - 1][1];
        double length = std::sqrt(dx * dx + dy * dy);
        segment_lengths.push_back(length);
    }

    std::vector<double> accumulative_lengths;
    accumulative_lengths.reserve(curve.size() - 1);
    double total_length = 0.0;
    for (double length : segment_lengths) {
        total_length += length;
        accumulative_lengths.push_back(total_length);
    }

    size_t index = 0;
    while (index < accumulative_lengths.size() - 1 && accumulative_lengths[index] < l) {
        ++index;
    }

    std::vector<double> start_point(2);
    if (index == 0) {
        start_point = curve[0];
    } else {
        double excess_length = accumulative_lengths[index] - l;
        double segment_ratio = excess_length / segment_lengths[index];
        start_point[0] = curve[index][0] - (curve[index][0] - curve[index - 1][0]) * segment_ratio;
        start_point[1] = curve[index][1] - (curve[index][1] - curve[index - 1][1]) * segment_ratio;
    }

    double dx = curve[index + 1][0] - start_point[0];
    double dy = curve[index + 1][1] - start_point[1];
    double ex = start_point[0] + dx;
    double ey = start_point[1] + dy;

    double tangent_heading_rad = std::atan2(dy, dx);
    double heading_error_rad = tangent_heading_rad - start_theta;

    if (heading_error_rad > M_PI) {
        heading_error_rad -= 2 * M_PI;
    } else if (heading_error_rad < -M_PI) {
        heading_error_rad += 2 * M_PI;
    }

    if (heading_error_rad > M_PI / 2) {
        heading_error_rad = M_PI / 2;
    }
    else if (heading_error_rad < - M_PI / 2) {
        heading_error_rad = - M_PI / 2;
    }

    return heading_error_rad;
}

void MPCNode::publishBezierCurve(const std::vector<std::vector<double>>& bezierCurve)
{
    nav_msgs::Path pathMsg;
    pathMsg.header.stamp = ros::Time::now();
    pathMsg.header.frame_id = _odom_frame;

    for (const auto& point : bezierCurve)
    {
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.pose.position.x = point[0];
        poseStamped.pose.position.y = point[1];
        poseStamped.pose.position.z = 0.0;
        poseStamped.header.stamp = ros::Time::now();
        poseStamped.header.frame_id = _odom_frame;

        pathMsg.poses.push_back(poseStamped);
    }
    _pub_bazier_path.publish(pathMsg);
}

void MPCNode::publishDesirePose(double x_des, double y_des, double theta_des) {
    geometry_msgs::PoseStamped desired_pose;

    desired_pose.header.frame_id = _odom_frame;
    desired_pose.header.stamp = ros::Time::now();
    desired_pose.pose.position.x = x_des;
    desired_pose.pose.position.y = y_des;
    desired_pose.pose.position.z = 0;

    desired_pose.pose.orientation.x = 0.0;
    desired_pose.pose.orientation.y = 0.0;
    desired_pose.pose.orientation.z = sin(theta_des / 2);
    desired_pose.pose.orientation.w = cos(theta_des / 2);

    _pub_desire_pose.publish(desired_pose);


    _desired_pose_path.header.frame_id = _odom_frame;
    _desired_pose_path.header.stamp = ros::Time::now();

    _desired_pose_path.poses.push_back(desired_pose);

    _pub_desirePointPath.publish(_desired_pose_path);
}

void MPCNode::publishMPCTraj() {
    _mpc_traj = nav_msgs::Path();
    _mpc_traj.header.frame_id = _car_frame; // points in car coordinate        
    _mpc_traj.header.stamp = ros::Time::now();

    geometry_msgs::PoseStamped tempPose;
    tf2::Quaternion myQuaternion;

    for(int i=0; i<_mpc.mpc_x.size(); i++)
    {
        tempPose.header = _mpc_traj.header;
        tempPose.pose.position.x = _mpc.mpc_x[i];
        tempPose.pose.position.y = _mpc.mpc_y[i];

        myQuaternion.setRPY( 0, 0, _mpc.mpc_theta[i] );  
        tempPose.pose.orientation.x = myQuaternion[0];
        tempPose.pose.orientation.y = myQuaternion[1];
        tempPose.pose.orientation.z = myQuaternion[2];
        tempPose.pose.orientation.w = myQuaternion[3];
            
        _mpc_traj.poses.push_back(tempPose); 
    }     
    // publish the mpc trajectory
    _pub_mpctraj.publish(_mpc_traj);
}

void MPCNode::followControlLoopCB(const ros::TimerEvent&)
{
    if( _target_received ) //received target
    {
        nav_msgs::Odometry odom = _odom; 
        nav_msgs::Path odom_path = _odom_path;   
        geometry_msgs::PoseStamped target_pos = _target_pose;
        
        // ************************* update variables **************************************

        // update follow car , odom frame
        const double xF = odom.pose.pose.position.x;
        const double yF = odom.pose.pose.position.y;

        tf::Pose pose;
        tf::poseMsgToTF(odom.pose.pose, pose);
        // double thetaF = tf::getYaw(pose.getRotation()) + M_PI;  // -pi ~ pi
        double thetaF = tf::getYaw(pose.getRotation());
        
        const double vF = _speed;
        const double wF = _w;
        const double dt = _dt;
        const double aF = (_speed - _pre_speed) / dt;       // acceleration
        const double dwF= (_w - _pre_w) / dt;               // delta of w

        // update refrence car, odom frame
        const double xR = _xR;
        const double yR = _yR;
        const double thetaR = _thetaR;
        const double vR = _vR;
        const double wR = _wR;
        const double aR = _aR;
        const double xd = _distance * cos(_alpha);
        const double yd = - _distance * sin(_alpha);

        // update error
        double temp_thetaR = thetaR;

        if(temp_thetaR <= -M_PI + thetaF) 
        {
            temp_thetaR = thetaR + 2 * M_PI;
        } 
        else if(temp_thetaR >= M_PI + thetaF) 
        {
            temp_thetaR = thetaR - 2 * M_PI;
        }

        double thetaE = temp_thetaR - thetaF;
        const double xE = xd * cos(thetaE) - yd * sin(thetaE) + (xR - xF) * cos(thetaF) + (yR - yF) * sin(thetaF) - _l;
        const double yE = xd * sin(thetaE) + yd * cos(thetaE) - (xR - xF) * sin(thetaF) + (yR - yF) * cos(thetaF);

        // update desire pose
        const double x_des  = xR + cos(thetaR) * xd - sin(thetaR) * yd;
        const double y_des  = yR + sin(thetaR) * xd + cos(thetaR) * yd;

        std::vector<std::vector<double>> bazierCurve = bezier_curve(xF, yF, thetaF, x_des, y_des, thetaR, 100);

        const double eBazier = calculate_heading_error(bazierCurve, thetaF + M_PI, thetaR + M_PI, _l);

        // ************************* variabels at action time, in car frame, MPC init value *********************
        const double xF_act = vF * dt;
        const double yF_act = 0;
        const double thetaF_act = wF * dt;
        const double ev = cos(thetaE) * (vR - yd * wR) - xd * sin(thetaE) * wR - vF;
        const double ew = sin(thetaE) * (vR - yd * wR) + xd * cos(thetaE) * wR - _l * wF;
        const double xE_act = xE + (ev + yE * wF) * dt;
        const double yE_act = yE + (ew - xE * wF) * dt;

        double thetaE_act = thetaE;

        if (abs(xE) > _distance_in_queue) // 远距离时跟随贝塞尔曲线
        {                 
            thetaE_act = eBazier - thetaF_act;
            publishBezierCurve(bazierCurve);
        }
        
        // if (thetaE_act > M_PI / 2)
        // {
        //     thetaE_act = M_PI / 2;
        // }
        // if (thetaE_act < - M_PI / 2) 
        // {
        //     thetaE_act = M_PI / 2;
        // }

        // ******************************************* solve MPC *************************************************
        VectorXd target_params(10);
        VectorXd state(6);
        target_params << xR, yR, thetaR, vR, wR, xd, yd, aR, aF, dwF;
        state << xF_act, yF_act, thetaF_act, xE_act, yE_act, thetaE_act;
        // pubTargetPrediction(target_params);     // show prediction pose
        vector<double> mpc_results = _mpc.Solve(state, target_params);

        // ******************************************* 处理并发布话题 ********************************************
        _pre_speed = _speed;
        _pre_w = _w;
        _speed = mpc_results[0]; // radian/sec, angular velocity
        _w = mpc_results[1]; // acceleration

        if(_speed >= _max_speed) _speed = _max_speed;
        if(_speed <= 0.0) _speed = 0.0;
        if(_w >= _max_angvel) _w = _max_angvel;
        else if(_w <= - _max_angvel) _w = - _max_angvel;

        // ******************************************* Display & public ********************************************
        if(_debug_info)
        {
        cout << "------------------- Debug ----------------------------" <<endl;
        cout << "\t xE = \t" << xE << ", " << "\t yE = \t" << yE  << endl;
        cout << "\t xR = \t" << xR << ", " << "\t yR = \t" << yR << "\t thetaR = \t" << thetaR << endl;
        cout << "\t xF = \t" << xF << ", " << "\t yF = \t" << yF << "\t thetaF = \t" << thetaF << endl;
        cout << "\t xd = \t" << xd << ", " << "\t yd = \t" << yd << endl;
        cout << "\t x_des = \t" << x_des << ", " << "\t y_des = \t" << y_des << endl;
        cout << "\t x_error = \t" << x_des - xF << ", " << "\t y_error = \t" << y_des - yF << "\t thetaE = \t" << thetaE << endl;
        cout << "\t eBazier = \t" << eBazier << "\t temp_thetaR = \t" << temp_thetaR <<  endl;
        cout << "\t vF = \t" << vF << "\t wF = \t" << wF <<  endl;
        }

        publishMPCTraj();
        publishDesirePose(x_des, y_des, thetaR);
        pubFollowPath(xF, yF, thetaF);

    }else {
        _speed = 0.0;
        _w = 0;
        cout << "Target not recevied!" << endl;
    }
    // publish general cmd_vel 
    if(_pub_twist_flag) {
        _twist_msg.linear.x  = _speed;
        _twist_msg.angular.z = _w;
        _pub_twist.publish(_twist_msg);
    }
}


/****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "MPC_Node");
    MPCNode mpc_node;
    ROS_INFO("Waiting for global path msgs ~");
    ros::AsyncSpinner spinner(mpc_node.get_thread_numbers()); // Use multi threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
