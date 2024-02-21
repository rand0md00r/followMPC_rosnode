
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

#include <fstream>

#include "navMpc.h"
#include <Eigen/Core>
#include <Eigen/QR>

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
        ros::Subscriber _sub_odom, _sub_path, _sub_goal, _sub_target, _sub_desirePos, _sub_marker;
        ros::Publisher _pub_globalpath,_pub_odompath, _pub_twist, _pub_mpctraj, _pub_target_prediction, _pub_follow_path, _pub_desire_pose, _pub_desirePointPath, _pub_fake_pose;
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
        geometry_msgs::PoseStamped _target_pose, _desire_pose, _pre_target_pose, _fake_target;
        visualization_msgs::Marker _marker_pose, _pre_marker_pose;
        ros::Time _prev_time = ros::Time::now();

        string _odom_topic, _target_topic, _desirePos_topic, _marker_topic;
        string _map_frame, _odom_frame, _car_frame;

        MPC _mpc;   // MPC类实例化，用于求解MPC
        map<string, double> _mpc_params;

        // 声明MPC求解参数
        double _mpc_steps, _w_ex, _w_etheta, _w_vel, 
               _w_angvel, _w_angvel_d, _w_vel_d, _max_angvel, _max_throttle, _bound_value,
               _distance, _alpha;

        // 声明控制回环变量
        double _vR, _wR, _thetaR, _vF, _wF, _thetaF, _xE, _yE, _xR, _yR, _aR, _prev_vx, _prev_vy, _l;
        double _dt, _w, _throttle, _speed, _max_speed;
        int _controller_freq, _downSampling, _thread_numbers;
        bool _goal_received, _goal_reached, _path_computed, _pub_twist_flag, _debug_info;
        bool _target_received, _desirePos_received;
        double polyeval(Eigen::VectorXd coeffs, double x);
        Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

        // 声明成员函数
        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void desiredPathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        void controlLoopCB(const ros::TimerEvent&);
        void followControlLoopCB(const ros::TimerEvent&);
        void targetCB(const geometry_msgs::PoseStamped::ConstPtr &targetMsg);
        void desirePosCB(const geometry_msgs::PoseStamped::ConstPtr &desirePosMsg);
        void markerCB(const visualization_msgs::Marker::ConstPtr &targetMsg);
        void pubTargetPrediction(const VectorXd& target_params);
        void pubFollowPath(double x, double y, double theta);
        void publishDesirePose(double x_des, double y_des, double theta_des);
        void publishFakePose(double x, double y, double theta);
}; // end of class

MPCNode::MPCNode()
{
    //Private parameters handler
    ros::NodeHandle pn("~");

    // 加载控制回环参数
    pn.param("thread_numbers", _thread_numbers, 2); // number of threads for this ROS node
    pn.param("pub_twist_cmd", _pub_twist_flag, true);
    pn.param("debug_info", _debug_info, true);
    pn.param("max_speed", _max_speed, 0.50); // unit: m/s
    pn.param("controller_freq", _controller_freq, 10);
    _dt = double(1.0/_controller_freq); // time step duration dt in s 
    pn.param("l_hand", _l, 0.5);  //手端模型

    // 加载求解MPC参数
    pn.param("target_distance", _distance, 1.0);
    pn.param("target_alpha", _alpha, 3.14);
    pn.param("mpc_steps", _mpc_steps, 20.0);
    pn.param("mpc_w_ex", _w_ex, 500.0);
    pn.param("mpc_w_etheta", _w_etheta, 500.0);
    pn.param("mpc_w_vel", _w_vel, 500.0);
    pn.param("mpc_w_vel_d", _w_vel_d, 1.0);
    pn.param("mpc_w_angvel", _w_angvel, 10.0);
    pn.param("mpc_w_angvel_d", _w_angvel_d, 1.0);
    pn.param("mpc_max_angvel", _max_angvel, 3.0); // Maximal angvel radian (~30 deg)
    pn.param("mpc_max_throttle", _max_throttle, 1.0);
    pn.param("mpc_bound_value", _bound_value, 1.0e3);

    // 话题&坐标系参数
    pn.param<std::string>("target_topic", _target_topic, "/target_point/pose" );
    
    pn.param<std::string>("desirePos_topic", _desirePos_topic, "/desired_pose/pose" );
    pn.param<std::string>("odom_topic", _odom_topic, "/odom_gazebo" );
    pn.param<std::string>("marker_topic", _marker_topic, "/visualization_marker" );

    pn.param<std::string>("map_frame", _map_frame, "map" ); //*****for mpc, "odom"
    pn.param<std::string>("global_frame", _odom_frame, "odom");
    pn.param<std::string>("car_frame", _car_frame, "base_footprint" );



    // 发布和订阅
    _sub_odom   = _nh.subscribe( _odom_topic, 1, &MPCNode::odomCB, this);
    _sub_target = _nh.subscribe( _target_topic, 1, &MPCNode::targetCB, this);
    _sub_marker = _nh.subscribe( _marker_topic, 1, &MPCNode::markerCB, this);
    _sub_desirePos = _nh.subscribe( _desirePos_topic, 1, &MPCNode::desirePosCB, this);
    _pub_globalpath  = _nh.advertise<nav_msgs::Path>("/global_path", 1); // Global path generated from another source
    _pub_odompath  = _nh.advertise<nav_msgs::Path>("/mpc_reference", 1); // reference path for MPC ///mpc_reference 
    _pub_mpctraj   = _nh.advertise<nav_msgs::Path>("/mpc_trajectory", 1);// MPC trajectory output
    if(_pub_twist_flag)
        _pub_twist = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1); //for stage (Ackermann msg non-supported)
    _pub_target_prediction = _nh.advertise<visualization_msgs::Marker>("/target_prediction", 1);    // 显示目标车预测状态
    _pub_follow_path = _nh.advertise<nav_msgs::Path>("/followCar_path", 1);    // 显示目标车预测状态
    _pub_desirePointPath = _nh.advertise<nav_msgs::Path>("/desire_pose_path", 1);
    _pub_desire_pose = _nh.advertise<geometry_msgs::PoseStamped>("/desire_pose", 1);
    _pub_fake_pose = _nh.advertise<geometry_msgs::PoseStamped>("/fake_bunker_pose", 1);

    ros::Duration(0.5).sleep(); //等待接受目标信息
    _timer1 = _nh.createTimer(ros::Duration((1.0)/_controller_freq), &MPCNode::followControlLoopCB, this); // 10Hz //*****mpc
    
    // 初始化变量
    _target_received = false;
    _desirePos_received = false;
    _goal_received = false;
    _goal_reached  = false;
    _path_computed = false;
    _throttle = 0.0; 
    _w = 0.0;
    _speed = 0.0;
    
    _xR = 0.0;
    _yR = 0.0;
    _thetaR = 0.0;
    _vR = 0.0;
    _wR = 0.0;
    _prev_vx = 0.0;
    _prev_vx = 0.0;
    
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
    _mpc_params["MAXTHR"]   = _max_throttle;
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

// Evaluate a polynomial.
double MPCNode::polyeval(Eigen::VectorXd coeffs, double x) {}

// CallBack: Update odometry
void MPCNode::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg) {
    _odom = *odomMsg;

    // 创建并填充当前车辆位置的消息
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = _odom_frame; // 设置坐标系为 "map"

    pose.pose.position.x = _odom.pose.pose.position.x;
    pose.pose.position.y = _odom.pose.pose.position.y;
    pose.pose.position.z = 0.0; // 设置位置z

    pose.pose.orientation.x = _odom.pose.pose.orientation.x;
    pose.pose.orientation.y = _odom.pose.pose.orientation.y;
    pose.pose.orientation.z = _odom.pose.pose.orientation.z;
    pose.pose.orientation.w = _odom.pose.pose.orientation.w;

    _pub_fake_pose.publish(pose);
}

// CallBack: Update path waypoints (conversion to odom frame)
void MPCNode::pathCB(const nav_msgs::Path::ConstPtr& pathMsg){}

// CallBack: Update goal status
void MPCNode::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg){}

// Timer: Control Loop (closed loop nonlinear MPC)
void MPCNode::controlLoopCB(const ros::TimerEvent&){}

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
        _thetaR = yaw + M_PI;
        // _thetaR = yaw;

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
        _aR = sqrt(ax * ax + ay * ay);

        // 更新前一次位姿 时间 速度
        _prev_vx = vx;
        _prev_vy = vy;
        _pre_target_pose = _target_pose;
        _prev_time = current_time;

    }
}

void MPCNode::markerCB(const visualization_msgs::Marker::ConstPtr &targetMsg) {

}

void MPCNode::desirePosCB(const geometry_msgs::PoseStamped::ConstPtr& desirePosMsg) {
    _desire_pose = *desirePosMsg;
    _desirePos_received = true;
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


void MPCNode::followControlLoopCB(const ros::TimerEvent&)
{          
    if( _target_received ) //received target
    {
        nav_msgs::Odometry odom = _odom; 
        nav_msgs::Path odom_path = _odom_path;   
        geometry_msgs::PoseStamped target_pos = _target_pose;
        geometry_msgs::PoseStamped desire_pos = _desire_pose;
        
        // ************************* 更新参数 **************************************
        const double xF = odom.pose.pose.position.x;    // odom frame
        const double yF = odom.pose.pose.position.y;
        tf::Pose pose;
        tf::poseMsgToTF(odom.pose.pose, pose);
        double thetaF = tf::getYaw(pose.getRotation()) + M_PI;
        const double vF = _speed;
        const double wF = _w;
        const double dt = _dt;
        pubFollowPath(xF, yF, thetaF);
        const double xR = _xR;      // 更新目标车的系统状态,odom frame，由观测获得
        const double yR = _yR;
        const double thetaR = _thetaR;
        const double vR = _vR;
        const double wR = _wR;
        // 目标位置参数
        const double xd = - _distance * cos(_alpha);
        const double yd = _distance * sin(_alpha);
        // 误差项更新
        double temp_thetaR = thetaR;
        if(temp_thetaR <= -M_PI + thetaF) {
            temp_thetaR = thetaR + 2 * M_PI;
        } else if (temp_thetaR >= M_PI + thetaF) {
            temp_thetaR = thetaR - 2 * M_PI;
        }
        const double thetaE = temp_thetaR - thetaF;

        const double xE = xd * cos(thetaE) - yd * sin(thetaE) + (xR - xF) * cos(thetaF) + (yR - yF) * sin(thetaF) - _l;
        const double yE = xd * sin(thetaE) + yd * cos(thetaE) - (xR - xF) * sin(thetaF) + (yR - yF) * cos(thetaF);

        VectorXd target_params(9);
        target_params << xR, yR, thetaR, vR, wR, xd, yd, xE, yE;
        pubTargetPrediction(target_params);     // show prediction pose
        // update desire pose
        const double x_des  = xR + cos(thetaR) * xd - sin(thetaR) * yd;
        const double y_des  = yR + sin(thetaR) * xd + cos(thetaR) * yd;
        publishDesirePose(x_des, y_des, thetaR);

        // ************************* 控制时刻状态, in car frame, MPC初值 **************************************

        VectorXd state(6);
        const double xF_act = vF * dt;
        const double yF_act = 0;
        const double thetaF_act = wF * dt;
        const double ev = cos(thetaE) * (vR - yd * wR) - xd * sin(thetaE) * wR - vF;
        const double ew = sin(thetaE) * (vR - yd * wR) + xd * cos(thetaE) * wR - _l * wF;
        const double xE_act = xE + (ev + yE * wF) * dt;
        const double yE_act = yE + (ew - xE * wF) * dt;
        const double thetaE_act = thetaE - thetaF_act;
        
        state << xF_act, yF_act, thetaF_act, xE_act, yE_act, thetaE_act;


        // ******************************************* 求解MPC *************************************************
        vector<double> mpc_results = _mpc.Solve(state, target_params);

        // ******************************************* 处理并发布话题 ********************************************
        // MPC result (all described in car frame), output = (v, w)        
        _speed = mpc_results[0]; // radian/sec, angular velocity
        if (vR == 0.0) {
            _speed = 0.0;
        }
        _w = mpc_results[1]; // acceleration

        if (_speed >= _max_speed)
            _speed = _max_speed;
        if(_speed <= 0.0)
            _speed = 0.0;
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
        }
        // Display the MPC predicted trajectory
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
    else
    {
        _throttle = 0.0;
        _speed = 0.0;
        _w = 0;
        cout << "Target not recevied!" << endl;
    }
    // publish general cmd_vel 
    if(_pub_twist_flag)
    {
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
