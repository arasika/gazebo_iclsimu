#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/SetLinkState.h"
#include "gazebo_msgs/GetLinkState.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Dense>
#include <iostream>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/common/transforms.h>
#include <random>
#include <chrono>
#include <mutex>


#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,  
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (uint16_t, ring, ring)
                                   (float, time, time)
)


using namespace Eigen;
typedef PointXYZIRT  PointType;


geometry_msgs::Pose start_pose;
geometry_msgs::Pose velodyne_pose;
double pi=3.1415926;
ros::ServiceClient client;
ros::Publisher pub_linkstate;
Eigen::Affine3f Twl_0;
  static double t_lidar =0;
int cur_img_time = 0;
bool flag_img_first = 0;
bool flag_img_lidar=0;
std::deque<double> imgtimeQueue;
std::deque<nav_msgs::Odometry> lidarOdomQueue;
std::mutex img_buf;
double t_start;
float ellipse_x = 0; 
float ellipse_y = 0; 
float z = 0.5; // z轴做sin运动 
float K1 = 5; // z轴的正弦频率是x，y的k1倍 
float K = M_PI/ 10; // 2 
float Ke = M_PI; 
double Kr = M_PI/ 4; 
double k_roll = 0; 
double k_pitch = 0; 
double k_yaw = 0.6; 
float Horizon_SCAN = 1800;
ros::Publisher pubCloud_distort;



int getIndex(std::vector<std::string> v, std::string value){
    for(int i =0;i<v.size();i++){
        if(v[i].compare(value) == 0)
        return i;
    }
    return -1;
}
// euler2Rotation:   body frame to interitail frame
Eigen::Matrix3d euler2Rotation( Eigen::Vector3d  eulerAngles)
{
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);
    double yaw = eulerAngles(2);

    double cr = cos(roll); double sr = sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);
    double cy = cos(yaw); double sy = sin(yaw);

    //RIb=RbI^−1=RbI^T
    Eigen::Matrix3d RIb;//这里对应的是公式 是从i系到b系的旋转矩阵的转置
    //i系到b系的旋转矩阵的公式见ppt 就是Rbi=R(yaw,pith,roll)
    RIb<< cy*cp ,   cy*sp*sr - sy*cr,   sy*sr + cy* cr*sp,
            sy*cp,    cy *cr + sy*sr*sp,  sp*sy*cr - cy*sr,
            -sp,         cp*sr,           cp*cr;
    return RIb;
}

Eigen::Vector3d Rotation2euler(Eigen::Matrix3d R){
    double r = atan(R(2,1)/R(2,2));
    double p=asin(-R(2,0));
    double y= atan(R(1,0)/R(0,0));
    Eigen::Vector3d euler;
    euler =Eigen::Vector3d(r,p,y);
    return euler;
}
//变换矩阵--将欧拉角速度从I坐标变到body坐标
Eigen::Matrix3d eulerRates2bodyRates(Eigen::Vector3d eulerAngles)
{
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);

    double cr = cos(roll); double sr = sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);
//公式见ppt
    Eigen::Matrix3d R;
    R<<  1,   0,    -sp,
            0,   cr,   sr*cp,
            0,   -sr,  cr*cp;

    return R;
}

Eigen::Quaterniond Rotation2quat(Eigen::Matrix3d R){
    Eigen::Quaterniond quat_R;
    double x,y,z,w;
    if (R(0,0)>=R(1,1)+R(2,2))
        {x = 0.5*sqrt(1+R(0,0)-R(1,1)-R(2,2));y=0.25*(R(0,1)+R(1,0))/x;z=0.25*(R(0,2)+R(2,0))/x;w=0.25*(R(2,1)-R(1,2))/x;}
    else if (R(1,1)>=R(0,0)+R(2,2))
        {y = 0.5*sqrt(1-R(0,0)+R(1,1)-R(2,2));x=0.25*(R(0,1)+R(1,0))/y;w=0.25*(R(0,2)-R(2,0))/y;z=0.25*(R(2,1)+R(1,2))/y;}
    else if (R(2,2)>=R(1,1)+R(0,0))
        {z = 0.5*sqrt(1-R(0,0)-R(1,1)+R(2,2));w=0.25*(-R(0,1)+R(1,0))/z;x=0.25*(R(0,2)+R(2,0))/z;y=0.25*(R(2,1)+R(1,2))/z;}
    else   
        {w = 0.5*sqrt(1+R(0,0)+R(1,1)+R(2,2));z=0.25*(-R(0,1)+R(1,0))/w;y=0.25*(R(0,2)-R(2,0))/w;x=0.25*(R(2,1)-R(1,2))/w;}
    quat_R = Eigen::Quaterniond(w,x,y,z);
    return quat_R;
}

void link_states_callback(gazebo_msgs::LinkStates link_states){
    int link_index = getIndex(link_states.name, "vodyne");
    velodyne_pose = link_states.pose[link_index];
    // std::cout << "得到vodyne的坐标"<< std::endl;
 }


void img_callback(const sensor_msgs::ImageConstPtr img_msg){
    // std::cout << "输入图像信息"<< std::endl;
        // auto start = std::chrono::high_resolution_clock::now();
        double cur_img_time = img_msg->header.stamp.toSec();
        std::cout << "cur_img_time"<<  cur_img_time<< std::endl;
        img_buf.lock();
        imgtimeQueue.push_back(cur_img_time);
        img_buf.unlock();
        // auto end = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<float> duration = end - start;
        // std::cout << "Time taken: " << duration.count() << " seconds" << std::endl;
     
}

    double imu_timestep = 1./200;
    double gyro_bias_sigma = 1.0e-5;
    double acc_bias_sigma = 0.001;
    double gyro_noise_sigma = 0.0017;    // rad/s * 1/sqrt(hz)
    double acc_noise_sigma = 0.0060;      //　m/(s^2) * 1/sqrt(hz)
    Eigen::Vector3d imu_gyro_bias;
    Eigen::Vector3d imu_acc_bias;
    Eigen::Vector3d gyro_bias_= Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_bias_= Eigen::Vector3d::Zero();
    Eigen::Vector3d constant_gyro_bias(0.02,0.02,0.02);
    Eigen::Vector3d constant_acc_bias(0.02,0.02,0.02);


 void addIMUnoise(Eigen::Vector3d imu_gyro,Eigen::Vector3d imu_acc)
{
    std::random_device rd;
    std::default_random_engine generator_(rd());
    std::normal_distribution<double> noise(0.0, 1.0);

    Eigen::Vector3d noise_gyro(noise(generator_),noise(generator_),noise(generator_));
    Eigen::Matrix3d gyro_sqrt_cov = gyro_noise_sigma * Eigen::Matrix3d::Identity();
    imu_gyro = imu_gyro + gyro_sqrt_cov * noise_gyro / sqrt( imu_timestep ) + gyro_bias_;

    Eigen::Vector3d noise_acc(noise(generator_),noise(generator_),noise(generator_));
    Eigen::Matrix3d acc_sqrt_cov = acc_noise_sigma * Eigen::Matrix3d::Identity();
    imu_acc = imu_acc + acc_sqrt_cov * noise_acc / sqrt( imu_timestep ) + acc_bias_;

    // gyro_bias update
    Eigen::Vector3d noise_gyro_bias(noise(generator_),noise(generator_),noise(generator_));
    gyro_bias_ += gyro_bias_sigma * sqrt(imu_timestep ) * noise_gyro_bias;
    imu_gyro_bias = gyro_bias_;

    // acc_bias update
    Eigen::Vector3d noise_acc_bias(noise(generator_),noise(generator_),noise(generator_));
    acc_bias_ += acc_bias_sigma * sqrt(imu_timestep ) * noise_acc_bias;
    imu_acc_bias = acc_bias_;

}

// 给激光雷达加运动畸变
// 1.计算点的time
// 2.计算time对应的畸变点
// 3.发布畸变点云
void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{   
    pcl::PointCloud<PointType>::Ptr laserCloudIn(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr Cloud_distort(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);

    float  horizonAngle;
    size_t columnIdn, cloudSize; 
    PointType thisPoint;

    cloudSize = laserCloudIn->points.size();
    double timeScanstart = laserCloudMsg->header.stamp.toSec();

    // 计算帧首对应的位姿affine_transform1
    //t_pose = 当前帧开始时间-控制载体运动开始时间
    double t_pose = timeScanstart-t_start;
    Eigen::Vector3d position1;
    Eigen::Vector3d eulerAngles1;
    Eigen::Matrix3d Rwb1;
    eulerAngles1 = Eigen::Vector3d(k_roll * sin(Kr*t_pose) , k_pitch * sin(Kr*t_pose) , sin(Ke*t_pose ) );   // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
    position1=Eigen::Vector3d( ellipse_x * sin( K * t_pose) , ellipse_y * sin( K * t_pose/2) ,    0.5*sin( K * t_pose*2)+1);
    Rwb1 = euler2Rotation(eulerAngles1);
    Eigen::Affine3d affine_transform1 = Eigen::Affine3d::Identity();
    affine_transform1.linear() = Rwb1;
    affine_transform1.translation() = position1;

    // 遍历点云，根据点时间增加畸变
    for (size_t i = 0; i < cloudSize; ++i){

        thisPoint.x = laserCloudIn->points[i].x;
        thisPoint.y = laserCloudIn->points[i].y;
        thisPoint.z = laserCloudIn->points[i].z;

        horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
        static float ang_res_x = 360.0/float(Horizon_SCAN);
        columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
        if (columnIdn >= Horizon_SCAN)
            columnIdn -= Horizon_SCAN;

        if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
            continue;
    
        
        // 根据扫描时间计算相对于初始扫描时刻的位姿
        float t_distort = columnIdn/Horizon_SCAN*0.1+t_pose;
        Eigen::Vector3d position2;
        Eigen::Vector3d eulerAngles2;
        Eigen::Matrix3d Rwb2;
        eulerAngles2 = Eigen::Vector3d(k_roll * sin(Kr*t_distort) , k_pitch * sin(Kr*t_distort) , sin(Ke*t_distort ) );   // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
        position2 =Eigen::Vector3d( ellipse_x * sin( K * t_distort) , ellipse_y * sin( K * t_distort/2) ,    0.5*sin( K * t_distort*2)+1);
        Rwb2 = euler2Rotation(eulerAngles2);
        Eigen::Affine3d affine_transform2 = Eigen::Affine3d::Identity();
        affine_transform2.linear() = Rwb2;
        affine_transform2.translation() = position2;
        Eigen::Affine3d affine_transform;
        affine_transform = affine_transform2.inverse()*affine_transform1;

        Eigen::Vector3d originalPoint;
        originalPoint.x() = laserCloudIn->points[i].x;
        originalPoint.y() = laserCloudIn->points[i].y;
        originalPoint.z() = laserCloudIn->points[i].z;
        Eigen::Vector3d transformedPoint;
        transformedPoint = affine_transform * originalPoint;
        // Now transformedPoint contains the transformed coordinates (x', y', z')
        // Optionally, you can update the original point with the transformed coordinates
        thisPoint.x = transformedPoint.x();
        thisPoint.y = transformedPoint.y();
        thisPoint.z = transformedPoint.z();
        thisPoint.intensity = laserCloudIn->points[i].intensity;
        thisPoint.ring = laserCloudIn->points[i].ring;
        thisPoint.time = columnIdn/Horizon_SCAN*0.1;

        Cloud_distort->push_back(thisPoint);
    
    }
    sensor_msgs::PointCloud2 laserCloudTemp;
    pcl::toROSMsg(*Cloud_distort, laserCloudTemp);
    laserCloudTemp.header.stamp = laserCloudMsg->header.stamp;
    laserCloudTemp.header.frame_id = "base_link";
    pubCloud_distort.publish(laserCloudTemp);
}

int main(int argc, char **argv){
    ros::init(argc,argv,"sin_trj");
    ros::NodeHandle n;
    ros::Rate loop_rate(200);
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state",10);
    ros::Subscriber sub_link_state = n.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states",100,link_states_callback);
    ros::Subscriber subLaserCloud = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 5, &cloudHandler, ros::TransportHints().tcpNoDelay());
    ros::ServiceClient root_link_state_client = n.serviceClient<gazebo_msgs::GetLinkState>("gazebo/get_link_state"); 
    ros::Publisher pub_imu_info =  n.advertise<sensor_msgs::Imu>("imu_new", 5);
    pubCloud_distort = n.advertise<sensor_msgs::PointCloud2> ("/cloud_distort", 1);
    double t,delta_t,flag;
    flag = 0;
    double t_now;

    // float ellipse_x = 2; 
    // float ellipse_y = 1; 
    // float z = 0.5; // z轴做sin运动 
    // float K1 = 5; // z轴的正弦频率是x，y的k1倍 
    // float K = M_PI/ 10; // 2 
    // float Ke = M_PI/ 6; 
    // double Kr = M_PI/ 4; 
    // double k_roll = 0.6; 
    // double k_pitch = 0.6; 
    // double k_yaw = 0.6; 

    
    Eigen::Vector3d gn (0,0,-9.8);                                   //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
    Eigen::Vector3d position;
    Eigen::Vector3d eulerAngles;
    Eigen::Matrix3d Rwb;
    Eigen::Quaterniond quat;
    Eigen::Vector3d imu_acc;
    Eigen::Vector3d ddp,dp;
    position = Eigen::Vector3d(0,0,1);
    ddp = Eigen::Vector3d(0,0,0);
    dp = Eigen::Vector3d(0,0,0);
    Eigen::Vector3d eulerAnglesRates;
    Eigen::Vector3d imu_gyro;
    double lidar_flag = 0;
    
    while(ros::ok()){   
                        t_now =  ros::Time::now().toSec();
                        std::cout << "t_now" << t <<std::endl;
                        t = t_now-t_start;
                        // std::cout << "t" << t <<std::endl;
                        if (flag<5){
                                    t_start=  ros::Time::now().toSec();
                                    std::cout << "t_start" << t_start <<std::endl;
                                    loop_rate.sleep();
                                    flag ++;
                                    continue;
                        }
                        //先旋转，后旋转和平移
                        eulerAngles = Eigen::Vector3d(k_roll * sin(Kr*t) , k_pitch * sin(Kr*t) , sin(Ke*t ) );   // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
                        eulerAnglesRates = Eigen::Vector3d(k_roll *Kr* cos(Kr* t) , k_pitch *Kr* cos(Kr* t) , Ke * cos(Ke*t));      // euler angles 的导数   
                        Rwb = euler2Rotation(eulerAngles);         // body frame to world frame
                        imu_gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates;   //  euler rates trans to body gyro
                        position=Eigen::Vector3d( ellipse_x * sin( K * t) , ellipse_y * sin( K * t/2) ,    0.5*sin( K * t*2)+1);
                        dp=Eigen::Vector3d( K * ellipse_x * cos(K*t),  K * ellipse_y * cos(K*t/2)*0.5, 0.5*K * cos(K*t*2)*2);              // position导数　in world frame
                        double K2 = K*K;
                        ddp=Eigen::Vector3d( -K2 * ellipse_x * sin(K*t),  -K2 * ellipse_y * sin(K*t/2)*0.25,  -0.5*K2 * sin(K*t*2)*4); 
                        
                        imu_acc = Rwb.transpose() * ( ddp -  gn );  //  Rbw * Rwn * gn = gs
                        quat = Eigen::Quaterniond(Rwb);
                        addIMUnoise(imu_gyro,imu_acc);
                        std::random_device rd;
                        std::default_random_engine generator_(rd());
                        std::normal_distribution<double> noise(0.0, 1.0);
                        Eigen::Vector3d noise_gyro(noise(generator_),noise(generator_),noise(generator_));
                        Eigen::Matrix3d gyro_sqrt_cov = gyro_noise_sigma * Eigen::Matrix3d::Identity();
                        imu_gyro = imu_gyro + gyro_sqrt_cov * noise_gyro / sqrt( imu_timestep ) + gyro_bias_ + constant_gyro_bias;
                        Eigen::Vector3d noise_acc(noise(generator_),noise(generator_),noise(generator_));
                        Eigen::Matrix3d acc_sqrt_cov = acc_noise_sigma * Eigen::Matrix3d::Identity();
                        imu_acc = imu_acc + acc_sqrt_cov * noise_acc / sqrt( imu_timestep ) + acc_bias_ + constant_acc_bias;
                        // gyro_bias update
                        Eigen::Vector3d noise_gyro_bias(noise(generator_),noise(generator_),noise(generator_));
                        gyro_bias_ += gyro_bias_sigma * sqrt(imu_timestep ) * noise_gyro_bias;
                        imu_gyro_bias = gyro_bias_;
                        // acc_bias update
                        Eigen::Vector3d noise_acc_bias(noise(generator_),noise(generator_),noise(generator_));
                        acc_bias_ += acc_bias_sigma * sqrt(imu_timestep ) * noise_acc_bias;
                        imu_acc_bias = acc_bias_;
                        //
                        //generate trajectory by using link_state client
                        gazebo_msgs::SetLinkState objstate;
                        objstate.request.link_state.link_name = "base_link";
                        objstate.request.link_state.pose.position.x = position[0];
                        objstate.request.link_state.pose.position.y = position[1];
                        objstate.request.link_state.pose.position.z = position[2];
                        objstate.request.link_state.pose.orientation.w = quat.w(); 
                        objstate.request.link_state.pose.orientation.x = quat.x();
                        objstate.request.link_state.pose.orientation.y = quat.y();
                        objstate.request.link_state.pose.orientation.z = quat.z();
                        objstate.request.link_state.reference_frame = "world";
                        client.call(objstate);
                        //simulate IMU info
                        sensor_msgs::Imu imu_info;
                        imu_info.header.stamp = ros::Time().fromSec(t_now);
                        imu_info.angular_velocity.x = imu_gyro[0];
                        imu_info.angular_velocity.y = imu_gyro[1];
                        imu_info.angular_velocity.z = imu_gyro[2];
                        imu_info.linear_acceleration.x = imu_acc[0];
                        imu_info.linear_acceleration.y = imu_acc[1];
                        imu_info.linear_acceleration.z = imu_acc[2];
                        imu_info.angular_velocity_covariance[0] = dp[0];
                        imu_info.angular_velocity_covariance[1] = dp[1];
                        imu_info.angular_velocity_covariance[2] = dp[2];
                        imu_info.angular_velocity_covariance[3] = position[0];
                        imu_info.angular_velocity_covariance[4] = position[1];
                        imu_info.angular_velocity_covariance[5] = position[2];
                        imu_info.angular_velocity_covariance[6] = eulerAngles[0];
                        imu_info.angular_velocity_covariance[7] = eulerAngles[1];
                        imu_info.angular_velocity_covariance[8] = eulerAngles[2];
                        imu_info.orientation.w = quat.w();
                        imu_info.orientation.x = quat.x();
                        imu_info.orientation.y = quat.y();
                        imu_info.orientation.z = quat.z();
                        pub_imu_info.publish(imu_info);

                        loop_rate.sleep();
                  ros::spinOnce();  // 处理消息并触发回调函数
            


    }

        return 0;
}
