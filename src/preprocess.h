#ifndef PREPROCESS_H
#define PREPROCESS_H


#include <pcl_conversions/pcl_conversions.h>

#ifdef USE_ROS1
>>>>>>> f2b7c87427cad0f41ec34381e229d73df25013b0
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver2/CustomMsg.h>
#elif defined(USE_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif


#ifdef USE_ROS1
using LivoxCustomMsgConstPtr = livox_ros_driver2::CustomMsg::ConstPtr;
using Pcl2MsgConstPtr = sensor_msgs::PointCloud2::ConstPtr;
using TimeType = ros::Time;
using PointCloud2Msg = sensor_msgs::PointCloud2;
using Pcl2Publisher = ros::Publisher;
#elif defined(USE_ROS2)
using LivoxCustomMsgConstPtr = livox_ros_driver2::msg::CustomMsg::ConstPtr;
using Pcl2MsgConstPtr = sensor_msgs::msg::PointCloud2::ConstPtr;
using TimeType = rclcpp::Time;
using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
using Pcl2Publisher = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;
#endif

using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
enum LID_TYPE{AVIA = 1, VELO16, OUST64, AIRY, MARSIM}; //{1, 2, 3}
enum TIME_UNIT{SEC = 0, MS = 1, US = 2, NS = 3};
enum Feature{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};
enum Surround{Prev, Next};
enum E_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};


enum TIME_UNIT
{
  SEC = 0,
  MS = 1,
  US = 2,
  NS = 3
};
// 枚举类型：表示点云中特征点的类型
enum Feature
{
  Nor,
  Poss_Plane,
  Real_Plane,
  Edge_Jump,
  Edge_Plane,
  Wire,
  ZeroPoint
};
// 枚举类型：位置标识（前一个、后一个）
enum Surround
{
  Prev,
  Next
};
// 枚举类型：表示有跨越边的类型
enum E_jump
{
  Nr_nor,
  Nr_zero,
  Nr_180,
  Nr_inf,
  Nr_blind
};

// 用于存储激光雷达点的一些其他属性
struct orgtype
{
  double range; // 点云在xy平面距离雷达中心的距离
  double dista; // 当前点与后一个点的距离
  // 假设雷达原点为O 前一个点为M 当前点为A 后一个点为N
  double angle[2];  // 这个是角OAM和角OAN的cos值
  double intersect; // 这个是角MAN的cos值
  E_jump edj[2];    // 前后两点的类型
  Feature ftype;    // 点类型
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};
// velodyne数据结构
namespace velodyne_ros
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;                // 4D点坐标类型
    float intensity;                // 强度
    float time;                     // 时间戳
    uint16_t ring;                  // 点所属的圈数
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 进行内存对齐
  };
} // namespace velodyne_ros
// 注册velodyne点云类型
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, time, time)(uint16_t, ring, ring))

// ouster数据结构
namespace ouster_ros
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;                // 4D点坐标类型
    float intensity;                // 强度
    uint32_t t;                     // 时间戳
    uint16_t reflectivity;          // 反射率
    uint8_t ring;                   // 点所属的圈数
    uint16_t ambient;               // 环境光,没用到
    uint32_t range;                 // 距离
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 进行内存对齐
  };
} // namespace ouster_ros

// clang-format off
//注册ouster Point 类型
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)


namespace robosense_ros { // RoboSense pointcloud registration
  struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

POINT_CLOUD_REGISTER_POINT_STRUCT(robosense_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
    (double, timestamp, timestamp)
)
class Preprocess
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();

  
  void process(const LivoxCustomMsgConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void process(const Pcl2MsgConstPtr &msg, PointCloudXYZI::Ptr &pcl_out); 
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf;// 全部点、边缘点、平面点
  PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
  vector<orgtype> typess[128]; //maximum 128 line lidar
  float time_unit_scale;

  int lidar_type, point_filter_num, N_SCANS, SCAN_RATE, time_unit;
  double blind;
  bool feature_enabled, given_offset_time;

    
  Pcl2Publisher pub_full, pub_surf, pub_corn;

  private:

  void avia_handler(const LivoxCustomMsgConstPtr &msg);
  void oust64_handler(const Pcl2MsgConstPtr &msg);
  void velodyne_handler(const Pcl2MsgConstPtr &msg);
  void airy_handler(const Pcl2MsgConstPtr& msg);
  void sim_handler(const Pcl2MsgConstPtr &msg);

  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  void pub_func(const Pcl2Publisher& pub, PointCloudXYZI &pl, const TimeType &ct);
  int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
    //判断小平面，没有用到
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);
  
  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;//两个小区域（small patch）的相交面积,比例阈值
  double vx, vy, vz;
};

#endif