//#include "utility.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/pcl_config.h>


// rslidar和velodyne的格式有微小的区别
// rslidar的点云格式
struct RsPointXYZIRT {
    PCL_ADD_POINT4D;
    uint8_t intensity;
    uint16_t ring = 0;
    double timestamp = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)
                                          (uint16_t, ring, ring)(double, timestamp, timestamp))

// velodyne的点云格式
struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                           (uint16_t, ring, ring)(float, time, time)
)


struct PointXYZIRADT
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  float azimuth;
  float distance;
  uint8_t return_type;
  double time_stamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;


POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZIRADT,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(
    float, azimuth, azimuth)(float, distance, distance)(std::uint8_t, return_type, return_type)(double, time_stamp, time_stamp))


class RsToVelodyne : public rclcpp::Node
{
  public:
    RsToVelodyne()
    : Node("ros_to_velodyne_ros2")
    {
        using std::placeholders::_1;

        // subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //         "/sensing/lidar/top/pointcloud_raw_ex", rclcpp::QoS{1}, std::bind(&RsToVelodyne::rsHandler_XYZIRADT, this, _1));

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/sensing/lidar/top/pointcloud_raw_ex", 
                    rclcpp::SensorDataQoS().keep_last(5),
                std::bind(&RsToVelodyne::rsHandler_XYZIRADT, this, _1));
    }

  private:
    void rsHandler_XYZIRADT(const sensor_msgs::msg::PointCloud2 &pc_msg) const 
    {
        pcl::PointCloud<PointXYZIRADT>::Ptr pc_in(new pcl::PointCloud<PointXYZIRADT>());
        pcl::fromROSMsg(pc_msg, *pc_in);

        for (int point_id = 0; point_id < pc_in->points.size() ; ++point_id) {
            if (has_nan(pc_in->points[point_id]))
                continue;

            float intensity      = pc_in->points[point_id].intensity;
            uint16_t ring        = pc_in->points[point_id].ring;
            float azimuth     = pc_in->points[point_id].azimuth;
            float distance    = pc_in->points[point_id].distance;
            /*uint8_t*/  uint16_t return_type = pc_in->points[point_id].return_type;
            double time_stamp  = pc_in->points[point_id].time_stamp;

            std::cout  <<   " ring : "  << ring ;
            std::cout  <<   ", azimuth : "  << azimuth;
            std::cout  <<   ", distance : "  << distance ;
            std::cout  <<   ", return_type : "  << return_type ;
            std::cout  <<   "   ---  time_stamp : "<< std::fixed << std::setprecision(12)  << time_stamp << std::endl;
            std::cout.unsetf(std::ios::fixed);
 
        }
    }


    bool has_nan(PointXYZIRADT point)  const
    {
        // remove nan point, or the feature assocaion will crash, the surf point will containing nan points
        // pcl remove nan not work normally
        // ROS_ERROR("Containing nan point!");
        #if 1
        // if (pcl_isnan(point.x) || pcl_isnan(point.y) || pcl_isnan(point.z)) {
        if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
            return true;
        } else {
            return false;
        }
        #else
            return true;
        #endif
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RsToVelodyne>());
    rclcpp::shutdown();
    return 0;

}
