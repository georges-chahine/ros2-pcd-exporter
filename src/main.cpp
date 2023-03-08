#define BOOST_BIND_NO_PLACEHOLDERS
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/octree/octree_search.h>
//#include <pcl_ros/point_cloud.hpp>
#include <pcl_ros/transforms.hpp>
#include <pcl/io/pcd_io.h>
//#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/transforms.h>
//#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class LidarMapper : public rclcpp::Node
{
private:

    /* ------------------------------ variables------------------------------------------------------------------------- */

    bool init;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;

    size_t count_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    Eigen::Matrix4d base_tf;
    pcl::PointCloud<pcl::PointXYZI>::Ptr rolling_cloud, rolling_cloud2;
    double leaf_size;
    Eigen::Vector3d prev_position;
    Eigen::Matrix4d map_tf;

    /* ------------------------------functions -------------------------------------------------------------------------*/

    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {


        std::string filename;
        //filename.precision(16);
        double filename_d= (double(msg->header.stamp.sec)+double(msg->header.stamp.nanosec)*1e-9);

        filename=std::to_string(filename_d);

        std::cout <<"filename is "<<filename<<" "<<filename<<std::endl;

        std::string currentPath="/home/georges/export";  // TODO: make it a rosparam

        pcl::PCLPointCloud2::Ptr pc2 (new pcl::PCLPointCloud2 ());

        pcl_conversions::toPCL(*msg,*pc2);

        std::vector<int> indices;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::fromPCLPointCloud2(*pc2,*temp_cloud);

        //pcl::removeNaNFromPointCloud(*temp_cloud, *temp_cloud, indices);

        std::cout<<"Saving "<<temp_cloud->points.size()<<" points"<<std::endl;

        temp_cloud->height=1;
        temp_cloud->width=temp_cloud->points.size();

        std::string fullPath= currentPath + "/" + filename + ".pcd" ;

        pcl::io::savePCDFileASCII (fullPath, *temp_cloud);

       // rclcpp::sleep_for(std::chrono::nanoseconds(100ms));
    }

public:
    LidarMapper()
        : Node("pcd_exporter"), count_(0)
    {


        subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    "cloud", 1, std::bind(&LidarMapper::lidar_callback, this, _1));

        base_tf=map_tf=Eigen::Matrix4d::Identity();

        rclcpp::TimerBase::SharedPtr timer_{nullptr};
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarMapper>());
    rclcpp::shutdown();
    return 0;
}
