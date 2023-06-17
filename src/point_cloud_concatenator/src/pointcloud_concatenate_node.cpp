
// MIT License

// Copyright(c) 2023 yusufy1ld1z

// Permission is hereby granted,
// free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute,
// sublicense, and / or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

// The above copyright notice and this permission notice shall be included in all copies
// or
// substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS",
// WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER
// LIABILITY,
// WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// Author : Yusuf YILDIZ 17.06.2023

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

class PointCloudConcatenator : public rclcpp::Node
{
public:
    PointCloudConcatenator()
        : Node("point_cloud_concatenator")
    {
        // Initialize the transformation buffer
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        rclcpp::QoS qos(10);
        qos.best_effort(); // To prevent the ROS2 QoS warning.
        auto rmw_qos_profile = qos.get_rmw_qos_profile();

        // Subscribe to the point cloud topics using message_filters
        cloud_subscriber1_.subscribe(this, "/sensing/lidar/top/pointcloud_raw", rmw_qos_profile);
        cloud_subscriber2_.subscribe(this, "/sensing/lidar/left/pointcloud_raw", rmw_qos_profile);
        cloud_subscriber3_.subscribe(this, "/sensing/lidar/right/pointcloud_raw", rmw_qos_profile);

        // Synchronize the subscribers using the ApproximateTime policy
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(100),
            cloud_subscriber1_, cloud_subscriber2_, cloud_subscriber3_);

        // Set the synchronized callback function
        sync_->registerCallback(std::bind(&PointCloudConcatenator::cloudCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        // Create the publisher for the concatenated point cloud
        cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensing/lidar/concatenated_point_cloud", 10);
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg1,
                       const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg2,
                       const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg3)
    {
        try
        {
            // Transform the point clouds to the "base_link" frame
            geometry_msgs::msg::TransformStamped transform_stamped1 =
                tf_buffer_->lookupTransform("base_link", cloud_msg1->header.frame_id, cloud_msg1->header.stamp);
            geometry_msgs::msg::TransformStamped transform_stamped2 =
                tf_buffer_->lookupTransform("base_link", cloud_msg2->header.frame_id, cloud_msg2->header.stamp);
            geometry_msgs::msg::TransformStamped transform_stamped3 =
                tf_buffer_->lookupTransform("base_link", cloud_msg3->header.frame_id, cloud_msg3->header.stamp);

            sensor_msgs::msg::PointCloud2 transformed_cloud_msg1;
            sensor_msgs::msg::PointCloud2 transformed_cloud_msg2;
            sensor_msgs::msg::PointCloud2 transformed_cloud_msg3;

            tf2::doTransform(*cloud_msg1, transformed_cloud_msg1, transform_stamped1);
            tf2::doTransform(*cloud_msg2, transformed_cloud_msg2, transform_stamped2);
            tf2::doTransform(*cloud_msg3, transformed_cloud_msg3, transform_stamped3);

            // Store the transformed point clouds in the corresponding frames
            transformed_clouds_["/sensing/lidar/top/pointcloud_raw"] = transformed_cloud_msg1;
            transformed_clouds_["/sensing/lidar/left/pointcloud_raw"] = transformed_cloud_msg2;
            transformed_clouds_["/sensing/lidar/right/pointcloud_raw"] = transformed_cloud_msg3;

            // Concatenate the transformed point clouds
            pcl::PointCloud<pcl::PointXYZ> concatenated_cloud;
            for (const auto &pair : transformed_clouds_)
            {
                const sensor_msgs::msg::PointCloud2 &cloud = pair.second;
                pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
                pcl::fromROSMsg(cloud, pcl_cloud);
                concatenated_cloud += pcl_cloud;
            }
            // Convert the concatenated point cloud back to ROS message
            sensor_msgs::msg::PointCloud2 concatenated_cloud_msg;
            pcl::toROSMsg(concatenated_cloud, concatenated_cloud_msg);

            // Publish the concatenated point cloud
            concatenated_cloud_msg.header.frame_id = "base_link";
            cloud_publisher_->publish(concatenated_cloud_msg);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to transform point cloud: %s", ex.what());
        }
    }

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> SyncPolicy;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unordered_map<std::string, sensor_msgs::msg::PointCloud2> transformed_clouds_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_subscriber1_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_subscriber2_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_subscriber3_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudConcatenator>());
    rclcpp::shutdown();
    return 0;
}
