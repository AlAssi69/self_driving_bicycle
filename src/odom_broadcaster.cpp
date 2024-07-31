#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_msgs/msg/tf_message.hpp"

using std::placeholders::_1;

class OdomPublisher : public rclcpp::Node
{
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void handleOdomTransform(const std::shared_ptr<tf2_msgs::msg::TFMessage> msg)
    {
        geometry_msgs::msg::TransformStamped odomTransform;

        odomTransform.header.stamp = this->get_clock()->now();
        odomTransform.header.frame_id = "odom";
        odomTransform.child_frame_id = "base_link";

        odomTransform.transform.translation.x = msg->transforms.data()->transform.translation.x;
        odomTransform.transform.translation.y = msg->transforms.data()->transform.translation.y;
        odomTransform.transform.translation.z = 0.0;

        odomTransform.transform.rotation.x = msg->transforms.data()->transform.rotation.x;
        odomTransform.transform.rotation.y = msg->transforms.data()->transform.rotation.y;
        odomTransform.transform.rotation.w = msg->transforms.data()->transform.rotation.w;
        odomTransform.transform.rotation.z = msg->transforms.data()->transform.rotation.z;

        tf_broadcaster_->sendTransform(odomTransform);
    }

public:
    OdomPublisher() : Node("odom_publisher")
    {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/bicycle_controller/tf_odometry",
            10,
            std::bind(&OdomPublisher::handleOdomTransform, this, _1));
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
    return 0;
}