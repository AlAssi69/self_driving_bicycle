#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_msgs/msg/tf_message.hpp"

class OdomPublisher : public rclcpp::Node
{
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void handleOdomTransform(const std::shared_ptr<tf2_msgs::msg::TFMessage>);

public:
    OdomPublisher();
};

OdomPublisher::OdomPublisher() : Node("odom_publisher")
{
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/bicycle_controller/tf_odometry",
        10,
        std::bind(&OdomPublisher::handleOdomTransform, this, std::placeholders::_1));
}

void OdomPublisher::handleOdomTransform(const std::shared_ptr<tf2_msgs::msg::TFMessage> msg)
{
    geometry_msgs::msg::TransformStamped odomTransform;

    odomTransform.header.stamp = msg->transforms.data()->header.stamp;
    odomTransform.header.frame_id = msg->transforms.data()->header.frame_id;
    odomTransform.child_frame_id = msg->transforms.data()->child_frame_id;

    odomTransform.transform.translation.x = msg->transforms.data()->transform.translation.x;
    odomTransform.transform.translation.y = msg->transforms.data()->transform.translation.y;
    odomTransform.transform.translation.z = msg->transforms.data()->transform.translation.z;

    odomTransform.transform.rotation.x = msg->transforms.data()->transform.rotation.x;
    odomTransform.transform.rotation.y = msg->transforms.data()->transform.rotation.y;
    odomTransform.transform.rotation.z = msg->transforms.data()->transform.rotation.z;
    odomTransform.transform.rotation.w = msg->transforms.data()->transform.rotation.w;

    tf_broadcaster_->sendTransform(odomTransform);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
    return 0;
}