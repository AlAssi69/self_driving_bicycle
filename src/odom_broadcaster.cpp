#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_msgs/msg/tf_message.hpp"

#include "rosgraph_msgs/msg/clock.hpp"

using std::placeholders::_1;

class OdomPublisher : public rclcpp::Node
{
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clkSubscription;
    rosgraph_msgs::msg::Clock clkPulse;

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr odomSubscription;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    geometry_msgs::msg::TransformStamped odomTransform;

    void handleOdomTransform(const std::shared_ptr<tf2_msgs::msg::TFMessage>);
    void handleClock(const rosgraph_msgs::msg::Clock &clkPulse) { this->clkPulse = clkPulse; }

public:
    OdomPublisher();
};

OdomPublisher::OdomPublisher() : Node("odom_publisher")
{
    tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    clkSubscription = this->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock",
        rclcpp::QoS(RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT).best_effort(),
        std::bind(&OdomPublisher::handleClock, this, _1));

    odomSubscription = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/bicycle_controller/tf_odometry",
        10,
        std::bind(&OdomPublisher::handleOdomTransform, this, _1));
}

void OdomPublisher::handleOdomTransform(const std::shared_ptr<tf2_msgs::msg::TFMessage> tfMsg)
{
    odomTransform.header.stamp.sec = clkPulse.clock.sec;
    odomTransform.header.stamp.nanosec = clkPulse.clock.nanosec;
    odomTransform.header.frame_id = tfMsg->transforms.data()->header.frame_id;

    odomTransform.child_frame_id = tfMsg->transforms.data()->child_frame_id;

    odomTransform.transform.translation.x = tfMsg->transforms.data()->transform.translation.x;
    odomTransform.transform.translation.y = tfMsg->transforms.data()->transform.translation.y;
    odomTransform.transform.translation.z = 0;

    odomTransform.transform.rotation.x = 0;
    odomTransform.transform.rotation.y = 0;
    odomTransform.transform.rotation.z = tfMsg->transforms.data()->transform.rotation.z;
    odomTransform.transform.rotation.w = tfMsg->transforms.data()->transform.rotation.w;

    tfBroadcaster->sendTransform(odomTransform);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
    return 0;
}