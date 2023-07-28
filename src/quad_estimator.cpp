#include <chrono>

#include <memory>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

using namespace std::placeholders;

class QuadEstimator : public rclcpp_lifecycle::LifecycleNode {
public:
    QuadEstimator();

private:
    // private methods
    void m_timer_odom_callback();

    // mag
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_pub_odom;
    double m_dt;
    rclcpp::TimerBase::SharedPtr m_timer_odom;
    nav_msgs::msg::Odometry m_msg_odom;
};

QuadEstimator::QuadEstimator()
    : LifecycleNode("quad_estimator")
{
    // parameters
    this->declare_parameter("dt", 0.005);

    // odometry
    m_pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    m_dt = this->get_parameter("dt").as_double();
    m_timer_odom = rclcpp::create_timer(this, this->get_clock(),
        std::chrono::nanoseconds(int64_t(1e9 * m_dt)),
        std::bind(&QuadEstimator::m_timer_odom_callback, this));
    m_msg_odom.header.frame_id = "map";
    m_msg_odom.child_frame_id = "base_link";
}

void QuadEstimator::m_timer_odom_callback()
{
    m_msg_odom.header.stamp = this->get_clock()->now();
    m_msg_odom.pose.pose.position.x = 1.0;
    m_msg_odom.pose.pose.position.y = 1.0;
    m_msg_odom.pose.pose.position.z = 1.0;
    m_msg_odom.pose.pose.orientation.x = 0.0;
    m_msg_odom.pose.pose.orientation.y = 0.0;
    m_msg_odom.pose.pose.orientation.z = 0.0;
    m_msg_odom.pose.pose.orientation.w = 1.0;
    m_pub_odom->publish(m_msg_odom);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    auto node = std::make_shared<QuadEstimator>();
    exe.add_node(node->get_node_base_interface());
    exe.spin();
    rclcpp::shutdown();
    return 0;
}

// vi: ts=4 sw=4 et
