#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <actuator_msgs/msg/actuators.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std::placeholders;

class QuadController : public rclcpp_lifecycle::LifecycleNode {
public:
    QuadController();

private:
    // private methods
    void m_timer_actuators_callback();
    void m_odometry_callback(const nav_msgs::msg::Odometry& msg);

    // actuators
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr m_pub_actuators;
    double m_dt;
    rclcpp::TimerBase::SharedPtr m_timer_actuators;
    actuator_msgs::msg::Actuators m_msg_actuators;

    // odometry
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_sub_odometry;
};

QuadController::QuadController()
    : LifecycleNode("quad_controller")
{
    // parameters
    this->declare_parameter("dt", 0.005);

    // actuators
    m_pub_actuators = this->create_publisher<actuator_msgs::msg::Actuators>("actuators", 10);
    m_dt = this->get_parameter("dt").as_double();
    m_timer_actuators = rclcpp::create_timer(this, this->get_clock(),
        std::chrono::nanoseconds(int64_t(1e9 * m_dt)),
        std::bind(&QuadController::m_timer_actuators_callback, this));
    m_msg_actuators.velocity = { 0, 0, 0, 0 };

    // odometry
    m_sub_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&QuadController::m_odometry_callback, this, _1));
}

void QuadController::m_timer_actuators_callback()
{
    m_msg_actuators.header.stamp = this->get_clock()->now();
    m_msg_actuators.velocity[0] = 1.0;
    m_msg_actuators.velocity[1] = 2.0;
    m_msg_actuators.velocity[2] = 3.0;
    m_msg_actuators.velocity[3] = 4.0;
    m_pub_actuators->publish(m_msg_actuators);
}

void QuadController::m_odometry_callback(const nav_msgs::msg::Odometry& msg)
{
    (void)msg;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    auto node = std::make_shared<QuadController>();
    exe.add_node(node->get_node_base_interface());
    exe.spin();
    rclcpp::shutdown();
    return 0;
}

// vi: ts=4 sw=4 et
