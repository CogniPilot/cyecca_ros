#include <chrono>

#include <actuator_msgs/msg/actuators.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

using namespace std::placeholders;

class QuadSimulator : public rclcpp_lifecycle::LifecycleNode {
public:
    QuadSimulator();

private:
    // private methods
    void m_timer_clock_callback();
    void m_update_dynamics();
    void m_timer_imu_callback();
    void m_timer_gnss_callback();
    void m_timer_mag_callback();
    void m_actuators_callback(const actuator_msgs::msg::Actuators& msg);

    // clock
    double m_dt;
    double m_speed;
    int m_i { 0 };
    rclcpp::TimerBase::SharedPtr m_timer_clock;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr m_pub_clock;
    rclcpp::Time m_system_time_prev;
    rosgraph_msgs::msg::Clock m_msg_clock;
    rclcpp::Clock::SharedPtr m_system_clock;

    // imu
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_pub_imu;
    double m_dt_imu;
    rclcpp::TimerBase::SharedPtr m_timer_imu;
    sensor_msgs::msg::Imu m_msg_imu;

    // gnss
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr m_pub_gnss;
    double m_dt_gnss;
    rclcpp::TimerBase::SharedPtr m_timer_gnss;
    sensor_msgs::msg::NavSatFix m_msg_gnss;

    // mag
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr m_pub_mag;
    double m_dt_mag;
    rclcpp::TimerBase::SharedPtr m_timer_mag;
    sensor_msgs::msg::MagneticField m_msg_mag;

    // control
    std::array<double, 4> m_motor_angular_velocity;
    rclcpp::Subscription<actuator_msgs::msg::Actuators>::SharedPtr m_sub_actuators;
};

QuadSimulator::QuadSimulator()
    : LifecycleNode("quad_simulator")
{
    // initialize system clock
    m_system_clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

    // parameters
    this->declare_parameter("dt", 0.005);
    this->declare_parameter("speed", 1.0);
    this->declare_parameter("dt_imu", 0.005);
    this->declare_parameter("dt_gnss", 0.1);
    this->declare_parameter("dt_mag", 0.02);
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    // clock
    m_pub_clock = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", 10);
    m_dt = this->get_parameter("dt").as_double();
    m_speed = this->get_parameter("speed").as_double();
    int64_t nanos_sleep = 1e9 * m_dt / m_speed;
    RCLCPP_INFO(this->get_logger(), "requested sim speed: %10.4f Hz", m_speed);
    RCLCPP_INFO(this->get_logger(), "sim sleep period: %ld ns", nanos_sleep);
    m_timer_clock = rclcpp::create_timer(
        this,
        m_system_clock,
        std::chrono::nanoseconds(nanos_sleep),
        std::bind(&QuadSimulator::m_timer_clock_callback, this));
    m_system_time_prev = m_system_clock->now();

    // imu
    m_pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    m_dt_imu = this->get_parameter("dt_imu").as_double();
    m_timer_imu = rclcpp::create_timer(
        this,
        this->get_clock(),
        std::chrono::nanoseconds(int64_t(1e9 * m_dt_imu)),
        std::bind(&QuadSimulator::m_timer_imu_callback, this));
    m_msg_imu.header.frame_id = "base_link";
    m_msg_imu.angular_velocity_covariance = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    };
    m_msg_imu.linear_acceleration_covariance = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    };

    // gnss
    m_pub_gnss = this->create_publisher<sensor_msgs::msg::NavSatFix>("gnss", 10);
    m_dt_gnss = this->get_parameter("dt_gnss").as_double();
    m_timer_gnss = rclcpp::create_timer(
        this,
        this->get_clock(),
        std::chrono::nanoseconds(int64_t(1e9 * m_dt_gnss)),
        std::bind(&QuadSimulator::m_timer_gnss_callback, this));
    m_msg_gnss.header.frame_id = "base_link";
    m_msg_gnss.position_covariance = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    };
    m_msg_gnss.position_covariance_type = 1; // diagonal known

    // mag
    m_pub_mag = this->create_publisher<sensor_msgs::msg::MagneticField>("mag", 10);
    m_dt_mag = this->get_parameter("dt_mag").as_double();
    m_timer_mag = rclcpp::create_timer(
        this,
        this->get_clock(),
        std::chrono::nanoseconds(int64_t(1e9 * m_dt_mag)),
        std::bind(&QuadSimulator::m_timer_mag_callback, this));
    m_msg_mag.header.frame_id = "base_link";
    m_msg_mag.magnetic_field_covariance = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    };

    // control
    m_sub_actuators = this->create_subscription<actuator_msgs::msg::Actuators>(
        "actuators", 10, std::bind(&QuadSimulator::m_actuators_callback, this, _1));
}

void QuadSimulator::m_timer_clock_callback()
{
    double t = m_i * m_dt;
    double sec = int(t);
    double nanosec = int(1e9 * (t - sec));
    m_msg_clock.clock.sec = sec;
    m_msg_clock.clock.nanosec = nanosec;
    m_pub_clock->publish(m_msg_clock);

    auto now = m_system_clock->now();
    double sim_rate = 1e9 * m_dt / (now - m_system_time_prev).nanoseconds();
    m_system_time_prev = now;
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *m_system_clock,
        1000, "sim speed:%10.3f Hz", sim_rate);

    // increment time for next loop
    m_i += 1;

    // update dynamics
    m_update_dynamics();
}

void QuadSimulator::m_update_dynamics()
{
}

void QuadSimulator::m_timer_imu_callback()
{
    m_msg_imu.header.stamp = this->get_clock()->now();
    m_msg_imu.angular_velocity.x = 1.0;
    m_msg_imu.angular_velocity.y = 2.0;
    m_msg_imu.angular_velocity.z = 3.0;
    m_pub_imu->publish(m_msg_imu);
}

void QuadSimulator::m_timer_gnss_callback()
{
    m_msg_gnss.header.stamp = this->get_clock()->now();
    m_msg_gnss.altitude = 1.0;
    m_msg_gnss.latitude = 2.0;
    m_msg_gnss.longitude = 3.0;
    m_pub_gnss->publish(m_msg_gnss);
}

void QuadSimulator::m_timer_mag_callback()
{
    m_msg_mag.header.stamp = this->get_clock()->now();
    m_msg_mag.magnetic_field.x = 1.0;
    m_msg_mag.magnetic_field.y = 2.0;
    m_msg_mag.magnetic_field.z = 3.0;
    m_pub_mag->publish(m_msg_mag);
}

void QuadSimulator::m_actuators_callback(const actuator_msgs::msg::Actuators& msg)
{
    for (int i = 0; i < 4; i++) {
        m_motor_angular_velocity[i] = msg.velocity[i];
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    auto quad_sim = std::make_shared<QuadSimulator>();
    exe.add_node(quad_sim->get_node_base_interface());
    exe.spin();
    rclcpp::shutdown();
    return 0;
}

// vi: ts=4 sw=4 et
