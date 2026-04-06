#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>

const double WHEEL_RADIUS = 0.2525;
const double L_R = 0.765;
const double L_F = 0.765;

class MotionUpdateNode : public rclcpp::Node
{
public:
    MotionUpdateNode() : Node("motion_update_node")
    {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            std::bind(&MotionUpdateNode::imuCallback, this, std::placeholders::_1));

        wheel_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/wheel_speed_avg", 10,
            std::bind(&MotionUpdateNode::wheelSpeedCallback, this, std::placeholders::_1));

        track_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/ground_truth/track/viz", 10,
            std::bind(&MotionUpdateNode::trackCallback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/car_marker", 10);

        RCLCPP_INFO(this->get_logger(), "Motion update node started. Waiting for messages...");
    }

private:
    double x_ = 0.0, y_ = 0.0, psi_ = 0.0;
    rclcpp::Time last_imu_time_, last_wheel_time_;
    bool first_imu_ = true, first_wheel_ = true;

    std::vector<std::pair<double, double>> cone_positions_;
    bool track_received_ = false;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr wheel_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr track_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    void publishArrowHead()
    {
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = "map";
        arrow.header.stamp = this->get_clock()->now();
        arrow.ns = "car_pose";
        arrow.id = 0;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;

        arrow.pose.position.x = x_;
        arrow.pose.position.y = y_;
        arrow.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, psi_);
        arrow.pose.orientation.x = q.x();
        arrow.pose.orientation.y = q.y();
        arrow.pose.orientation.z = q.z();
        arrow.pose.orientation.w = q.w();

        arrow.scale.x = 1.0;
        arrow.scale.y = 0.2;
        arrow.scale.z = 0.2;

        arrow.color.r = 0.0f;
        arrow.color.g = 0.0f;
        arrow.color.b = 1.0f;
        arrow.color.a = 1.0f;

        marker_pub_->publish(arrow);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        rclcpp::Time current_time = this->get_clock()->now();
        if (first_imu_)
        {
            last_imu_time_ = current_time;
            first_imu_ = false;
            return;
        }

        double dt = (current_time - last_imu_time_).seconds();
        double omega = msg->angular_velocity.z;
        psi_ += omega * dt;
        last_imu_time_ = current_time;
        publishArrowHead();

        RCLCPP_INFO(this->get_logger(), "IMU received! omega: %f", msg->angular_velocity.z);
    }

    void wheelSpeedCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        rclcpp::Time current_time = this->get_clock()->now();
        if (first_wheel_)
        {
            last_wheel_time_ = current_time;
            first_wheel_ = false;
            return;
        }

        double dt = (current_time - last_wheel_time_).seconds();
        double rpm = msg->data;
        double v = rpm * (2.0 * M_PI * WHEEL_RADIUS) / 60.0;

        x_ += v * cos(psi_) * dt;
        y_ += v * sin(psi_) * dt;

        last_wheel_time_ = current_time;
        publishArrowHead();
    }

    void trackCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        (void)msg;
        RCLCPP_INFO_ONCE(this->get_logger(),
                         "Track topic received — add /ground_truth/track/viz directly in RViz");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionUpdateNode>());
    rclcpp::shutdown();
    return 0;
}
