#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cmath>

using namespace std;

const double WHEEL_RADIUS = 0.2525;
const double L_R = 0.765;
const double L_F = 0.765;

class ExtendedKalmanFilterNode : public rclcpp::Node
{
public:
    ExtendedKalmanFilterNode() : Node("motion_update_node")
    {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            std::bind(&ExtendedKalmanFilterNode::imuCallback, this, std::placeholders::_1));

        wheel_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/wheel_speed_avg", 10,
            std::bind(&ExtendedKalmanFilterNode::wheelSpeedCallback, this, std::placeholders::_1));

        track_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/ground_truth/track/viz", 10,
            std::bind(&ExtendedKalmanFilterNode::trackCallback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/car_marker", 10);

        cone_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/ground_truth_cones", 10,
            std::bind(&ExtendedKalmanFilterNode::measurementCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Motion update node started. Waiting for messages...");

        X_ << 0.0, 0.0, 0.0;

        P_ = Eigen::Matrix3d::Identity() * 0.1;
        Q_ = Eigen::Matrix3d::Identity() * 0.01;
        R_ = Eigen::Matrix2d::Identity() * 0.5;
    }

private:
    rclcpp::Time last_imu_time_, last_wheel_time_;
    bool first_imu_ = true, first_wheel_ = true;
    double last_v_ = 0.0;
    Eigen::Vector3d X_;

    Eigen::Matrix3d P_;

    Eigen::Matrix3d Q_;

    Eigen::Matrix2d R_;

    std::vector<std::pair<double, double>> cone_positions_;
    bool track_received_ = false;
    std::vector<std::pair<double, double>> gt_cones_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr wheel_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr track_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr cone_sub_;

    void publishArrowHead()
    {
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = "map";
        arrow.header.stamp = this->get_clock()->now();
        arrow.ns = "car_pose";
        arrow.id = 0;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;

        arrow.pose.position.x = X_(0);
        arrow.pose.position.y = X_(1);
        arrow.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, X_(2));
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

        double v= last_v_;

        double psi = X_(2);

        
    Eigen::Matrix3d F;
    F << 1, 0, -v * sin(psi) * dt,
         0, 1,  v * cos(psi) * dt,
         0, 0,  1;

    P_ = F * P_ * F.transpose() + Q_;

    X_(2) += omega * dt;


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

        last_v_ = v;

        double psi = X_(2);

        
        Eigen::Matrix3d F;

        F << 1, 0, -v * sin(X_(2)) * dt,
            0, 1, v * cos(X_(2)) * dt,
            0, 0, 1;

        X_(0) += v * cos(psi) * dt;
        X_(1) += v * sin(psi) * dt;


        P_ = F * P_ * F.transpose() + Q_;

        last_wheel_time_ = current_time;
        publishArrowHead();
    }
    void measurementCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        gt_cones_.clear();
    for (auto &marker : msg->markers)
    {
        gt_cones_.emplace_back(marker.pose.position.x, marker.pose.position.y);
    }
        if (gt_cones_.empty())
            return;
        double x = X_(0);
        double y = X_(1);
        double psi = X_(2);
        
         for (auto &cone : gt_cones_){
        double Cx = cone.first;
        double Cy = cone.second;
       
        double dx = Cx - x;
        double dy = Cy - y;

        Eigen::Vector2d z;

        z(0) = cos(psi) * dx + sin(psi) * dy;
        z(1) = -sin(psi) * dx + cos(psi) * dy;

        //z(0) += 0.1;
        //z(1) += -0.1;

        Eigen::Vector2d z_pred;

        z_pred(0) = cos(psi) * dx + sin(psi) * dy;
        z_pred(1) = -sin(psi) * dx + cos(psi) * dy;

        
      
        Eigen::Vector2d y_k;
        y_k = z - z_pred;

        Eigen::Matrix<double, 2, 3> H;

        H(0, 0) = -cos(psi);
        H(0, 1) = -sin(psi);
        H(0, 2) = -sin(psi) * dx + cos(psi) * dy;

        H(1, 0) = sin(psi);
        H(1, 1) = -cos(psi);
        H(1, 2) = -cos(psi) * dx - sin(psi) * dy;

        Eigen::Matrix2d S;
        S = H * P_ * H.transpose() + R_;

        Eigen::Matrix<double, 3, 2> K;
        K = P_ * H.transpose() * S.ldlt().solve(Eigen::Matrix2d::Identity());

        std::cout << "Kalman Gain K:\n"
                  << K << std::endl;

        X_ = X_ + K * y_k;

        x   = X_(0);
        y   = X_(1);
        psi = X_(2);


        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        P_ = (I - K * H) * P_;

        while (X_(2) > M_PI)
            X_(2) -= 2 * M_PI;
        while (X_(2) < -M_PI)
            X_(2) += 2 * M_PI;
         

        RCLCPP_INFO(this->get_logger(),
                    "EKF Update → x: %.2f, y: %.2f, psi: %.2f",
                    X_(0), X_(1), X_(2));

        publishArrowHead();

        cout << "Z:\n"
             << z << endl;
        cout << "Z_pred:\n"
             << z_pred << endl;
        cout << "Innovation y:\n"
             << y_k << endl;
        cout << "S:\n"
             << S << endl;
        cout << "K:\n"
             << K << endl;
         }
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
    rclcpp::spin(std::make_shared<ExtendedKalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}
