#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <fstream>
#include <cmath>
#include <iomanip>
#include <vector>
#include <string>
#include <sstream>

struct Robot {
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;

    double x = 0.0, y = 0.0, yaw = 0.0;
    bool pose_ready = false;

    double init_x = 0.0, init_y = 0.0, init_yaw = 0.0;

    double max_v = 0.5;
    double max_w = 0.6;
};

class RigidControllerNode : public rclcpp::Node {
public:
    RigidControllerNode() : Node("rigid_controller_node") {
        // leader
        robots_.push_back(Robot());
        robots_[0].pose_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot_1/odom", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg){ poseCallback(msg, 0); });
        robots_[0].cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/robot_1/cmd_vel", 10);

        // coleader
        robots_.push_back(Robot());
        robots_[1].pose_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot_2/odom", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg){ poseCallback(msg, 1); });
        robots_[1].cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/robot_2/cmd_vel", 10);
        // robots_[1].init_x = -1.0;
        // robots_[1].init_y = -0.5;

        // follower
        robots_.push_back(Robot());
        robots_[2].pose_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot_3/odom", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg){ poseCallback(msg, 2); });
        robots_[2].cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/robot_3/cmd_vel", 10);
        // robots_[2].init_x = -1.0;
        // robots_[2].init_y = 1.0;

        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&RigidControllerNode::controlLoop, this)
        );

        log_str_rows_.push_back("t,d01_err,d02_err,d12_err,o_error,o_error_deg");
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<Robot> robots_;
    std::vector<std::string> log_str_rows_;

    float akm_offset = -0.2;
    double k_coleader_ = 0.6;
    double alpha_ = 0.6;
    double k_follower_ = 3.0;
    double beta_soft_ = 0.37;
    double tanh_k_ = 3.0;

    double A_ = 0.06;
    double w_ = 0.5;
    double v_straight_ = 0.2;
    double init_time_ = 5.0;
    double leader_start_time_ = 5.0;
    double go_straight_time_ = 2.0;

    Eigen::Vector2d p_o_;
    Eigen::Vector2d dp_o_;

    double d_01_ = 0.7;
    double d_02_ = 0.7;
    double d_12_ = 0.7;

    Eigen::Vector2d u_0_, u_1_, u_2_;
    double t_global_ = 0.0;
    bool stop_signal_ = false;
    bool log_dumped_ = false;

    void poseCallback(nav_msgs::msg::Odometry::SharedPtr msg, int index) {
        double x_r = msg->pose.pose.position.x;
        double y_r = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        // yaw += M_PI;

        robots_[index].x = robots_[index].init_x + x_r + akm_offset * cos(yaw);
        robots_[index].y = robots_[index].init_y + y_r + akm_offset * sin(yaw);
        robots_[index].yaw = yaw + robots_[index].init_yaw;
        robots_[index].pose_ready = true;
    }

    Eigen::Vector2d tanhVec(const Eigen::Vector2d& v) {
        Eigen::Vector2d res;
        for (int i = 0; i < 2; ++i) {
            res[i] = std::tanh(tanh_k_ * v[i]);
        }
        return res;
    }

    void controlLoop() {
        if (!robots_[0].pose_ready || !robots_[1].pose_ready || !robots_[2].pose_ready){
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for all robots' pose...");
            return;
        }

        // print all robot pose
        RCLCPP_INFO_ONCE(this->get_logger(), "Robot 0: (%.2f, %.2f, %.2f)", robots_[0].x, robots_[0].y, robots_[0].yaw);
        RCLCPP_INFO_ONCE(this->get_logger(), "Robot 1: (%.2f, %.2f, %.2f)", robots_[1].x, robots_[1].y, robots_[1].yaw);
        RCLCPP_INFO_ONCE(this->get_logger(), "Robot 2: (%.2f, %.2f, %.2f)", robots_[2].x, robots_[2].y, robots_[2].yaw);

        t_global_ += 0.02; // simple timer approximation

        if (t_global_ < init_time_) {
            u_0_ = {0.0, 0.0};
            u_1_ = {0.0, 0.0};
            u_2_ = {0.0, 0.0};
        } else {
            getLeaderCmd();
            getColeaderCmd();
            getFollowerCmd();
        }

        publishCmd(0, u_0_);
        publishCmd(1, u_1_);
        publishCmd(2, u_2_);

        record_to_log_str();
    }

    void record_to_log_str() {
        double e_01 = std::abs(std::hypot(robots_[0].x - robots_[1].x,
                                          robots_[0].y - robots_[1].y) - d_01_);
        double e_02 = std::abs(std::hypot(robots_[0].x - robots_[2].x,
                                          robots_[0].y - robots_[2].y) - d_02_);
        double e_12 = std::abs(std::hypot(robots_[1].x - robots_[2].x,
                                          robots_[1].y - robots_[2].y) - d_12_);

        Eigen::Vector2d p_01(robots_[0].x - robots_[1].x,
                             robots_[0].y - robots_[1].y);
        double o_err = (p_01-p_o_).norm();
        double angle1 = std::atan2(p_01.y(), p_01.x());
        double angle2 = std::atan2(p_o_.y(), p_o_.x());
        double o_err_rad = std::abs(angle1 - angle2);
        double o_err_deg = o_err_rad * 180.0 / M_PI;

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << t_global_ << ",";
        oss << std::fixed << std::setprecision(4)
            << e_01 << "," << e_02 << "," << e_12 << "," << o_err << ",";
        oss << std::fixed << std::setprecision(2) << o_err_deg;

        log_str_rows_.push_back(oss.str());

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "T:%.2f | Err | d01: %.3f | d02: %.3f | d12: %.3f | o: %.3f | o_deg: %.2f", 
            t_global_, e_01, e_02, e_12, o_err, o_err_deg);

        if (stop_signal_ && !log_dumped_) {
            dump_log("fmc.log");
            log_dumped_ = true;
        }
    }

    void dump_log(const std::string& filename) {
        std::ofstream log_file(filename);
        if (!log_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open log file: %s", filename.c_str());
            return;
        }
        for (const auto& row : log_str_rows_) {
            log_file << row << "\n";
        }
        log_file.close();
        RCLCPP_INFO(this->get_logger(), "Log file saved: %s", filename.c_str());
    }

    void getLeaderCmd() {
        if (robots_[0].x > 4.0) {
            u_0_ = {0.0, 0.0};
            dp_o_ = {0.0, 0.0};
            stop_signal_ = true;
            return;
        }

        if (t_global_ < leader_start_time_) {
            p_o_ = {d_01_, 0.0};
            dp_o_ = {0.0, 0.0};
            u_0_ = {0.0, 0.0};
            return;
        } else if (t_global_ < leader_start_time_ + go_straight_time_) {
            u_0_ = {v_straight_, 0.0};
            p_o_ = {d_01_, 0.0};
            dp_o_ = {0.0, 0.0};
            return;
        } else {
            double sin_start_time = leader_start_time_ + go_straight_time_;
            double vx = v_straight_;
            double vy = -A_ * sin(w_ * (t_global_ - sin_start_time));
            double dvx = 0.0;
            double dvy = -A_ * w_ * cos(w_ * (t_global_ - sin_start_time));

            u_0_ = {vx, vy};
            double theta = std::atan2(vy, vx);
            double dtheta = (vx*dvy - vy*dvx)/(vx*vx + vy*vy);

            p_o_ = {d_01_ * cos(theta), d_01_ * sin(theta)};
            dp_o_ = {-d_01_ * sin(theta)*dtheta, d_01_ * cos(theta)*dtheta};
        }
    }

    void getColeaderCmd() {
        Eigen::Vector2d p_01(robots_[0].x - robots_[1].x, robots_[0].y - robots_[1].y);
        Eigen::Vector2d p_o_bar = p_01 - p_o_;

        Eigen::Vector2d p_10 = -p_01;
        Eigen::Vector2d p_12(robots_[1].x - robots_[2].x, robots_[1].y - robots_[2].y);
        double d_12 = p_12.norm();
        double sigma_10 = p_10.squaredNorm() - d_01_*d_01_;
        double sigma_12 = p_12.squaredNorm() - d_12_*d_12_;
        Eigen::Vector2d r1 = sigma_10 * p_10;

        double eta = alpha_ * (p_o_bar.dot(dp_o_)) / std::pow((r1 - alpha_ * p_o_bar).norm(), 2);
        u_1_ = -(k_coleader_ - eta) * (r1 - alpha_ * p_o_bar) + u_0_;
    }

    void getFollowerCmd() {
        Eigen::Vector2d p_20(robots_[2].x - robots_[0].x, robots_[2].y - robots_[0].y);
        Eigen::Vector2d p_21(robots_[2].x - robots_[1].x, robots_[2].y - robots_[1].y);
        Eigen::Vector2d r2 = (p_20.squaredNorm() - d_02_*d_02_)*p_20 +
                             (p_21.squaredNorm() - d_12_*d_12_)*p_21;

        double k_f = k_follower_;
        if (t_global_ < leader_start_time_ + 2.0) k_f = 0.5;

        u_2_ = -k_f*r2 - beta_soft_*tanhVec(r2);
    }

    void publishCmd(int index, const Eigen::Vector2d& v) {
        geometry_msgs::msg::Twist twist;

        double vx_local = cos(robots_[index].yaw)*v[0] + sin(robots_[index].yaw)*v[1];
        double vy_local = -sin(robots_[index].yaw)*v[0] + cos(robots_[index].yaw)*v[1];
        double omega = vy_local / akm_offset;

        if (vx_local > robots_[index].max_v) vx_local = robots_[index].max_v;
        if (vx_local < -robots_[index].max_v) vx_local = -robots_[index].max_v;
        if (omega > robots_[index].max_w) omega = robots_[index].max_w;
        if (omega < -robots_[index].max_w) omega = -robots_[index].max_w;

        twist.linear.x = vx_local;
        twist.angular.z = omega;
        robots_[index].cmd_pub->publish(twist);
    }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RigidControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
