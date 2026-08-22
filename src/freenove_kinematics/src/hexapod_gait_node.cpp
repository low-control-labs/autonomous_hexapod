#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <std_msgs/msg/float64_multi_array.hpp>

class HexapodGaitNode : public rclcpp::Node {
public:
    HexapodGaitNode() : Node("hexapod_gait_node"), gait_phase_(0), head_pan_rad_(0.0), head_tilt_rad_(0.0) {
        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&HexapodGaitNode::cmdVelCallback, this, std::placeholders::_1));

        sub_cmd_head_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/cmd_head", 10, std::bind(&HexapodGaitNode::cmdHeadCallback, this, std::placeholders::_1));

        pub_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        joint_names_ = {
            "motor0", "motor1",
            "motor16", "motor17", "motor18",
            "motor19", "motor20", "motor21",
            "motor22", "motor23", "motor27",
            "motor9", "motor8", "motor31",
            "motor12", "motor11", "motor10",
            "motor15", "motor14", "motor13"
        };

        body_points_ = {
            {137.1, 189.4, body_height_}, {225.0, 0.0, body_height_}, {137.1, -189.4, body_height_},
            {-137.1, -189.4, body_height_}, {-225.0, 0.0, body_height_}, {-137.1, 189.4, body_height_}
        };

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&HexapodGaitNode::updateGaitCycle, this));
        
        pub_gazebo_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/position_controller/commands", 10);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_cmd_head_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_gazebo_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::string> joint_names_;
    std::vector<std::vector<double>> body_points_;
    
    double body_height_ = -25.0;
    double vx_ = 0.0, vy_ = 0.0, wz_ = 0.0;
    double head_pan_rad_, head_tilt_rad_;
    int gait_phase_;
    const int F_steps_ = 32;

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        vx_ = std::clamp(msg->linear.x * 35.0, -35.0, 35.0);
        vy_ = std::clamp(msg->linear.y * 35.0, -35.0, 35.0);
        wz_ = std::clamp(msg->angular.z * 20.0, -20.0, 20.0);
    }

    void cmdHeadCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        head_pan_rad_ = std::clamp(msg->x, -90.0, 90.0) * M_PI / 180.0;
        head_tilt_rad_ = std::clamp(msg->y, -90.0, 90.0) * M_PI / 180.0;
    }

    bool coordinateToAngle(double x, double y, double z, double &a_deg, double &b_deg, double &c_deg) {
        double l1 = 33.0, l2 = 90.0, l3 = 110.0;
        
        double a = M_PI_2 - std::atan2(z, y);
        double x_4 = l1 * std::sin(a);
        double x_5 = l1 * std::cos(a);
        
        double l23 = std::sqrt(std::pow(z - x_5, 2) + std::pow(y - x_4, 2) + std::pow(x, 2));
        if (l23 > (l2 + l3) || l23 < std::abs(l2 - l3)) return false;

        double v = std::clamp((l2 * l2 + l23 * l23 - l3 * l3) / (2.0 * l2 * l23), -1.0, 1.0);
        double u = std::clamp((l2 * l2 + l3 * l3 - l23 * l23) / (2.0 * l3 * l2), -1.0, 1.0);

        double b = std::asin(std::clamp(x / l23, -1.0, 1.0)) - std::acos(v);
        double c = M_PI - std::acos(u);

        a_deg = a * 180.0 / M_PI;
        b_deg = b * 180.0 / M_PI;
        c_deg = c * 180.0 / M_PI;
        return true;
    }

    void transformCoordinates(const std::vector<std::vector<double>> &pts, std::vector<std::vector<double>> &leg_pos) {
        static const double angles[6] = {54.0, 0.0, -54.0, -126.0, 180.0, 126.0};
        static const double offsets[6] = {94.0, 85.0, 94.0, 94.0, 85.0, 94.0};

        for (int i = 0; i < 6; ++i) {
            double rad = angles[i] * M_PI / 180.0;
            leg_pos[i][0] = pts[i][0] * std::cos(rad) + pts[i][1] * std::sin(rad) - offsets[i];
            leg_pos[i][1] = -pts[i][0] * std::sin(rad) + pts[i][1] * std::cos(rad);
            leg_pos[i][2] = pts[i][2] - 14.0;
        }
    }

    void updateGaitCycle() {
        std::vector<std::vector<double>> pts = body_points_;
        double step_z = 30.0;

        if (std::abs(vx_) > 0.1 || std::abs(vy_) > 0.1 || std::abs(wz_) > 0.1) {
            gait_phase_ = (gait_phase_ + 1) % F_steps_;
            double progress = static_cast<double>(gait_phase_) / F_steps_;

            for (int i = 0; i < 3; ++i) {
                int leg_a = 2 * i;
                int leg_b = 2 * i + 1;

                if (progress < 0.5) {
                    double p_a = progress * 2.0;
                    pts[leg_a][0] += vx_ * p_a;
                    pts[leg_a][1] += vy_ * p_a;
                    pts[leg_a][2] += std::sin(p_a * M_PI) * step_z;

                    pts[leg_b][0] -= vx_ * p_a;
                    pts[leg_b][1] -= vy_ * p_a;
                } else {
                    double p_b = (progress - 0.5) * 2.0;
                    pts[leg_a][0] -= vx_ * p_b;
                    pts[leg_a][1] -= vy_ * p_b;

                    pts[leg_b][0] += vx_ * p_b;
                    pts[leg_b][1] += vy_ * p_b;
                    pts[leg_b][2] += std::sin(p_b * M_PI) * step_z;
                }
            }
        }

        std::vector<std::vector<double>> leg_positions(6, std::vector<double>(3, 0.0));
        transformCoordinates(pts, leg_positions);

        auto joint_state = sensor_msgs::msg::JointState();
        joint_state.header.stamp = this->now();
        joint_state.name = joint_names_;

        joint_state.position.push_back(head_pan_rad_);
        joint_state.position.push_back(head_tilt_rad_);

        for (int i = 0; i < 6; ++i) {
            double a, b, c;
            coordinateToAngle(-leg_positions[i][2], leg_positions[i][0], leg_positions[i][1], a, b, c);
            
            joint_state.position.push_back((a - 90.0) * M_PI / 180.0);
            joint_state.position.push_back(b * M_PI / 180.0);
            joint_state.position.push_back(c * M_PI / 180.0);
        }

        pub_joint_states_->publish(joint_state);
        auto cmd_msg = std_msgs::msg::Float64MultiArray();
        cmd_msg.data = joint_state.position;
        pub_gazebo_cmd_->publish(cmd_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HexapodGaitNode>());
    rclcpp::shutdown();
    return 0;
}