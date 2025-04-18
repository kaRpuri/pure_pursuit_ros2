#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nanoflann.hpp"
#include "Eigen/Dense"
#include "Eigen/StdVector"  

class PurePursuit : public rclcpp::Node
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  

    PurePursuit() : Node("pure_pursuit_node"), 
                    sim(true), 
                    L(1.72), // 1.72 for simulation, 2.7 on car considering slip
                    P(0.435), 
                    kd_tree_adaptor(waypoints)
    {
        // Load waypoints FIRST before anything else
        loadWaypoints();
        
        // Subscribers
        if (sim) {
            odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                "/ego_racecar/odom", 10,
                std::bind(&PurePursuit::odom_callback, this, std::placeholders::_1));
        } else {
            pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/pf/viz/inferred_pose", 10,
                std::bind(&PurePursuit::pose_callback, this, std::placeholders::_1));
        }

        drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoints_markers", 10);

        // Build KDTree only after waypoints are loaded
        if (!waypoints.empty()) {
            kd_tree = std::make_unique<KDTree>(2, kd_tree_adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
            kd_tree->buildIndex();
            RCLCPP_INFO(this->get_logger(), "KDTree built with %ld waypoints", waypoints.size());
        } else {
            RCLCPP_ERROR(this->get_logger(), "No waypoints loaded! KDTree not built.");
        }

        // Timer for marker publishing
        timer = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PurePursuit::publish_waypoints_markers, this));
    }

private:
    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
    rclcpp::TimerBase::SharedPtr timer;

    // Parameters
    bool sim;
    double L, P;

    // use the alligned allocator for waypoint and KD tree
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> waypoints;
    
    struct KDTreeAdaptor {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Alignment macro
        const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& data;
        
        KDTreeAdaptor(const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& data) : data(data) {}
        
        inline size_t kdtree_get_point_count() const { return data.size(); }
        inline double kdtree_get_pt(const size_t idx, const size_t dim) const { return data[idx][dim]; }
        template <class BBOX> bool kdtree_get_bbox(BBOX&) const { return false; }
    };
    
    KDTreeAdaptor kd_tree_adaptor;
    
    using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, KDTreeAdaptor>,
        KDTreeAdaptor, 2, unsigned int>;
        
    std::unique_ptr<KDTree> kd_tree;

    // Rotation matrix
    Eigen::Matrix3d rot;

    // Member Functions
    void loadWaypoints();
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void process_pose(double car_x, double car_y, const geometry_msgs::msg::Quaternion& quat);
    Eigen::Vector2d translate_point(const Eigen::Vector2d& curr, const Eigen::Vector2d& target);
    void publish_waypoints_markers();
};

void PurePursuit::loadWaypoints() {
    // Get path from package share directory
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("pure_pursuit");
    std::string csv_path = pkg_share_dir + "/levine2.csv";
    std::ifstream file(csv_path);
    
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open waypoints file: %s", csv_path.c_str());
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Successfully opened waypoints file: %s", csv_path.c_str());
    
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string x_str, y_str;
        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');
        
        try {
            double x = std::stod(x_str);
            double y = std::stod(y_str);
            waypoints.emplace_back(x, y);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to parse waypoint: %s", line.c_str());
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Loaded %ld waypoints", waypoints.size());
}


void PurePursuit::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    try {
        process_pose(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.orientation);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in odom_callback: %s", e.what()); // test odom callback
    }
}

void PurePursuit::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    try { // test the pose callback
        process_pose(msg->pose.position.x, msg->pose.position.y, msg->pose.orientation);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in pose_callback: %s", e.what());
    }
}

void PurePursuit::process_pose(double car_x, double car_y, const geometry_msgs::msg::Quaternion& quat) {
    if (waypoints.empty() || !kd_tree) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                              "Waypoints or KDTree not initialized");
        return;
    }

    // Quaternion to rotation matrix
    Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);
    rot = q.normalized().toRotationMatrix().transpose();

    // Find closest waypoint
    std::vector<double> query{car_x, car_y};
    unsigned int idx;
    double dist;
    
    try {
        kd_tree->knnSearch(query.data(), 1, &idx, &dist);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "KDTree search error: %s", e.what());
        return;
    }

    // Vectorized lookahead search improved for faster computation
    Eigen::Vector2d car_pos(car_x, car_y);
    size_t goal_idx = waypoints.size();
    for (size_t i = idx; i < waypoints.size(); ++i) {
        if ((waypoints[i] - car_pos).norm() >= L) {
            goal_idx = i;
            break;
        }
    }
    if (goal_idx == waypoints.size()) {
        for (size_t i = 0; i < idx; ++i) {
            if ((waypoints[i] - car_pos).norm() >= L) {
                goal_idx = i;
                break;
            }
        }
        if (goal_idx == waypoints.size()) return;
    }
    Eigen::Vector2d goal = waypoints[goal_idx];

    // Transform goal to vehicle frame
    Eigen::Vector2d translated = translate_point(car_pos, goal);
    double curvature = 2 * translated.y() / (L * L);
    double steering_angle = P * curvature;

    // Compute speed (simple model) need to improve by checking curvature for more comtrol
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    const double max_steering = 0.4;
    const double max_speed = 4.5;
    const double min_speed = 1.5;
    double steer_fraction = std::abs(steering_angle) / max_steering;
    double speed = max_speed - (max_speed - min_speed) * steer_fraction;
    if (speed > 4.3) speed = 5.0;
    if (speed < 2.0) speed = 2.0;

    drive_msg.drive.speed = speed;
    drive_msg.drive.steering_angle = steering_angle;
    drive_pub->publish(drive_msg);
}

Eigen::Vector2d PurePursuit::translate_point(const Eigen::Vector2d& curr, const Eigen::Vector2d& target) {
    Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
    H.block<3, 3>(0, 0) = rot;
    H(0, 3) = curr.x();
    H(1, 3) = curr.y();
    Eigen::Vector4d dir(target.x() - curr.x(), target.y() - curr.y(), 0, 0);
    Eigen::Vector4d translated = H * dir;
    return Eigen::Vector2d(translated.x(), translated.y());
}

void PurePursuit::publish_waypoints_markers() {
    if (waypoints.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                            "No waypoints to visualize");
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "waypoints";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = waypoints[i][0];
        marker.pose.position.y = waypoints[i][1];
        marker.pose.position.z = 0.1;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker_array.markers.push_back(marker);
    }
    marker_pub->publish(marker_array); // dont publish in the race, reduces computation time slightly
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<PurePursuit>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in main: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
