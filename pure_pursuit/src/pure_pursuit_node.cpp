#include <chrono>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "Eigen/Dense"
#include "Eigen/StdVector"

#include "pure_pursuit/pure_pursuit_core.hpp"
#include "pure_pursuit/third_party/nanoflann.hpp"

class PurePursuit : public rclcpp::Node
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PurePursuit() : Node("pure_pursuit_node"), kd_tree_adaptor(waypoints)
    {
        sim = this->declare_parameter("use_sim", true);
        L = this->declare_parameter("lookahead_distance", 1.72);
        P = this->declare_parameter("steering_gain", 0.435);
        speed_profile.max_steering = this->declare_parameter("max_steering", 0.4);
        speed_profile.max_speed = this->declare_parameter("max_speed", 4.5);
        speed_profile.min_speed = this->declare_parameter("min_speed", 1.5);
        waypoints_file = this->declare_parameter<std::string>("waypoints_file", "levine2.csv");
        const std::string odom_topic = this->declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");
        const std::string pose_topic = this->declare_parameter<std::string>("pose_topic", "/pf/viz/inferred_pose");
        const std::string drive_topic = this->declare_parameter<std::string>("drive_topic", "/drive");

        // Load waypoints FIRST before anything else
        loadWaypoints();

        // Subscribers
        if (sim) {
            odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                odom_topic, 10,
                std::bind(&PurePursuit::odom_callback, this, std::placeholders::_1));
        } else {
            pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                pose_topic, 10,
                std::bind(&PurePursuit::pose_callback, this, std::placeholders::_1));
        }

        drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
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
    pure_pursuit::SpeedProfile speed_profile;
    std::string waypoints_file;

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

    // Member Functions
    void loadWaypoints();
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void process_pose(double car_x, double car_y, const geometry_msgs::msg::Quaternion& quat);
    void publish_waypoints_markers();
};

void PurePursuit::loadWaypoints() {
    // Get path from package share directory
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("pure_pursuit");
    std::string csv_path = pkg_share_dir + "/waypoints/" + waypoints_file;
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
        RCLCPP_ERROR(this->get_logger(), "Error in odom_callback: %s", e.what());
    }
}

void PurePursuit::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    try {
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
    const Eigen::Matrix3d rot = pure_pursuit::worldToVehicleRotation(q);

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

    // Walk forward from the nearest waypoint until one is at least L away,
    // wrapping around the end of the lap.
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

    // Transform goal to vehicle frame, then steer along the arc through it.
    const Eigen::Vector2d translated = pure_pursuit::goalPointInVehicleFrame(rot, car_pos, goal);
    const double curvature = pure_pursuit::arcCurvature(translated.y(), L);
    const double steering_angle = pure_pursuit::steeringAngle(P, curvature);
    const double speed = pure_pursuit::speedFromSteering(steering_angle, speed_profile);

    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.drive.speed = speed;
    drive_msg.drive.steering_angle = steering_angle;
    drive_pub->publish(drive_msg);
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
    marker_pub->publish(marker_array);
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
