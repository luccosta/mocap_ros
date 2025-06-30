#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <memory>

using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;

struct Robot {
    std::shared_ptr<PointCloud> reference_cloud;
    std::string name;

    Eigen::Matrix4f last_pose;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;

    Robot(std::shared_ptr<PointCloud> reference_cloud, std::string name) : 
        reference_cloud(reference_cloud), name(name) {
            last_pose = Eigen::Matrix4f::Identity();
        }
};

class ICPNode : public rclcpp::Node {
public:
    ICPNode() : Node("icp_pose_estimator") {        
        std::string clouds_file_path = this->declare_parameter<std::string>(
            "clouds_file_path", "config/clouds.yaml");
        
        double icp_max_correspondence = this->declare_parameter<double>(
            "icp_max_correspondence", 0.05);
        double icp_transformation_epsilon = this->declare_parameter<double>(
            "icp_transformation_epsilon", 2.0);
        double icp_euclidian_fitness_epsilon = this->declare_parameter<double>(
            "icp_euclidian_fitness_epsilon", 1.0);
        int icp_max_iters = this->declare_parameter<int>(
            "icp_max_iters", 30);
        
        icp_.setMaxCorrespondenceDistance(icp_max_correspondence);
        icp_.setMaximumIterations(icp_max_iters);
        icp_.setTransformationEpsilon(icp_transformation_epsilon);
        icp_.setEuclideanFitnessEpsilon(icp_euclidian_fitness_epsilon);

        load_clouds_from_yaml(clouds_file_path);
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "input_cloud", 1, [this](const sensor_msgs::msg::PointCloud2 & msg) {
                cloud_callback(msg);
            });
        
        for (auto & robot : robots_) {
            robot.pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(robot.name + "/icp_pose", 10);
        }

        initialized_ = false;
    }

private:
    std::vector<Robot> robots_;
    pcl::IterativeClosestPoint<PointT, PointT> icp_;
    bool initialized_;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

    void load_clouds_from_yaml(const std::string& filepath) {
        YAML::Node root = YAML::LoadFile(filepath);
    
        for (const auto& cloud_node : root["clouds"]) {
            auto cloud = std::make_shared<PointCloud>();
            for (const auto& pt : cloud_node["points"]) {
                PointT p;
                p.x = pt[0].as<float>();
                p.y = pt[1].as<float>();
                p.z = pt[2].as<float>();
                cloud->points.push_back(p);
            }
            cloud->width = cloud->points.size();
            cloud->height = 1;
            cloud->is_dense = true;
            robots_.push_back(Robot(cloud, cloud_node["name"].as<std::string>()));
        }
    }

    geometry_msgs::msg::Pose pose_from_matrix(const Eigen::Matrix4f & matrix) {
        geometry_msgs::msg::Pose pose_msg;

        pose_msg.position.x = matrix(0, 3);
        pose_msg.position.y = matrix(1, 3);
        pose_msg.position.z = matrix(2, 3);

        Eigen::Matrix3f rotation_matrix = matrix.block<3,3>(0,0);
        Eigen::Quaternionf quaternion(rotation_matrix);
        pose_msg.orientation.x = quaternion.x();
        pose_msg.orientation.y = quaternion.y();
        pose_msg.orientation.z = quaternion.z();
        pose_msg.orientation.w = quaternion.w();

        return pose_msg;
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2 & msg) {
        auto input_cloud = std::make_shared<PointCloud>();
        pcl::fromROSMsg(msg, *input_cloud);

        if (!initialized_) {
            initial_search(input_cloud);
            initialized_ = true;
        }

        for (auto & robot : robots_) {
            icp_.setInputTarget(input_cloud);
            icp_.setInputSource(robot.reference_cloud);

            PointCloud output_cloud;
            icp_.align(output_cloud, robot.last_pose);
            
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header = msg.header;
            pose_msg.header.frame_id = "map";

            if (icp_.hasConverged()) {
                Eigen::Matrix4f map_to_base_link = icp_.getFinalTransformation();
                robot.last_pose = map_to_base_link;
                
                pose_msg.pose = pose_from_matrix(map_to_base_link);

                robot.pose_pub->publish(pose_msg);

                RCLCPP_DEBUG_STREAM(this->get_logger(), "ICP for " << robot.name << " converged. Fitness: " << icp_.getFitnessScore());
            } else {
                pose_msg.pose = pose_from_matrix(robot.last_pose);

                robot.pose_pub->publish(pose_msg);

                RCLCPP_WARN_STREAM(this->get_logger(), "ICP for " << robot.name << " did not converge");
            }

            robot.pose_pub->publish(pose_msg);
        }
    }

    void initial_search(std::shared_ptr<PointCloud> cloud) {
        for (auto & robot : robots_) {
            double lower_fitness = std::numeric_limits<double>::max();
            Eigen::Matrix4f robot_initial_pose;

            icp_.setInputTarget(cloud);
            icp_.setInputSource(robot.reference_cloud);
            
            for (const auto& pt : cloud->points) {
                for (int i = 0; i < 360; i++) {
                    auto yaw = i * M_PI / 180;
                    
                    Eigen::Affine3f tf = pcl::getTransformation(
                        pt.x, pt.y, pt.z,
                        0.0f, 0.0f, yaw
                    );

                    Eigen::Matrix4f initial_guess = tf.matrix();

                    PointCloud output_cloud;
                    icp_.align(output_cloud, initial_guess);
                    auto fitness_score = icp_.getFitnessScore();

                    if (fitness_score < lower_fitness) {
                        lower_fitness = fitness_score;
                        robot_initial_pose = initial_guess;
                    }
                }
            }
            robot.last_pose = robot_initial_pose;
        }
    }
};

int main(int argc, char** argv) {
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICPNode>());
    rclcpp::shutdown();
    return 0;
}
