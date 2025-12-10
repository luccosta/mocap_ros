#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>
#include <limits>
#include <cmath>
#include <vector>
#include <string>
#include <memory>
#include <algorithm>
#include <stdexcept>

using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;

struct Robot {
    std::shared_ptr<PointCloud> reference_cloud;
    std::string name;

    Eigen::Matrix4f last_pose;
    Eigen::Matrix4f last_movement;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_swarm_pub;

    Robot(std::shared_ptr<PointCloud> reference_cloud, std::string name) : 
        reference_cloud(reference_cloud), name(name) {
            last_pose = Eigen::Matrix4f::Identity();
        }
};

class ICPNode : public rclcpp::Node {
public:
    ICPNode() : Node("icp_pose_estimator"), random_num_gen_(rand_device_()) {        
        std::string clouds_file_path = this->declare_parameter<std::string>(
            "clouds_file_path", "config/clouds.yaml");
        
        // TODO(lucas): tunning!!!!
        double icp_max_correspondence = this->declare_parameter<double>(
            "icp_max_correspondence", 0.05);
        double icp_transformation_epsilon = this->declare_parameter<double>(
            "icp_transformation_epsilon", 0.1);
        double icp_euclidian_fitness_epsilon = this->declare_parameter<double>(
            "icp_euclidian_fitness_epsilon", 1.0);
        random_particle_x_stddev_ = this->declare_parameter<double>(
            "random_particle_x_stddev", 0.1);
        random_particle_y_stddev_ = this->declare_parameter<double>(
            "random_particle_y_stddev", 0.1);
        random_particle_z_stddev_ = this->declare_parameter<double>(
            "random_particle_z_stddev", 0.1);
        random_particle_yaw_stddev_ = this->declare_parameter<double>(
            "random_particle_yaw_stddev", 0.1);
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
            robot.particle_swarm_pub = this->create_publisher<geometry_msgs::msg::PoseArray>(robot.name + "/particle_swarm", 10);
        }

        initialized_ = false;
    }

private:
    std::vector<Robot> robots_;
    pcl::IterativeClosestPoint<PointT, PointT> icp_;
    bool initialized_;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_swarm_pub_;
    std::random_device rand_device_;
    std::mt19937 random_num_gen_;
    double random_particle_x_stddev_{0.0};
    double random_particle_y_stddev_{0.0};
    double random_particle_z_stddev_{0.0};
    double random_particle_yaw_stddev_{0.0};

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

            apply_constant_velocity_model(robot);

            /*if (icp_.hasConverged()) {
                Eigen::Matrix4f map_to_base_link = icp_.getFinalTransformation();
                robot.last_pose = map_to_base_link;
                
                pose_msg.pose = pose_from_matrix(map_to_base_link);

                robot.pose_pub->publish(pose_msg);

                RCLCPP_DEBUG_STREAM(this->get_logger(), "ICP for " << robot.name << " converged. Fitness: " << icp_.getFitnessScore());
            } else {
                pose_msg.pose = pose_from_matrix(robot.last_pose);

                robot.pose_pub->publish(pose_msg);

                RCLCPP_WARN_STREAM(this->get_logger(), "ICP for " << robot.name << " did not converge");
            }*/

            pose_msg.pose = pose_from_matrix(robot.last_pose);

            robot.pose_pub->publish(pose_msg);
        }
    }

    std::vector<Eigen::Matrix4f> generate_pose_particles(const Eigen::Matrix4f& pose, int number_of_particles=10) {
        auto particles = std::vector<Eigen::Matrix4f>();
        auto random_num_gen = std::make_unique<std::mt19937>(rand_device_());

        auto x_noise_dist = std::normal_distribution<double>(0.0, random_particle_x_stddev_);
        auto y_noise_dist = std::normal_distribution<double>(0.0, random_particle_y_stddev_);
        auto z_noise_dist = std::normal_distribution<double>(0.0, random_particle_z_stddev_);
        auto yaw_noise_dist = std::normal_distribution<double>(0.0, random_particle_yaw_stddev_);

        for (size_t i = 0; i < number_of_particles; ++i) {
            Eigen::Matrix4f particle = pose;

            Eigen::Vector3f translation_noise(
                x_noise_dist(*random_num_gen),
                y_noise_dist(*random_num_gen),
                z_noise_dist(*random_num_gen));
            particle.block<3,1>(0,3) += translation_noise;

            float yaw_noise = yaw_noise_dist(*random_num_gen);
            Eigen::Matrix3f yaw_rot =
                Eigen::AngleAxisf(yaw_noise, Eigen::Vector3f::UnitZ()).toRotationMatrix();
            particle.block<3,3>(0,0) = particle.block<3,3>(0,0) * yaw_rot;

            particles.push_back(particle);
        }

        return particles;
    }

    std::vector<Eigen::Matrix4f> full_particle_swarm(std::shared_ptr<PointCloud> cloud) {
        auto particle_swarm = std::vector<Eigen::Matrix4f>();
        for (const auto& pt : cloud->points) {
            for (int i = 0; i < 360; i++) {
                auto yaw = i * M_PI / 180;
                
                Eigen::Affine3f tf = pcl::getTransformation(
                    pt.x, pt.y, pt.z,
                    0.0f, 0.0f, yaw
                );

                Eigen::Matrix4f initial_guess = tf.matrix();

                auto particles = generate_pose_particles(initial_guess);
                particle_swarm.insert(
                    particle_swarm.end(), particles.begin(), particles.end());
            }
        }
        return particle_swarm;
    }

    std::vector<Eigen::Matrix4f> apply_constant_velocity_model(Robot & robot) {
        std::vector<Eigen::Matrix4f> particle_swarm;
        double lower_fitness = std::numeric_limits<double>::max();
        Eigen::Matrix4f next_pose = robot.last_pose;

        next_pose.block<3,1>(0,3).noalias() +=
            robot.last_pose.block<3,3>(0,0) * robot.last_movement.block<3,1>(0,3);

        auto robot_particle_swarm = generate_pose_particles(next_pose);
        particle_swarm.insert(
            particle_swarm.end(), robot_particle_swarm.begin(), robot_particle_swarm.end());

        for (const auto & particle : robot_particle_swarm) {
            PointCloud output_cloud;
            icp_.align(output_cloud, particle);
            auto fitness_score = icp_.getFitnessScore();

            if (fitness_score < lower_fitness) {
                lower_fitness = fitness_score;
                robot.last_pose = icp_.getFinalTransformation();
            }
        }

        geometry_msgs::msg::PoseArray swarm_msg;
        swarm_msg.header.frame_id = "map";
        swarm_msg.header.stamp = now();
        swarm_msg.poses.reserve(particle_swarm.size());
        for (const auto & particle_pose : particle_swarm) {
            swarm_msg.poses.push_back(pose_from_matrix(particle_pose));
        }

        robot.particle_swarm_pub->publish(swarm_msg);

        return particle_swarm;
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

    float points_distance(const pcl::PointXYZ& point_a, const pcl::PointXYZ& point_b) {
        float dx = point_a.x - point_b.x;
        float dy = point_a.y - point_b.y;
        float dz = point_a.z - point_b.z;

        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    std::vector<float> distances_between_points(pcl::PointCloud<pcl::PointXYZ> cloud){
        size_t n = cloud.points.size();
        std::vector<float> distances;

        for (size_t i = 0; i < n; ++i)
            for (size_t j = i + 1; j < n; ++j)
                distances.push_back(points_distance(cloud.points[i], cloud.points[j]));

        return distances;
    }

    std::vector<size_t> get_points_indexes_from_distance_index(size_t k, size_t n) {
        size_t i = 0;

        while (k >= n - i - 1) {
            k -= (n - i - 1);
            ++i;
        }

        size_t j = i + 1 + k;

        return {i, j};
    }

    size_t find_common_element(const std::vector<size_t>& a, const std::vector<size_t>& b)
    {
        for (const size_t& x : a)
        {
            for (const size_t& y : b)
            {
                if (x == y)
                    return x;
            }
        }
        throw std::runtime_error("The choosen sides should contain a vertice in common.");
    }

    size_t find_opposite_element(const std::vector<size_t>& a, size_t current_element)
    {
        for (const size_t& x : a)
        {
            if (x != current_element)
                return x;
        }
        throw std::runtime_error("No opposite element found.");
    }

    Eigen::Matrix3f rotation_from_a_to_b(const Eigen::Vector3f& a, const Eigen::Vector3f& b)
    {
        Eigen::Vector3f v1 = a.normalized();
        Eigen::Vector3f v2 = b.normalized();

        float cos_theta = v1.dot(v2);
        Eigen::Vector3f axis = v1.cross(v2);

        if (axis.norm() < 1e-6) {
            if (cos_theta > 0.9999f) {
                return Eigen::Matrix3f::Identity();
            } else {
                Eigen::Vector3f ortho = v1.unitOrthogonal();
                return Eigen::AngleAxisf(M_PI, ortho).toRotationMatrix();
            }
        }

        float sin_theta = axis.norm();
        Eigen::Matrix3f K;
        K <<     0, -axis.z(),  axis.y(),
            axis.z(),     0, -axis.x(),
            -axis.y(), axis.x(),     0;

        Eigen::Matrix3f R = Eigen::Matrix3f::Identity() + K + K * K * ((1 - cos_theta) / (sin_theta * sin_theta));
        return R;
    }

    Eigen::Matrix4f build_homogeneous_matrix(const Eigen::Matrix3f& rotation, const Eigen::Vector3f& translation)
    {
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

        T.block<3,3>(0,0) = rotation;
        T.block<3,1>(0,3) = translation;

        return T;
    }

    Eigen::Vector3f eigen_from_pcl_point(const pcl::PointXYZ& point) {
        return Eigen::Vector3f{
            point.x,
            point.y,
            point.z
        };
    }

    void template_to_cluster_transform(const pcl::PointCloud<pcl::PointXYZ>& template_pc,
        const pcl::PointCloud<pcl::PointXYZ>& cluster) {

        auto template_distances = distances_between_points(template_pc);
        auto cluster_distances = distances_between_points(cluster);

        auto template_largest_side = std::max_element(template_distances.begin(), template_distances.end());
        auto cluster_largest_side = std::max_element(cluster_distances.begin(), cluster_distances.end());

        auto template_shortest_side = std::min_element(template_distances.begin(), template_distances.end());
        auto cluster_shortest_side = std::min_element(cluster_distances.begin(), cluster_distances.end());

        auto template_largest_side_index = std::distance(template_distances.begin(), template_largest_side);
        auto template_shortest_side_index = std::distance(template_distances.begin(), template_shortest_side);

        auto cluster_largest_side_index = std::distance(cluster_distances.begin(), cluster_largest_side);
        auto cluster_shortest_side_index = std::distance(cluster_distances.begin(), cluster_shortest_side);
        
        // Template transform
        auto template_largest_side_points_index = 
            get_points_indexes_from_distance_index(template_largest_side_index, template_pc.points.size());
        auto template_shortest_side_points_index = 
            get_points_indexes_from_distance_index(template_shortest_side_index, template_pc.points.size());

        auto template_common_point_index = 
            find_common_element(template_largest_side_points_index, template_shortest_side_points_index);
        auto template_other_point_index = 
            find_opposite_element(template_largest_side_points_index, template_common_point_index);

        auto template_translation = eigen_from_pcl_point(template_pc.points[template_common_point_index]);
        auto template_rotation = rotation_from_a_to_b(template_translation,
            eigen_from_pcl_point(template_pc.points[template_other_point_index]));

        auto template_transform = build_homogeneous_matrix(template_rotation, template_translation);
        
        // Cluster transform
        auto cluster_largest_side_points_index = 
            get_points_indexes_from_distance_index(cluster_largest_side_index, cluster.points.size());
        auto cluster_shortest_side_points_index = 
            get_points_indexes_from_distance_index(cluster_shortest_side_index, cluster.points.size());

        auto cluster_common_point_index = 
            find_common_element(cluster_largest_side_points_index, cluster_shortest_side_points_index);
        auto cluster_other_point_index = 
            find_opposite_element(cluster_largest_side_points_index, cluster_common_point_index);

        auto cluster_translation = eigen_from_pcl_point(cluster.points[cluster_common_point_index]);
        auto cluster_rotation = rotation_from_a_to_b(cluster_translation,
            eigen_from_pcl_point(cluster.points[cluster_other_point_index]));

        auto cluster_transform = build_homogeneous_matrix(cluster_rotation, cluster_translation);
        
        // Template to cluster transform
        auto template_to_cluster = cluster_transform * template_transform.inverse();
        
        Eigen::Matrix4f template_base_link = Eigen::Matrix4f::Identity();
        template_base_link.block<3,1>(0,3) = template_translation;
        auto cluster_base_link = template_to_cluster * template_base_link.inverse();
    }

    void find_template_in_cluster(
        pcl::PointCloud<pcl::PointXYZ> template_pc, 
        pcl::PointCloud<pcl::PointXYZ> cluster) {

        auto estimates = pcl::PointCloud<pcl::PointXYZ>{};
        std::vector<pcl::PointCloud<pcl::PointXYZ>> point_count;
        for (const auto& pt_cluster : cluster.points) {
            for (const auto& pt_template : template_pc.points) {
                auto estimate = pcl::PointXYZ{};

                estimate.x = pt_cluster.x + pt_template.x;
                estimate.y = pt_cluster.y + pt_template.y;
                estimate.z = pt_cluster.z + pt_template.z;
                
                auto found_pair = false;
                for (auto& pair : point_count) {
                    if (points_distance(pair.points[0], estimate) < 0.01) {
                        found_pair = true;
                        pair.points.push_back(estimate);
                    }
                }
                if (!found_pair) {
                    auto pc_estimate = pcl::PointCloud<pcl::PointXYZ>{};
                    pc_estimate.points.push_back(estimate);
                    point_count.push_back(pc_estimate);
                }
            }
        }

        auto largest_cluster = std::max_element(
            point_count.begin(), point_count.end(),
            [](const PointCloud& a, const PointCloud& b) {
                return a.points.size() < b.points.size();
            });

        largest_cluster->width = largest_cluster->points.size();
        largest_cluster->height = 1;
        largest_cluster->is_dense = true;

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*largest_cluster, centroid);
        
        
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
    segment_dense_regions(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
    {
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(input_cloud);

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.05);       
        ec.setMinClusterSize(3);           
        ec.setMaxClusterSize(4);        
        ec.setSearchMethod(tree);
        ec.setInputCloud(input_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        ec.extract(cluster_indices);

        for (const auto& indices : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (int idx : indices.indices)
            {
                cluster->points.push_back(input_cloud->points[idx]);
            }
            cluster->width = cluster->points.size();
            cluster->height = 1;
            cluster->is_dense = true;
            clusters.push_back(cluster);
        }

        return clusters;
    }
};

int main(int argc, char** argv) {
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICPNode>());
    rclcpp::shutdown();
    return 0;
}
