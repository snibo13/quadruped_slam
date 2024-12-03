#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <deque>
#include <fstream>
#include <filesystem>
#include <string>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

using namespace gtsam;
using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

class FactorGraphNode{
    public:
        FactorGraphNode()
        : correction_count_(0), output_time_(0.0), dt_(0.01), window_size_(10), isam_(initializeISAM2Params()){
            // Initialize the graph and IMU preintegration
            initializeGraph();
            initializeIMUIntegrator();
            // ROS_INFO_STREAM("Graph and IMU preintegration initialized");
            ROS_INFO_STREAM("Current working directory: " << std::filesystem::current_path());
            // Initialize ROS subscribers and publishers
            imu_sub_ = nh_.subscribe("/cmu_rc7/imu/data", 1000, &FactorGraphNode::imuCallback, this);
            lidar_sub_ = nh_.subscribe("/cmu_rc7/aft_mapped_to_init_imu", 1000, &FactorGraphNode::lidarCallback, this);
            visual_sub_ = nh_.subscribe("/cmu_rc7/integrated_to_init", 1000, &FactorGraphNode::visualCallback, this);
            ground_truth_sub_ = nh_.subscribe("/cmu_rc7/velodyne_cloud_registered_imu", 1000, &FactorGraphNode::groundTruthCallback, this);\
            estimated_trajectory_pub_ = nh_.advertise<nav_msgs::Odometry>("/estimated_trajectory", 1000);
            // ROS_INFO_STREAM("Subscribers initialized");
        }

        void spin(){
            ros::spin();
            // output_file_.close();
        }

    private:
        ros::NodeHandle nh_;
        ros::Subscriber imu_sub_; 
        ros::Subscriber lidar_sub_;
        ros::Subscriber visual_sub_;
        ros::Subscriber ground_truth_sub_;
        ros::Publisher estimated_trajectory_pub_;

        NonlinearFactorGraph graph_;
        Values initial_values_;
        std::shared_ptr<PreintegratedImuMeasurements> preintegrated_;
        ISAM2 isam_; // Use ISAM2 to Marginalize out Old Factors over Time (Sliding Window)

        std::deque<size_t> factor_window_; // Uncessary Factors
        size_t window_size_;

        double output_time_;
        double dt_;
        NavState prev_state_;
        NavState prop_state_;
        imuBias::ConstantBias prev_bias_;
        imuBias::ConstantBias prop_bias_;

        int correction_count_;
        int batch_size_ = 1; // Try 10 later
        std::ofstream output_file_;
        std::string output_file_path = "/home/rml/quadruped_slam/src/gtsam_bag/src/output.csv";
        Matrix3 I_3x3 = Matrix3::Identity();

        static ISAM2Params initializeISAM2Params() {
            // Define Sliding Window Size, and other ISAM2 parameters
            ISAM2Params params;
            params.relinearizeThreshold = 0.01;  // Adjust as needed
            params.relinearizeSkip = 1;          // Relinearize every step
            return params;
        }


        void initializeGraph() {
            // TO DO: Ask Shibo how what the intial graph pose should be during deployment
            Pose3 prior_pose(Rot3::Identity(), Point3(0, 0, 0));
            Vector3 prior_velocity(0, 0, 0);
            imuBias::ConstantBias prior_imu_bias;

            auto pose_noise = noiseModel::Diagonal::Sigmas(
                (Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished());
            auto velocity_noise = noiseModel::Isotropic::Sigma(3, 0.1);
            auto bias_noise = noiseModel::Isotropic::Sigma(6, 1e-3);

            graph_.addPrior(X(0), prior_pose, pose_noise);
            graph_.addPrior(V(0), prior_velocity, velocity_noise);
            graph_.addPrior(B(0), prior_imu_bias, bias_noise);

            initial_values_.insert(X(0), prior_pose);
            initial_values_.insert(V(0), prior_velocity);
            initial_values_.insert(B(0), prior_imu_bias);

            prev_state_ = NavState(prior_pose, prior_velocity);
            prev_bias_ = prior_imu_bias;

            // Open output file
            output_file_.open(output_file_path);
            if (!output_file_.is_open()) {
                ROS_ERROR_STREAM("Failed to open output.csv for writing.");
                return;
            } else {
                ROS_INFO_STREAM("Successfully opened output.csv for writing.");
            }
            output_file_ << "time,x,y,z,qx,qy,qz,qw\n";
        }

        // Initialize the IMU preintegration
        void initializeIMUIntegrator() {
            auto params = PreintegratedImuMeasurements::Params::MakeSharedU(9.81);
            params->accelerometerCovariance = I_3x3 * pow(0.01, 2);
            params->gyroscopeCovariance = I_3x3 * pow(0.01, 2);
            params->integrationCovariance = I_3x3 * 1e-5;

            preintegrated_ = std::make_shared<PreintegratedImuMeasurements>(params, prev_bias_);
        }
        
        // Handle TF Transformation from Sensor Frame to IMU (Body Frame)
        void tfConversion(){
            // TO DO: Implement TF Transformation
        }


        // Process IMU messages
        void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
            // ROS_INFO_STREAM("IMU DATA RECEIVED");
            Vector3 accel(imu_msg->linear_acceleration.x,
                        imu_msg->linear_acceleration.y,
                        imu_msg->linear_acceleration.z);
            Vector3 gyro(imu_msg->angular_velocity.x,
                        imu_msg->angular_velocity.y,
                        imu_msg->angular_velocity.z);

            preintegrated_->integrateMeasurement(accel, gyro, dt_);
            correction_count_++;
            addIMUFactor();

            prop_state_ = preintegrated_->predict(prev_state_, prev_bias_);
            initial_values_.insert(X(correction_count_), prop_state_.pose());
            initial_values_.insert(V(correction_count_), prop_state_.v());
            initial_values_.insert(B(correction_count_), prev_bias_);

            if (correction_count_ % batch_size_ == 0) {
                // Batch Optimization after batch_size steps
                optimizeGraph();
            }
            preintegrated_->resetIntegrationAndSetBias(prev_bias_);
            output_time_ += dt_;
        }

        // Process LiDAR messages
        void lidarCallback(const nav_msgs::Odometry::ConstPtr& lidar_msg) {
            // TO DO: Extract and process LiDAR constraints here
            // Add constraints to the graph in the form of Between Factors
            // ROS_INFO("LiDAR data received.");
        }

        // Process visual odometry messages
        void visualCallback(const nav_msgs::Odometry::ConstPtr& visual_msg) {
            // TO DO: Include the Visual Odometry Constraints
            // Adding Between Factors between IMU Poses
            Pose3 visual_pose = Pose3(Rot3::Quaternion(
                                        visual_msg->pose.pose.orientation.w,
                                        visual_msg->pose.pose.orientation.x,
                                        visual_msg->pose.pose.orientation.y,
                                        visual_msg->pose.pose.orientation.z),
                                    Point3(visual_msg->pose.pose.position.x,
                                            visual_msg->pose.pose.position.y,
                                            visual_msg->pose.pose.position.z));

            auto noise = noiseModel::Diagonal::Sigmas(Vector6::Constant(0.1));
            // graph_.add(BetweenFactor<Pose3>(X(correction_count_ - 1),
            //                                 X(correction_count_), visual_pose, noise));
            // ROS_INFO("Visual odometry data added to graph.");
            
        }

        void groundTruthCallback(const sensor_msgs::PointCloud2::ConstPtr& ground_truth_msg) {
            // Extract and process ground truth 
            // Probably dont need this unless we do some graphing or post processing comparisons
            // ROS_INFO("Ground truth data received.");
        }

        // Add an IMU factor to the graph
        void addIMUFactor() {
            PreintegratedImuMeasurements preint_imu = *preintegrated_;
            graph_.add(ImuFactor(X(correction_count_ - 1), V(correction_count_ - 1),
                                X(correction_count_), V(correction_count_),
                                B(correction_count_ - 1), preint_imu));
            imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
            graph_.add(BetweenFactor<imuBias::ConstantBias>( B(correction_count_ - 1), 
            B(correction_count_), zero_bias, noiseModel::Isotropic::Sigma(6, 1e-3)));
        }

        // Optimize the graph using ISAM2
        void optimizeGraph() {
            isam_.update(graph_, initial_values_);
            graph_.resize(0);  // Clear factors after optimization
            initial_values_.clear();

            Values results = isam_.calculateEstimate();
            prev_state_ = NavState(results.at<Pose3>(X(correction_count_)),
                                results.at<Vector3>(V(correction_count_)));
            prev_bias_ = results.at<imuBias::ConstantBias>(B(correction_count_));
            logStateToFile(prev_state_, output_time_);
        }

        // Log the state to a CSV file
        void logStateToFile(const NavState& state, double time) {
            auto pos = state.pose().translation();
            auto quat = state.pose().rotation().toQuaternion();
            output_file_ << time << "," << pos.x() << "," << pos.y() << "," << pos.z() << ","
                        << quat.x() << "," << quat.y() << "," << quat.z() << "," << quat.w()
                        << "\n";
        }

        // Publish Trajectoy of Solved Poses
        void publishTrajectory(){
            // TO DO: Implement Trajectory Publisher for GAzebo Visualization
        }
    };

int main(int argc, char** argv) {
    ros::init(argc, argv, "factor_graph_node");
    FactorGraphNode node;
    node.spin();
    return 0;
}