#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>

// #include <boost/program_options.hpp>



using namespace gtsam;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)


boost::shared_ptr<PreintegratedCombinedMeasurements::Params> imuParams() {
  // We use the sensor specs to build the noise model for the IMU factor.
  double accel_noise_sigma = 0.0003924;
  double gyro_noise_sigma = 0.000205689024915;
  double accel_bias_rw_sigma = 0.004905;
  double gyro_bias_rw_sigma = 0.000001454441043;
  Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma, 2);
  Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma, 2);
  Matrix33 integration_error_cov =
      I_3x3 * 1e-8;  // error committed in integrating position from velocities
  Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma, 2);
  Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma, 2);
  Matrix66 bias_acc_omega_init =
      I_6x6 * 1e-5;  // error in the bias used for preintegration

  auto p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
  // PreintegrationBase params:
  p->accelerometerCovariance =
      measured_acc_cov;  // acc white noise in continuous
  p->integrationCovariance =
      integration_error_cov;  // integration uncertainty continuous
  // should be using 2nd order integration
  // PreintegratedRotation params:
  p->gyroscopeCovariance =
      measured_omega_cov;  // gyro white noise in continuous
  // PreintegrationCombinedMeasurements params:
  p->biasAccCovariance = bias_acc_cov;      // acc bias in continuous
  p->biasOmegaCovariance = bias_omega_cov;  // gyro bias in continuous
  p->biasAccOmegaInt = bias_acc_omega_init;

  return p;
}
int main(int argc, char** argv)
{

    rosbag::Bag bag;
    if (argc < 2)
    {
        std::cout << "Missing path to bag file: usage rosrun gtsam_bag gtsam_bag <path_to_file>" << std::endl;
        return -1;
    }
    char* bag_filename = argv[1];
    std::cout << bag_filename << std::endl;
    bag.open(bag_filename);  // BagMode is Read by default

    // std::vector<rosbag::MessageInstance> imuMessages;

    Vector10 initial_state; // X, Y, Z, Qx, Qy, Qz, Qw, Vx, Vy, Vz

    // Priors
    Rot3 prior_rotation = Rot3::Quaternion(initial_state(6), initial_state(3),
                                           initial_state(4), initial_state(5));
    Point3 prior_point(initial_state.head<3>());
    Pose3 prior_pose(prior_rotation, prior_point);
    Vector3 prior_velocity(initial_state.tail<3>());
    imuBias::ConstantBias prior_imu_bias;  // assume zero initial bias

    Values initial_values;
    int correction_count = 0;
    initial_values.insert(X(correction_count), prior_pose);
    initial_values.insert(V(correction_count), prior_velocity);
    initial_values.insert(B(correction_count), prior_imu_bias);

    // Assemble prior noise model and add it the graph.`
    auto pose_noise_model = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5)
            .finished());  // rad,rad,rad,m, m, m
    auto velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);  // m/s
    auto bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);

    // Add all prior factors (pose, velocity, bias) to the graph.
    NonlinearFactorGraph* graph = new NonlinearFactorGraph();
    graph->addPrior(X(correction_count), prior_pose, pose_noise_model);
    graph->addPrior(V(correction_count), prior_velocity, velocity_noise_model);
    graph->addPrior(B(correction_count), prior_imu_bias, bias_noise_model);

    auto p = imuParams();

    std::shared_ptr<PreintegrationType> preintegrated =
        std::make_shared<PreintegratedImuMeasurements>(p, prior_imu_bias);

    assert(preintegrated);

    // Store previous state for imu integration and latest predicted outcome.
    NavState prev_state(prior_pose, prior_velocity);
    NavState prop_state = prev_state;
    imuBias::ConstantBias prev_bias = prior_imu_bias;

    // Keep track of total error over the entire run as simple performance metric.
    double current_position_error = 0.0, current_orientation_error = 0.0;

    double output_time = 0.0;
    double dt = 0.01;  // The real system has noise, but here, results are nearly
                        // exactly the same, so keeping this for simplicity.


    // Initialising file for output
    const char* output_filename = "bagged_imu.csv";
    FILE* fp_out = fopen(output_filename, "w+");
    fprintf(fp_out, "time(s),x(m),y(m),z(m),qx,qy,qz,qw\n");

    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        if (m.getTopic() == "/dvs/imu")
        {
            // imuMessages.push_back(m);
            sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
            if (imu != nullptr) {
                if ((int) (output_time) % 100 == 0)
                {
                    std::cout << "Adding IMU " << output_time << std::endl;
                }

                Vector6 imu_vec;
                imu_vec(0) = imu->linear_acceleration.x;
                imu_vec(1) = imu->linear_acceleration.y;
                imu_vec(2) = imu->linear_acceleration.z;
                imu_vec(3) = imu->angular_velocity.x;
                imu_vec(4) = imu->angular_velocity.y;
                imu_vec(5) = imu->angular_velocity.z;
                preintegrated->integrateMeasurement(imu_vec.head<3>(), imu_vec.tail<3>(), dt);
                correction_count++;

                auto preint_imu = dynamic_cast<const PreintegratedImuMeasurements&>(*preintegrated);
                ImuFactor imu_factor(X(correction_count - 1), V(correction_count - 1),
                                    X(correction_count), V(correction_count),
                                    B(correction_count - 1), preint_imu);
                graph->add(imu_factor);
                imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
                graph->add(BetweenFactor<imuBias::ConstantBias>(
                B(correction_count - 1), B(correction_count), zero_bias,
                bias_noise_model));

                prop_state = preintegrated->predict(prev_state, prev_bias);
                initial_values.insert(X(correction_count), prop_state.pose());
                initial_values.insert(V(correction_count), prop_state.v());
                initial_values.insert(B(correction_count), prev_bias);

                Values result;

                LevenbergMarquardtOptimizer optimizer(*graph, initial_values);
                result = optimizer.optimize();

                // Overwrite the beginning of the preintegration for the next step.
                prev_state = NavState(result.at<Pose3>(X(correction_count)),
                                    result.at<Vector3>(V(correction_count)));
                prev_bias = result.at<imuBias::ConstantBias>(B(correction_count));

                // Reset the preintegration object.
                preintegrated->resetIntegrationAndSetBias(prev_bias);

                Vector3 gtsam_position = prev_state.pose().translation();
                Quaternion gtsam_quat = prev_state.pose().rotation().toQuaternion();

                fprintf(fp_out, "%f,%f,%f,%f,%f,%f,%f,%f\n",
                        output_time, gtsam_position(0), gtsam_position(1),
                        gtsam_position(2), gtsam_quat.x(), gtsam_quat.y(), gtsam_quat.z(),
                        gtsam_quat.w());

                output_time += 1.0;
            }
        }
    }
    bag.close();
    fclose(fp_out);
    std::cout << "Complete" << std::endl;

    return 1;
}