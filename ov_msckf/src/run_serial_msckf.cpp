/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

// ============================================================================
// Deterministic offline runner for ROS2 bags.
//
// run_subscribe_msckf goes bag -> DDS -> subscriber callbacks, so the filter
// sees whatever survives transport under the CPU load of that particular run.
// Measured on urban39: repeats of ONE configuration produced between 14513 and
// 15335 output poses, i.e. up to 5% of the trajectory was lost, differently
// each time.
//
// This binary removes transport from the loop: it reads the bag with
// rosbag2_cpp and hands measurements to VioManager directly, in bag order, on
// one thread. Nothing is dropped and nothing races, so two runs of the same bag
// and config produce byte-identical trajectories.
//
// The feed order mirrors ROS2Visualizer's online semantics (images buffered,
// drained once an IMU sample passes the image time) so results stay comparable
// to the subscribe path rather than being a different estimator.
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>

#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "types/IMU.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

using namespace ov_msckf;

std::shared_ptr<VioManager> sys;

namespace {

struct StampedImage {
  double timestamp = -1;
  cv::Mat image;
};

std::deque<StampedImage> buf_cam0, buf_cam1;
std::deque<ov_core::CameraData> camera_queue;
std::map<int, double> camera_last_timestamp;
std::ofstream of_traj;

size_t n_poses = 0, n_pairs = 0, n_unmatched = 0, n_dropped_rate = 0;
size_t n_imu = 0, n_wheel = 0;

/// Append the current state in TUM format, matching what ROS2Visualizer
/// publishes on /poseimu (same time base, same quaternion convention), so the
/// existing evo pipeline compares like with like.
void write_pose() {
  if (!sys->initialized() || !of_traj.is_open())
    return;
  std::shared_ptr<State> state = sys->get_state();
  double timestamp_inI = state->_timestamp + state->_calib_dt_CAMtoIMU->value()(0);
  const Eigen::Vector4d q = state->_imu->quat();
  const Eigen::Vector3d p = state->_imu->pos();
  of_traj << std::fixed << std::setprecision(9) << timestamp_inI << " " << p(0) << " " << p(1) << " " << p(2) << " " << q(0) << " "
          << q(1) << " " << q(2) << " " << q(3) << "\n";
  n_poses++;
}

/// Drain every buffered image that the just-fed IMU sample has passed.
/// Same condition as the online callback, minus the threading.
void feed_pending_cameras(double timestamp_imu) {
  double timestamp_imu_inC = timestamp_imu - sys->get_state()->_calib_dt_CAMtoIMU->value()(0);
  while (!camera_queue.empty() && camera_queue.at(0).timestamp < timestamp_imu_inC) {
    sys->feed_measurement_camera(camera_queue.at(0));
    write_pose();
    camera_queue.pop_front();
  }
}

/// Pair the two image streams by header stamp. message_filters' ApproximateTime
/// does this online; here an unmatched image is dropped explicitly and counted.
void try_pair_stereo(double tol) {
  while (!buf_cam0.empty() && !buf_cam1.empty()) {

    const double t0 = buf_cam0.front().timestamp;
    const double t1 = buf_cam1.front().timestamp;

    if (std::fabs(t0 - t1) > tol) {
      if (t0 < t1)
        buf_cam0.pop_front();
      else
        buf_cam1.pop_front();
      n_unmatched++;
      continue;
    }

    // Decimation to track_frequency, mirroring ROS2Visualizer::callback_stereo
    const double time_delta = 1.0 / sys->get_params().track_frequency;
    if (camera_last_timestamp.find(0) != camera_last_timestamp.end() && t0 < camera_last_timestamp.at(0) + time_delta) {
      buf_cam0.pop_front();
      buf_cam1.pop_front();
      n_dropped_rate++;
      continue;
    }
    camera_last_timestamp[0] = t0;

    ov_core::CameraData message;
    message.timestamp = t0;
    message.sensor_ids.push_back(0);
    message.sensor_ids.push_back(1);
    message.images.push_back(buf_cam0.front().image);
    message.images.push_back(buf_cam1.front().image);

    if (sys->get_params().use_mask) {
      message.masks.push_back(sys->get_params().masks.at(0));
      message.masks.push_back(sys->get_params().masks.at(1));
    } else {
      message.masks.push_back(cv::Mat::zeros(buf_cam0.front().image.rows, buf_cam0.front().image.cols, CV_8UC1));
      message.masks.push_back(cv::Mat::zeros(buf_cam1.front().image.rows, buf_cam1.front().image.cols, CV_8UC1));
    }

    buf_cam0.pop_front();
    buf_cam1.pop_front();
    camera_queue.push_back(message);
    std::sort(camera_queue.begin(), camera_queue.end());
    n_pairs++;
  }
}

/// Monocular: no pairing, just decimation and enqueue.
void push_monocular(const StampedImage &img) {
  const double time_delta = 1.0 / sys->get_params().track_frequency;
  if (camera_last_timestamp.find(0) != camera_last_timestamp.end() && img.timestamp < camera_last_timestamp.at(0) + time_delta) {
    n_dropped_rate++;
    return;
  }
  camera_last_timestamp[0] = img.timestamp;

  ov_core::CameraData message;
  message.timestamp = img.timestamp;
  message.sensor_ids.push_back(0);
  message.images.push_back(img.image);
  if (sys->get_params().use_mask) {
    message.masks.push_back(sys->get_params().masks.at(0));
  } else {
    message.masks.push_back(cv::Mat::zeros(img.image.rows, img.image.cols, CV_8UC1));
  }
  camera_queue.push_back(message);
  std::sort(camera_queue.begin(), camera_queue.end());
  n_pairs++;
}

bool decode_image(const rclcpp::SerializedMessage &serialized, StampedImage &out) {
  static rclcpp::Serialization<sensor_msgs::msg::Image> ser;
  sensor_msgs::msg::Image msg;
  ser.deserialize_message(&serialized, &msg);
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    PRINT_ERROR(RED "[SERIAL]: cv_bridge exception: %s\n" RESET, e.what());
    return false;
  }
  out.timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
  out.image = cv_ptr->image;
  return true;
}

} // namespace

int main(int argc, char **argv) {

  std::string config_path = "unset_path_to_config.yaml";
  if (argc > 1) {
    config_path = argv[1];
  }

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("run_serial_msckf", options);
  node->get_parameter<std::string>("config_path", config_path);

  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
  parser->set_node(node);

  std::string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  // Create our VIO system.
  //
  // These three are forced, not read from config: the whole point of this
  // binary is a repeatable run, and each of them is a source of nondeterminism.
  VioManagerOptions params;
  params.print_and_load(parser);
  params.use_multi_threading_subs = false;
  params.use_multi_threading_pubs = false;
  params.num_opencv_threads = 1;
  sys = std::make_shared<VioManager>(params);

  if (!parser->successful()) {
    PRINT_ERROR(RED "[SERIAL]: unable to parse all parameters, please fix\n" RESET);
    sys.reset();
    std::exit(EXIT_FAILURE);
  }

  //===================================================================================
  // Topics and run options
  //===================================================================================

  std::string topic_imu;
  node->get_parameter<std::string>("topic_imu", topic_imu);
  parser->parse_external("relative_config_imu", "imu0", "rostopic", topic_imu);

  std::vector<std::string> topic_cameras;
  for (int i = 0; i < params.state_options.num_cameras; i++) {
    std::string cam_topic = "/cam" + std::to_string(i) + "/image_raw";
    node->get_parameter<std::string>("topic_camera" + std::to_string(i), cam_topic);
    parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "rostopic", cam_topic);
    topic_cameras.emplace_back(cam_topic);
  }

  const std::string topic_wheel = params.wheel_options.topic;
  const bool use_wheel = params.state_options.do_wheel_odometry;

  std::string path_to_bag;
  node->get_parameter<std::string>("path_bag", path_to_bag);

  std::string path_traj = "/tmp/traj_estimate.tum";
  node->get_parameter<std::string>("path_traj", path_traj);

  double bag_start = 0.0, bag_durr = -1.0;
  node->get_parameter<double>("bag_start", bag_start);
  node->get_parameter<double>("bag_durr", bag_durr);

  double stereo_sync_tol = 0.02;
  node->get_parameter<double>("stereo_sync_tol", stereo_sync_tol);

  PRINT_INFO("[SERIAL]: bag        : %s\n", path_to_bag.c_str());
  PRINT_INFO("[SERIAL]: trajectory : %s\n", path_traj.c_str());
  PRINT_INFO("[SERIAL]: imu topic  : %s\n", topic_imu.c_str());
  for (size_t i = 0; i < topic_cameras.size(); i++)
    PRINT_INFO("[SERIAL]: cam%zu topic : %s\n", i, topic_cameras.at(i).c_str());
  if (use_wheel)
    PRINT_INFO("[SERIAL]: wheel topic: %s\n", topic_wheel.c_str());
  PRINT_INFO("[SERIAL]: start=%.1fs duration=%.1fs\n", bag_start, bag_durr);

  if (path_to_bag.empty()) {
    PRINT_ERROR(RED "[SERIAL]: no bag given, set the path_bag parameter\n" RESET);
    sys.reset();
    return EXIT_FAILURE;
  }

  of_traj.open(path_traj, std::ofstream::out | std::ofstream::trunc);
  if (!of_traj.is_open()) {
    PRINT_ERROR(RED "[SERIAL]: could not open %s for writing\n" RESET, path_traj.c_str());
    sys.reset();
    return EXIT_FAILURE;
  }

  //===================================================================================
  // Read the bag and feed, streaming so a 25 GiB bag does not need to fit in RAM
  //===================================================================================

  rosbag2_cpp::Reader reader;
  try {
    reader.open(path_to_bag);
  } catch (const std::exception &e) {
    PRINT_ERROR(RED "[SERIAL]: could not open bag: %s\n" RESET, e.what());
    of_traj.close();
    sys.reset();
    return EXIT_FAILURE;
  }

  rosbag2_storage::StorageFilter filter;
  filter.topics.push_back(topic_imu);
  for (const auto &t : topic_cameras)
    filter.topics.push_back(t);
  if (use_wheel)
    filter.topics.push_back(topic_wheel);
  reader.set_filter(filter);

  rclcpp::Serialization<sensor_msgs::msg::Imu> ser_imu;
  rclcpp::Serialization<nav_msgs::msg::Odometry> ser_odom;

  const auto wall_start = std::chrono::steady_clock::now();
  double time_first = -1, time_last = -1;
  size_t n_msgs = 0;

  while (reader.has_next() && rclcpp::ok()) {

    auto bag_msg = reader.read_next();
    const double t_bag = 1e-9 * (double)bag_msg->time_stamp;

    if (time_first < 0)
      time_first = t_bag;
    if (t_bag < time_first + bag_start)
      continue;
    if (bag_durr > 0 && t_bag > time_first + bag_start + bag_durr)
      break;
    time_last = t_bag;
    n_msgs++;

    rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);

    // ---- IMU: feed, then release any image it has passed ----
    if (bag_msg->topic_name == topic_imu) {
      sensor_msgs::msg::Imu msg;
      ser_imu.deserialize_message(&serialized, &msg);
      ov_core::ImuData message;
      message.timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
      message.wm << msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z;
      message.am << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;
      sys->feed_measurement_imu(message);
      n_imu++;
      feed_pending_cameras(message.timestamp);
      continue;
    }

    // ---- Wheel odometry ----
    if (use_wheel && bag_msg->topic_name == topic_wheel) {
      nav_msgs::msg::Odometry msg;
      ser_odom.deserialize_message(&serialized, &msg);
      ov_core::OdometryData data;
      data.timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
      data.linear_velocity << msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z;
      data.angular_velocity << msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z;
      sys->feed_measurement_wheel(data);
      n_wheel++;
      continue;
    }

    // ---- Images ----
    if (bag_msg->topic_name == topic_cameras.at(0)) {
      StampedImage img;
      if (!decode_image(serialized, img))
        continue;
      if (params.state_options.num_cameras == 2) {
        buf_cam0.push_back(img);
        try_pair_stereo(stereo_sync_tol);
      } else {
        push_monocular(img);
      }
    } else if (params.state_options.num_cameras == 2 && bag_msg->topic_name == topic_cameras.at(1)) {
      StampedImage img;
      if (!decode_image(serialized, img))
        continue;
      buf_cam1.push_back(img);
      try_pair_stereo(stereo_sync_tol);
    }

    // Progress, keyed off images so it ticks at a readable rate
    if (n_pairs > 0 && n_pairs % 500 == 0 && camera_queue.size() <= 1) {
      const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
      PRINT_INFO("[SERIAL]: %.1fs of bag processed (%zu frames, %zu poses, %.2fx realtime)\n", time_last - time_first, n_pairs, n_poses,
                 (time_last - time_first) / std::max(elapsed, 1e-6));
    }
  }

  of_traj.close();

  const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
  PRINT_INFO("\n");
  PRINT_INFO("[SERIAL]: ==================== done ====================\n");
  PRINT_INFO("[SERIAL]: messages read      : %zu (imu %zu, wheel %zu)\n", n_msgs, n_imu, n_wheel);
  PRINT_INFO("[SERIAL]: frames tracked     : %zu\n", n_pairs);
  PRINT_INFO("[SERIAL]: frames decimated   : %zu (track_frequency=%.1f)\n", n_dropped_rate, params.track_frequency);
  PRINT_INFO("[SERIAL]: images unmatched   : %zu\n", n_unmatched);
  PRINT_INFO("[SERIAL]: poses written      : %zu -> %s\n", n_poses, path_traj.c_str());
  PRINT_INFO("[SERIAL]: bag time processed : %.1fs in %.1fs wall (%.2fx realtime)\n", time_last - time_first, elapsed,
             (time_last - time_first) / std::max(elapsed, 1e-6));

  // Release before the exit handlers run, while the context is still alive
  sys.reset();
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
