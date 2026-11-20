#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <lesta_interfaces/srv/save_training_data.hpp>
#include "lesta/core/GlobalMapper.h"
#include "lesta/core/FeatureExtractor.h"
#include "lesta/core/LabelGenerator.h"
#include "lesta/core/TraversabilityEstimator.h"
#include "lesta/core/TraversabilityMapper.h"
#include "lesta/types/label_point.h"
#include <filesystem>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <cstdint>
#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <unordered_set>

namespace {
namespace fs = std::filesystem;
using Cloud = pcl::PointCloud<Laser>;
using Map = height_mapping::HeightMap;
using Save = lesta_interfaces::srv::SaveTrainingData;

std::vector<grid_map::Index> collectIndices(
    const Map &map, const Cloud &cloud) {
  std::vector<grid_map::Index> result;
  result.reserve(cloud.size());
  for (const auto &point : cloud.points) {
    grid_map::Index index;
    if (map.getIndex(grid_map::Position(point.x, point.y), index)) {
      result.push_back(index);
    }
  }
  return result;
}

Cloud::Ptr transform(const Cloud::Ptr &input,
                     const geometry_msgs::msg::TransformStamped &tf) {
  auto out = std::make_shared<Cloud>();
  const Eigen::Affine3d pose = tf2::transformToEigen(tf.transform);
  pcl::transformPointCloud(*input, *out, pose.cast<float>().matrix());
  // PCL transforms coordinates but leaves frame_id unchanged; the mapper
  // explicitly rejects clouds not tagged with its map frame.
  out->header.frame_id = tf.header.frame_id;
  return out;
}

Cloud::Ptr cropXY(const Cloud::Ptr &cloud, double range) {
  auto temp = std::make_shared<Cloud>();
  auto out = std::make_shared<Cloud>();
  pcl::PassThrough<Laser> filter;
  filter.setInputCloud(cloud);
  filter.setFilterFieldName("x");
  filter.setFilterLimits(-range, range);
  filter.filter(*temp);
  filter.setInputCloud(temp);
  filter.setFilterFieldName("y");
  filter.setFilterLimits(-range, range);
  filter.filter(*out);
  return out;
}

class LeSTANode : public rclcpp::Node {
public:
  LeSTANode()
      : Node("lesta"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    mode_ = declare_parameter<std::string>("mode", "prediction");
    if (mode_ != "prediction" && mode_ != "mapping" && mode_ != "labels")
      throw std::invalid_argument("mode must be prediction, mapping or labels");

    const std::string map_frame = declare_parameter<std::string>("map_frame", "map");
    map_frame_ = map_frame;
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
    const std::string topic = declare_parameter<std::string>("lidarscan_topic", "/velodyne/points");
    const bool global = mode_ != "prediction";
    const auto dims = global ? 200.0 : 10.0;
    const auto map_x = declare_parameter<double>("map_length_x", dims);
    const auto map_y = declare_parameter<double>("map_length_y", dims);
    const auto resolution = declare_parameter<double>("grid_resolution", 0.1);
    const auto min_height = declare_parameter<double>("min_height_threshold", -0.2);
    const auto max_height = declare_parameter<double>("max_height_threshold", 1.5);
    const auto estimator = declare_parameter<std::string>("estimator_type", "StatMean");
    scan_range_ = declare_parameter<double>("scan_filter_range", global ? 6.0 : 5.0);
    remove_backpoints_ = declare_parameter<bool>("remove_backpoints", true);
    publish_rate_ = declare_parameter<double>("map_publish_rate", global ? 1.0 : 10.0);
    footprint_radius_ = declare_parameter<double>("footprint_radius", 0.3);
    max_traversable_step_ = declare_parameter<double>("max_traversable_step", 0.2);
    const auto pca_radius = declare_parameter<double>("pca_radius", 0.2);
    auto model_path = declare_parameter<std::string>("model_path", "");
    const auto feature_fields = declare_parameter<std::vector<std::string>>(
        "feature_fields", {"step", "slope", "roughness", "curvature"});
    const auto binary_threshold = declare_parameter<double>("binary_threshold", 0.5);
    const auto input_dimension = declare_parameter<int>("input_dimension", 4);
    const auto use_log_odds = declare_parameter<bool>("use_log_odds_update", true);
    const auto learning_rate = declare_parameter<double>("learning_rate", 0.7);
    const auto prob_min = declare_parameter<double>("prob_min", 0.05);
    const auto prob_max = declare_parameter<double>("prob_max", 0.95);
    const auto run_raycasting = declare_parameter<bool>("raycasting", true);
    raycasting_ = run_raycasting;
    if (publish_rate_ <= 0.0 || map_x <= 0 || map_y <= 0 || resolution <= 0)
      throw std::invalid_argument("invalid map geometry or publish rate");
    if (scan_range_ <= 0 || min_height >= max_height)
      throw std::invalid_argument("invalid scan range or height thresholds");

    lesta::HeightMapper::Config hcfg{
        estimator, map_frame, map_x, map_y, resolution, min_height, max_height};
    if (global) {
      lesta::GlobalMapper::Config gcfg{};
      static_cast<lesta::HeightMapper::Config &>(gcfg) = hcfg;
      gcfg.map_save_dir = "";
      global_mapper_ = std::make_unique<lesta::GlobalMapper>(gcfg);
      mapper_ = global_mapper_.get();
    } else {
      local_mapper_ = std::make_unique<lesta::HeightMapper>(hcfg);
      mapper_ = local_mapper_.get();
    }
    extractor_ = std::make_unique<lesta::FeatureExtractor>(
        lesta::FeatureExtractor::Config{pca_radius});
    if (mode_ == "labels") {
      labeler_ = std::make_unique<lesta::LabelGenerator>(
          lesta::LabelGenerator::Config{footprint_radius_, max_traversable_step_});
    } else {
      if (model_path.empty())
        throw std::invalid_argument("model_path is required for prediction/mapping");
      if (!fs::exists(model_path))
        throw std::invalid_argument("TorchScript model file not found: " + model_path);
      lesta::TraversabilityEstimator::Config cfg{
          model_path, static_cast<int>(input_dimension), static_cast<float>(binary_threshold), feature_fields};
      if (global) {
        lesta::TraversabilityMapper::Config mcfg{};
        static_cast<lesta::TraversabilityEstimator::Config &>(mcfg) = cfg;
        mcfg.use_log_odds_update = use_log_odds;
        mcfg.learning_rate = static_cast<float>(learning_rate);
        mcfg.prob_min = static_cast<float>(prob_min);
        mcfg.prob_max = static_cast<float>(prob_max);
        trav_mapper_ = std::make_unique<lesta::TraversabilityMapper>(mcfg);
      } else {
        trav_estimator_ = std::make_unique<lesta::TraversabilityEstimator>(cfg);
      }
    }

    const auto prefix = std::string("/lesta/") +
        (mode_ == "labels" ? "label_generation" :
         mode_ == "mapping" ? "mapping" : "prediction");
    map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(
        prefix + (mode_ == "labels" ? "/heightmap" : "/traversability"), 1);
    if (mode_ == "labels") {
      cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          prefix + "/map_labeled", 1);
      save_service_ = create_service<Save>(
          "/lesta/save_label_map",
          [this](const std::shared_ptr<Save::Request> request,
                 std::shared_ptr<Save::Response> response) {
            std::lock_guard<std::mutex> lock(mutex_);
            response->success = saveTrainingData(*request);
          });
    }
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          onScan(msg);
        });
    const auto interval = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(interval),
        [this] { publishMaps(); });
    RCLCPP_INFO(get_logger(), "LeSTA ROS2 mode=%s topic=%s", mode_.c_str(), topic.c_str());
  }

private:
  bool lookup(const std::string &target, const std::string &source,
              const rclcpp::Time &timestamp,
              geometry_msgs::msg::TransformStamped &out) {
    try {
      out = tf_buffer_.lookupTransform(target, source, timestamp,
                                       rclcpp::Duration::from_seconds(0.05));
      return true;
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "TF %s <- %s unavailable: %s",
                           target.c_str(), source.c_str(), e.what());
      return false;
    }
  }

  void onScan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    geometry_msgs::msg::TransformStamped sensor_to_base, base_to_map;
    const auto stamp = rclcpp::Time(msg->header.stamp);
    if (!lookup(robot_frame_, msg->header.frame_id, stamp, sensor_to_base) ||
        !lookup(map_frame_, robot_frame_, stamp, base_to_map))
      return;
    Cloud::Ptr scan = std::make_shared<Cloud>();
    pcl::fromROSMsg(*msg, *scan);
    if (scan->empty())
      return;
    auto base_scan = transform(scan, sensor_to_base);
    Cloud::Ptr filtered = std::make_shared<Cloud>();
    mapper_->fastHeightFilter<Laser>(base_scan, filtered);
    if (filtered->empty()) return;
    filtered = cropXY(filtered, scan_range_);
    if (remove_backpoints_) {
      auto front = std::make_shared<Cloud>();
      front->header = filtered->header;
      const auto angle_limit = 135.0 * std::acos(-1.0) / 180.0;
      for (const auto &p : filtered->points) {
        const auto a = std::atan2(p.y, p.x);
        if (a >= -angle_limit && a <= angle_limit) front->push_back(p);
      }
      filtered = front;
    }
    if (filtered->empty()) return;
    auto map_scan = transform(filtered, base_to_map);

    const auto base_tf = tf2::transformToEigen(base_to_map.transform);
    const auto sensor_tf = tf2::transformToEigen(sensor_to_base.transform);
    const Eigen::Vector3d sensor_origin = (base_tf * sensor_tf).translation();
    if (mode_ == "prediction") {
      const auto &p = base_to_map.transform.translation;
      mapper_->moveMapOrigin(grid_map::Position(p.x, p.y));
    } else if (!ready_) {
      // Global maps keep the origin chosen from the first usable scan.
      mapper_->setMapPosition(sensor_origin.head<2>());
    }
    // GlobalMapper tracks measured cells for label export and global mapping.
    auto raster = global_mapper_ ? global_mapper_->heightMapping<Laser>(map_scan)
                                 : mapper_->heightMapping<Laser>(map_scan);
    if (!raster || raster->empty()) return;
    auto indices = collectIndices(mapper_->getHeightMap(), *raster);
    if (indices.empty()) return;
    if (raycasting_) mapper_->raycasting<Laser>(sensor_origin.cast<float>(), raster);
    auto &map = mapper_->getHeightMap();
    map.setTimestamp(stamp.nanoseconds());
    extractor_->extractFeatures(map, indices);
    if (mode_ == "labels") {
      labeler_->addObstacles(map, indices);
      const auto &p = base_to_map.transform.translation;
      auto position = grid_map::Position(p.x, p.y);
      labeler_->addFootprint(map, position);
    } else if (mode_ == "mapping") {
      trav_mapper_->traversabilityMapping(map, indices);
    } else {
      trav_estimator_->estimateTraversability(map, indices);
    }
    ready_ = true;
  }

  pcl::PointCloud<lesta_types::LabelPoint> labelCloud() const {
    pcl::PointCloud<lesta_types::LabelPoint> out;
    const auto &map = mapper_->getHeightMap();
    if (!global_mapper_) return out;
    out.reserve(global_mapper_->getMeasuredGridIndices().size());
    for (const auto &index : global_mapper_->getMeasuredGridIndices()) {
      if (map.isEmptyAt(index)) continue;
      const auto &req = std::vector<std::string>{
        lesta::layers::Feature::STEP, lesta::layers::Feature::SLOPE,
        lesta::layers::Feature::ROUGHNESS, lesta::layers::Feature::CURVATURE,
        lesta::layers::Label::FOOTPRINT, lesta::layers::Label::TRAVERSABILITY};
      if (std::any_of(req.begin(), req.end(),
          [&](const std::string &layer) { return map.isEmptyAt(layer, index); }))
        continue;
      grid_map::Position3 pos;
      if (!map.getPosition3(height_mapping::layers::Height::ELEVATION, index, pos))
        continue;
      lesta_types::LabelPoint p{};
      p.x = pos.x(); p.y = pos.y(); p.z = pos.z();
      p.elevation = map.at(height_mapping::layers::Height::ELEVATION, index);
      p.step = map.at(lesta::layers::Feature::STEP, index);
      p.slope = map.at(lesta::layers::Feature::SLOPE, index);
      p.roughness = map.at(lesta::layers::Feature::ROUGHNESS, index);
      p.curvature = map.at(lesta::layers::Feature::CURVATURE, index);
      p.variance = map.at(height_mapping::layers::Height::ELEVATION_VARIANCE, index);
      p.footprint = map.at(lesta::layers::Label::FOOTPRINT, index);
      p.traversability_label = map.at(lesta::layers::Label::TRAVERSABILITY, index);
      out.push_back(p);
    }
    out.width = static_cast<std::uint32_t>(out.size());
    out.height = 1;
    out.is_dense = false;
    return out;
  }

  bool saveTrainingData(const Save::Request &request) {
    if (!ready_) return false;
    try {
      fs::path directory = request.directory.empty()
          ? fs::current_path() : fs::path(request.directory);
      if (!directory.empty() && directory.string().front() == '~') {
        const char *home = std::getenv("HOME");
        if (!home) return false;
        directory = fs::path(home) / directory.string().substr(1);
      }
      fs::create_directories(directory);
      fs::path name = request.filename.empty()
          ? fs::path("labelmap.pcd") : fs::path(request.filename).filename();
      if (name.extension() != ".pcd") name += ".pcd";
      const auto output = directory / name;
      const auto cloud = labelCloud();
      return !cloud.empty() && pcl::io::savePCDFileASCII(output.string(), cloud) == 0;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Save failed: %s", e.what());
      return false;
    }
  }

  void publishMaps() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ready_) return;
    const auto &map = mapper_->getHeightMap();
    std::vector<std::string> layers = map.getLayers();
    try {
      auto msg = grid_map::GridMapRosConverter::toMessage(map, layers);
      map_pub_->publish(*msg);
      if (cloud_pub_) {
        auto cloud = labelCloud();
        sensor_msgs::msg::PointCloud2 msg_cloud;
        pcl::toROSMsg(cloud, msg_cloud);
        msg_cloud.header.frame_id = map_frame_;
        msg_cloud.header.stamp = now();
        cloud_pub_->publish(msg_cloud);
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Map publication failed: %s", e.what());
    }
  }

  std::string mode_, map_frame_, robot_frame_;
  double scan_range_{6}, publish_rate_{1}, footprint_radius_{0.3}, max_traversable_step_{0.2};
  bool raycasting_{true}, ready_{false}, remove_backpoints_{true};
  std::mutex mutex_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<lesta::HeightMapper> local_mapper_;
  std::unique_ptr<lesta::GlobalMapper> global_mapper_;
  lesta::HeightMapper *mapper_{nullptr};
  std::unique_ptr<lesta::FeatureExtractor> extractor_;
  std::unique_ptr<lesta::LabelGenerator> labeler_;
  std::unique_ptr<lesta::TraversabilityEstimator> trav_estimator_;
  std::unique_ptr<lesta::TraversabilityMapper> trav_mapper_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Service<Save>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LeSTANode>());
  } catch (const std::exception &e) {
    std::cerr << "LeSTA ROS2 fatal: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
