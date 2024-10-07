#include <memory>
#include <chrono>
#include <stdexcept>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <swri_transform_util/local_xy_util.h>

#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <tier4_map_msgs/msg/map_projector_info.hpp>
#include <satellite_msgs/msg/monitor.hpp>

#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include "projector.hpp"

uint32_t calculate_xor_checksum(const std::array<uint8_t, 16> &uuid)
{
  uint32_t checksum = 0x00000000;
  for (const auto &byte : uuid)
  {
    checksum ^= byte;
  }
  return checksum;
}

class AutowareToSatellite : public rclcpp::Node
{
public:
  AutowareToSatellite()
      : Node("autoware_to_satellite"), tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())), tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    this->declare_parameter<std::string>("map_frame_id", "map");
    this->declare_parameter<std::string>("vehicle_frame_id", "base_link");
    this->declare_parameter<double>("max_steering", 38.05);
    this->declare_parameter<double>("target_rate", 10.);

    this->get_parameter("map_frame_id", map_frame_);
    this->get_parameter("vehicle_frame_id", vehicle_frame_);
    this->get_parameter("max_steering", max_steering_);
    this->get_parameter("target_rate", target_rate_);

    rclcpp::QoS qos_in(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));

    monitor_pub_ = this->create_publisher<satellite_msgs::msg::Monitor>("monitor", qos_in);

    steering_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>(
        "/vehicle/status/steering_status", qos_in,
        std::bind(&AutowareToSatellite::steering_cb, this, std::placeholders::_1));
    gear_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::GearReport>(
        "/vehicle/status/gear_status", qos_in, std::bind(&AutowareToSatellite::gear_cb, this, std::placeholders::_1));
    velocity_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
        "/vehicle/status/velocity_status", qos_in,
        std::bind(&AutowareToSatellite::velocity_cb, this, std::placeholders::_1));
    turn_indicators_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(
        "/vehicle/status/turn_indicators_status", qos_in,
        std::bind(&AutowareToSatellite::turn_indicators_cb, this, std::placeholders::_1));
    predicted_objects_sub_ = this->create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
        "/perception/object_recognition/objects", qos_in,
        std::bind(&AutowareToSatellite::predicted_objects_cb, this, std::placeholders::_1));

    rclcpp::QoS map_projector_qos = qos_in.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    map_projector_sub_ = this->create_subscription<tier4_map_msgs::msg::MapProjectorInfo>(
        "/map/map_projector_info", map_projector_qos,
        std::bind(&AutowareToSatellite::map_projector_cb, this, std::placeholders::_1));

    auto timer_duration = std::chrono::duration<double>(1.0 / target_rate_);
    timer_ = this->create_wall_timer(timer_duration, std::bind(&AutowareToSatellite::construct_monitor_msg, this));
  }

  void construct_monitor_msg()
  {
    if (projector_received_ == false)
      return;

    satellite_msgs::msg::Monitor msg;
    try
    {
      msg.coordinate = get_vehicle_pose();
    }
    catch (...)
    {
      RCLCPP_INFO(this->get_logger(), "Failed to get vehicle pose");
    }

    msg.header.frame_id = "/wgs84";
    msg.header.stamp = this->get_clock()->now();
    msg.vehicle = get_vehicle_info();
    msg.objects = get_satellite_objects(predicted_objects_);
    msg.road_line = 0;

    monitor_pub_->publish(msg);
  }

private:
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turn_indicators_sub_;
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr predicted_objects_sub_;
  rclcpp::Subscription<tier4_map_msgs::msg::MapProjectorInfo>::SharedPtr map_projector_sub_;
  rclcpp::Publisher<satellite_msgs::msg::Monitor>::SharedPtr monitor_pub_;

  autoware_auto_vehicle_msgs::msg::SteeringReport steering_report_;
  autoware_auto_vehicle_msgs::msg::GearReport gear_report_;
  autoware_auto_vehicle_msgs::msg::VelocityReport velocity_report_;
  autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport turn_indicators_report_;
  autoware_auto_perception_msgs::msg::PredictedObjects predicted_objects_;

  std::unique_ptr<CoordinateConvertor> projector_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool projector_received_ = false;
  std::string map_frame_, vehicle_frame_;
  double max_steering_, target_rate_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  satellite_msgs::msg::Coordinate get_vehicle_pose()
  {
    geometry_msgs::msg::TransformStamped transform_stamped =
        tf_buffer_->lookupTransform(map_frame_, vehicle_frame_, tf2::TimePointZero);

    geometry_msgs::msg::Pose vp;
    vp.position.x = transform_stamped.transform.translation.x;
    vp.position.y = transform_stamped.transform.translation.y;
    vp.position.z = transform_stamped.transform.translation.z;
    vp.orientation = transform_stamped.transform.rotation;

    satellite_msgs::msg::Coordinate coordinate = projector_->xy_to_lat_lon(vp);
    return coordinate;
  }

  satellite_msgs::msg::Vehicle get_vehicle_info()
  {
    satellite_msgs::msg::Vehicle vehicle;

    vehicle.velocity = velocity_report_.longitudinal_velocity;
    vehicle.steering = steering_report_.steering_tire_angle * max_steering_;

    // TODO: Need to get values from the raw_vehicle_convertor
    vehicle.accel_target = 0.0;
    vehicle.brake_target = 0.0;

    if (turn_indicators_report_.report == autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT)
      vehicle.turn_left = true;
    else
      vehicle.turn_left = false;

    if (turn_indicators_report_.report == autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT)
      vehicle.turn_right = true;
    else
      vehicle.turn_right = false;

    if ((gear_report_.report == autoware_auto_vehicle_msgs::msg::GearReport::NEUTRAL) ||
        (gear_report_.report == autoware_auto_vehicle_msgs::msg::GearReport::PARK))
    {
      vehicle.gear = satellite_msgs::msg::Vehicle::GEAR_NEUTRAL;
    }
    else if (gear_report_.report == autoware_auto_vehicle_msgs::msg::GearReport::REVERSE ||
             gear_report_.report == autoware_auto_vehicle_msgs::msg::GearReport::REVERSE_2)
    {
      vehicle.gear = satellite_msgs::msg::Vehicle::GEAR_REVERSE;
    }
    else if (gear_report_.report == autoware_auto_vehicle_msgs::msg::GearReport::NONE)
    {
      vehicle.gear = satellite_msgs::msg::Vehicle::GEAR_UNKNOWN;
    }
    else
    {
      vehicle.gear = satellite_msgs::msg::Vehicle::GEAR_FORWARD;
    }

    return vehicle;
  }

  std::vector<satellite_msgs::msg::Object>
  get_satellite_objects(autoware_auto_perception_msgs::msg::PredictedObjects msg)
  {
    std::vector<satellite_msgs::msg::Object> objects;
    bool should_transform = false;

    if (msg.header.frame_id != map_frame_)
    {
      should_transform = true;
    }

    for (auto predicted_obj : msg.objects)
    {
      satellite_msgs::msg::Object satellite_object;
      geometry_msgs::msg::Pose pose;
      if (should_transform)
      {
        try
        {
          geometry_msgs::msg::TransformStamped transform_stamped =
              tf_buffer_->lookupTransform(map_frame_, msg.header.frame_id, msg.header.stamp);
          tf2::doTransform(predicted_obj.kinematics.initial_pose_with_covariance.pose, pose, transform_stamped);
        }
        catch (tf2::TransformException &ex)
        {
          RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
          continue;
        }
      }
      else
      {
        pose = predicted_obj.kinematics.initial_pose_with_covariance.pose;
      }
      satellite_object.coordinate = projector_->xy_to_lat_lon(pose);

      if (predicted_obj.classification.size() > 0)
      {
        if (predicted_obj.classification.at(0).label == autoware_auto_perception_msgs::msg::ObjectClassification::CAR)
        {
          satellite_object.classification = satellite_msgs::msg::Object::CLASSIFICATION_CAR;
        }
        else if ((predicted_obj.classification.at(0).label ==
                  autoware_auto_perception_msgs::msg::ObjectClassification::TRUCK) ||
                 (predicted_obj.classification.at(0).label ==
                  autoware_auto_perception_msgs::msg::ObjectClassification::BUS) ||
                 (predicted_obj.classification.at(0).label ==
                  autoware_auto_perception_msgs::msg::ObjectClassification::TRAILER))
        {
          satellite_object.classification = satellite_msgs::msg::Object::CLASSIFICATION_TRUCK;
        }
        else if (predicted_obj.classification.at(0).label ==
                 autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE)
        {
          satellite_object.classification = satellite_msgs::msg::Object::CLASSIFICATION_MOTORCYCLE;
        }
        else if (predicted_obj.classification.at(0).label ==
                 autoware_auto_perception_msgs::msg::ObjectClassification::BICYCLE)
        {
          satellite_object.classification = satellite_msgs::msg::Object::CLASSIFICATION_BIKE;
        }
        else if (predicted_obj.classification.at(0).label ==
                 autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN)
        {
          satellite_object.classification = satellite_msgs::msg::Object::CLASSIFICATION_PEDESTRIAN;
        }
      }

      satellite_object.velocity =
          std::sqrt(std::pow(predicted_obj.kinematics.initial_twist_with_covariance.twist.linear.x, 2) +
                    std::pow(predicted_obj.kinematics.initial_twist_with_covariance.twist.linear.y, 2) +
                    std::pow(predicted_obj.kinematics.initial_twist_with_covariance.twist.linear.z, 2));

      satellite_object.id = calculate_xor_checksum(predicted_obj.object_id.uuid);

      objects.push_back(satellite_object);
    }
    return objects;
  }

  void steering_cb(const autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr msg)
  {
    steering_report_ = *msg;
  }

  void gear_cb(const autoware_auto_vehicle_msgs::msg::GearReport::SharedPtr msg)
  {
    gear_report_ = *msg;
  }

  void velocity_cb(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg)
  {
    velocity_report_ = *msg;
  }

  void turn_indicators_cb(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::SharedPtr msg)
  {
    turn_indicators_report_ = *msg;
  }

  void predicted_objects_cb(const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr msg)
  {
    predicted_objects_ = *msg;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutowareToSatellite>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}