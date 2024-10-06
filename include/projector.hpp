#include <memory>
#include <stdexcept>
#include <vector>

#include <tier4_map_msgs/msg/map_projector_info.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <satellite_msgs/msg/coordinate.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
const double RAD_TO_DEG = 180.0 / M_PI;

double quaternion_to_azimuth(const geometry_msgs::msg::Quaternion& orientation) {
    tf2::Quaternion quat(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    );

    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    return yaw;
}

class CoordinateConvertor
{
public:
    explicit CoordinateConvertor(const tier4_map_msgs::msg::MapProjectorInfo& msg)
        : z(msg.map_origin.altitude) {}

    static std::unique_ptr<CoordinateConvertor> create(const tier4_map_msgs::msg::MapProjectorInfo& msg);
    virtual ~CoordinateConvertor() = default;
    virtual satellite_msgs::msg::Coordinate xy_to_lat_lon(const geometry_msgs::msg::Pose& pose) = 0;
protected:
    double z;
};

class TranverseMercator : public CoordinateConvertor
{
public:
    TranverseMercator(const tier4_map_msgs::msg::MapProjectorInfo& msg)
        : CoordinateConvertor(msg),
          wgst_(std::make_unique<swri_transform_util::LocalXyWgs84Util>(
              msg.map_origin.latitude,
              msg.map_origin.longitude,
              0,
              msg.map_origin.altitude
          ))
    {}
    satellite_msgs::msg::Coordinate xy_to_lat_lon(const geometry_msgs::msg::Pose& pose) override;

private:
    std::unique_ptr<swri_transform_util::LocalXyWgs84Util> wgst_;
};

class MGRS : public CoordinateConvertor
{
public:
    MGRS(const tier4_map_msgs::msg::MapProjectorInfo& msg)
        : CoordinateConvertor(msg) {} // Call base class constructor

    satellite_msgs::msg::Coordinate xy_to_lat_lon(const geometry_msgs::msg::Pose& pose) override;
};

std::unique_ptr<CoordinateConvertor> CoordinateConvertor::create(const tier4_map_msgs::msg::MapProjectorInfo& msg)
{
    if (msg.vertical_datum != tier4_map_msgs::msg::MapProjectorInfo::WGS84)
    {
        throw std::runtime_error("Unsupported vertical datum: " + msg.vertical_datum);
    }

    if (msg.projector_type == tier4_map_msgs::msg::MapProjectorInfo::TRANSVERSE_MERCATOR)
    {
        return std::make_unique<TranverseMercator>(msg);
    }
    // else if (msg.projector_type == tier4_map_msgs::msg::MapProjectorInfo::MGRS)
    // {
    //     return std::make_unique<MGRS>(msg);
    // }
    else
    {
        throw std::runtime_error("Unsupported projector_type: " + msg.projector_type);
    }
}

satellite_msgs::msg::Coordinate TranverseMercator::xy_to_lat_lon(const geometry_msgs::msg::Pose& pose)
{
    double lat, lon;
    wgst_->ToWgs84(pose.position.x, pose.position.y, lat, lon);

    satellite_msgs::msg::Coordinate coordinate;
    coordinate.latitude = lat;
    coordinate.longitude = lon;
    coordinate.altitude = z + pose.position.z;
    coordinate.azimuth = -1* quaternion_to_azimuth(pose.orientation) * RAD_TO_DEG + 90; // respect to north
    return coordinate;
}

satellite_msgs::msg::Coordinate MGRS::xy_to_lat_lon(const geometry_msgs::msg::Pose& pose)
{
    satellite_msgs::msg::Coordinate coordinate;
    // Add conversion logic here
    return coordinate;
}
