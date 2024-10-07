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
        : CoordinateConvertor(msg), mgrs_code_(msg.mgrs_grid) {} 

    satellite_msgs::msg::Coordinate xy_to_lat_lon(const geometry_msgs::msg::Pose& pose) override;

private:
    std::string mgrs_code_; // Store the MGRS code from the MapProjectorInfo message
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
    else if (msg.projector_type == tier4_map_msgs::msg::MapProjectorInfo::MGRS)
    {
        return std::make_unique<MGRS>(msg);
    }
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

satellite_msgs::msg::Coordinate MGRS::xy_to_lat_lon(const geometry_msgs::msg::Pose& pose) {
    double lat, lon;

    // Split easting and northing from the pose's position (x = easting, y = northing)
    std::stringstream ss;
    ss << pose.position.x << "," << pose.position.y;

    std::string easting_str, northing_str;

    // Parse the easting and northing values from the comma-delimited string
    if (!std::getline(ss, easting_str, ',') || !std::getline(ss, northing_str, ',')) {
        RCLCPP_ERROR(this->get_logger(), "Invalid pose position format: x=%.6f, y=%.6f", 
                     pose.position.x, pose.position.y);
        return satellite_msgs::msg::Coordinate{};
    }

    // Convert the parsed easting and northing to double
    double MGRSEasting = std::stod(easting_str);
    double MGRSNorthing = std::stod(northing_str);

    // Format the MGRS code with full precision
    std::stringstream mgrs_code_ss;
    mgrs_code_ss << mgrs_code_ << std::fixed << std::setprecision(6)
                 << MGRSEasting << MGRSNorthing;

    // Create the final MGRS code string
    std::string mgrs_code = mgrs_code_ss.str();

    // Remove any spaces or dots from the MGRS code string
    mgrs_code.erase(std::remove_if(mgrs_code.begin(), mgrs_code.end(), [](char c) {
        return c == ' ' || c == '.';
    }), mgrs_code.end());

    // MGRS to latitude and longitude conversion logic
    try {
        int zone;
        bool northp;
        double easting, northing;
        int prec;

        // Convert MGRS to UTM coordinates
        GeographicLib::MGRS::Reverse(mgrs_code, zone, northp, easting, northing, prec);

        // Convert UTM coordinates to latitude and longitude
        GeographicLib::UTMUPS::Reverse(zone, northp, easting, northing, lat, lon);
    } catch (const GeographicLib::GeographicErr& e) {
        RCLCPP_ERROR(this->get_logger(), "Error converting MGRS to LL: %s", e.what());
        return satellite_msgs::msg::Coordinate{};
    }

    // Populate the Coordinate message with the converted lat/lon and altitude (from z position)
    satellite_msgs::msg::Coordinate coordinate;
    coordinate.latitude = lat;
    coordinate.longitude = lon;
    coordinate.altitude = z + pose.position.z; // Assume 'z' is a pre-defined altitude offset
    coordinate.azimuth = -1 * quaternion_to_azimuth(pose.orientation) * RAD_TO_DEG + 90; // Convert quaternion to azimuth

    return coordinate;
}