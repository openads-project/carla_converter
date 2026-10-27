// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

// Conversion of perception_msgs to ETSI ITS messages, ported from the subset of ad2etsi
// (egodata2cam, objectlist2cpm) used by this package.

#include <carla_converter/etsi_conversion.hpp>

#include <GeographicLib/UTMUPS.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <etsi_its_msgs_utils/cam_access.hpp>
#include <etsi_its_msgs_utils/cpm_ts_access.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <perception_msgs_utils/object_access.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_perception_msgs/tf2_perception_msgs.hpp>

namespace carla_converter::etsi {

namespace {

namespace gm = geometry_msgs::msg;
namespace pi = perception_msgs::msg;
namespace oa = perception_msgs::object_access;
namespace etsi_cam = etsi_its_cam_msgs::msg;
namespace etsi_cpm = etsi_its_cpm_ts_msgs::msg;
namespace cam_access = etsi_its_cam_msgs::access;
namespace cpm_access = etsi_its_cpm_ts_msgs::access;

using st = etsi_cam::StationType;
using oc = pi::ObjectClassification;

constexpr uint8_t kProtocolVersion = 2;

// maximum number of classifications per perceived object and minimum probability of a reported classification
constexpr size_t kMaxClassificationsPerObject = 8;
constexpr double kMinClassificationProbability = 0.01;

// map from perception_msgs ObjectClassification to CAM StationType
const std::map<uint8_t, uint8_t> kClassificationToStationType{{oc::UNCLASSIFIED, st::UNKNOWN},
                                                              {oc::PEDESTRIAN, st::PEDESTRIAN},
                                                              {oc::BICYCLE, st::CYCLIST},
                                                              {oc::MOTORCYCLE, st::MOTORCYCLE},
                                                              {oc::CAR, st::PASSENGER_CAR},
                                                              {oc::UTILITY, st::HEAVY_TRUCK},
                                                              {oc::VAN, st::PASSENGER_CAR},
                                                              {oc::BUS, st::BUS},
                                                              {oc::ANIMAL, st::UNKNOWN},
                                                              {oc::ROAD_OBSTACLE, st::UNKNOWN},
                                                              {oc::TRAIN, st::TRAM},
                                                              {oc::TRAILER, st::TRAILER},
                                                              {oc::VRU, st::PEDESTRIAN},
                                                              {oc::MICRO, st::CYCLIST},
                                                              {oc::CAR_UNION, st::PASSENGER_CAR},
                                                              {oc::TRUCK_UNION, st::HEAVY_TRUCK},
                                                              {oc::BIKE_UNION, st::CYCLIST},
                                                              {oc::UNKNOWN, st::UNKNOWN}};

/**
 * @brief Returns the logger used for ETSI conversion warnings
 *
 * @return logger
 */
rclcpp::Logger logger() { return rclcpp::get_logger("carla_converter"); }

/**
 * @brief Returns the TF frame name of a UTM zone, e.g. "utm_32N"
 *
 * @param zone UTM zone
 * @param is_northp UTM hemisphere
 * @return UTM frame name
 */
std::string utmFrame(const int zone, const bool is_northp) { return "utm_" + std::to_string(zone) + (is_northp ? "N" : "S"); }

/**
 * @brief Converts a variance to a standard deviation, mapping invalid (negative or non-finite) variances to an
 *        infinite standard deviation, which the ETSI setters encode as unavailable
 *
 * @param variance variance
 * @return standard deviation
 */
double stdDev(const double variance) {
  if (!std::isfinite(variance) || variance < 0.0) return std::numeric_limits<double>::infinity();
  return std::sqrt(variance);
}

/**
 * @brief Converts a UTM position to WGS84 and returns the meridian convergence at that position
 *
 * @param zone UTM zone
 * @param is_northp UTM hemisphere
 * @param easting UTM easting [m]
 * @param northing UTM northing [m]
 * @param latitude output latitude [deg]
 * @param longitude output longitude [deg]
 * @return meridian convergence [deg]
 */
double utmToWgs84(
    const int zone, const bool is_northp, const double easting, const double northing, double& latitude, double& longitude) {
  double grid_convergence = 0.0;
  double scale = 0.0;
  try {
    GeographicLib::UTMUPS::Reverse(zone, is_northp, easting, northing, latitude, longitude, grid_convergence, scale);
  } catch (const GeographicLib::GeographicErr& e) {
    throw std::invalid_argument(e.what());
  }
  return grid_convergence;
}

/**
 * @brief Converts a UTM yaw (counterclockwise from grid east) to an ETSI heading (clockwise from true north)
 *
 * @param yaw_deg UTM yaw [deg]
 * @param grid_convergence meridian convergence at the position [deg]
 * @return ETSI heading in [0, 360) [deg]
 */
double utmYawToEtsiHeading(const double yaw_deg, const double grid_convergence) {
  double heading = std::fmod(90.0 - yaw_deg + grid_convergence, 360.0);
  if (heading < 0.0) heading += 360.0;
  return heading;
}

/**
 * @brief Converts a perception_msgs classification type to a CPM object class
 *
 * @param type perception_msgs ObjectClassification type
 * @param object_id id of the object, used for logging
 * @return CPM object class
 */
etsi_cpm::ObjectClass toCpmObjectClass(const uint8_t type, const uint32_t object_id) {
  etsi_cpm::ObjectClass object_class;
  switch (type) {
    case oc::UNCLASSIFIED:
    case oc::UNKNOWN:
      object_class.choice = etsi_cpm::ObjectClass::CHOICE_VEHICLE_SUB_CLASS;
      object_class.vehicle_sub_class.value = etsi_cpm::TrafficParticipantType::UNKNOWN;
      break;
    case oc::PEDESTRIAN:
    case oc::VRU:
      object_class.choice = etsi_cpm::ObjectClass::CHOICE_VRU_SUB_CLASS;
      object_class.vru_sub_class.choice = etsi_cpm::VruProfileAndSubprofile::CHOICE_PEDESTRIAN;
      object_class.vru_sub_class.pedestrian.value = etsi_cpm::VruSubProfilePedestrian::UNAVAILABLE;
      break;
    case oc::BICYCLE:
    case oc::BIKE_UNION:
    case oc::MICRO:
      object_class.choice = etsi_cpm::ObjectClass::CHOICE_VRU_SUB_CLASS;
      object_class.vru_sub_class.choice = etsi_cpm::VruProfileAndSubprofile::CHOICE_BICYCLIST_AND_LIGHT_VRU_VEHICLE;
      object_class.vru_sub_class.bicyclist_and_light_vru_vehicle.value = etsi_cpm::VruSubProfileBicyclist::UNAVAILABLE;
      break;
    case oc::MOTORCYCLE:
      object_class.choice = etsi_cpm::ObjectClass::CHOICE_VRU_SUB_CLASS;
      object_class.vru_sub_class.choice = etsi_cpm::VruProfileAndSubprofile::CHOICE_MOTORCYCLIST;
      object_class.vru_sub_class.motorcyclist.value = etsi_cpm::VruSubProfileMotorcyclist::UNAVAILABLE;
      break;
    case oc::CAR:
    case oc::CAR_UNION:
    case oc::VAN:
      object_class.choice = etsi_cpm::ObjectClass::CHOICE_VEHICLE_SUB_CLASS;
      object_class.vehicle_sub_class.value = etsi_cpm::TrafficParticipantType::PASSENGER_CAR;
      break;
    case oc::UTILITY:
    case oc::TRUCK_UNION:
      object_class.choice = etsi_cpm::ObjectClass::CHOICE_VEHICLE_SUB_CLASS;
      object_class.vehicle_sub_class.value = etsi_cpm::TrafficParticipantType::HEAVY_TRUCK;
      break;
    case oc::BUS:
      object_class.choice = etsi_cpm::ObjectClass::CHOICE_VEHICLE_SUB_CLASS;
      object_class.vehicle_sub_class.value = etsi_cpm::TrafficParticipantType::BUS;
      break;
    case oc::ANIMAL:
      object_class.choice = etsi_cpm::ObjectClass::CHOICE_VRU_SUB_CLASS;
      object_class.vru_sub_class.choice = etsi_cpm::VruProfileAndSubprofile::CHOICE_ANIMAL;
      object_class.vru_sub_class.animal.value = etsi_cpm::VruSubProfileAnimal::UNAVAILABLE;
      break;
    case oc::ROAD_OBSTACLE:
      object_class.choice = etsi_cpm::ObjectClass::CHOICE_OTHER_SUB_CLASS;
      object_class.other_sub_class.value = etsi_cpm::OtherSubClass::UNKNOWN;
      break;
    case oc::TRAIN:
      object_class.choice = etsi_cpm::ObjectClass::CHOICE_VEHICLE_SUB_CLASS;
      object_class.vehicle_sub_class.value = etsi_cpm::TrafficParticipantType::TRAM;
      break;
    case oc::TRAILER:
      object_class.choice = etsi_cpm::ObjectClass::CHOICE_VEHICLE_SUB_CLASS;
      object_class.vehicle_sub_class.value = etsi_cpm::TrafficParticipantType::TRAILER;
      break;
    default:
      RCLCPP_WARN_STREAM(logger(), "Object with id " << object_id << " has unknown classification type " << static_cast<int>(type)
                                                     << ", reporting it as unknown in CPM");
      object_class.choice = etsi_cpm::ObjectClass::CHOICE_VEHICLE_SUB_CLASS;
      object_class.vehicle_sub_class.value = etsi_cpm::TrafficParticipantType::UNKNOWN;
      break;
  }
  return object_class;
}

/**
 * @brief Converts a perception_msgs object, given in the CPM's UTM frame, to a CPM perceived object
 *
 * @param object object to convert
 * @param utm_frame UTM frame the object is given in
 * @param cpm CPM the perceived object is added to, whose reference position the object position is relative to
 * @return CPM perceived object
 */
etsi_cpm::PerceivedObject toPerceivedObject(const pi::Object& object,
                                            const std::string& utm_frame,
                                            etsi_cpm::CollectivePerceptionMessage& cpm) {
  const pi::ObjectState& state = object.state;
  const auto covariance_at = [&state](const int index) { return oa::getContinuousStateCovarianceAt(state, index, index); };

  etsi_cpm::PerceivedObject perceived_object;

  // position (mandatory, relative to the CPM reference position) and measurement delta time
  gm::PointStamped utm_position;
  utm_position.header.frame_id = utm_frame;
  utm_position.point = oa::getPosition(state);
  cpm_access::initPerceivedObjectWithUTMPosition(cpm, perceived_object, utm_position, 0);
  cpm_access::setUTMPositionOfPerceivedObject(
      cpm, perceived_object, utm_position, stdDev(covariance_at(oa::indexX(state.model_id))),
      stdDev(covariance_at(oa::indexY(state.model_id))), stdDev(covariance_at(oa::indexZ(state.model_id))));

  cpm_access::setIdOfPerceivedObject(perceived_object, object.id);

  // covariance indices depend on the state model (e.g. HEXAMOTION as published by this node, or ISCACTR)
  cpm_access::setYawOfPerceivedObject(perceived_object, oa::getYaw(state), stdDev(covariance_at(oa::indexYaw(state.model_id))));

  const gm::PoseWithCovariance velocity_covariance = oa::getVelocityXYZWithCovariance(state);
  cpm_access::setVelocityOfPerceivedObject(perceived_object, oa::getVelocityXYZ(state), stdDev(velocity_covariance.covariance[0]),
                                           stdDev(velocity_covariance.covariance[7]), 0);

  const gm::PoseWithCovariance acceleration_covariance = oa::getAccelerationXYZWithCovariance(state);
  cpm_access::setAccelerationOfPerceivedObject(perceived_object, oa::getAccelerationXYZ(state),
                                               stdDev(acceleration_covariance.covariance[0]),
                                               stdDev(acceleration_covariance.covariance[7]), 0);

  cpm_access::setYawRateOfPerceivedObject(perceived_object, oa::getYawRate(state),
                                          stdDev(covariance_at(oa::indexYawRate(state.model_id))));

  gm::Vector3 dimensions;
  dimensions.x = oa::getLength(state);
  dimensions.y = oa::getWidth(state);
  dimensions.z = oa::getHeight(state);
  cpm_access::setDimensionsOfPerceivedObject(perceived_object, dimensions, stdDev(covariance_at(oa::indexLength(state.model_id))),
                                             stdDev(covariance_at(oa::indexWidth(state.model_id))),
                                             stdDev(covariance_at(oa::indexHeight(state.model_id))));

  // report the most probable classifications, ignoring negligible ones
  std::vector<pi::ObjectClassification> classifications = state.classifications;
  std::sort(classifications.begin(), classifications.end(),
            [](const pi::ObjectClassification& a, const pi::ObjectClassification& b) { return a.probability > b.probability; });
  for (const pi::ObjectClassification& classification : classifications) {
    if (perceived_object.classification.array.size() >= kMaxClassificationsPerObject ||
        classification.probability < kMinClassificationProbability) {
      break;
    }
    auto& cpm_classification = perceived_object.classification.array.emplace_back();
    cpm_classification.object_class = toCpmObjectClass(classification.type, object.id);
    cpm_classification.confidence.value = static_cast<uint8_t>(classification.probability * 100.0);
  }
  perceived_object.classification_is_present = !perceived_object.classification.array.empty();

  return perceived_object;
}

}  // namespace

void egoDataToCam(const perception_msgs::msg::EgoData& ego_data_raw,
                  etsi_its_cam_msgs::msg::CAM& cam,
                  tf2_ros::Buffer& tf_buffer,
                  const int zone,
                  const bool is_northp) {
  const std::string utm_frame = utmFrame(zone, is_northp);

  // transform the entire EgoData into the UTM frame
  pi::EgoData ego_data = ego_data_raw;
  if (ego_data_raw.header.frame_id != utm_frame) {
    const gm::TransformStamped tf = tf_buffer.lookupTransform(utm_frame, ego_data_raw.header.frame_id, ego_data_raw.header.stamp,
                                                              rclcpp::Duration::from_seconds(1.0));
    tf2::doTransform(ego_data_raw, ego_data, tf);
  }
  const pi::ObjectState& state = ego_data.state;

  // header and generation delta time
  cam_access::setItsPduHeader(cam, ego_data.vehicle_id, kProtocolVersion);
  cam_access::setGenerationDeltaTime(
      cam, rclcpp::Time(ego_data.header.stamp).nanoseconds(),
      etsi_its_msgs::getLeapSecondInsertionsSince2004(static_cast<uint64_t>(ego_data.header.stamp.sec)));

  // basic container: station type
  uint8_t station_type = st::UNKNOWN;
  const uint8_t classification = oa::getClassWithHighestProbability(state).type;
  const auto station_type_it = kClassificationToStationType.find(classification);
  if (station_type_it != kClassificationToStationType.end()) {
    station_type = station_type_it->second;
  } else {
    RCLCPP_WARN_STREAM(logger(), "EgoData classification type " << static_cast<int>(classification)
                                                                << " cannot be converted to a CAM station type, "
                                                                   "using unknown");
  }
  cam_access::setStationType(cam, station_type);

  // basic container: reference position
  gm::PointStamped reference_position;
  reference_position.header = ego_data.header;
  reference_position.point = oa::getCenterPosition(state);
  if (st::MOPED <= station_type && station_type <= st::TRAM) {
    // for station types 3 to 11 the reference point is the ground position of the centre of the front side of the
    // bounding box of the vehicle
    // https://www.etsi.org/deliver/etsi_en/302600_302699/30263702/01.03.01_30/en_30263702v010301v.pdf
    tf2::Quaternion orientation;
    tf2::fromMsg(oa::getPose(state).orientation, orientation);
    const tf2::Vector3 offset = tf2::Matrix3x3(orientation) * tf2::Vector3(ego_data.length / 2.0, 0.0, -ego_data.height / 2.0);
    reference_position.point.x += offset.x();
    reference_position.point.y += offset.y();
    reference_position.point.z += offset.z();
  }
  cam_access::setFromUTMPosition(cam, reference_position, zone, is_northp);
  cam_access::setWGSRefPosConfidence(cam,
                                     std::array<double, 4>{oa::getContinuousStateCovarianceAt(state, pi::EGO::X, pi::EGO::X),
                                                           oa::getContinuousStateCovarianceAt(state, pi::EGO::X, pi::EGO::Y),
                                                           oa::getContinuousStateCovarianceAt(state, pi::EGO::Y, pi::EGO::X),
                                                           oa::getContinuousStateCovarianceAt(state, pi::EGO::Y, pi::EGO::Y)});

  // UTM yaw is relative to grid north, while the CAM heading is relative to true north, so the heading is corrected
  // by the meridian convergence at the transmitted reference position
  double latitude = 0.0;
  double longitude = 0.0;
  const double grid_convergence =
      utmToWgs84(zone, is_northp, reference_position.point.x, reference_position.point.y, latitude, longitude);

  // high frequency container (basic vehicle)
  auto& high_frequency_container = cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency;
  cam_access::setHeading(cam, utmYawToEtsiHeading(oa::getYawInDeg(state), grid_convergence),
                         stdDev(oa::getContinuousStateCovarianceAt(state, pi::EGO::YAW, pi::EGO::YAW)));
  cam_access::setSpeed(cam, oa::getVelocityMagnitude(state),
                       stdDev(oa::getContinuousStateCovarianceAt(state, pi::EGO::VEL_LON, pi::EGO::VEL_LON)));
  high_frequency_container.drive_direction.value = etsi_cam::DriveDirection::FORWARD;
  cam_access::setVehicleDimensions(cam, ego_data.length, ego_data.width);
  cam_access::setLongitudinalAcceleration(cam, oa::getAccLon(state),
                                          stdDev(oa::getContinuousStateCovarianceAt(state, pi::EGO::ACC_LON, pi::EGO::ACC_LON)));
  cam_access::setLateralAcceleration(cam, oa::getAccLat(state),
                                     stdDev(oa::getContinuousStateCovarianceAt(state, pi::EGO::ACC_LAT, pi::EGO::ACC_LAT)));
  cam_access::setYawRate(cam, oa::getYawRate(state) * 180.0 / M_PI,
                         stdDev(oa::getContinuousStateCovarianceAt(state, pi::EGO::YAW_RATE, pi::EGO::YAW_RATE)) * 180.0 / M_PI);

  // low frequency container (basic vehicle) with all exterior lights off
  cam.cam.cam_parameters.low_frequency_container_is_present = true;
  cam.cam.cam_parameters.low_frequency_container.choice =
      etsi_cam::LowFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_LOW_FREQUENCY;
  try {
    cam_access::setExteriorLights(cam, std::vector<bool>(8, false));
  } catch (const std::invalid_argument& e) {
    RCLCPP_WARN_STREAM(logger(), "CAM exterior lights cannot be set: " << e.what());
  }
}

void objectListToCpm(const perception_msgs::msg::ObjectList& object_list_raw,
                     etsi_its_cpm_ts_msgs::msg::CollectivePerceptionMessage& cpm,
                     tf2_ros::Buffer& tf_buffer,
                     const uint8_t originating_container_id,
                     const uint32_t station_id,
                     const int zone,
                     const bool is_northp) {
  const std::string utm_frame = utmFrame(zone, is_northp);

  cpm_access::setItsPduHeader(cpm, station_id, kProtocolVersion);

  // transform the entire ObjectList into the UTM frame
  pi::ObjectList object_list = object_list_raw;
  if (object_list_raw.header.frame_id != utm_frame) {
    object_list = tf_buffer.transform(object_list_raw, utm_frame, tf2::durationFromSec(0.1));
  }

  // reference position: origin of the ObjectList frame
  gm::PoseStamped reference_pose;
  reference_pose.header = object_list_raw.header;
  reference_pose = tf_buffer.transform(reference_pose, utm_frame, tf2::durationFromSec(0.1));
  double latitude = 0.0;
  double longitude = 0.0;
  const double grid_convergence =
      utmToWgs84(zone, is_northp, reference_pose.pose.position.x, reference_pose.pose.position.y, latitude, longitude);
  cpm_access::setReferencePosition(cpm, latitude, longitude);

  // originating station container
  etsi_cpm::WrappedCpmContainer originating_container;
  if (originating_container_id == etsi_cpm::WrappedCpmContainer::CHOICE_CONTAINER_DATA_ORIGINATING_VEHICLE_CONTAINER) {
    originating_container.container_id.value = originating_container_id;
    const tf2::Quaternion orientation(reference_pose.pose.orientation.x, reference_pose.pose.orientation.y,
                                      reference_pose.pose.orientation.z, reference_pose.pose.orientation.w);
    const double orientation_deg = utmYawToEtsiHeading(tf2::getYaw(orientation) * 180.0 / M_PI, grid_convergence);
    auto& vehicle_container = originating_container.container_data_originating_vehicle_container;
    vehicle_container.orientation_angle.value.value =
        static_cast<uint16_t>(std::lround(orientation_deg * 10.0)) % etsi_cpm::Wgs84AngleValue::DO_NOT_USE;
    vehicle_container.orientation_angle.confidence.value = etsi_cpm::Wgs84AngleConfidence::UNAVAILABLE;
  } else if (originating_container_id == etsi_cpm::WrappedCpmContainer::CHOICE_CONTAINER_DATA_ORIGINATING_RSU_CONTAINER) {
    // the optional MAP reference of the RSU container is left absent
    originating_container.container_id.value = originating_container_id;
  } else {
    throw std::invalid_argument("Invalid originating container for CPM: " + std::to_string(originating_container_id));
  }
  cpm_access::addContainerToCPM(cpm, originating_container);

  // reference time
  cpm_access::setReferenceTime(
      cpm, rclcpp::Time(object_list.header.stamp).nanoseconds(),
      etsi_its_msgs::getLeapSecondInsertionsSince2004(static_cast<uint64_t>(object_list.header.stamp.sec)));

  // perceived object container
  etsi_cpm::WrappedCpmContainer perceived_object_container;
  cpm_access::initPerceivedObjectContainer(perceived_object_container, object_list.objects.size());
  for (const pi::Object& object : object_list.objects) {
    cpm_access::addPerceivedObjectToContainer(perceived_object_container, toPerceivedObject(object, utm_frame, cpm));
  }
  cpm_access::addContainerToCPM(cpm, perceived_object_container);
}

}  // namespace carla_converter::etsi
