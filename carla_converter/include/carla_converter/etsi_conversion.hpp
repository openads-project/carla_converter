// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <tf2_ros/buffer.h>
#include <cstdint>

#include <etsi_its_cam_msgs/msg/cam.hpp>
#include <etsi_its_cpm_ts_msgs/msg/collective_perception_message.hpp>
#include <perception_msgs/msg/ego_data.hpp>
#include <perception_msgs/msg/object_list.hpp>

namespace carla_converter::etsi {

/**
 * @brief Converts an EgoData message to an ETSI CAM message
 *
 * @param ego_data EgoData to convert, in any frame that can be transformed into the given UTM frame
 * @param cam output CAM message
 * @param tf_buffer TF buffer used to transform the EgoData into the UTM frame
 * @param zone UTM zone to convert in
 * @param is_northp UTM hemisphere to convert in
 * @throws tf2::TransformException if the EgoData cannot be transformed into the UTM frame
 */
void egoDataToCam(const perception_msgs::msg::EgoData& ego_data,
                  etsi_its_cam_msgs::msg::CAM& cam,
                  tf2_ros::Buffer& tf_buffer,
                  int zone,
                  bool is_northp);

/**
 * @brief Converts an ObjectList message to an ETSI CPM message, reporting the origin of the ObjectList
 *        frame as the reference position of the CPM
 *
 * @param object_list ObjectList to convert, in any frame that can be transformed into the given UTM frame
 * @param cpm output CollectivePerceptionMessage
 * @param tf_buffer TF buffer used to transform the ObjectList into the UTM frame
 * @param originating_container_id originating vehicle or originating RSU container (WrappedCpmContainer choice)
 * @param station_id ETSI station id to report the CPM under
 * @param zone UTM zone to convert in
 * @param is_northp UTM hemisphere to convert in
 * @throws tf2::TransformException if the ObjectList cannot be transformed into the UTM frame
 * @throws std::invalid_argument if originating_container_id is not supported
 */
void objectListToCpm(const perception_msgs::msg::ObjectList& object_list,
                     etsi_its_cpm_ts_msgs::msg::CollectivePerceptionMessage& cpm,
                     tf2_ros::Buffer& tf_buffer,
                     uint8_t originating_container_id,
                     uint32_t station_id,
                     int zone,
                     bool is_northp);

}  // namespace carla_converter::etsi
