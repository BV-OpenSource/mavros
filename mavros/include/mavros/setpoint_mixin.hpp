/**
 * @brief Mixin for setpoint plugins
 * @file setpoint_mixin.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#ifndef MAVROS__SETPOINT_MIXIN_HPP_
#define MAVROS__SETPOINT_MIXIN_HPP_

#include <functional>
#include <string>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"

namespace mavros
{
namespace plugin
{

/**
 * @brief This mixin adds set_position_target_local_ned()
 */
template<class D>
class SetPositionTargetLocalNEDMixin
{
public:
  //! Message specification: @p https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
  void set_position_target_local_ned(
    uint32_t time_boot_ms, uint8_t coordinate_frame,
    uint16_t type_mask,
    Eigen::Vector3d p,
    Eigen::Vector3d v,
    Eigen::Vector3d af,
    float yaw, float yaw_rate)
  {
    static_assert(
      std::is_base_of<plugin::Plugin, D>::value,
      "SetPositionTargetLocalNEDMixin should be used by mavros::plugin::Plugin child");

    plugin::UASPtr uas_ = static_cast<D *>(this)->uas;

    mavlink::common::msg::SET_POSITION_TARGET_LOCAL_NED sp = {};
    uas_->msg_set_target(sp);

    // [[[cog:
    // for f in ('time_boot_ms', 'coordinate_frame', 'type_mask', 'yaw', 'yaw_rate'):
    //     cog.outl(f"sp.{f} = {f};")
    // for fp, vp in (('', 'p'), ('v', 'v'), ('af', 'af')):
    //     for a in ('x', 'y', 'z'):
    //         cog.outl(f"sp.{fp}{a} = {vp}.{a}();")
    // ]]]
    sp.time_boot_ms = time_boot_ms;
    sp.coordinate_frame = coordinate_frame;
    sp.type_mask = type_mask;
    sp.yaw = yaw;
    sp.yaw_rate = yaw_rate;
    sp.x = p.x();
    sp.y = p.y();
    sp.z = p.z();
    sp.vx = v.x();
    sp.vy = v.y();
    sp.vz = v.z();
    sp.afx = af.x();
    sp.afy = af.y();
    sp.afz = af.z();
    // [[[end]]] (checksum: f8942bb13a7463a2cbadc9b745df25d0)

    uas_->send_message(sp);
  }
};

/**
 * @brief This mixin adds set_position_target_global_int()
 */
template<class D>
class SetPositionTargetGlobalIntMixin
{
public:
  //! Message specification: @p https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT
  void set_position_target_global_int(
    uint32_t time_boot_ms, uint8_t coordinate_frame,
    uint16_t type_mask,
    int32_t lat_int, int32_t lon_int, float alt,
    Eigen::Vector3d v,
    Eigen::Vector3d af,
    float yaw, float yaw_rate)
  {
    static_assert(
      std::is_base_of<plugin::Plugin, D>::value,
      "SetPositionTargetGlobalIntMixin should be used by mavros::plugin::Plugin child");

    plugin::UASPtr uas_ = static_cast<D *>(this)->uas;

    mavlink::common::msg::SET_POSITION_TARGET_GLOBAL_INT sp = {};
    uas_->msg_set_target(sp);

    // [[[cog:
    // for f in ('time_boot_ms', 'coordinate_frame', 'type_mask',
    //     'lat_int', 'lon_int', 'alt', 'yaw', 'yaw_rate'):
    //     cog.outl(f"sp.{f} = {f};")
    // for fp, vp in (('v', 'v'), ('af', 'af')):
    //     for a in ('x', 'y', 'z'):
    //         cog.outl(f"sp.{fp}{a} = {vp}.{a}();")
    // ]]]
    sp.time_boot_ms = time_boot_ms;
    sp.coordinate_frame = coordinate_frame;
    sp.type_mask = type_mask;
    sp.lat_int = lat_int;
    sp.lon_int = lon_int;
    sp.alt = alt;
    sp.yaw = yaw;
    sp.yaw_rate = yaw_rate;
    sp.vx = v.x();
    sp.vy = v.y();
    sp.vz = v.z();
    sp.afx = af.x();
    sp.afy = af.y();
    sp.afz = af.z();
    // [[[end]]] (checksum: 82301ecd7936657d65e006cf7525e82a)

    uas_->send_message(sp);
  }
};

/**
 * @brief This mixin adds set_attitude_target()
 */
template<class D>
class SetAttitudeTargetMixin
{
public:
  //! Message sepecification: @p https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET
  void set_attitude_target(
    uint32_t time_boot_ms,
    uint8_t type_mask,
    Eigen::Quaterniond orientation,
    Eigen::Vector3d body_rate,
    float thrust)
  {
    static_assert(
      std::is_base_of<plugin::Plugin, D>::value,
      "SetAttitudeTargetMixin should be used by mavros::plugin::Plugin child");

    plugin::UASPtr uas_ = static_cast<D *>(this)->uas;

    mavlink::common::msg::SET_ATTITUDE_TARGET sp = {};

    uas_->msg_set_target(sp);
    mavros::ftf::quaternion_to_mavlink(orientation, sp.q);

    // [[[cog:
    // for f in ('time_boot_ms', 'type_mask', 'thrust'):
    //     cog.outl(f"sp.{f} = {f};")
    // for f, v in (('roll', 'x'), ('pitch', 'y'), ('yaw', 'z')):
    //     cog.outl(f"sp.body_{f}_rate = body_rate.{v}();")
    // ]]]
    sp.time_boot_ms = time_boot_ms;
    sp.type_mask = type_mask;
    sp.thrust = thrust;
    sp.body_roll_rate = body_rate.x();
    sp.body_pitch_rate = body_rate.y();
    sp.body_yaw_rate = body_rate.z();
    // [[[end]]] (checksum: d0910b0f92d233024163ebf957a3d642)

    uas_->send_message(sp);
  }
};

#if 0  // XXX TODO(vooon): port me pls
/**
 * @brief This mixin adds TF2 listener thread to plugin
 *
 * It requires tf_frame_id, tf_child_frame_id strings
 * tf_rate double and uas object pointer
 */
template<class D>
class TF2ListenerMixin
{
public:
  std::thread tf_thread;
  std::string tf_thd_name;

  /**
   * @brief start tf listener
   *
   * @param _thd_name  listener thread name
   * @param cbp        plugin callback function
   */
  void tf2_start(const char * _thd_name, void (D::* cbp)(const geometry_msgs::TransformStamped &) )
  {
    tf_thd_name = _thd_name;
    auto tf_transform_cb = std::bind(cbp, static_cast<D *>(this), std::placeholders::_1);

    tf_thread = std::thread(
      [this, tf_transform_cb]() {
        mavconn::utils::set_this_thread_name("%s", tf_thd_name.c_str());

        mavros::UAS * m_uas_ = static_cast<D *>(this)->m_uas;
        std::string & _frame_id = static_cast<D *>(this)->tf_frame_id;
        std::string & _child_frame_id = static_cast<D *>(this)->tf_child_frame_id;

        ros::Rate rate(static_cast<D *>(this)->tf_rate);
        while (ros::ok()) {
          // Wait up to 3s for transform
          if (m_uas_->tf2_buffer.canTransform(
            _frame_id, _child_frame_id, ros::Time(0),
            ros::Duration(3.0)))
          {
            try {
              auto transform = m_uas_->tf2_buffer.lookupTransform(
                _frame_id, _child_frame_id, ros::Time(0), ros::Duration(3.0));
              tf_transform_cb(transform);
            } catch (tf2::LookupException & ex) {
              ROS_ERROR_NAMED("tf2_buffer", "%s: %s", tf_thd_name.c_str(), ex.what());
            }
          }
          rate.sleep();
        }
      });
  }

  /**
   * @brief start tf listener syncronized with another topic
   *
   * @param _thd_name  listener thread name
   * @param cbp        plugin callback function
   */
  template<class T>
  void tf2_start(
    const char * _thd_name, message_filters::Subscriber<T> & topic_sub, void (D::* cbp)(
      const geometry_msgs::TransformStamped &, const typename T::ConstPtr &))
  {
    tf_thd_name = _thd_name;

    tf_thread = std::thread(
      [this, cbp, &topic_sub]() {
        mavconn::utils::set_this_thread_name("%s", tf_thd_name.c_str());

        mavros::UAS * m_uas_ = static_cast<D *>(this)->m_uas;
        ros::NodeHandle & _sp_nh = static_cast<D *>(this)->sp_nh;
        std::string & _frame_id = static_cast<D *>(this)->tf_frame_id;
        std::string & _child_frame_id = static_cast<D *>(this)->tf_child_frame_id;

        tf2_ros::MessageFilter<T> tf2_filter(topic_sub, m_uas_->tf2_buffer, _frame_id, 10, _sp_nh);

        ros::Rate rate(static_cast<D *>(this)->tf_rate);
        while (ros::ok()) {
          // Wait up to 3s for transform
          if (m_uas_->tf2_buffer.canTransform(
            _frame_id, _child_frame_id, ros::Time(0),
            ros::Duration(3.0)))
          {
            try {
              auto transform = m_uas_->tf2_buffer.lookupTransform(
                _frame_id, _child_frame_id, ros::Time(0), ros::Duration(3.0));

              tf2_filter.registerCallback(boost::bind(cbp, static_cast<D *>(this), transform, _1));
            } catch (tf2::LookupException & ex) {
              ROS_ERROR_NAMED("tf2_buffer", "%s: %s", tf_thd_name.c_str(), ex.what());
            }
          }
          rate.sleep();
        }
      });
  }
};
#endif

}       // namespace plugin
}       // namespace mavros

#endif  // MAVROS__SETPOINT_MIXIN_HPP_
