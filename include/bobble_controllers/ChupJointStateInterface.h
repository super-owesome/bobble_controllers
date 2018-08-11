//
// Created by mike on 8/11/18.
//

#ifndef CHUP_JOINT_STATE_INTERFACE_H
#define CHUP_JOINT_STATE_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <cassert>
#include <string>

namespace hardware_interface
{

/** A handle used to read the state of a Chup motor joint. */
class ChupJointStateHandle
{
public:
  ChupJointStateHandle() : name_(), pos_(0), vel_(0), eff_(0) {}

  /**
   * \param name The name of the joint
   * \param pos A pointer to the storage for this joint's position
   * \param vel A pointer to the storage for this joint's velocity
   * \param eff A pointer to the storage for this joint's effort (force or torque)
   */
  ChupJointStateHandle(const std::string& name, const float* pos, const float* vel, const float* eff)
    : name_(name), pos_(pos), vel_(vel), eff_(eff)
  {
    if (!pos)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Position data pointer is null.");
    }
    if (!vel)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Velocity data pointer is null.");
    }
    if (!eff)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Effort data pointer is null.");
    }
  }

  std::string getName() const {return name_;}
  float getPosition()  const {assert(pos_); return *pos_;}
  float getVelocity()  const {assert(vel_); return *vel_;}
  float getEffort()    const {assert(eff_); return *eff_;}

private:
  std::string name_;
  const float* pos_;
  const float* vel_;
  const float* eff_;
};

/** \brief Hardware interface to support reading the state of an array of joints
 *
 * This \ref HardwareInterface supports reading the state of an array of named
 * joints, each of which has some position, velocity, and effort (force or
 * torque).
 *
 */
class ChupJointStateInterface : public HardwareResourceManager<ChupJointStateHandle> {};

}

#endif //CHUP_JOINT_STATE_INTERFACE_H
