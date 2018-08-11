//
// Created by mike on 8/11/18.
//

#ifndef CHUP_JOINT_COMMAND_INTERFACE_H
#define CHUP_JOINT_COMMAND_INTERFACE_H

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <bobble_controllers/ChupJointStateInterface.h>

namespace hardware_interface
{

/** \brief A handle used to read and command a single joint. */
class ChupJointHandle : public ChupJointStateHandle
{
public:
  ChupJointHandle() : ChupJointStateHandle(), cmd_(0) {}

  /**
   * \param js This joint's state handle
   * \param cmd A pointer to the storage for this joint's output command
   */
  ChupJointHandle(const ChupJointStateHandle& js, float* cmd)
    : ChupJointStateHandle(js), cmd_(cmd)
  {
    if (!cmd_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command data pointer is null.");
    }
  }

  void setCommand(float command) {assert(cmd_); *cmd_ = command;}
  float getCommand() const {assert(cmd_); return *cmd_;}

private:
  float* cmd_;
};

/** \brief Hardware interface to support commanding an array of joints.
 *
 * This \ref HardwareInterface supports commanding the output of an array of
 * named joints. Note that these commands can have any semantic meaning as long
 * as they each can be represented by a single float, they are not necessarily
 * effort commands. To specify a meaning to this command, see the derived
 * classes like \ref EffortJointInterface etc.
 *
 * \note Getting a joint handle through the getHandle() method \e will claim that resource.
 *
 */
class ChupJointCommandInterface : public HardwareResourceManager<ChupJointHandle, ClaimResources> {};

/// \ref JointCommandInterface for commanding effort-based joints.
class ChupEffortJointInterface : public ChupJointCommandInterface {};

}

#endif //CHUP_JOINT_COMMAND_INTERFACE_H
