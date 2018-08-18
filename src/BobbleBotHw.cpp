//
#include <bobble_controllers/BobbleBotHw.h>

BobbleBotHw::BobbleBotHw()
 :
   left_motor_chup_can_interface(left_motor_chup_can_transporter),
   right_motor_chup_can_interface(right_motor_chup_can_transporter)
{
  ros::NodeHandle pnh("~");
  std::string default_can_channel = "vcan0";
  std::string default_left_joint_name = "left_wheel_hinge";
  std::string default_right_joint_name = "right_wheel_hinge";
  pnh.param("LeftMotorJointName", left_motor_joint_name, default_left_joint_name);
  pnh.param("LeftMotorChannel", left_motor_chup_can_interface.Channel, default_can_channel);
  pnh.param("LeftMotorChupId", left_motor_chup_can_interface.DeviceId, 291);
  pnh.param("LeftMotorRxOffset", left_motor_chup_can_interface.RxOffset, 0);
  pnh.param("RightMotorJointName", right_motor_joint_name, default_right_joint_name);
  pnh.param("RightMotorChannel", right_motor_chup_can_interface.Channel, default_can_channel);
  pnh.param("RightMotorChupId", right_motor_chup_can_interface.DeviceId, 292);
  pnh.param("RightMotorRxOffset", right_motor_chup_can_interface.RxOffset, 0);
  left_motor_chup_can_interface.ChannelName = left_motor_joint_name;
  right_motor_chup_can_interface.ChannelName = right_motor_joint_name;

  // define the joint state handles...supply the pointers needed
  hardware_interface::JointStateHandle left_joint_state_handle(left_motor_joint_name,
                                                               &LeftMotorPosition,
                                                               &LeftMotorVelocity,
                                                               &LeftMotorTorque);
  hardware_interface::JointStateHandle right_joint_state_handle(right_motor_joint_name,
                                                                &RightMotorPosition,
                                                                &RightMotorVelocity,
                                                                &RightMotorTorque);
  // connect and register the joint effort interfaces
  hardware_interface::JointHandle left_joint_effort_handle(left_joint_state_handle,
                                                            &LeftMotorCmdVoltage);
  hardware_interface::JointHandle right_joint_effort_handle(right_joint_state_handle,
                                                            &RightMotorCmdVoltage);

  jnt_effort_interface.registerHandle(left_joint_effort_handle);
  jnt_effort_interface.registerHandle(right_joint_effort_handle);
  registerInterface(&jnt_effort_interface);
}

BobbleBotHw::~BobbleBotHw()
{

}

void BobbleBotHw::init(){
  left_motor_chup_can_interface.InitChannel();
  right_motor_chup_can_interface.InitChannel();
}

void BobbleBotHw::read(){
  left_motor_chup_can_interface.ReadTelemetry();
  right_motor_chup_can_interface.ReadTelemetry();
  CommData left_chup_data = left_motor_chup_can_interface.getCommData();
  CommData right_chup_data = right_motor_chup_can_interface.getCommData();
  LeftMotorPosition = (double) left_chup_data.Position;
  LeftMotorVelocity = (double) left_chup_data.Velocity;
  LeftMotorTorque = (double) left_chup_data.Torque;
  RightMotorPosition = (double) right_chup_data.Position;
  RightMotorVelocity = (double) right_chup_data.Velocity;
  RightMotorTorque = (double) right_chup_data.Torque;
  hardware_interface::JointHandle left_jnt_handle = jnt_effort_interface.getHandle(left_motor_joint_name);
  hardware_interface::JointHandle right_jnt_handle = jnt_effort_interface.getHandle(right_motor_joint_name);
}

void BobbleBotHw::write(){
  CommData left_chup_cmd, right_chup_cmd;
  left_chup_cmd.CmdVoltage = (double) LeftMotorCmdVoltage;
  right_chup_cmd.CmdVoltage = (double) RightMotorCmdVoltage;
  left_motor_chup_can_interface.SendCommands(left_chup_cmd);
  right_motor_chup_can_interface.SendCommands(right_chup_cmd);
}
