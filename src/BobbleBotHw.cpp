//
#include <bobble_controllers/BobbleBotHw.h>

BobbleBotHw::BobbleBotHw()
 :
   left_motor_chup_can_interface(left_motor_chup_can_transporter),
   left_motor_chup_joint_data(),
   right_motor_chup_can_interface(right_motor_chup_can_transporter),
   right_motor_chup_joint_data()
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
  hardware_interface::ChupJointStateHandle left_joint_state_handle(left_motor_joint_name,
                                                               &left_motor_chup_joint_data.Position,
                                                               &left_motor_chup_joint_data.Velocity,
                                                               &left_motor_chup_joint_data.Torque);
  hardware_interface::ChupJointStateHandle right_joint_state_handle(right_motor_joint_name,
                                                                &right_motor_chup_joint_data.Position,
                                                                &right_motor_chup_joint_data.Velocity,
                                                                &right_motor_chup_joint_data.Torque);
  // connect and register the joint effort interfaces
  hardware_interface::ChupJointHandle left_joint_effort_handle(left_joint_state_handle,
                                                            &left_motor_chup_joint_data.CmdVoltage);
  hardware_interface::ChupJointHandle right_joint_effort_handle(right_joint_state_handle,
                                                            &right_motor_chup_joint_data.CmdVoltage);

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
 CommData left_chup_data = left_motor_chup_can_interface.getCommData();
 left_motor_chup_joint_data.Position = left_chup_data.Position;
 left_motor_chup_joint_data.Velocity = left_chup_data.Velocity;
 left_motor_chup_joint_data.Torque = left_chup_data.Torque;
 hardware_interface::ChupJointHandle left_jnt_handle = jnt_effort_interface.getHandle(left_motor_joint_name);
 right_motor_chup_can_interface.ReadTelemetry();
 CommData right_chup_data = right_motor_chup_can_interface.getCommData();
 right_motor_chup_joint_data.Position = right_chup_data.Position;
 right_motor_chup_joint_data.Velocity = right_chup_data.Velocity;
 right_motor_chup_joint_data.Torque = right_chup_data.Torque;
 hardware_interface::ChupJointHandle right_jnt_handle = jnt_effort_interface.getHandle(right_motor_joint_name);
 std::cout << "Left Motor Position : " << left_jnt_handle.getPosition() << std::endl;
 std::cout << "Left Motor Velocity : " << left_jnt_handle.getVelocity() << std::endl;
 std::cout << "Right Motor Position : " << right_jnt_handle.getPosition() << std::endl;
 std::cout << "Right Motor Velocity : " << right_jnt_handle.getVelocity() << std::endl;
}

void BobbleBotHw::write(){
  std::cout << "Write Left Motor Command : " << left_motor_chup_joint_data.CmdVoltage << std::endl;
  std::cout << "Write Right Motor Command : " << right_motor_chup_joint_data.CmdVoltage << std::endl;
  left_motor_chup_can_interface.SendCommands(left_motor_chup_joint_data);
  right_motor_chup_can_interface.SendCommands(right_motor_chup_joint_data);
}

