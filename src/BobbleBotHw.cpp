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

  // connect and register the joint state interfaces
  hardware_interface::JointStateHandle left_joint_state_handle(left_motor_joint_name,
                                                               (double*) &left_motor_chup_joint_data.Position,
                                                               (double*) &left_motor_chup_joint_data.Velocity,
                                                               (double*) &left_motor_chup_joint_data.Torque);
  jnt_state_interface.registerHandle(left_joint_state_handle);
  hardware_interface::JointStateHandle right_joint_state_handle(right_motor_joint_name,
                                                                (double*) &right_motor_chup_joint_data.Position,
                                                                (double*) &right_motor_chup_joint_data.Velocity,
                                                                (double*) &right_motor_chup_joint_data.Torque);
  jnt_state_interface.registerHandle(right_joint_state_handle);
  registerInterface(&jnt_state_interface);
  // connect and register the joint effort interfaces
  hardware_interface::JointHandle left_joint_effort_handle(left_joint_state_handle,
                                                            (double*) &left_motor_chup_joint_data.CmdVoltage);
  jnt_effort_interface.registerHandle(left_joint_effort_handle);
  hardware_interface::JointHandle right_joint_effort_handle(right_joint_state_handle,
                                                            (double*) &right_motor_chup_joint_data.CmdVoltage);
  jnt_effort_interface.registerHandle(right_joint_effort_handle);
  registerInterface(&jnt_effort_interface);
}

BobbleBotHw::~BobbleBotHw()
{

}

void BobbleBotHw::init(){
  // Comm channel init
  left_motor_chup_can_interface.InitChannel();
  //right_motor_chup_can_interface.InitChannel();
}

void BobbleBotHw::read(){
 left_motor_chup_can_interface.ReadTelemetry();
 CommData left_chup_data = left_motor_chup_can_interface.getCommData();
 left_motor_chup_joint_data.Position = left_chup_data.Position;
 left_motor_chup_joint_data.Velocity = left_chup_data.Velocity;
 left_motor_chup_joint_data.Torque = left_chup_data.Torque;
 hardware_interface::JointHandle jnt_handle = jnt_effort_interface.getHandle(left_motor_joint_name);
 //right_motor_chup_can_interface.ReadTelemetry();
 //right_motor_chup_joint_data = right_motor_chup_can_interface.getCommData();
 std::cout << "Left Motor Position In Struct : " << left_motor_chup_joint_data.Position << std::endl;
 std::cout << "Left Motor Position In Joint : " << jnt_handle.getPosition() << std::endl;
 //std::cout << "Right Motor Position : " << right_motor_chup_joint_data.Position << std::endl;
}

void BobbleBotHw::write(){
  std::cout << "Write Motor Commands : " << left_motor_chup_joint_data.CmdVoltage << std::endl;
  left_motor_chup_can_interface.SendCommands(left_motor_chup_joint_data);
  //right_motor_chup_can_interface.SendCommands(right_motor_chup_joint_data);
}

