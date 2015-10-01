/*
 * ROS interface for Dynamixel XL-320 servos
 */

#include <ros.h>
#include <time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <rosserial_msgs/RequestParam.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>

#define N_SERVOS 4

/* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

/* Control table */
#define PID_D_GAIN    27
#define PID_I_GAIN    28
#define PID_P_GAIN    29
#define GOAL_POSITION 30
#define GOAL_SPEED    32

Dynamixel Dxl(DXL_BUS_SERIAL1);

ros::NodeHandle  nh;

long next_frame_time = 0;
long msgs_seq = 1;

sensor_msgs::JointState jointstates_msg;
ros::Publisher pub_jointstates("joint_states", &jointstates_msg);

trajectory_msgs::JointTrajectoryPoint trajpoint;

uint16_t rad2pos(float rad)
{
  return (uint16_t) (rad * 195.378608); // deg * 1023 / 300
}

float pos2rad(uint16_t pos)
{
  return ((float) pos) / 195.378608;
}

uint16_t radpers2vel(float radpers)
{
  if (radpers < 0) {
    return 300; // ~33.3 RPM
  }
  return (uint16_t) (radpers * 85.692372); // 114 RPM -> 1023
}

void command_servo(const uint16_t servo_id, const float angle, const float velocity) {
  if (angle > 5.23598) { // 300 deg
    nh.logerror("Invalid angle");
    return;
  }
  if (velocity > 11.938052) { // 114 RPM
    nh.logerror("Invalid velocity");
    return;
  }
  
  uint16_t pos = rad2pos(angle);
  uint16_t vel = radpers2vel(velocity);
  
  nh.logdebug("Sending command to servo");
  Dxl.setPosition(servo_id, pos, vel);
}

void servo_cb(const uint16_t servo_id, const std_msgs::Float32& cmd_msg) {
  float angle = cmd_msg.data;
  command_servo(servo_id, angle, -1);
}

void trajpoint_cb(const trajectory_msgs::JointTrajectoryPoint& point) {
  if (point.positions_length != N_SERVOS) {
    nh.logerror("Wrong number of positions");
    return;
  }
  if (point.velocities_length > 0 && point.velocities_length != point.positions_length) {
    nh.logerror("Number of velocities (if provided) must match the number of positions");
    return;
  }
  
  for (int id=1; id<=N_SERVOS; id++) {
    float angle = point.positions[id-1];
    float velocity = -1;
    if (point.velocities_length > 0) {
      velocity = point.velocities[id-1];
    }
    command_servo(id, angle, velocity);
  }
}

void getpidparams_cb(const rosserial_msgs::RequestParamRequest& req, rosserial_msgs::RequestParamResponse& res) {
  uint8_t i = 0;
  
  if (res.ints_length < N_SERVOS*3) {
    res.ints = (int32_t*) realloc(res.ints, N_SERVOS*3*sizeof(int32_t));
  }
  res.ints_length = N_SERVOS*3;
  
  for (int id=1; id<=N_SERVOS; id++) {
    res.ints[i++] = (int32_t) Dxl.readByte(id, PID_P_GAIN);
    res.ints[i++] = (int32_t) Dxl.readByte(id, PID_I_GAIN);
    res.ints[i++] = (int32_t) Dxl.readByte(id, PID_D_GAIN);
  }
}

void setpidparams_cb(const std_msgs::UInt8MultiArray& params) {
  uint8_t pgain, igain, dgain;
  uint8_t i = 0;
  
  if (params.data_length != N_SERVOS*3) {
    nh.logerror("Incorrect number of PID parameters (expecting 3 params per servo)");
    return;
  }
  
  for (int id=1; id<=N_SERVOS; id++) {
    pgain = (uint8_t) params.data[i++];
    if (pgain > 1023) { nh.logerror("P gain must be <= 1023"); return; }
    igain = (uint8_t) params.data[i++];
    if (pgain > 254) { nh.logerror("I gain must be <= 254"); return; }
    dgain = (uint8_t) params.data[i++];
    if (pgain > 254) { nh.logerror("D gain must be <= 254"); return; }
    Dxl.writeByte(id, PID_P_GAIN, pgain);
    Dxl.writeByte(id, PID_I_GAIN, igain);
    Dxl.writeByte(id, PID_D_GAIN, dgain);
  }
}

void servo1(const std_msgs::Float32& cmd_msg) { servo_cb(1, cmd_msg); }
void servo2(const std_msgs::Float32& cmd_msg) { servo_cb(2, cmd_msg); }
void servo3(const std_msgs::Float32& cmd_msg) { servo_cb(3, cmd_msg); }
void servo4(const std_msgs::Float32& cmd_msg) { servo_cb(4, cmd_msg); }

ros::Subscriber<std_msgs::Float32> sub1("servo1", servo1);
ros::Subscriber<std_msgs::Float32> sub2("servo2", servo2);
ros::Subscriber<std_msgs::Float32> sub3("servo3", servo3);
ros::Subscriber<std_msgs::Float32> sub4("servo4", servo4);

ros::Subscriber<trajectory_msgs::JointTrajectoryPoint> sub_trajpoint("trajpoint", trajpoint_cb);

ros::ServiceServer<rosserial_msgs::RequestParamRequest, rosserial_msgs::RequestParamResponse> srv_pidparams("getpidparams_srv", &getpidparams_cb);
ros::Subscriber<std_msgs::UInt8MultiArray> sub_pidparams("pidparams", setpidparams_cb);

void setup()
{
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  Dxl.writeWord(BROADCAST_ID, GOAL_SPEED, 300);
  Dxl.jointMode(BROADCAST_ID); //jointMode() is to use position mode

  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);
  nh.subscribe(sub_trajpoint);
  nh.subscribe(sub_pidparams);
  nh.advertise(pub_jointstates);
  nh.advertiseService(srv_pidparams);
  
  while(!nh.connected()) nh.spinOnce();
  
  nh.loginfo("Startup complete");

  jointstates_msg.name_length = N_SERVOS;
  jointstates_msg.name = (char**) malloc(N_SERVOS * sizeof(char*));
  jointstates_msg.name[0] = "servo1";
  jointstates_msg.name[1] = "servo2";
  jointstates_msg.name[2] = "servo3";
  jointstates_msg.name[3] = "servo4";
  jointstates_msg.position_length = N_SERVOS;
  jointstates_msg.position = (float*) malloc(N_SERVOS * sizeof(float));
  //jointstates_msg.velocity_length = N_SERVOS;
  //jointstates_msg.effort_length = N_SERVOS;
}

void publish_jointstates()
{
  boolean read_error = false;
  
  jointstates_msg.header.seq = msgs_seq ++;
  jointstates_msg.header.stamp = nh.now();
  
  for (int id=1; id<=N_SERVOS; id++) {
    uint16_t pos = Dxl.getPosition(id);
    if (pos == 0xffff) {
      read_error = true;
      break;
    }
    float rad = pos2rad(pos);
    jointstates_msg.position[id-1] = rad;
  }
  
  if (!read_error) {
    pub_jointstates.publish(&jointstates_msg);
  }
}

void loop()
{
  if (millis() > next_frame_time)
  {
    publish_jointstates();
    next_frame_time = millis() + 100; // 10Hz
  }
  nh.spinOnce();
  delay(1);
}

