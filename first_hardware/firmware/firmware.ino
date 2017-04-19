#include <ros.h>
#include <std_msgs/Float32.h>
#include <first_msgs/GetMotorSpeed.h>

ros::NodeHandle nh;
float currentEffort = 0.0;

void commandCallback(const std_msgs::Float32& command_message)
{
  currentEffort = command_message.data;
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

void getMotorVelocity(const first_msgs::GetMotorSpeed::Request  &request, first_msgs::GetMotorSpeed::Response &response)
{
  response.velocity = 10.0 * currentEffort;
}

ros::Subscriber<std_msgs::Float32> effort_command_subscriber("send_motor_1_effort", &commandCallback);
ros::ServiceServer<first_msgs::GetMotorSpeed::Request, first_msgs::GetMotorSpeed::Response> motor_velocity_service(
  "get_motor_velocity", &getMotorVelocity);

void setup()
{
  nh.initNode();
  pinMode(13, OUTPUT);
  nh.subscribe(effort_command_subscriber);
  nh.advertiseService(motor_velocity_service);
}

void loop()
{
  nh.spinOnce();
  delay(10);
}
