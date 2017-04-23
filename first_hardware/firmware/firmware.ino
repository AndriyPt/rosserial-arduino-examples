#include <ros.h>
#include <std_msgs/Float32.h>
#include <first_msgs/GetMotorSpeed.h>

//declare variables for the motor pins (based on http://www.4tronix.co.uk/arduino/Stepper-Motors.php)
int motorPin1 = 38;
int motorPin2 = 40;
int motorPin3 = 42;
int motorPin4 = 44;
int lookup[8] = {B01000, B01100, B00100, B00110, B00010, B00011, B00001, B01001};

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
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  
  nh.initNode();
  pinMode(13, OUTPUT);
  nh.subscribe(effort_command_subscriber);
  nh.advertiseService(motor_velocity_service);
}

void loop()
{
  int integerEffort = min((int)(abs(currentEffort)), 5000);
  int timeout = 5010 - integerEffort;
  if (0 != integerEffort) {
    if (currentEffort > 0) {
      clockwise(timeout);
    }
    else {
      anticlockwise(timeout);
    }
  }
  currentEffort = 0.0;
  
  nh.spinOnce();
}

void anticlockwise(int timeout)
{
  for(int i = 0; i < 8; i++)
  {
    setOutput(i);
    delayMicroseconds(timeout);
  }
}

void clockwise(int timeout)
{
  for(int i = 7; i >= 0; i--)
  {
    setOutput(i);
    delayMicroseconds(timeout);
  }
}

void setOutput(int out)
{
  digitalWrite(motorPin1, bitRead(lookup[out], 0));
  digitalWrite(motorPin2, bitRead(lookup[out], 1));
  digitalWrite(motorPin3, bitRead(lookup[out], 2));
  digitalWrite(motorPin4, bitRead(lookup[out], 3));
}
