#include <ros.h>
#include <std_msgs/Float32.h>
#include <first_msgs/JointState.h>

ros::NodeHandle node_handle;
first_msgs::JointState joint_state_message;
const int motorPwmPin = 10;
const int buildInLedPin = 13;
const int encoderPin = 50;
const float radiansPerEncoderPulse = PI / 10;

int motorDutyCycle = 0;
volatile byte encoderPulses = 0;
unsigned int updateFrequency = 50;
unsigned long lastMeasuredTime = 0;

void commandCallback(const std_msgs::Float32& command_message)
{
  joint_state_message.effort = command_message.data;
  motorDutyCycle = map((int)joint_state_message.effort, 0, 10, 0, 255);
  analogWrite(motorPwmPin, motorDutyCycle);
  digitalWrite(buildInLedPin, HIGH-digitalRead(buildInLedPin));
}

void encoderPulsesCounter()
{
  encoderPulses++;    
}

ros::Subscriber<std_msgs::Float32> effort_command_subscriber("hardware_set_motor_effort", &commandCallback);
ros::Publisher joint_state_publisher("hardware_motor_state", &joint_state_message);

void setup()
{
  pinMode(motorPwmPin, OUTPUT);
  pinMode(buildInLedPin, OUTPUT);
  pinMode(encoderPin, INPUT);

  joint_state_message.position = 0.0;
  joint_state_message.velocity = 0.0;
  joint_state_message.effort = 0.0;
   
  attachInterrupt(0, encoderPulsesCounter, FALLING);

  node_handle.initNode();
  node_handle.subscribe(effort_command_subscriber);
  node_handle.advertise(joint_state_publisher);
}

void loop()
{
  unsigned long duration = millis() - lastMeasuredTime;
  if (duration >= 1000 / updateFrequency) 
  {
    detachInterrupt(0);

    float fullPosition = joint_state_message.position + radiansPerEncoderPulse * encoderPulses;
    int fullCirclesCount = (int)(fullPosition / TWO_PI);
    joint_state_message.position = fullPosition - fullCirclesCount * TWO_PI;
    joint_state_message.velocity = radiansPerEncoderPulse * encoderPulses / duration; 
  
    lastMeasuredTime = millis();
    encoderPulses = 0;
    attachInterrupt(0, encoderPulsesCounter, FALLING);

    joint_state_publisher.publish(&joint_state_message);
  }
  
  node_handle.spinOnce();
}

