#include <ros.h>
#include <std_msgs/Float32.h>
#include <first_msgs/JointState.h>
// Library from https://storage.googleapis.com/google-code-archive-downloads/v2/code.google.com/arduino-timerone/TimerOne-r11.zip
#include <TimerOne.h>

ros::NodeHandle node_handle;
first_msgs::JointState joint_state_message;

const int motorPwmPin = 10;
const int buildInLedPin = 13;
const int encoderPin = 2;
const float radiansPerEncoderPulse = PI * 0.1;

int motorDutyCycle = 0;
volatile unsigned int encoderPulses = 0;
volatile unsigned long lastInterrupt = 0;
unsigned int updateFrequency = 10;

void commandCallback(const std_msgs::Float32& command_message)
{
  joint_state_message.effort = command_message.data;
  if (joint_state_message.effort < 0.1)
  {
    motorDutyCycle = 0;
  }
  else {
    motorDutyCycle = map((int)(joint_state_message.effort * 100), 0, 1000, 80, 127); // limit PWM due to interuptions overload :)
  }
  analogWrite(motorPwmPin, motorDutyCycle);
}

ros::Subscriber<std_msgs::Float32> effort_command_subscriber("hardware_set_motor_effort", &commandCallback);
ros::Publisher joint_state_publisher("hardware_motor_state", &joint_state_message);

void encoderPulsesCounter()
{
//  if(millis() - lastInterrupt > 1)
//  {
    encoderPulses++;
    lastInterrupt = millis();
//  }
}

void onTimer()
{
  detachInterrupt(digitalPinToInterrupt(encoderPin));
  Timer1.detachInterrupt();

  digitalWrite(buildInLedPin, HIGH-digitalRead(buildInLedPin));

  float fullPosition = joint_state_message.position + radiansPerEncoderPulse * encoderPulses;
  int fullCirclesCount = (int)(fullPosition / TWO_PI);
  joint_state_message.position = fullPosition - fullCirclesCount * TWO_PI;
  joint_state_message.velocity = radiansPerEncoderPulse * encoderPulses * updateFrequency;

  encoderPulses = 0;
  joint_state_publisher.publish(&joint_state_message);

  Timer1.attachInterrupt(onTimer);
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderPulsesCounter, RISING);
}

void setup()
{
  pinMode(motorPwmPin, OUTPUT);
  pinMode(buildInLedPin, OUTPUT);
  pinMode(encoderPin, INPUT);

  joint_state_message.position = 0.0;
  joint_state_message.velocity = 0.0;
  joint_state_message.effort = 0.0;

  node_handle.initNode();
  node_handle.subscribe(effort_command_subscriber);
  node_handle.advertise(joint_state_publisher);

  Timer1.initialize(1000000 / updateFrequency);
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderPulsesCounter, RISING);
  Timer1.attachInterrupt(onTimer);
}

void loop()
{
  node_handle.spinOnce();
}
