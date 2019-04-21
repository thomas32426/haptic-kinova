/*
Read voltages from flex sensors (Thumb, Middle, Index) (Respective pins A2, A1, A0)
Move servos based upon ROS message.
*/

#define USE_USBCON
#include <ros.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>
#include <Servo.h>

// Create ROS variables.
std_msgs::Int16MultiArray fingers_position;
std_msgs::MultiArrayDimension myDim;
std_msgs::MultiArrayLayout myLayout;

// Create servo variables and initial positions. (180 is relaxed and 0 is full force backward)
Servo thumbServo;
Servo indexServo;
Servo middleServo;

int thumbAngle = 0;
int indexAngle = 0;
int middleAngle = 180;

// Create the node handle
ros::NodeHandle nh;

// When a servo_angles message comes in, this function runs.
void fingersCallback(const std_msgs::Int16MultiArray& fingerMsg)
{
  thumbAngle = fingerMsg.data[0];
  indexAngle = fingerMsg.data[1];
  middleAngle = fingerMsg.data[2];
}

// Subscribe to the topic and respond with the callback function
ros::Subscriber<std_msgs::Int16MultiArray> sub("servo_angles", fingersCallback);

// Publish the glove data as a topic called "glove".
ros::Publisher glove("glove", &fingers_position);

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  // Arduino Pro Micro
  thumbServo.attach(10);
  indexServo.attach(9);
  middleServo.attach(6);

  // Initialize ROS node, publish FromArduino glove data, and subscribe to ToArduino message.
  nh.initNode();
  nh.advertise(glove);
  nh.subscribe(sub);

  myDim.label = "flex_sensors_readings";
  myDim.size = 3;
  myDim.stride = 3;
  myLayout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 1);
  myLayout.dim[0] = myDim;
  myLayout.data_offset = 0;
  fingers_position.layout = myLayout;
  fingers_position.data = (int *)malloc(sizeof(int)*myDim.size);
  fingers_position.data_length = myDim.size;
}

// the loop routine runs over and over again forever:
void loop() {
  // Read the input on analog pins
  int indexSensor = analogRead(A0); // Index finger
  int middleSensor = analogRead(A1); // Middle finger
  int thumbSensor = analogRead(A2); // Thumb

  // Update data to be published through sensors' reads.
  fingers_position.data[0] = thumbSensor;
  fingers_position.data[1] = indexSensor;
  fingers_position.data[2] = middleSensor;
  
  // Publish data
  glove.publish(&fingers_position);

  // Set servo angles
  thumbServo.write(thumbAngle);
  indexServo.write(indexAngle);
  middleServo.write(middleAngle);

  // Wait for a bit
  nh.spinOnce();
}
