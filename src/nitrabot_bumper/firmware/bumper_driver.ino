#include <ros.h>
#include <std_msgs/Bool.h>


ros::NodeHandle nh;

std_msgs::Bool left_bumper_msg;
std_msgs::Bool right_bumper_msg;
ros::Publisher left_bumper_publisher("left_bumper", &left_bumper_msg);
ros::Publisher right_bumper_publisher("left_bumper", &right_bumper_msg);

const int left_bumper_pin = A1;
const int right_bumper_pin = A2;

int value = false; 

void setup()
{
  nh.initNode();

  nh.advertise(left_bumper_publisher);
  
  nh.advertise(right_bumper_publisher);
}


void loop()
{
  value = analogRead(left_bumper_pin)

  if (value != 0) 
  {
	left_bumper_msg.data = true;

    left_bumper_publisher.push(&left_bumper_msg)
  } 
  else 
  {
	left_bumper_msg.data = false;

    left_bumper_publisher.push(&left_bumper_msg)
  }

  value = analogRead(right_bumper_pin)

  if (value != 0) 
  {
	right_bumper_msg.data = true;

    right_bumper_publisher.push(&right_bumper_msg)
  } 
  else 
  {
	right_bumper_msg.data = false;

    right_bumper_publisher.push(&right_bumper_msg)
  }

  nh.spinOnce();
}