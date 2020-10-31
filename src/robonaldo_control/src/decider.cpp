// Copyright 2020 USC Makers Robonaldo Inc.

#include "ros/ros.h"
#include "robonaldo_msgs/keyboard_input.h"
#include "robonaldo_msgs/motor_speeds.h"
#include "robonaldo_msgs/ball_positions.h"

const float MAX_RIGHT_SPEED = 1.0f;
const float MIN_RIGHT_SPEED = 1.0f;
const float MAX_LEFT_SPEED = 1.0f;
const float MIN_LEFT_SPEED = 1.0f;

const float KEYBOARD_SPEED = 0.25f;

const float ANGLE_THRESHOLD = 45.0f;  // the angle that motor speed will start to decrease
const float ANGLE_MAX_SPEED = 0.25f;  // represents motor max speed for turning

robonaldo_msgs::motor_speeds motor_speeds_msg;

void userInputCallback(const robonaldo_msgs::keyboard_input::ConstPtr& msg)
{
  // ROS_INFO("Recieved %u %u %u %u", msg->up, msg->down, msg->left, msg->right);

  motor_speeds_msg.left_speed = 0.0f;
  motor_speeds_msg.right_speed = 0.0f;

  if (msg->up == true)
  {
    motor_speeds_msg.left_speed += KEYBOARD_SPEED;
    motor_speeds_msg.right_speed += KEYBOARD_SPEED;
  }

  if (msg->down == true)
  {
    motor_speeds_msg.left_speed += -KEYBOARD_SPEED;
    motor_speeds_msg.right_speed += -KEYBOARD_SPEED;
  }

  if (msg->left == true)
  {
    motor_speeds_msg.left_speed += -KEYBOARD_SPEED;
    motor_speeds_msg.right_speed += KEYBOARD_SPEED;
  }

  if (msg->right == true)
  {
    motor_speeds_msg.left_speed += KEYBOARD_SPEED;
    motor_speeds_msg.right_speed += -KEYBOARD_SPEED;
  }
}
void ballPositionCallback(const robonaldo_msgs::ball_positions::ConstPtr& msg)
{
  // Want to turn motors in direction of ball
  if (msg->angle <= ANGLE_THRESHOLD)
  {
    // max speed 25% ish, at what angle do we stop going at max speed and slow down etc
    // divide 50% by theta
    motor_speeds_msg.left_speed = (-1 * ANGLE_MAX_SPEED * msg->angle) / ANGLE_THRESHOLD;
    motor_speeds_msg.right_speed = (ANGLE_MAX_SPEED * msg->angle) / ANGLE_THRESHOLD;
  }
  else
  {   // keep turning until we find the ball in the direction of the angle
    if (msg->angle < 0.0f)
    {  // clockwise (turn left) if angle is negative
      motor_speeds_msg.left_speed = -1 * ANGLE_MAX_SPEED;
      motor_speeds_msg.right_speed = ANGLE_MAX_SPEED;
    }
    else
    {   // counterclockwise (turn right) if angle is positive
      motor_speeds_msg.left_speed = ANGLE_MAX_SPEED;
      motor_speeds_msg.right_speed = -1 * ANGLE_MAX_SPEED;
    }
  }

  // And later go to ball too maybe using encoder data but idk
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "decider");
  ros::NodeHandle n;
  ros::Publisher motor_control_pub = n.advertise<robonaldo_msgs::motor_speeds>("motor_control", 1000);
  ros::Subscriber user_sub = n.subscribe("user_input", 1000, userInputCallback);
  ros::Subscriber ball_sub = n.subscribe("ball_position", 1000, ballPositionCallback);
  ros::Rate loop_rate(60);

  motor_speeds_msg.left_speed = 0.0;
  motor_speeds_msg.right_speed = 0.0;

  while (ros::ok())
  {
    // This is a message object. stuff it with data and publish it

    // ROS_INFO("Sending %f %f", motor_speeds_msg.left_speed, motor_speeds_msg.right_speed);

    motor_control_pub.publish(motor_speeds_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
