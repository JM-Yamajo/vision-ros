#include <stdio.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int16.h>

rcl_node_t node;
rcl_timer_t timer;
rcl_publisher_t publisher;
rcl_allocator_t allocator;

rclc_support_t support;
rclc_executor_t executor;

std_msgs__msg__Int32 msg;
