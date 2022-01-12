// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcutils/error_handling.h>

#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/compressed_image.h>

#include <stdio.h>
#include <unistd.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
std_msgs__msg__Int32 int32_msg;
rcl_publisher_t image_publisher;
sensor_msgs__msg__CompressedImage image_msg_static;
#define BUF_LEN 32
#define BUF_CAP 32
#define STR_CAP 32
uint8_t buf[BUF_CAP];
uint8_t image_buf[BUF_CAP];
rcutils_error_state_t *error;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &int32_msg, NULL));
    printf("Sent: %d\n", int32_msg.data);
    int32_msg.data++;

    memcpy(image_msg_static.data.data, buf, BUF_LEN);
    image_msg_static.data.size = BUF_LEN;
    RCSOFTCHECK(rcl_publish(&image_publisher, &image_msg_static, NULL));
    printf("Sent %d bytes of image\n", image_msg_static.data.size);
  }
}

int main(int argc, char ** argv)
{
  printf("setup frame_id\n");
  image_msg_static.header.frame_id.capacity = STR_CAP;
  image_msg_static.header.frame_id.data = (char*) malloc(image_msg_static.header.frame_id.capacity * sizeof(char));
  strcpy(image_msg_static.header.frame_id.data, "my_image_topic");
  image_msg_static.header.frame_id.size = strlen(image_msg_static.header.frame_id.data);

  printf("setup format\n");
  image_msg_static.format.capacity = STR_CAP;
  image_msg_static.format.data = (char*) malloc(image_msg_static.format.capacity * sizeof(char));
  strcpy(image_msg_static.format.data, "jpeg");
  image_msg_static.format.size = strlen(image_msg_static.format.data);

  printf("setup data\n");
  image_msg_static.data.capacity = BUF_CAP;
  image_msg_static.data.data = image_buf;
  image_msg_static.data.size = 0;
  for(int32_t i = 0; i < BUF_CAP; i++){
    image_msg_static.data.data[i] = i;
    image_msg_static.data.size += 1;
  }


  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;

  // create init_options
  printf("init\n");
  RCSOFTCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  printf("create node\n");
  rcl_node_t node;
  RCCHECK(rclc_node_init_default(&node, "int32_publisher_rclc", "", &support));

  // create publisher
  printf("create publisher\n");
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "std_msgs_msg_Int32"));
  
  // create publisher
  printf("create image publisher\n");
  RCCHECK(rclc_publisher_init_default(
    &image_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
    "sensor_msgs_msg_CompressedImage"));
  
  // create timer
  rcl_timer_t timer;
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
  
  // create executor
  rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  int32_msg.data = 0;

  rclc_executor_spin(&executor);

  RCCHECK(rcl_publisher_fini(&publisher, &node));
  RCCHECK(rcl_publisher_fini(&image_publisher, &node));
  RCCHECK(rcl_node_fini(&node));
  return 0;
}
