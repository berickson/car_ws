#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>

#include <tuple> // for std::ignore

#include "car_msgs/msg/update.h"

#define LED_PIN 13
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t update_publisher;
car_msgs__msg__Update update_msg;
bool micro_ros_init_successful;

enum uros_states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} uros_state;

void publish_update_message() {
    msg.data++;

    update_msg.ms = millis();
    update_msg.us = micros();
    std::ignore = rcl_publish(&update_publisher, &update_msg, NULL);

}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {
    std::ignore = rcl_publish(&publisher, &msg, NULL);
  }
}



// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library with
// - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3

bool create_uros_entities()
{
  allocator = rcl_get_default_allocator();

  // create init_options
  rclc_support_init(&support, 0, NULL, &allocator);

  // create node
  rclc_node_init_default(&node, "car_controller", "", &support);

  rclc_publisher_init_best_effort(
    &update_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(car_msgs, msg, Update),
    "car_update");

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, 1, &allocator);

  return true;
}

void destroy_uros_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  
  std::ignore = rcl_publisher_fini(&publisher, &node);
  std::ignore = rcl_timer_fini(&timer);
  std::ignore = rclc_executor_fini(&executor);
  std::ignore = rcl_node_fini(&node);
  std::ignore = rclc_support_fini(&support);
}

void setup() {
  set_microros_serial_transports(Serial);
  pinMode(LED_PIN, OUTPUT);

  uros_state = WAITING_AGENT;

  msg.data = 0;
}

void maintain_uros_connection() {
  switch (uros_state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, uros_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      uros_state = (true == create_uros_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (uros_state == WAITING_AGENT) {
        destroy_uros_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, uros_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (uros_state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_uros_entities();
      uros_state = WAITING_AGENT;
      break;
    default:
      break;
  }
}

bool every_n_ms(uint32_t last_loop_ms, uint32_t loop_ms, uint32_t ms, uint32_t offset = 0) {
  return ((last_loop_ms-offset) % ms) + (loop_ms - last_loop_ms) >= ms;
}

void loop() {
  static uint32_t last_loop_ms = 0;
  uint32_t loop_ms = millis();

  maintain_uros_connection();

  if(every_n_ms(last_loop_ms, loop_ms, 10)) {
    publish_update_message();
  }

  if (uros_state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, 1);
  } else {
    digitalWrite(LED_PIN, 0);
  }

  last_loop_ms = loop_ms;
}