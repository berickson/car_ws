#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <tuple> // for std::ignore

#include "car_msgs/msg/update.h"
#include "car_msgs/msg/rc_command.h"
#include "std_srvs/srv/empty.h"
#include "sensor_msgs/msg/battery_state.h"


#include "pwm_input.h"
#include "servo2.h"
#include "motor_encoder.h"
#include "quadrature_encoder.h"
#include "blinker.h"



#if defined(BLUE_CAR)
const int pin_led = 13;
const int pin_motor_a = 14;
const int pin_motor_b = 12;
const int pin_motor_c = 11;
//const int pin_motor_temp = A13;

const int pin_odo_fl_a = 23;
const int pin_odo_fl_b = 22;
const int pin_odo_fr_a = 21;
const int pin_odo_fr_b = 20;

const int pin_str = 26;
const int pin_esc = 27;
const int pin_esc_aux = 15;

const int pin_rx_str = 25;
const int pin_rx_esc = 24;

const int pin_mpu_interrupt = 17;

const int pin_vbat_sense = A13;
const int pin_cell1_sense = A14;
const int pin_cell2_sense = A15;
const int pin_cell3_sense = A16;
const int pin_cell4_sense = A17;
const int pin_cell0_sense = A18;
#endif

///////////////////////////////////////////////
// Globals

PwmInput rx_str;
PwmInput rx_esc;

Servo2 str;
Servo2 esc;

QuadratureEncoder odo_fl(pin_odo_fl_a, pin_odo_fl_b);
QuadratureEncoder odo_fr(pin_odo_fr_a, pin_odo_fr_b);

MotorEncoder motor(pin_motor_a, pin_motor_b, pin_motor_c);

Blinker blinker;

float cell_voltages[4];

///////////////////////////////////////////////
// Interrupt handlers

void rx_str_handler() {
  rx_str.handle_change();
}

void rx_esc_handler() {
  rx_esc.handle_change();
}

void odo_fl_a_changed() {
  odo_fl.sensor_a_changed();
}

void odo_fl_b_changed() {
  odo_fl.sensor_b_changed();
}

void odo_fr_a_changed() {
  odo_fr.sensor_a_changed();
}

void odo_fr_b_changed() {
  odo_fr.sensor_b_changed();
}

void motor_a_changed() {
  motor.on_a_change();
}

void motor_b_changed() {
  motor.on_b_change();
}

void motor_c_changed() {
  motor.on_c_change();
}

///////////////////////////////////////////////
// Misc Code
class BatterySensor {
public:

  int resolution_bits = 10;
  float r1 = 102;
  float r2 = 422;
  float max_reading = 1024;
  float scale = (r1+r2) * 3.3 / (max_reading * r2);

  float v_bat = 0;
  float v_cell0 = 0;
  float v_cell1 = 0;
  float v_cell2 = 0;
  float v_cell3 = 0;
  float v_cell4 = 0;



  void init() {
    analogReadResolution(resolution_bits);	
    max_reading = pow(2, resolution_bits);
    scale = (r1+r2) * 3.3 / (max_reading * r1);
  }

  void execute() {
#if defined(BLUE_CAR)
    v_bat = analogRead(pin_vbat_sense) * 12.47/744;
    v_cell0 = analogRead(pin_cell0_sense) * scale;
    v_cell1 = analogRead(pin_cell1_sense)  * 4.161/246;
    v_cell2 = analogRead(pin_cell2_sense)  * 8.32/497;
    v_cell3 = analogRead(pin_cell3_sense) * 12.48/744;
    v_cell4 = analogRead(pin_cell4_sense) * 12.48/744;

    /* 
    // calibration logging
    char buffer[200];
    sprintf(buffer, "V Cells: bat %4.3f cell0: %4.3f cell1: %4.3f cell2: %4.3f cell3: %4.3f cell4: %4.3f", v_bat, v_cell0, v_cell1, v_cell2, v_cell3, v_cell4);
    nh.loginfo(buffer);
    sprintf(buffer, "Raw Cells: bat %d cell0: %d cell1: %d cell2: %d cell3: %d cell4: %d", analogRead(pin_vbat_sense), analogRead(pin_cell0_sense), analogRead(pin_cell1_sense), analogRead(pin_cell2_sense), analogRead(pin_cell3_sense), analogRead(pin_cell4_sense));
    nh.loginfo(buffer);
    */
#elif defined(ORANGE_CAR)
    // constants below based on 220k and 1M resistor, 1023 steps and 3.3 reference voltage
    v_bat = analogRead(pin_vbat_sense) * ((3.3/1023.) / 220.)*(220.+1000.);
#else
#error "voltage not defined for this car"
#endif
  }
};

BatterySensor battery_sensor;



///////////////////////////////////////////////
// ROS

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
rcl_publisher_t battery_state_publisher;
rcl_subscription_t rc_command_subscription;
rcl_service_t enable_rc_mode_service;
rcl_service_t disable_rc_mode_service;
car_msgs__msg__Update update_message;
car_msgs__msg__RcCommand rc_command_message;
sensor_msgs__msg__BatteryState battery_state_message;
std_srvs__srv__Empty_Request empty_request_message;
std_srvs__srv__Empty_Response empty_response_message;


bool micro_ros_init_successful;

enum uros_states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} uros_state;

void rc_command_received(const void * msg)
{
  // Cast received message to used type
  const car_msgs__msg__RcCommand * rc_command = (const car_msgs__msg__RcCommand *)msg;

  // Process message
    // hack: just to debug whether the messages arrive
    update_message.ax = rc_command->str_us;
    update_message.ay = rc_command->esc_us;
    update_message.az = 999;

}

void enable_rc_mode_service_callback(const void * /*request_msg*/, void * /*response_msg*/) {
  update_message.go = true;
}

void disable_rc_mode_service_callback(const void * /*request_msg*/, void * /*response_msg*/) {
  update_message.go = false;

}


void publish_update_message() {

    update_message.ms = millis();
    update_message.us = micros();
    update_message.str = str.readMicroseconds();
    update_message.esc = esc.readMicroseconds();

    update_message.rx_esc = rx_esc.pulse_us();
    update_message.rx_str = rx_str.pulse_us();

    noInterrupts();
    update_message.spur_us = motor.last_change_us;
    update_message.spur_odo = motor.odometer;
    interrupts();

    noInterrupts();
    update_message.odo_fl_a = odo_fl.odometer_a;
    update_message.odo_fl_a_us = odo_fl.last_odometer_a_us;
    update_message.odo_fl_b = odo_fl.odometer_b;
    update_message.odo_fl_b_us = odo_fl.last_odometer_b_us;
    update_message.odo_fl_ab_us = odo_fl.odometer_ab_us;
    interrupts();

    noInterrupts();
    update_message.odo_fr_a = odo_fr.odometer_a;
    update_message.odo_fr_a_us = odo_fr.last_odometer_a_us;
    update_message.odo_fr_b = odo_fr.odometer_b;
    update_message.odo_fr_b_us = odo_fr.last_odometer_b_us;
    update_message.odo_fr_ab_us = odo_fr.odometer_ab_us;
    interrupts();


    std::ignore = rcl_publish(&update_publisher, &update_message, NULL);
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
    "car/update");
  
  rclc_publisher_init_best_effort(
    &battery_state_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
    "car/battery"
  );

  rclc_subscription_init_best_effort(
    &rc_command_subscription,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(car_msgs, msg, RcCommand),
    "car/rc_command"
  );

  auto empty_type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Empty);
  
  rclc_service_init_best_effort(
    &enable_rc_mode_service, 
    &node,
    empty_type_support, 
    "/enable_rc_mode");

  rclc_service_init_default(
    &disable_rc_mode_service, 
    &node,
    empty_type_support, 
    "/disable_rc_mode");
  

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();

  rclc_executor_init(&executor, &support.context, 3, &allocator);


  rclc_executor_add_service(
    &executor,
    &enable_rc_mode_service,
    &empty_request_message,
    &empty_response_message,
    &enable_rc_mode_service_callback
  );

  rclc_executor_add_service(
    &executor,
    &disable_rc_mode_service,
    &empty_request_message,
    &empty_response_message,
    &disable_rc_mode_service_callback
  );

  rclc_executor_add_subscription(
    &executor,
    &rc_command_subscription,
    &rc_command_message,
    &rc_command_received,
    ON_NEW_DATA
  );


  return true;
}

void destroy_uros_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  
  std::ignore = rcl_publisher_fini(&update_publisher, &node);
  std::ignore = rcl_publisher_fini(&battery_state_publisher, &node);
  std::ignore = rcl_timer_fini(&timer);
  std::ignore = rclc_executor_fini(&executor);
  std::ignore = rcl_node_fini(&node);
  std::ignore = rclc_support_fini(&support);
}

void setup() {
  Serial.begin(921600);

  set_microros_serial_transports(Serial);

  uros_state = WAITING_AGENT;

  rx_str.attach(pin_rx_str);
  rx_esc.attach(pin_rx_esc);
  str.attach(pin_str);
  esc.attach(pin_esc);

  // 1500 is usually safe...
  esc.writeMicroseconds(1500);
  str.writeMicroseconds(1500);

  attachInterrupt(pin_rx_str, rx_str_handler, CHANGE);
  attachInterrupt(pin_rx_esc, rx_esc_handler, CHANGE);

  pinMode(pin_motor_a, INPUT);
  pinMode(pin_motor_b, INPUT);
  pinMode(pin_motor_c, INPUT);

  attachInterrupt(pin_motor_a, motor_a_changed, CHANGE);
  attachInterrupt(pin_motor_b, motor_b_changed, CHANGE);
  attachInterrupt(pin_motor_c, motor_c_changed, CHANGE);

  pinMode(pin_odo_fl_a, INPUT);
  pinMode(pin_odo_fl_b, INPUT);
  pinMode(pin_odo_fr_a, INPUT);
  pinMode(pin_odo_fr_a, INPUT);
  
  attachInterrupt(pin_odo_fl_a, odo_fl_a_changed, CHANGE);
  attachInterrupt(pin_odo_fl_b, odo_fl_b_changed, CHANGE);
  attachInterrupt(pin_odo_fr_a, odo_fr_a_changed, CHANGE);
  attachInterrupt(pin_odo_fr_b, odo_fr_b_changed, CHANGE);

  blinker.init(pin_led);

  // battery
  battery_state_message.cell_voltage.data = cell_voltages;
  battery_state_message.cell_voltage.size = 4;
  battery_state_message.cell_voltage.capacity = 4;
  battery_sensor.init();

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
        auto timeout_ns = 0;
        rclc_executor_spin_some(&executor, timeout_ns);
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

  blinker.execute();
  maintain_uros_connection();


  if(every_n_ms(last_loop_ms, loop_ms, 10)) {

    if(rx_str.pulse_us() > 0 && rx_esc.pulse_us() > 0) {
      str.writeMicroseconds(rx_str.pulse_us());
      esc.writeMicroseconds(rx_esc.pulse_us());
    } else {
      esc.writeMicroseconds(1500);
      str.writeMicroseconds(1500);
    }

    publish_update_message();
  }

  if(every_n_ms(last_loop_ms, loop_ms, 1000)) {
    battery_sensor.execute();

    const char * frame = "base_link";
    const char * location = "blue-crash4";
    battery_state_message.header.stamp.nanosec = rmw_uros_epoch_nanos() | 0xffff;
    battery_state_message.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
    battery_state_message.header.frame_id.capacity = sizeof(frame);
    battery_state_message.header.frame_id.size = sizeof(frame);
    battery_state_message.header.frame_id.data = frame;
    battery_state_message.charge = NAN;
    battery_state_message.current = NAN;
    battery_state_message.location.capacity = sizeof(location);
    battery_state_message.location.size = sizeof(location);
    battery_state_message.location.data = location;
    
    ;


    battery_state_message.cell_voltage.data[0] = battery_sensor.v_cell1 - battery_sensor.v_cell0;
    battery_state_message.cell_voltage.data[1] = battery_sensor.v_cell2 - battery_sensor.v_cell1;
    battery_state_message.cell_voltage.data[2] = battery_sensor.v_cell3 - battery_sensor.v_cell2;
    battery_state_message.cell_voltage.data[3] = battery_sensor.v_cell4 - battery_sensor.v_cell3;
    battery_state_message.voltage = battery_sensor.v_bat;
    if(battery_sensor.v_cell4 > 2.0) {
      battery_state_message.cell_voltage.size = 4;
    } else if(battery_sensor.v_cell3 > 2.0 ) {
       battery_state_message.cell_voltage.size = 3;
    } else if(battery_sensor.v_cell2 > 2.0 ) {
       battery_state_message.cell_voltage.size = 2;
    } else if(battery_sensor.v_cell1 > 2.0 ) {
      battery_state_message.cell_voltage.size = 1;
    } else {
       battery_state_message.cell_voltage.size = 0;
    }

    battery_state_message.present = battery_state_message.cell_voltage.size > 0;

    if(battery_state_message.cell_voltage.size > 0) {
      float cell_average = battery_sensor.v_bat / battery_state_message.cell_voltage.size;
      battery_state_message.percentage = constrain(map(cell_average,3.5,4.2,0.0,1.0),0.0,1.0);
    } else {
      battery_state_message.percentage = NAN;
    }

    std::ignore = rcl_publish(&battery_state_publisher, &battery_state_message, NULL);

  }

  last_loop_ms = loop_ms;
}