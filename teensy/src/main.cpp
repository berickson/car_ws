#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include "qmc5883.h" // magnetometer

#undef PLATFORMIO
#include "CRSFforArduino.hpp" // CRSF


#include <tuple> // for std::ignore

#include "nmea_msgs/msg/sentence.h"

#include "car_msgs/msg/update.h"
#include "car_msgs/msg/rc_command.h"
#include "std_srvs/srv/empty.h"
#include "sensor_msgs/msg/battery_state.h"
#include "sensor_msgs/msg/joy.h"


#include "pwm_input.h"
#include "Servo.h"
#include "motor_encoder.h"
#include "quadrature_encoder.h"
#include "task.h"
#include "fsm.h"
#include "hand_mode.h"
#include "auto_mode.h"
#include "off_mode.h"
#include "rx_events.h"
#include "blinker.h"



// all these ugly pushes are because the 9150 has a lot of warnings
// the .h file must be included in one time in a source file
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-value"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#include <MPU9150_9Axis_MotionApps41.h>
#pragma GCC diagnostic pop

#include "mpu9150.h"

#if defined(BLUE_CAR)
#define HAS_MOTOR_ODOM

//const int pin_motor_temp = A13;

const int pin_odo_fl_a = 0;
const int pin_odo_fl_b = 1;
const int pin_odo_fr_a = 2;
const int pin_odo_fr_b = 3;

const int pin_str = 7;
const int pin_esc = 8;

const int pin_motor_a = 10;
const int pin_motor_b = 11;
const int pin_motor_c = 12;

const int pin_led = 13;

const int pin_gps_rx = 14;
const int pin_gps_tx = 15;

#define WireMag Wire1
const int pin_mag_scl = 16; // Wire1 SCL
const int pin_mag_sda = 17; // Wire1 SDA

const int pin_elrs_rx = 20; // tx5
const int pin_elrs_tx = 21; // rx5

const int pin_mpu_interrupt = 22;
const int pin_vbat_sense = A9;

#endif

#if defined(SETH_CAR)
#define HAS_MOTOR_ODOM

const int pin_odo_fl_a = 0;
const int pin_odo_fl_b = 1;
const int pin_odo_fr_a = 2;
const int pin_odo_fr_b = 3;

const int pin_str = 7;
const int pin_esc = 8;

const int pin_motor_c = 10;
const int pin_motor_b = 11;
const int pin_motor_a = 12;


const int pin_led = 13;

const int pin_gps_rx = 14;
const int pin_gps_tx = 15;

#define WireMag Wire1
const int pin_mag_scl = 16; // Wire1 SCL
const int pin_mag_sda = 17; // Wire1 SDA

const int pin_elrs_rx = 20; // tx5
const int pin_elrs_tx = 21; // rx5

const int pin_mpu_interrupt = 22;

const int pin_vbat_sense = 23; // A9;

#endif

#if defined(BEEBOT_CAR)

const int pin_odo_fl_a = 0;
const int pin_odo_fl_b = 1;
const int pin_odo_fr_a = 2;
const int pin_odo_fr_b = 3;

const int pin_str = 7;
const int pin_esc = 8;

const int pin_motor_a = 10;
const int pin_motor_b = 11;
const int pin_motor_c = 12;

const int pin_led = 13;

const int pin_gps_rx = 14;
const int pin_gps_tx = 15;

#define WireMag Wire1
const int pin_mag_scl = 16; // Wire1 SCL
const int pin_mag_sda = 17; // Wire1 SDA

const int pin_elrs_rx = 20; // tx5
const int pin_elrs_tx = 21; // rx5

const int pin_mpu_interrupt = 22;
const int pin_vbat_sense = A9;

#endif

///////////////////////////////////////////////
// helpers

#define count_of(a) (sizeof(a)/sizeof(a[0]))

///////////////////////////////////////////////
// Globals

Mpu9150 mpu9150;
QMC5883 magnetometer(WireMag);
CRSFforArduino crsf = CRSFforArduino(&Serial5);
int16_t mag_x, mag_y, mag_z;


PwmInput rx_str;
PwmInput rx_esc;
PwmInput rx_aux;

RxEvents rx_events;

Servo str;
Servo esc;

QuadratureEncoder odo_fl(pin_odo_fl_a, pin_odo_fl_b);
QuadratureEncoder odo_fr(pin_odo_fr_a, pin_odo_fr_b);

#ifdef HAS_MOTOR_ODOM
MotorEncoder motor(pin_motor_a, pin_motor_b, pin_motor_c);
#endif

Blinker blinker;

float cell_voltages[4];

String nmea_sentence;

///////////////////////////////////////////////
// Interrupt handlers

void rx_str_handler() {
  rx_str.handle_change();
}

void rx_esc_handler() {
  rx_esc.handle_change();
}

void rx_aux_handler() {
  rx_aux.handle_change();
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

#ifdef HAS_MOTOR_ODOM
void motor_a_changed() {
  motor.on_a_change();
}

void motor_b_changed() {
  motor.on_b_change();
}

void motor_c_changed() {
  motor.on_c_change();
}
#endif

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
    pinMode(pin_vbat_sense, INPUT);
  }

  void execute() {
    analogReadAveraging(32);
    analogReadResolution(12);
#if defined(BLUE_CAR)
    v_bat = analogRead(pin_vbat_sense) * 0.018829 / 4.0;
#elif defined(SETH_CAR)
    v_bat = analogRead(pin_vbat_sense) * 0.00467307255;
#elif defined(BEEBOT_CAR)
    v_bat = analogRead(pin_vbat_sense) * 0.0023750386;
#else
#error "voltage not defined for this car"
#endif
  }
};

BatterySensor battery_sensor;


bool every_n_ms(uint32_t last_loop_ms, uint32_t loop_ms, uint32_t ms, uint32_t offset = 0) {
  return ((last_loop_ms-offset) % ms) + (loop_ms - last_loop_ms) >= ms;
}

///////////////////////////////////////////////
// modes

HandMode hand_mode;
OffMode off_mode;
AutoMode auto_mode;


Task * tasks[] = {&hand_mode, &off_mode, &auto_mode};

Fsm::Edge edges[] = {
  {"hand", "auto", "auto"},
  {"hand", "off", "off"},
  {"off", "hand", "hand"},
  {"off", "auto", "auto"},
  {"auto", "off", "off"},
  {"auto", "hand", "hand"},
  {"auto", "non-neutral", "hand"},
  {"auto", "steer-change", "hand"},
  {"auto", "throttle-change", "hand"},
  {"auto", "done", "hand"}
};

Fsm modes(tasks, count_of(tasks), edges, count_of(edges));

void command_hand() {
  modes.set_event("hand");
}

void command_remote_control() {
  modes.set_event("auto");
}

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
rcl_publisher_t joy_publisher;
rcl_publisher_t nmea_sentence_publisher;
rcl_subscription_t rc_command_subscription;
rcl_service_t enable_rc_mode_service;
rcl_service_t disable_rc_mode_service;
car_msgs__msg__Update update_message;
car_msgs__msg__RcCommand rc_command_message;
sensor_msgs__msg__BatteryState battery_state_message;
sensor_msgs__msg__Joy joy_message;
bool joy_message_ready = false;
nmea_msgs__msg__Sentence nmea_sentence_message;
std_srvs__srv__Empty_Request empty_request_message;
std_srvs__srv__Empty_Response empty_response_message;


bool micro_ros_init_successful;

enum uros_states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};
uros_states uros_state = WAITING_AGENT;


void rc_command_received(const void * msg) {
  // Cast received message to used type
  const car_msgs__msg__RcCommand * rc_command = (const car_msgs__msg__RcCommand *)msg;
  auto_mode.command_steer_and_esc(rc_command->str_us,  rc_command->esc_us);
}


void command_remote_control();  // forward decl
void command_hand();          // forward decl

void enable_rc_mode_service_callback(const void * /*request_msg*/, void * /*response_msg*/) {

  command_remote_control();
}

void disable_rc_mode_service_callback(const void * /*request_msg*/, void * /*response_msg*/) {

  command_hand();
}


void publish_update_message() {

    static char frame[] = "base_link";
    update_message.header.stamp.nanosec = rmw_uros_epoch_nanos() % 1000000000ULL;
    update_message.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
    update_message.header.frame_id.capacity = sizeof(frame);
    update_message.header.frame_id.size = sizeof(frame);
    update_message.header.frame_id.data = frame;

    update_message.ms = millis();
    update_message.us = micros();
    update_message.str = str.readMicroseconds();
    update_message.esc = esc.readMicroseconds();

    update_message.ax = mpu9150.ax;
    update_message.ay = mpu9150.ay;
    update_message.az = mpu9150.az;

    update_message.rx_esc = rx_esc.pulse_us();
    update_message.rx_str = rx_str.pulse_us();
    update_message.rx_aux = rx_aux.pulse_us();

    noInterrupts();
  #ifdef HAS_MOTOR_ODOM
    update_message.motor_us = motor.last_change_us;
    update_message.motor_odo = motor.odometer;
  #endif
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

    update_message.v_bat = battery_state_message.voltage;

    update_message.mpu_deg_yaw = mpu9150.heading();
    update_message.mpu_deg_pitch = mpu9150.pitch * 180. / M_PI;
    update_message.mpu_deg_roll = mpu9150.roll * 180. / M_PI;

    update_message.mpu_deg_c = mpu9150.temperature /340.0 + 35.0;

    update_message.mag_x = mag_x;
    update_message.mag_y = mag_y;
    update_message.mag_z = mag_z;
    update_message.mag_deg_yaw = magnetometer.azimuth(mag_x, mag_y);

    update_message.mode.capacity = sizeof(modes.current_task->name);
    update_message.mode.size = sizeof(modes.current_task->name);
    update_message.mode.data = (char *)modes.current_task->name;
  
    std::ignore = rcl_publish(&update_publisher, &update_message, NULL);
}

void publish_nmea_sentence_message() {
  static char frame[] = "base_link";
  nmea_sentence_message.header.stamp.nanosec = rmw_uros_epoch_nanos() | 0xffff;
  nmea_sentence_message.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
  nmea_sentence_message.header.frame_id.capacity = sizeof(frame);
  nmea_sentence_message.header.frame_id.size = sizeof(frame);
  nmea_sentence_message.header.frame_id.data = frame;
  nmea_sentence_message.sentence.capacity = nmea_sentence.length();
  nmea_sentence_message.sentence.size = nmea_sentence.length();
  nmea_sentence_message.sentence.data = (char *) nmea_sentence.c_str();

  std::ignore = rcl_publish(&nmea_sentence_publisher, &nmea_sentence_message, NULL);
}

void publish_battery_state_message() {
  static char frame[] = "base_link";
  static char location[] = "car";
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

  battery_state_message.cell_voltage.data = cell_voltages;
  battery_state_message.cell_voltage.size = 4;
  battery_state_message.cell_voltage.capacity = 4;

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
    battery_state_message.percentage = battery_sensor.v_bat/12.6;
  }

  std::ignore = rcl_publish(&battery_state_publisher, &battery_state_message, NULL);
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
  

  // unfortunately, the nmea sentence message doesn't accept best effort,
  // publishing to a separate topic so they can be relayed by gps_fixer
  rclc_publisher_init_best_effort(
    &nmea_sentence_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nmea_msgs, msg, Sentence),
    "car/gps/nmea_sentence_best_effort"
  );

  // // make a super small ack timeout to make sure we don't hang
  // int ack_timeout = 5;;
  // rmw_uros_set_publisher_session_timeout(&nmea_sentence_publisher, ack_timeout);

  rclc_publisher_init_best_effort(
    &battery_state_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
    "car/battery"
  );

  rclc_publisher_init_best_effort(
    &joy_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
    "car/joy"
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
    "/car/enable_ros_control");

  rclc_service_init_best_effort(
    &disable_rc_mode_service, 
    &node,
    empty_type_support, 
    "/car/disable_ros_control");
  

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
 std::ignore = rcl_publisher_fini(&joy_publisher, &node);
  std::ignore = rcl_publisher_fini(&nmea_sentence_publisher, &node);
  std::ignore = rcl_timer_fini(&timer);
  std::ignore = rclc_executor_fini(&executor);
  std::ignore = rcl_node_fini(&node);
  std::ignore = rclc_support_fini(&support);
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
      } else {
        rmw_uros_sync_session(100);
      }
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


void scan_i2c_devices(TwoWire & wire) {

  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    wire.beginTransmission(address);
    error = wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
 
}

void print_i2c_register_numbers_and_values(TwoWire & wire, int address, int start, int end) {
  Serial.printf("i2c register numbers and values for device at 0x%02X\n", address);
  for(int i=start; i<=end; i++) {
    wire.beginTransmission(address);
    wire.write(i);
    wire.endTransmission();
    wire.requestFrom(address, 1);
    if(wire.available()) {
      Serial.printf("  0x%02X: 0x%02X\n", i, wire.read());
    }
  }
}


///////////////////////////////////////////////
// setup and loop


void setup() {
  Serial.begin(921600);
  Serial3.begin(38400);
  delay(1000);

  // Initialize CRSF
  if (!crsf.begin())
  {
    // TODO: report setup errors some better way, maybe blink LED
      Serial.println("CRSF for Arduino initialization failed!");
      while (1)
      {
          ;
      }
  }

  crsf.setRcChannelsCallback([](serialReceiverLayer::rcChannels_t * rc_channels) {
    const int axis_count = 15;
    static float axes[axis_count];

    if(rc_channels->failsafe ){
      rx_str.set_from_crsf(0);
      rx_esc.set_from_crsf(0);
      rx_aux.set_from_crsf(0);
      for(int i=0; i<axis_count; i++) {
        axes[i] = 0;
      }
    } else {
      rx_str.set_from_crsf(crsf.rcToUs(crsf.getChannel(1)));
      rx_esc.set_from_crsf(crsf.rcToUs(crsf.getChannel(2)));
      rx_aux.set_from_crsf(crsf.rcToUs(crsf.getChannel(3)));

      for(int i=0; i<axis_count; i++) {
        // remap 1000-2000 to -1 to 1, clamping if necessary
        axes[i] = constrain(map(crsf.rcToUs(crsf.getChannel(i+1)), 1000, 2000, -1000, 1000) / 1000.0, -1.0, 1.0);
      }
    }


    // also send as joy message
    joy_message.header.stamp.nanosec = rmw_uros_epoch_nanos() % 1000000000ULL;
    joy_message.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
    joy_message.header.frame_id.capacity = 0;
    joy_message.header.frame_id.size = 0;
    joy_message.header.frame_id.data = NULL;
    joy_message.axes.data = axes;
    joy_message.axes.size = axis_count;
    joy_message.axes.capacity = axis_count;
    joy_message.buttons.data = NULL;
    joy_message.buttons.size = 0;
    joy_message.buttons.capacity = 0;
    joy_message_ready = true;
  });

  Serial.println("setup");
  Wire1.begin();
  magnetometer.setAddress(0x0D);
  magnetometer.softReset();
  magnetometer.init();
  magnetometer.setMode(Mode_Continuous,ODR_200Hz,RNG_8G,OSR_512);

  set_microros_serial_transports(Serial);
  
  str.attach(pin_str);
  esc.attach(pin_esc);

  // 1500 is usually safe...
  esc.writeMicroseconds(1500);
  str.writeMicroseconds(1500);

#ifdef HAS_MOTOR_ODOM
  pinMode(pin_motor_a, INPUT);
  pinMode(pin_motor_b, INPUT);
  pinMode(pin_motor_c, INPUT);

  attachInterrupt(pin_motor_a, motor_a_changed, CHANGE);
  attachInterrupt(pin_motor_b, motor_b_changed, CHANGE);
  attachInterrupt(pin_motor_c, motor_c_changed, CHANGE);
#endif
  pinMode(pin_odo_fl_a, INPUT);
  pinMode(pin_odo_fl_b, INPUT);
  pinMode(pin_odo_fr_a, INPUT);
  pinMode(pin_odo_fr_a, INPUT);
  
  attachInterrupt(pin_odo_fl_a, odo_fl_a_changed, CHANGE);
  attachInterrupt(pin_odo_fl_b, odo_fl_b_changed, CHANGE);
  attachInterrupt(pin_odo_fr_a, odo_fr_a_changed, CHANGE);
  attachInterrupt(pin_odo_fr_b, odo_fr_b_changed, CHANGE);

  blinker.init(pin_led);

  battery_sensor.init();

  modes.begin();
  Wire.begin();

  mpu9150.setup();

#if defined(BLUE_CAR)
#define HAS_MOTOR_ODOM
  mpu9150.ax_bias = 0;
  mpu9150.ay_bias = 0;
  mpu9150.az_bias = 7893.51;
  mpu9150.rest_a_mag =  7893.51;
  mpu9150.zero_adjust = Quaternion(1.0, 0.0, 0.0, 0.0);
  mpu9150.yaw_slope_rads_per_ms  = -0.0000000680;
  mpu9150.yaw_actual_per_raw = 1;
#elif defined(SETH_CAR)
  mpu9150.ax_bias = 0;
  mpu9150.ay_bias = 0;
  mpu9150.az_bias = 7893.51;
  mpu9150.rest_a_mag =  7893.51;
  mpu9150.zero_adjust = Quaternion(1.0, 0.0, 0.0, 0.0);
  mpu9150.yaw_slope_rads_per_ms  = -0.0000000680;
  mpu9150.yaw_actual_per_raw = 1;
#elif defined(BEEBOT_CAR)
  mpu9150.ax_bias = 0;
  mpu9150.ay_bias = 0;
  mpu9150.az_bias = 7893.51;
  mpu9150.rest_a_mag =  7893.51;
  mpu9150.zero_adjust = Quaternion(1.0, 0.0, 0.0, 0.0);
  mpu9150.yaw_slope_rads_per_ms  = -0.0000000680;
  mpu9150.yaw_actual_per_raw = 1;
#else
#error "Car not defined for MPU"
#endif
  mpu9150.zero_heading();

}

char last_hoa_mode = '?'; // hand/off/auto indicator
RxEvent last_rx_event; 
void loop() {
  static uint32_t last_loop_ms = 0;
  uint32_t loop_ms = millis();

  blinker.execute();

  rx_events.process_pulses(rx_str.pulse_us(), rx_esc.pulse_us(), rx_aux.pulse_us());
  bool new_rx_event = rx_events.get_event();
  // send events through modes state machine
  if(new_rx_event) {
    auto c = rx_events.current;
    if(! (c.steer == 'C' && c.aux=='N')) {
      modes.set_event("non-neutral");
    }
    if(c.steer != last_rx_event.steer) {
      modes.set_event("steer-change");
    }
    if(c.speed != last_rx_event.speed) {
      modes.set_event("throttle-change");
    }
    if(c.aux != last_hoa_mode) {
      if(c.aux == 'H') {
        modes.set_event("hand");
      } else if(c.aux == 'A') {
        modes.set_event("auto");
      } else {
        modes.set_event("off");
      }
      last_hoa_mode = c.aux;
    } 
  }

  maintain_uros_connection();

  if(every_n_ms(last_loop_ms, loop_ms, 10)) {
    modes.execute();
  }

  if(every_n_ms(last_loop_ms, loop_ms, 10)) {
    if(joy_message_ready) {
      std::ignore = rcl_publish(&joy_publisher, &joy_message, NULL);
      joy_message_ready = false;
    }
  }

  crsf.update(); // update as fast as possible, will call callbacks

  if(every_n_ms(last_loop_ms, loop_ms, 200)) {
      crsf.telemetryWriteBattery(battery_sensor.v_bat * 100,0,0,0);
      crsf.telemetryWriteCustomFlightMode(modes.current_task->name);
  }

  if(every_n_ms(last_loop_ms, loop_ms, 10, 3)) {
    // read magnetometer
    magnetometer.read(&mag_x, &mag_y, &mag_z);
  }

  while(Serial3.available()) {
    char c = Serial3.read();
    if(c=='\r') {
      continue;
    }
    if(c=='\n') {
      // send nmea_sentence
      publish_nmea_sentence_message();
      nmea_sentence = "";
      continue;
    }
    nmea_sentence.append(c);
    if(nmea_sentence.length() > 300) {
      nmea_sentence = "";
    }
  }

  // mpu9150 execute takes about 3ms when there is an interrupt,
  // and this messes up the perfect 10ms update timings.  Running it at
  // 2ms offset from the updates keeps it from interfering
  if(every_n_ms(last_loop_ms, loop_ms, 10, 2)) {
     mpu9150.execute();
  }

  if(every_n_ms(last_loop_ms, loop_ms, 10)) {

    // if(rx_str.pulse_us() > 0 && rx_esc.pulse_us() > 0) {
    //   str.writeMicroseconds(rx_str.pulse_us());
    //   esc.writeMicroseconds(rx_esc.pulse_us());
    // } else {
    //   esc.writeMicroseconds(1500);
    //   str.writeMicroseconds(1500);
    // }

    publish_update_message();
  }

  if(every_n_ms(last_loop_ms, loop_ms, 1000)) {
    battery_sensor.execute();
    publish_battery_state_message();
  }

  // sync clock every 10 minutes
  if(every_n_ms(last_loop_ms, loop_ms, 10*60*1000)) {
    rmw_uros_sync_session(10);
  }

  last_loop_ms = loop_ms;
}