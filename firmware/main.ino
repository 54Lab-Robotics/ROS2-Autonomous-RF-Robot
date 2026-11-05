#include <Arduino.h>
#include <Wire.h>
#include <FastAccelStepper.h>
#include <Adafruit_NeoPixel.h>
#include <INA226.h>
#include <TMCStepper.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <string.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/battery_state.h>

#ifndef INA226_MINIMAL_SHUNT_OHM
#define INA226_MINIMAL_SHUNT_OHM (0.010)
#endif

// Pins
#define NEOPIXEL_PIN 21
#define NUMPIXELS    1

#define PIN_EN 12
#define PIN_RX 3
#define PIN_TX 2

#define L_STEP 13
#define L_DIR  1
#define R_STEP 4
#define R_DIR  5

#define INVERT_L_MOTOR false
#define INVERT_R_MOTOR true

// Drive & motor config
#define ACCEL             8000
#define R_SENSE           0.11f
#define MICROSTEPS        16
#define FULL_STEPS_REV    200.0f
#define MOTOR_RMS_CURRENT 450
#define GEAR_RATIO        1.777f
#define WHEEL_RADIUS      0.05f
#define LX                0.0711f   // half track width (m)

#define CMD_VEL_TIMEOUT_MS 500
#define LOW_VOLTAGE_WARN   10.5f

const float STEPS_PER_REV   = FULL_STEPS_REV * (float)MICROSTEPS * GEAR_RATIO;
const float WHEEL_CIRCUMF   = 2.0f * PI * WHEEL_RADIUS;
const float STEPS_PER_METER = STEPS_PER_REV / WHEEL_CIRCUMF;

Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
INA226 INA(0x40);
HardwareSerial SerialTMC(1);

TMC2209Stepper driver_L(&SerialTMC, R_SENSE, 0);
TMC2209Stepper driver_R(&SerialTMC, R_SENSE, 1);

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper_L = NULL;
FastAccelStepper *stepper_R = NULL;

struct TwistCommand {
  float linear_x;
  float angular_z;
  uint32_t timestamp_ms;
};

struct SharedRoverState {
  float x;
  float y;
  float theta;
  float vx;
  float wz;
  float voltage;
  float current;
  bool is_moving;
};

QueueHandle_t cmdVelQueue;
SemaphoreHandle_t stateMutex;

SharedRoverState roverState = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};

TaskHandle_t kinematicsTaskHandle = NULL;
TaskHandle_t microROSTaskHandle   = NULL;
TaskHandle_t hardwareTaskHandle   = NULL;

rcl_publisher_t odom_pub;
rcl_publisher_t battery_pub;
rcl_subscription_t cmd_vel_sub;
rcl_timer_t publish_timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

geometry_msgs__msg__Twist twist_msg;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__BatteryState battery_msg;

enum AgentState {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};

volatile AgentState agentState = WAITING_AGENT;
volatile uint32_t last_cmd_rx_ms = 0;
uint8_t battery_pub_divider = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }

void setPixelColor(uint8_t r, uint8_t g, uint8_t b) {
  static uint8_t last_r = 255, last_g = 255, last_b = 255;
  if (r != last_r || g != last_g || b != last_b) {
    last_r = r;
    last_g = g;
    last_b = b;
    pixels.setPixelColor(0, pixels.Color(r, g, b));
    pixels.show();
  }
}

void setupTMC(TMC2209Stepper &driver) {
  driver.begin();
  driver.toff(3);
  driver.blank_time(24);
  driver.rms_current(MOTOR_RMS_CURRENT);
  driver.ihold(3);          // low hold current so motors don't cook at idle
  driver.iholddelay(10);
  driver.TPOWERDOWN(20);
  driver.microsteps(MICROSTEPS);

  driver.en_spreadCycle(false);
  driver.pwm_autoscale(true);
  driver.pwm_autograd(true);
  driver.TPWMTHRS(0);

  driver.semin(0);
  driver.SGTHRS(100);
}

void setStepperVelocity(FastAccelStepper *stepper, float velocity_m_s, bool invert) {
  if (stepper == NULL) return;
  if (invert) velocity_m_s = -velocity_m_s;

  uint32_t steps_per_sec = (uint32_t)(fabsf(velocity_m_s) * STEPS_PER_METER);

  if (steps_per_sec < 5) {
    stepper->stopMove();
    return;
  }

  stepper->setSpeedInHz(steps_per_sec);
  if (velocity_m_s > 0.0f) {
    stepper->runForward();
  } else {
    stepper->runBackward();
  }
}

void stopMotors() {
  if (stepper_L) stepper_L->stopMove();
  if (stepper_R) stepper_R->stopMove();
}

// 50 Hz loop
void kinematicsTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20);

  int32_t last_steps_L = 0;
  int32_t last_steps_R = 0;
  uint32_t last_time_us = micros();

  TwistCommand latestCmd = {0.0f, 0.0f, 0};
  float local_x = 0.0f;
  float local_y = 0.0f;
  float local_theta = 0.0f;

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    uint32_t now_ms = millis();
    uint32_t now_us = micros();
    float dt = (now_us - last_time_us) / 1000000.0f;
    if (dt <= 0.0001f || dt > 0.5f) dt = 0.02f;
    last_time_us = now_us;

    xQueuePeek(cmdVelQueue, &latestCmd, 0);

    bool active_motion = false;

    if (agentState == AGENT_CONNECTED && (now_ms - latestCmd.timestamp_ms <= CMD_VEL_TIMEOUT_MS)) {
      float v_L = latestCmd.linear_x - (latestCmd.angular_z * LX);
      float v_R = latestCmd.linear_x + (latestCmd.angular_z * LX);

      setStepperVelocity(stepper_L, v_L, INVERT_L_MOTOR);
      setStepperVelocity(stepper_R, v_R, INVERT_R_MOTOR);

      active_motion = (fabsf(latestCmd.linear_x) > 0.001f || fabsf(latestCmd.angular_z) > 0.001f);
    } else {
      stopMotors();
      active_motion = false;
    }

    int32_t cur_steps_L = stepper_L ? stepper_L->getCurrentPosition() : 0;
    int32_t cur_steps_R = stepper_R ? stepper_R->getCurrentPosition() : 0;

    int32_t d_steps_L = cur_steps_L - last_steps_L;
    int32_t d_steps_R = cur_steps_R - last_steps_R;
    last_steps_L = cur_steps_L;
    last_steps_R = cur_steps_R;

    if (INVERT_L_MOTOR) d_steps_L = -d_steps_L;
    if (INVERT_R_MOTOR) d_steps_R = -d_steps_R;

    float dist_L = (float)d_steps_L / STEPS_PER_METER;
    float dist_R = (float)d_steps_R / STEPS_PER_METER;

    float d_center = (dist_R + dist_L) / 2.0f;
    float d_theta  = (dist_R - dist_L) / (2.0f * LX);

    local_x     += d_center * cosf(local_theta + (d_theta / 2.0f));
    local_y     += d_center * sinf(local_theta + (d_theta / 2.0f));
    local_theta += d_theta;

    while (local_theta > PI)  local_theta -= 2.0f * PI;
    while (local_theta < -PI) local_theta += 2.0f * PI;

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      roverState.x = local_x;
      roverState.y = local_y;
      roverState.theta = local_theta;
      roverState.vx = d_center / dt;
      roverState.wz = d_theta / dt;
      roverState.is_moving = active_motion;
      xSemaphoreGive(stateMutex);
    }
  }
}

void hardwareTask(void *pvParameters) {
  for (;;) {
    float busVoltage = INA.getBusVoltage();
    float current    = INA.getCurrent();
    bool moving      = false;

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      roverState.voltage = busVoltage;
      roverState.current = current;
      moving = roverState.is_moving;
      xSemaphoreGive(stateMutex);
    }

    // Status LED: Red = low bat, Yellow = no agent, Blue = moving, Green = idle
    if (busVoltage > 1.0f && busVoltage < LOW_VOLTAGE_WARN) {
      setPixelColor(255, 0, 0);
    } else if (agentState != AGENT_CONNECTED) {
      setPixelColor(200, 100, 0);
    } else if (moving) {
      setPixelColor(0, 100, 255);
    } else {
      setPixelColor(0, 180, 0);
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  uint32_t now = millis();
  TwistCommand cmd;
  cmd.linear_x     = msg->linear.x;
  cmd.angular_z    = msg->angular.z;
  cmd.timestamp_ms = now;

  last_cmd_rx_ms = now;

  xQueueOverwrite(cmdVelQueue, &cmd);
}

void publish_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer == NULL) return;

  SharedRoverState snapshot;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    snapshot = roverState;
    xSemaphoreGive(stateMutex);
  } else {
    return;
  }

  int64_t time_ns = rmw_uros_epoch_nanos();
  if (time_ns > 0) {
    odom_msg.header.stamp.sec     = time_ns / 1000000000LL;
    odom_msg.header.stamp.nanosec = time_ns % 1000000000LL;
  } else {
    uint32_t ms = millis();
    odom_msg.header.stamp.sec     = ms / 1000;
    odom_msg.header.stamp.nanosec = (ms % 1000) * 1000000ULL;
  }

  odom_msg.pose.pose.position.x    = snapshot.x;
  odom_msg.pose.pose.position.y    = snapshot.y;
  odom_msg.pose.pose.position.z    = 0.0;
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sinf(snapshot.theta / 2.0f);
  odom_msg.pose.pose.orientation.w = cosf(snapshot.theta / 2.0f);

  odom_msg.twist.twist.linear.x  = snapshot.vx;
  odom_msg.twist.twist.angular.z = snapshot.wz;

  RCSOFTCHECK(rcl_publish(&odom_pub, &odom_msg, NULL));

  battery_pub_divider++;
  if (battery_pub_divider >= 20) {
    battery_pub_divider = 0;

    battery_msg.header.stamp = odom_msg.header.stamp;
    battery_msg.voltage      = snapshot.voltage;
    battery_msg.current      = snapshot.current;
    battery_msg.present      = true;

    RCSOFTCHECK(rcl_publish(&battery_pub, &battery_msg, NULL));
  }
}

bool create_entities() {
  allocator = rcl_get_default_allocator();

  support       = {};
  node          = rcl_get_zero_initialized_node();
  odom_pub      = rcl_get_zero_initialized_publisher();
  battery_pub   = rcl_get_zero_initialized_publisher();
  cmd_vel_sub   = rcl_get_zero_initialized_subscription();
  publish_timer = rcl_get_zero_initialized_timer();
  executor      = rclc_executor_get_zero_initialized_executor();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "diff_rover_rtos_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &odom_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));

  RCCHECK(rclc_publisher_init_default(
    &battery_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), "battery_state"));

  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  RCCHECK(rclc_timer_init_default(
    &publish_timer, &support, RCL_MS_TO_NS(50), publish_timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &publish_timer));

  rmw_uros_sync_session(500);
  return true;
}

void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCSOFTCHECK(rcl_publisher_fini(&odom_pub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&battery_pub, &node));
  RCSOFTCHECK(rcl_subscription_fini(&cmd_vel_sub, &node));
  RCSOFTCHECK(rcl_timer_fini(&publish_timer));
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));
}

void microROSTask(void *pvParameters) {
  uint32_t last_ping_check_ms = 0;
  uint8_t ping_fail_count = 0;

  for (;;) {
    switch (agentState) {
      case WAITING_AGENT:
        if (rmw_uros_ping_agent(250, 1) == RMW_RET_OK) {
          agentState = AGENT_AVAILABLE;
        } else {
          vTaskDelay(pdMS_TO_TICKS(300));
        }
        break;

      case AGENT_AVAILABLE:
        if (create_entities()) {
          agentState = AGENT_CONNECTED;
          last_ping_check_ms = millis();
          last_cmd_rx_ms     = millis();
          ping_fail_count    = 0;
        } else {
          destroy_entities();
          agentState = WAITING_AGENT;
          vTaskDelay(pdMS_TO_TICKS(500));
        }
        break;

      case AGENT_CONNECTED: {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

        uint32_t now = millis();
        // Skip agent pings if we're actively receiving cmd_vel to avoid blocking serial
        if (now - last_cmd_rx_ms < 2000) {
          last_ping_check_ms = now;
          ping_fail_count = 0;
        } else if (now - last_ping_check_ms > 3000) {
          last_ping_check_ms = now;
          if (rmw_uros_ping_agent(200, 2) == RMW_RET_OK) {
            ping_fail_count = 0;
          } else {
            ping_fail_count++;
            if (ping_fail_count >= 3) {
              agentState = AGENT_DISCONNECTED;
            }
          }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
        break;
      }

      case AGENT_DISCONNECTED:
        stopMotors();
        destroy_entities();
        agentState = WAITING_AGENT;
        vTaskDelay(pdMS_TO_TICKS(200));
        break;
    }
  }
}

// Custom transport needed for ESP32-S3 native USB CDC
bool uros_custom_transport_open(struct uxrCustomTransport * transport) {
  (void)transport;
  return true;
}

bool uros_custom_transport_close(struct uxrCustomTransport * transport) {
  (void)transport;
  return true;
}

size_t uros_custom_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err) {
  (void)transport;
  if (!Serial) {
    if (err) *err = 1;
    return 0;
  }

  size_t total_written = 0;
  uint32_t start_ms = millis();

  while (total_written < len) {
    int avail = Serial.availableForWrite();
    if (avail > 0) {
      size_t to_write = len - total_written;
      if (to_write > (size_t)avail) {
        to_write = (size_t)avail;
      }
      size_t written = Serial.write(buf + total_written, to_write);
      total_written += written;
      start_ms = millis();
    } else {
      if (millis() - start_ms > 40) {
        if (err) *err = 1;
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
  return total_written;
}

size_t uros_custom_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err) {
  (void)transport;
  if (!Serial) {
    if (err) *err = 1;
    return 0;
  }

  if (err) *err = 0;
  uint32_t start_time = millis();
  if (timeout < 0) timeout = 0;

  size_t read_bytes = 0;
  while (read_bytes < len) {
    int avail = Serial.available();
    if (avail > 0) {
      buf[read_bytes++] = (uint8_t)Serial.read();
    } else {
      if ((millis() - start_time) >= (uint32_t)timeout) {
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
  return read_bytes;
}

void setup() {
  Serial.setRxBufferSize(2048);
  Serial.setTxBufferSize(2048);
  Serial.setTxTimeoutMs(0);
  Serial.begin(115200);

  rmw_uros_set_custom_transport(
    true,
    NULL,
    uros_custom_transport_open,
    uros_custom_transport_close,
    uros_custom_transport_write,
    uros_custom_transport_read
  );

  pixels.begin();
  pixels.setBrightness(50);
  setPixelColor(200, 100, 0);

  Wire.begin();
  Wire.setTimeOut(20); // keep I2C from hanging on EMI spikes
  INA.begin();
  INA.setMaxCurrentShunt(5.0, 0.010);

  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, LOW);

  SerialTMC.begin(115200, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(100);
  setupTMC(driver_L);
  setupTMC(driver_R);

  engine.init();
  stepper_L = engine.stepperConnectToPin(L_STEP);
  if (stepper_L) {
    stepper_L->setDirectionPin(L_DIR);
    stepper_L->setAcceleration(ACCEL);
  }

  stepper_R = engine.stepperConnectToPin(R_STEP);
  if (stepper_R) {
    stepper_R->setDirectionPin(R_DIR);
    stepper_R->setAcceleration(ACCEL);
  }

  memset(&twist_msg, 0, sizeof(twist_msg));
  memset(&odom_msg, 0, sizeof(odom_msg));
  memset(&battery_msg, 0, sizeof(battery_msg));

  odom_msg.header.frame_id.data = (char *)"odom";
  odom_msg.header.frame_id.size = strlen("odom");
  odom_msg.header.frame_id.capacity = odom_msg.header.frame_id.size + 1;

  odom_msg.child_frame_id.data = (char *)"base_link";
  odom_msg.child_frame_id.size = strlen("base_link");
  odom_msg.child_frame_id.capacity = odom_msg.child_frame_id.size + 1;

  battery_msg.header.frame_id.data = (char *)"base_link";
  battery_msg.header.frame_id.size = strlen("base_link");
  battery_msg.header.frame_id.capacity = battery_msg.header.frame_id.size + 1;

  stateMutex  = xSemaphoreCreateMutex();
  cmdVelQueue = xQueueCreate(1, sizeof(TwistCommand));

  xTaskCreatePinnedToCore(microROSTask, "microROS", 12288, NULL, 2, &microROSTaskHandle, 0);
  xTaskCreatePinnedToCore(kinematicsTask, "Kinematics", 4096, NULL, 3, &kinematicsTaskHandle, 1);
  xTaskCreatePinnedToCore(hardwareTask, "HardwareMon", 4096, NULL, 1, &hardwareTaskHandle, 1);
}

void loop() {
  vTaskDelete(NULL);
}
