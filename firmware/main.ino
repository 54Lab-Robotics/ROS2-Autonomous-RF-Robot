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

#include <AS5600.h>

#ifndef INA226_MINIMAL_SHUNT_OHM
#define INA226_MINIMAL_SHUNT_OHM (0.010)
#endif

#define NEOPIXEL_PIN 21
#define NUMPIXELS    1

#define PIN_EN 12
#define PIN_RX 3
#define PIN_TX 2

#define L_STEP 4
#define L_DIR  5
#define R_STEP 13
#define R_DIR  1

#define I2C0_SDA 7   // Wire:  Left AS5600 (0x36) + INA226 (0x40)
#define I2C0_SCL 6
#define I2C1_SDA 11  // Wire1: Right AS5600 (0x36)
#define I2C1_SCL 10

#define FAN_PIN 15

#define INVERT_L_MOTOR false
#define INVERT_R_MOTOR true

#define INVERT_L_ENCODER true
#define INVERT_R_ENCODER false

// Keep 'true' if the AS5600 magnet is mounted on the motor shaft (before the 1.777 gear reduction).
// Set to 'false' if the magnet is mounted directly on the wheel axle
#define ENCODER_ON_MOTOR_SHAFT true

#define ACCEL                  8000
#define R_SENSE                0.11f
#define MICROSTEPS             16
#define FULL_STEPS_REV         200.0f
#define MOTOR_RMS_CURRENT      500
#define GEAR_RATIO             1.777f
#define WHEEL_RADIUS           0.025f
#define LX                     0.0711f        // Half track width (wheel to center in m)
#define WHEEL_BASE             (2.0f * LX)    // Full track width

#define CMD_VEL_TIMEOUT_MS     500
#define LOW_VOLTAGE_WARN       11.5f
#define MAX_TICKS_PER_10MS     800            // Base limit per 10ms against I2C glitches
#define VEL_FILTER_ALPHA       0.70f          // Fast response for minimal phase lag in EKF
#define ODOM_PUBLISH_PERIOD_MS 20             // 20 ms = 50 Hz
#define TIME_SYNC_INTERVAL_MS  15000          // Re-synchronize clock with ROS2 host every 15 s

const float STEPS_PER_REV   = FULL_STEPS_REV * (float)MICROSTEPS * GEAR_RATIO;
const float WHEEL_CIRCUMF   = 2.0f * PI * WHEEL_RADIUS;
const float STEPS_PER_METER = STEPS_PER_REV / WHEEL_CIRCUMF;

#if ENCODER_ON_MOTOR_SHAFT
const float TICKS_TO_RAD    = (2.0f * PI) / (4096.0f * GEAR_RATIO);
#else
const float TICKS_TO_RAD    = (2.0f * PI) / 4096.0f;
#endif

Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
INA226 INA(0x40, &Wire);
HardwareSerial SerialTMC(1);

TMC2209Stepper driver_L(&SerialTMC, R_SENSE, 0);
TMC2209Stepper driver_R(&SerialTMC, R_SENSE, 1);

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper_L = NULL;
FastAccelStepper *stepper_R = NULL;

AS5600 as1(&Wire);   // Left encoder on I2C0
AS5600 as2(&Wire1);  // Right encoder on I2C1

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

QueueHandle_t cmdVelQueue;         // Queue for /cmd_vel
SemaphoreHandle_t stateMutex;      // Protects SharedRoverState
SemaphoreHandle_t i2cMutex;        // Protects Wire (INA226 and as1 share bus)

SharedRoverState roverState = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};

TaskHandle_t kinematicsTaskHandle = NULL;
TaskHandle_t urosTaskHandle       = NULL;
TaskHandle_t hardwareTaskHandle   = NULL;

rcl_publisher_t odom_pub;
rcl_publisher_t battery_pub;
rcl_subscription_t cmd_vel_sub;
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
volatile uint32_t last_comm_ok_ms = 0;
volatile uint8_t publish_fail_streak = 0;
uint8_t battery_pub_divider = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }

static inline int16_t getDeltaTicks(uint16_t current, uint16_t last) {
  int16_t delta = (int16_t)current - (int16_t)last;
  if (delta > 2048) {
    delta -= 4096;
  } else if (delta < -2048) {
    delta += 4096;
  }
  return delta;
}

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
  driver.pdn_disable(true);
  driver.I_scale_analog(false);
  driver.mstep_reg_select(true);

  driver.toff(3);
  driver.blank_time(24);
  driver.rms_current(MOTOR_RMS_CURRENT);
  driver.ihold(3);
  driver.iholddelay(10);
  driver.TPOWERDOWN(20);
  driver.microsteps(MICROSTEPS);
  driver.intpol(true);

  driver.en_spreadCycle(false);
  driver.pwm_autoscale(true);
  driver.pwm_autograd(true);

  driver.semin(0);
  driver.SGTHRS(100);
}

void setStepperVelocity(FastAccelStepper *stepper, float velocity_m_s, bool invert, int32_t &last_signed_hz) {
  if (stepper == NULL) return;
  if (invert) velocity_m_s = -velocity_m_s;

  uint32_t steps_per_sec = (uint32_t)(fabsf(velocity_m_s) * STEPS_PER_METER + 0.5f);

  if (steps_per_sec < 5) {
    if (last_signed_hz != 0) {
      stepper->stopMove();
      last_signed_hz = 0;
    }
    return;
  }

  int32_t signed_hz = (velocity_m_s > 0.0f) ? (int32_t)steps_per_sec : -(int32_t)steps_per_sec;
  if (signed_hz == last_signed_hz && stepper->isRunning()) {
    return;
  }

  last_signed_hz = signed_hz;
  stepper->setSpeedInHz(steps_per_sec);
  if (velocity_m_s > 0.0f) {
    stepper->runForward();
  } else {
    stepper->runBackward();
  }
}

void stopMotors(int32_t &last_hz_L, int32_t &last_hz_R) {
  if (stepper_L && (last_hz_L != 0 || stepper_L->isRunning())) stepper_L->stopMove();
  if (stepper_R && (last_hz_R != 0 || stepper_R->isRunning())) stepper_R->stopMove();
  last_hz_L = 0;
  last_hz_R = 0;
}

bool readBothEncoders(uint16_t &out_left, uint16_t &out_right, TickType_t timeout_ticks) {
  if (xSemaphoreTake(i2cMutex, timeout_ticks) != pdTRUE) {
    return false;
  }
  uint16_t raw_l = as1.rawAngle();
  int err_l = as1.lastError();
  xSemaphoreGive(i2cMutex);

  uint16_t raw_r = as2.rawAngle();
  int err_r = as2.lastError();

  if (err_l != AS5600_OK || err_r != AS5600_OK) {
    return false;
  }

  out_left  = raw_l & 0x0FFF;
  out_right = raw_r & 0x0FFF;
  return true;
}

void kinematicsTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10 ms = 100 Hz

  uint16_t last_ticks_left  = 0;
  uint16_t last_ticks_right = 0;

  while (!readBothEncoders(last_ticks_left, last_ticks_right, pdMS_TO_TICKS(100))) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  uint32_t last_odom_time_us = micros();

  TwistCommand latestCmd = {0.0f, 0.0f, 0};
  float local_x     = 0.0f;
  float local_y     = 0.0f;
  float local_theta = 0.0f;
  float filt_vx     = 0.0f;
  float filt_wz     = 0.0f;

  int32_t last_hz_L = 0;
  int32_t last_hz_R = 0;
  uint8_t zero_tick_streak = 0;
  uint8_t glitch_streak    = 0;

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    uint32_t now_ms = millis();

    // Read drive command
    xQueuePeek(cmdVelQueue, &latestCmd, 0);

    bool cmd_active = (agentState == AGENT_CONNECTED) &&
                      ((now_ms - latestCmd.timestamp_ms) <= CMD_VEL_TIMEOUT_MS);

    if (cmd_active) {
      float v_L = latestCmd.linear_x - (latestCmd.angular_z * LX);
      float v_R = latestCmd.linear_x + (latestCmd.angular_z * LX);

      setStepperVelocity(stepper_L, v_L, INVERT_L_MOTOR, last_hz_L);
      setStepperVelocity(stepper_R, v_R, INVERT_R_MOTOR, last_hz_R);
    } else {
      stopMotors(last_hz_L, last_hz_R);
    }

    // Read both encoders synchronously
    uint16_t current_ticks_left  = 0;
    uint16_t current_ticks_right = 0;
    if (!readBothEncoders(current_ticks_left, current_ticks_right, pdMS_TO_TICKS(3))) {
      continue;
    }

    uint32_t now_us = micros();
    float dt = (now_us - last_odom_time_us) * 1e-6f;
    if (dt <= 0.001f || dt > 0.5f) {
      dt = 0.01f;
    }

    int16_t delta_ticks_left  = getDeltaTicks(current_ticks_left, last_ticks_left);
    int16_t delta_ticks_right = getDeltaTicks(current_ticks_right, last_ticks_right);

    int16_t max_valid_ticks = (int16_t)(MAX_TICKS_PER_10MS * (dt / 0.01f));
    if (max_valid_ticks < MAX_TICKS_PER_10MS) max_valid_ticks = MAX_TICKS_PER_10MS;

    if (abs(delta_ticks_left) > max_valid_ticks || abs(delta_ticks_right) > max_valid_ticks) {
      glitch_streak++;
      if (glitch_streak >= 3) {
        last_ticks_left   = current_ticks_left;
        last_ticks_right  = current_ticks_right;
        last_odom_time_us = now_us;
        glitch_streak     = 0;
      }
      continue;
    }
    glitch_streak = 0;
    last_odom_time_us = now_us;

    bool motors_running = (stepper_L && stepper_L->isRunning()) || (stepper_R && stepper_R->isRunning());

    if (!motors_running && abs(delta_ticks_left) <= 1) {
      delta_ticks_left = 0;
    } else {
      last_ticks_left = current_ticks_left;
    }

    if (!motors_running && abs(delta_ticks_right) <= 1) {
      delta_ticks_right = 0;
    } else {
      last_ticks_right = current_ticks_right;
    }

    if (INVERT_L_ENCODER) delta_ticks_left  = -delta_ticks_left;
    if (INVERT_R_ENCODER) delta_ticks_right = -delta_ticks_right;

    // Calculate distance traveled per wheel
    float ds_left  = (float)delta_ticks_left * TICKS_TO_RAD * WHEEL_RADIUS;
    float ds_right = (float)delta_ticks_right * TICKS_TO_RAD * WHEEL_RADIUS;

    float ds     = (ds_left + ds_right) * 0.5f;
    float dtheta = (ds_right - ds_left) / WHEEL_BASE;

    float theta_mid = local_theta + (dtheta * 0.5f);
    local_x     += ds * cosf(theta_mid);
    local_y     += ds * sinf(theta_mid);
    local_theta  = atan2f(sinf(local_theta + dtheta), cosf(local_theta + dtheta));

    float raw_v     = ds / dt;
    float raw_omega = dtheta / dt;

    if (delta_ticks_left == 0 && delta_ticks_right == 0) {
      if (zero_tick_streak < 255) zero_tick_streak++;
    } else {
      zero_tick_streak = 0;
    }

    if (!motors_running && zero_tick_streak >= 2) {
      filt_vx = 0.0f;
      filt_wz = 0.0f;
    } else {
      filt_vx = (VEL_FILTER_ALPHA * raw_v)     + ((1.0f - VEL_FILTER_ALPHA) * filt_vx);
      filt_wz = (VEL_FILTER_ALPHA * raw_omega) + ((1.0f - VEL_FILTER_ALPHA) * filt_wz);
    }

    bool active_motion = motors_running || (fabsf(filt_vx) > 0.003f) || (fabsf(filt_wz) > 0.008f);

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
      roverState.x         = local_x;
      roverState.y         = local_y;
      roverState.theta     = local_theta;
      roverState.vx        = filt_vx;
      roverState.wz        = filt_wz;
      roverState.is_moving = active_motion;
      xSemaphoreGive(stateMutex);
    }
  }
}

void hardwareTask(void *pvParameters) {
  for (;;) {
    float busVoltage = 0.0f;
    float current    = 0.0f;
    bool moving      = false;
    bool i2c_ok      = false;

    // Two separate, short locks so the 100 Hz encoder task never waits
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      busVoltage = INA.getBusVoltage();
      i2c_ok = true;
      xSemaphoreGive(i2cMutex);
    }
    taskYIELD();
    if (i2c_ok && xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      current = INA.getCurrent();
      xSemaphoreGive(i2cMutex);
    }

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      if (i2c_ok) {
        roverState.voltage = busVoltage;
        roverState.current = current;
      }
      moving = roverState.is_moving;
      xSemaphoreGive(stateMutex);
    }

    if (busVoltage > 1.0f && busVoltage < LOW_VOLTAGE_WARN) {
      setPixelColor(255, 0, 0);    // Red: undervoltage
    } else if (agentState != AGENT_CONNECTED) {
      setPixelColor(200, 100, 0);  // Yellow: waiting for micro-ROS agent
    } else if (moving) {
      setPixelColor(0, 100, 255);  // Blue: rover moving
    } else {
      setPixelColor(0, 180, 0);    // Green: connected & ready
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

  last_comm_ok_ms = now;
  xQueueOverwrite(cmdVelQueue, &cmd);
}

void publishOdometryAndBattery() {
  SharedRoverState snapshot;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    snapshot = roverState;
    xSemaphoreGive(stateMutex);
  } else {
    return;
  }

  int64_t time_ns = rmw_uros_epoch_nanos();
  if (time_ns > 0) {
    odom_msg.header.stamp.sec     = (int32_t)(time_ns / 1000000000LL);
    odom_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000LL);
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
  odom_msg.pose.pose.orientation.z = sinf(snapshot.theta * 0.5f);
  odom_msg.pose.pose.orientation.w = cosf(snapshot.theta * 0.5f);

  odom_msg.twist.twist.linear.x  = snapshot.vx;
  odom_msg.twist.twist.angular.z = snapshot.wz;

  // Dynamic covariance
  if (!snapshot.is_moving) {
    odom_msg.twist.covariance[0]  = 1e-6; // vx
    odom_msg.twist.covariance[35] = 1e-6; // wz
  } else {
    odom_msg.twist.covariance[0]  = 0.002; // vx
    odom_msg.twist.covariance[35] = 0.20;  // wz
  }

  rcl_ret_t rc = rcl_publish(&odom_pub, &odom_msg, NULL);
  if (rc == RCL_RET_OK) {
    last_comm_ok_ms = millis();
    publish_fail_streak = 0;
  } else {
    if (publish_fail_streak < 255) publish_fail_streak++;
  }

  // At 50 Hz: 50 cycles = 1 second
  battery_pub_divider++;
  if (battery_pub_divider >= 50) {
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

  support     = {};
  node        = rcl_get_zero_initialized_node();
  odom_pub    = rcl_get_zero_initialized_publisher();
  battery_pub = rcl_get_zero_initialized_publisher();
  cmd_vel_sub = rcl_get_zero_initialized_subscription();
  executor    = rclc_executor_get_zero_initialized_executor();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "diff_rover_rtos_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &odom_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom_wh"));

  RCCHECK(rclc_publisher_init_best_effort(
    &battery_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), "battery_state"));

  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));

  rmw_uros_sync_session(500);
  return true;
}

void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCSOFTCHECK(rcl_publisher_fini(&odom_pub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&battery_pub, &node));
  RCSOFTCHECK(rcl_subscription_fini(&cmd_vel_sub, &node));
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));
}

void microROSTask(void *pvParameters) {
  uint32_t last_ping_check_ms = 0;
  uint32_t last_odom_pub_ms   = 0;
  uint32_t last_time_sync_ms  = 0;
  uint8_t ping_fail_count     = 0;

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
          uint32_t now        = millis();
          last_ping_check_ms  = now;
          last_odom_pub_ms    = now;
          last_time_sync_ms   = now;
          last_comm_ok_ms     = now;
          publish_fail_streak = 0;
          ping_fail_count     = 0;
        } else {
          destroy_entities();
          agentState = WAITING_AGENT;
          vTaskDelay(pdMS_TO_TICKS(500));
        }
        break;

      case AGENT_CONNECTED: {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

        uint32_t now = millis();

        // 50 Hz cycle for /odom_wh
        if ((now - last_odom_pub_ms) >= ODOM_PUBLISH_PERIOD_MS) {
          if ((now - last_odom_pub_ms) > (ODOM_PUBLISH_PERIOD_MS * 2)) {
            last_odom_pub_ms = now;
          } else {
            last_odom_pub_ms += ODOM_PUBLISH_PERIOD_MS;
          }
          publishOdometryAndBattery();
        }

        // re-synchronize time every 15s
        if ((now - last_time_sync_ms) >= TIME_SYNC_INTERVAL_MS) {
          last_time_sync_ms = now;
          rmw_uros_sync_session(20);
        }

        // Connection check only on errors or every 5 seconds
        if (publish_fail_streak >= 25 || (now - last_ping_check_ms > 5000)) {
          last_ping_check_ms = now;
          if (rmw_uros_ping_agent(100, 2) == RMW_RET_OK) {
            ping_fail_count = 0;
            publish_fail_streak = 0;
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
        // Motors are stopped automatically
        destroy_entities();
        agentState = WAITING_AGENT;
        vTaskDelay(pdMS_TO_TICKS(200));
        break;
    }
  }
}

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
      if (millis() - start_ms > 20) {
        if (err) *err = 1;
        break;
      }
      taskYIELD();
    }
  }
  if (err) *err = (total_written == len) ? 0 : 1;
  return total_written;
}

size_t uros_custom_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err) {
  (void)transport;
  if (!Serial) {
    if (err) *err = 1;
    return 0;
  }

  if (err) *err = 0;
  if (timeout < 0) timeout = 0;

  uint32_t start_us = micros();
  uint32_t timeout_us = (uint32_t)timeout * 1000UL;
  size_t read_bytes = 0;

  while (read_bytes < len) {
    int avail = Serial.available();
    if (avail > 0) {
      size_t to_read = len - read_bytes;
      if (to_read > (size_t)avail) {
        to_read = (size_t)avail;
      }
      read_bytes += Serial.read(buf + read_bytes, to_read);
    } else {
      if ((micros() - start_us) >= timeout_us) {
        break;
      }
      if (read_bytes > 0) {
        delayMicroseconds(40);
      } else {
        vTaskDelay(pdMS_TO_TICKS(1));
      }
    }
  }
  return read_bytes;
}

void setup() {
  Serial.setRxBufferSize(4096);
  Serial.setTxBufferSize(4096);
  Serial.setTxTimeoutMs(0);
  Serial.begin(921600);

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

  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, HIGH);

  Wire.begin(I2C0_SDA, I2C0_SCL, 400000);
  Wire.setTimeOut(10);

  Wire1.begin(I2C1_SDA, I2C1_SCL, 400000);
  Wire1.setTimeOut(10);

  INA.begin();
  INA.setMaxCurrentShunt(5.0, 0.010);

  as1.begin();
  as2.begin();

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

  odom_msg.pose.covariance[0]  = 0.005; // x
  odom_msg.pose.covariance[7]  = 0.005; // y
  odom_msg.pose.covariance[14] = 1e6;   // z
  odom_msg.pose.covariance[21] = 1e6;   // roll
  odom_msg.pose.covariance[28] = 1e6;   // pitch
  odom_msg.pose.covariance[35] = 0.20;  // yaw

  odom_msg.twist.covariance[0]  = 0.002; // vx
  odom_msg.twist.covariance[7]  = 1e-4;  // vy
  odom_msg.twist.covariance[14] = 1e6;   // vz
  odom_msg.twist.covariance[21] = 1e6;   // wx
  odom_msg.twist.covariance[28] = 1e6;   // wy
  odom_msg.twist.covariance[35] = 0.20;  // wz

  battery_msg.header.frame_id.data = (char *)"base_link";
  battery_msg.header.frame_id.size = strlen("base_link");
  battery_msg.header.frame_id.capacity = battery_msg.header.frame_id.size + 1;

  stateMutex  = xSemaphoreCreateMutex();
  i2cMutex    = xSemaphoreCreateMutex();
  cmdVelQueue = xQueueCreate(1, sizeof(TwistCommand));

  xTaskCreatePinnedToCore(microROSTask, "microROS", 12288, NULL, 2, &urosTaskHandle, 0);
  xTaskCreatePinnedToCore(kinematicsTask, "Kinematics", 4096, NULL, 3, &kinematicsTaskHandle, 1);
  xTaskCreatePinnedToCore(hardwareTask, "HardwareMon", 4096, NULL, 1, &hardwareTaskHandle, 1);
}

void loop() {
  vTaskDelete(NULL);
}
