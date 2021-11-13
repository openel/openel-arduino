/*
 * MIT License
 *
 * Copyright (c) 2017 M5Stack
 * Copyright (c) 2021 JASA(Japan Embedded Systems Technology Association)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#define M5STACK_MPU6886

#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>

#include <M5Stack.h>
#include <Preferences.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "calibration.h"
#include "imu_filter.h"
#include "MadgwickAHRS.h"
#include "pid.h"
#include "openEL.hpp"
#include "Actuator.hpp"
#include "Sensor.hpp"

#define BLYNK_PRINT Serial
#define BLYNK_USE_DIRECT_CONNECT
#define AUTH "My Blynk Auth Token" // Your Blynk Auth Token

Actuator *motor_l = 0;
Actuator *motor_r = 0;
Sensor *IMU = 0;
Sensor *BMM = 0;
Property imu_property;
ReturnCode ret;

char auth[] = AUTH;
float value;
float value_list[2048];
int32_t num;
uint32_t tim;

extern uint8_t openel_img[8455];

static float angle_point = -1.5;

static int16_t pwm_offset = 0;
static int16_t left_offset = 0;
static int16_t right_offset = 0;

float kp = 24.0f, ki = 0.0f, kd = 90.0f;
float s_kp = 15.0f, s_ki = 0.075f, s_kd = 0.0f;

PID pid(angle_point, kp, ki, kd);
PID speed_pid(0, s_kp, s_ki, s_kd);

// the setup routine runs once when M5Stack starts up
void setup() {  
  M5.begin(true, false, true, true);
  Blynk.setDeviceName("Blynk_bala2");
  Blynk.begin(auth);

  HALId halid1, halid2, halid3, halid4;
  halid1.deviceKindId = 0x1;
  halid1.vendorId = 0xA;
  halid1.productId = 0x1;
  halid1.instanceId = 0x1;

  halid2.deviceKindId = 0x1;
  halid2.vendorId = 0xA;
  halid2.productId = 0x1;
  halid2.instanceId = 0x2;

  halid3.deviceKindId = 0x2;
  halid3.vendorId = 0xA;
  halid3.productId = 0x1;
  halid3.instanceId = 0x1;

  halid4.deviceKindId = 0xD;
  halid4.vendorId = 0xA;
  halid4.productId = 0x1;
  halid4.instanceId = 0x1;

  motor_l = new Actuator(halid1);
  motor_r = new Actuator(halid2);
  IMU = new Sensor(halid3);
  BMM = new Sensor(halid4);

  M5.Lcd.clear();

  int16_t x_offset, y_offset, z_offset;
  float angle_center;
  calibrationInit();

  calibrationGet(&x_offset, &y_offset, &z_offset, &angle_center);

  angle_point = angle_center;
  pid.SetPoint(angle_point);

  SemaphoreHandle_t i2c_mutex;
  i2c_mutex = xSemaphoreCreateMutex();

  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  ret = motor_l->Init();
  ret = motor_r->Init();
  ret = IMU->Init();
  ret = BMM->Init();
  xSemaphoreGive(i2c_mutex);

  ImuTaskStart(x_offset, y_offset, z_offset, &i2c_mutex);
  xTaskCreatePinnedToCore(PIDTask, "pid_task", 4 * 1024, &i2c_mutex, 4, NULL, 1);

  M5.Lcd.drawJpg(openel_img, 8455);
}

// the loop routine runs over and over again forever
void loop() {
  static uint32_t next_show_time = 0;

  Blynk.run();
  vTaskDelay(pdMS_TO_TICKS(10));

  if(millis() > next_show_time) {
    draw_waveform();
    next_show_time = millis() + 10;
  }
  M5.update();
}

// Blynk Virtual PIN Data
BLYNK_WRITE(V0) {
  int x = param[0].asInt();
  int y = param[1].asInt();
  bool flag;

  if(y > 0) {
    move(200);
    flag = true;
  } else if (y < 0) {
    move(-200);
    flag = true;
  } else {
    move(0);
    flag = false;
  }

  if (x > 0) {
    if (flag == true) {
      turn(-100);
    } else {
      rotate(-200);
    }
  } else if (x < 0) {
    if (flag == true) {
      turn(100);
    } else {
      rotate(200);
    }
  } else {
    rotate(0);
  }
}

static void PIDTask(void *arg) {
  float bala_angle;
  float motor_speed = 0;
  float motor_encoder_l;
  float motor_encoder_r;

  int16_t pwm_speed;
  int16_t pwm_output_l;
  int16_t pwm_output_r;
  int16_t pwm_angle;

  int32_t encoder = 0;
  int32_t last_encoder = 0;
  uint32_t last_ticks = 0;

  SemaphoreHandle_t i2c_lock = *(SemaphoreHandle_t*)arg;

  pid.SetOutputLimits(1023, -1023);
  pid.SetDirection(-1);

  speed_pid.SetIntegralLimits(40, -40);
  speed_pid.SetOutputLimits(1023, -1023);
  speed_pid.SetDirection(1);

  for(;;) {
    // in imu task update, update freq is 200HZ
    bala_angle = getAngle();

    // Get motor encoder value
    xSemaphoreTake(i2c_lock, portMAX_DELAY);
    motor_l->GetValue(HAL_REQUEST_POSITION_CONTROL, &motor_encoder_l);
    motor_r->GetValue(HAL_REQUEST_POSITION_CONTROL, &motor_encoder_r);
    xSemaphoreGive(i2c_lock);

    encoder = (int32_t)(motor_encoder_l + motor_encoder_r);
    // motor_speed filter
    motor_speed = 0.8 * motor_speed + 0.2 * (encoder - last_encoder);
    last_encoder = encoder;

    if(fabs(bala_angle) < 70) {
      pwm_angle = (int16_t)pid.Update(bala_angle);
      pwm_speed = (int16_t)speed_pid.Update(motor_speed);
      pwm_output_l = pwm_speed + pwm_angle + pwm_offset + left_offset;
      if(pwm_output_l > 1023) { pwm_output_l = 1023; }
      if(pwm_output_l < -1023) { pwm_output_l = -1023; }
      pwm_output_r = pwm_speed + pwm_angle + pwm_offset + right_offset;
      if(pwm_output_r > 1023) { pwm_output_r = 1023; }
      if(pwm_output_r < -1023) { pwm_output_r = -1023; }
      xSemaphoreTake(i2c_lock, portMAX_DELAY);
      motor_l->SetValue(HAL_REQUEST_VELOCITY_CONTROL, pwm_output_l);
      motor_r->SetValue(HAL_REQUEST_VELOCITY_CONTROL, pwm_output_r);
      xSemaphoreGive(i2c_lock);
    } else {
      pwm_angle = 0;
      xSemaphoreTake(i2c_lock, portMAX_DELAY);
      motor_l->SetValue(HAL_REQUEST_POSITION_CONTROL, 0);
      motor_r->SetValue(HAL_REQUEST_POSITION_CONTROL, 0);
      xSemaphoreGive(i2c_lock);
      xSemaphoreTake(i2c_lock, portMAX_DELAY);
      motor_l->SetValue(HAL_REQUEST_VELOCITY_CONTROL, 0);
      motor_r->SetValue(HAL_REQUEST_VELOCITY_CONTROL, 0);
      xSemaphoreGive(i2c_lock);
      speed_pid.SetIntegral(0);
    }
    vTaskDelayUntil(&last_ticks, pdMS_TO_TICKS(10));
  }
}

static void draw_waveform() {
  #define MAX_LEN 120
  #define X_OFFSET 100
  #define Y_OFFSET 95
  #define X_SCALE 3
  static int16_t val_buf[MAX_LEN] = {0};
  static int16_t pt = MAX_LEN - 1;
  val_buf[pt] = constrain((int16_t)(getAngle() * X_SCALE), -50, 50);

  if (--pt < 0) {
    pt = MAX_LEN - 1;
  }

  for (int i = 1; i < (MAX_LEN); i++) {
    uint16_t now_pt = (pt + i) % (MAX_LEN);
    M5.Lcd.drawLine(i + X_OFFSET, val_buf[(now_pt + 1) % MAX_LEN] + Y_OFFSET, i + 1 + X_OFFSET, val_buf[(now_pt + 2) % MAX_LEN] + Y_OFFSET, TFT_BLACK);
    if (i < MAX_LEN - 1) {
      M5.Lcd.drawLine(i + X_OFFSET, val_buf[now_pt] + Y_OFFSET, i + 1 + X_OFFSET, val_buf[(now_pt + 1) % MAX_LEN] + Y_OFFSET, TFT_GREEN);
    }
  }
}

void move(int16_t speed) {
  pwm_offset = speed;
}

void stop() {
  pwm_offset = 0;
  left_offset = 0;
  right_offset = 0;
}

void turn(int16_t speed) {
  if (speed > 0) {
    left_offset = speed;
    right_offset = 0;
  } else if (speed < 0) {
    left_offset = 0;
    right_offset = -speed;
  }
}

void rotate(int16_t speed) {
  left_offset = speed;
  right_offset = -speed;
}
