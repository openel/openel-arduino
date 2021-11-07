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

#include "pid.h"

PID::PID(float point, float kp, float ki, float kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
  _point = point;
  output_max = 10000;
  output_min = -10000;
  integral_max = 10000;
  integral_min = -10000;
  error_integral = 0;
  _dir = 1;
}

float PID::Update(float input) {
  float output;
  float dinput;
  error = _point - input;
  error_integral += _ki * error + error_integral_offset;
  if(error_integral > integral_max) { error_integral = integral_max; }
  if(error_integral < integral_min) { error_integral = integral_min; }
  dinput = input - input_last;
  output = _kp * error + error_integral - _kd * dinput;
  input_last = input;
  if(output < output_min) { output = output_min; }
  if(output > output_max) { output = output_max; }
  return output;
} 

void PID::SetOutputLimits(float max_out, float min_out) {
  output_max = max_out;
  output_min = min_out;
} 

void PID::SetIntegralLimits(float max_out, float min_out) {
  integral_max = max_out;
  integral_min = min_out;
}

void PID::SetDirection(int8_t dir) {
  _dir = dir;
  _kp = fabs(_kp) * _dir;
  _ki = fabs(_ki) * _dir;
  _kd = fabs(_kd) * _dir;
}

void PID::SetIntegral(float integral) {
  error_integral = integral;
}

void PID::SetIntegralOffset(float offset) {
  error_integral_offset = offset;
}

void PID::UpdateParam(float kp, float ki, float kd) {
  _kp = fabs(kp) * _dir;
  _ki = fabs(ki) * _dir;
  _kd = fabs(kd) * _dir;
}

void PID::SetPoint(float point) {
  _point = point;
}
