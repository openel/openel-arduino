/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2018-2021, Japan Embedded Systems Technology Association(JASA)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vector>
#include <iostream>

#include "EventTimer.hpp"

hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;

std::vector<TimerObserver*> observerList;

void IRAM_ATTR onTimer()
{
    // Increment the counter and set the time of ISR
    portENTER_CRITICAL_ISR(&timerMux);
    isrCounter++;
    lastIsrAt = millis();
    portEXIT_CRITICAL_ISR(&timerMux);
    // Give a semaphore that we can check in the loop
    xSemaphoreGiveFromISR(timerSemaphore, NULL);

    for(int i = 0; i < observerList.size(); i++)
    {
        observerList[i]->notify_timer();
    }
}

EventTimer::EventTimer()
{
    // Create semaphore to inform us when the timer has fired
    timerSemaphore = xSemaphoreCreateBinary();

    // Use 1st timer of 4 (counted from zero).
    // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
    // info).
    timer = timerBegin(0, 80, true);

    // Attach onTimer function to our timer.
    timerAttachInterrupt(timer, &onTimer, true);
}

EventTimer::~EventTimer()
{
    timer = NULL;
}

ReturnCode EventTimer::StartTimer()
{
    // Start an alarm
    timerAlarmEnable(timer);

    return HAL_OK;
}

ReturnCode EventTimer::StopTimer()
{
    // Stop and free timer
    timerEnd(timer);

    return HAL_OK;
}

ReturnCode EventTimer::SetEventPeriod(int32_t eventPeriod)
{
    // Set alarm to call onTimer function every second (value in microseconds).
    // Repeat the alarm (third parameter)
    timerAlarmWrite(timer, (uint64_t)eventPeriod, true);

    return HAL_OK;
}

ReturnCode EventTimer::AddObserver(TimerObserver *timerObserver)
{
    observerList.push_back(timerObserver);

    return HAL_OK;
}

ReturnCode EventTimer::RemoveObserver(TimerObserver *timerObserver)
{
    for(int i = 0; i < observerList.size(); i++)
    {
        if (observerList[i] == timerObserver)
        {
            observerList.erase(observerList.begin() + i);
        }
    }

    return HAL_OK;
}

