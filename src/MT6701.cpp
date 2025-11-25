#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
// Prevent automated include reordering: FreeRTOS headers must remain above this include
// clang-format off
#include "MT6701.hpp"
// clang-format on

/**
 * @brief Constructs an MT6701 encoder object.
 *
 * @param device_address I2C address of the MT6701.
 * @param update_interval Interval in milliseconds at which to update the
 * encoder count.
 * @param rpm_filter_size Size of the RPM moving average filter.
 */
MT6701::MT6701(uint8_t device_address, int update_interval, int rpm_threshold,
               int rpm_filter_size)
    : address(device_address), updateIntervalMillis(update_interval),
      rpmThreshold(rpm_threshold), rpmFilterSize(rpm_filter_size)
{
    rpmFilterMutex = xSemaphoreCreateMutex();
}

MT6701::~MT6701()
{
    if (rpmFilterMutex)
    {
        vSemaphoreDelete(rpmFilterMutex);
    }
}

/**
 * @brief Initialises the MT6701 encoder.
 * @note This function must be called before any other MT6701 functions.
 */
void MT6701::begin(TwoWire *wire)
{
    if (wire == NULL)
    {
        this->i2c = &Wire;
        this->i2c->begin();
        // this->i2c->setClock(400000);
    }
    else
    {
        this->i2c = wire;
    }
    xTaskCreatePinnedToCore(updateTask, "MT6701 update task", 2048, this, 2, NULL,
                            portNUM_PROCESSORS - 1);
    xSemaphoreTake(rpmFilterMutex, portMAX_DELAY);
    rpmFilter.resize(rpmFilterSize);
    xSemaphoreGive(rpmFilterMutex);
}

/**
 * @brief Returns the shaft angle of the encoder in radians.
 *
 * @return Angle in radians withing the range [0, 2*PI).
 */
float MT6701::getAngleRadians() { return count * COUNTS_TO_RADIANS; }

/**
 * @brief Returns the shaft angle of the encoder in degrees.
 *
 * @return Angle in degrees withing the range [0, 360).
 */
float MT6701::getAngleDegrees() { return float(count) * COUNTS_TO_DEGREES; }

/**
 * @brief Returns the accumulated number of full turns of the encoder shaft
 * since initialisation.
 *
 * @return Number of full turns.
 */
int MT6701::getFullTurns() { return accumulator / COUNTS_PER_REVOLUTION; }

/**
 * @brief Returns the accumulated number of turns of the encoder shaft since
 * initialisation as a float.
 *
 * @return Number of turns.
 */
float MT6701::getTurns()
{
    return (float)accumulator / float(COUNTS_PER_REVOLUTION);
}

/**
 * @brief Returns the accumulated count the encoder has generated since
 * initialisation.
 *
 * @return Raw accumulator value.
 */
int MT6701::getAccumulator() { return accumulator; }

/**
 * @brief Returns the current RPM of the encoder shaft averaged over
 * 'rpmFilterSize' samples.
 *
 * @return RPM.
 */
float MT6701::getRPM()
{
    xSemaphoreTake(rpmFilterMutex, portMAX_DELAY);
    float sum = 0;
    for (float value : rpmFilter)
    {
        sum += value;
    }
    float rpm = sum / rpmFilter.size();
    xSemaphoreGive(rpmFilterMutex);
    return rpm;
}

/**
 * @brief Returns the current encoder count.
 *
 * @return Raw count value.
 */
int MT6701::getCount() { return count; }

/**
 * @brief Updates the encoder count.
 * @note This function is called automatically at regular intervals if hardware
 * permits.
 */
void MT6701::updateCount()
{
    int newCount = readCount();
    // give it four tries
    for (int i = 0; i < 3 && newCount < 0; i++, newCount = readCount())
        ;
    if (newCount < 0)
    {
        return;
    }
    int diff = newCount - count;
    if (diff > COUNTS_PER_REVOLUTION / 2)
    {
        diff -= COUNTS_PER_REVOLUTION;
    }
    else if (diff < -COUNTS_PER_REVOLUTION / 2)
    {
        diff += COUNTS_PER_REVOLUTION;
    }
    unsigned long currentTime = millis();
    unsigned long timeElapsed = currentTime - lastUpdateTime;
    if (timeElapsed > 0)
    {
        // Calculate RPM
        rpm = (diff / (float)COUNTS_PER_REVOLUTION) *
              (SECONDS_PER_MINUTE * 1000 / (float)timeElapsed);
        if (abs(rpm) < rpmThreshold)
        {
            updateRPMFilter(rpm);
        }
    }
    accumulator += diff;
    count = newCount;
    lastUpdateTime = currentTime;
}

void MT6701::updateRPMFilter(float newRPM)
{
    xSemaphoreTake(rpmFilterMutex, portMAX_DELAY);
    rpmFilter[rpmFilterIndex] = newRPM;
    rpmFilterIndex = (rpmFilterIndex + 1) % RPM_FILTER_SIZE;
    xSemaphoreGive(rpmFilterMutex);
}

int MT6701::readCount()
{
    uint8_t data[2];
    i2c->beginTransmission(address);
    i2c->write(0x03);                  // Starting register ANGLE_H
    i2c->endTransmission(false);       // End transmission, but keep the I2C bus active
    i2c->requestFrom((int)address, 2); // Request two bytes
    unsigned long startTime = millis();
    if (i2c->available() < 2)
    {
        return -1;
    }
    int angle_h = i2c->read();
    int angle_l = i2c->read();

    return (angle_h << 6) | angle_l; // returns value from 0 to 16383
}

void MT6701::updateTask(void *pvParameters)
{
    MT6701 *mt6701 = static_cast<MT6701 *>(pvParameters);
    TickType_t delay = pdMS_TO_TICKS(mt6701->updateIntervalMillis);
    while (true)
    {
        mt6701->updateCount();
        vTaskDelay(delay);
        //     unsigned int delayMS = mt6701->lastUpdateTime +
        //     mt6701->updateIntervalMillis - millis(); if (delayMS > 0)
        //     {
        //         TickType_t delay = pdMS_TO_TICKS(delayMS);
        //         vTaskDelay(delay);
        //     }
    }
}