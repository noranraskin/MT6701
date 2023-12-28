#pragma once
#include <atomic>
#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

class MT6701
{
public:
    static constexpr uint8_t DEFAULT_ADDRESS = 0b0000110; // I2C address of the MT6701
    static constexpr int COUNTS_PER_REVOLUTION = 16384;
    static constexpr float COUNTS_TO_RADIANS = 2.0 * PI / COUNTS_PER_REVOLUTION;
    static constexpr float COUNTS_TO_DEGREES = 360.0 / COUNTS_PER_REVOLUTION;

    MT6701(uint8_t device_address = DEFAULT_ADDRESS, int update_interval = 10);
    void begin();
    float getAngleRadians();
    float getAngleDegrees();
    int getFullTurns();
    float getTurns();
    int getAccumulator();
    void updateCount();

private:
    uint8_t address;
    int updateIntervalMillis;
    std::atomic<int> count{0};
    std::atomic<int> accumulator{0};

    int getCount();
#ifdef ESP32
    static void updateTask(void *pvParameters);
#endif
};
