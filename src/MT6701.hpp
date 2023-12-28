#pragma once
#include <atomic>
#include <vector>
#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

class MT6701
{
public:
    static constexpr uint8_t DEFAULT_ADDRESS = 0b0000110; // I2C address of the MT6701
    static constexpr int UPDATE_INTERVAL = 100;           // Update interval in milliseconds
    static constexpr int COUNTS_PER_REVOLUTION = 16384;
    static constexpr float COUNTS_TO_RADIANS = 2.0 * PI / COUNTS_PER_REVOLUTION;
    static constexpr float COUNTS_TO_DEGREES = 360.0 / COUNTS_PER_REVOLUTION;
    static constexpr float SECONDS_PER_MINUTE = 60.0f;
    static constexpr int RPM_FILTER_SIZE = 10; // Size of the RPM moving average filter

    MT6701(uint8_t device_address = DEFAULT_ADDRESS,
           int update_interval = UPDATE_INTERVAL,
           int rpm_filter_size = RPM_FILTER_SIZE);
    void begin();
    float getAngleRadians();
    float getAngleDegrees();
    int getFullTurns();
    float getTurns();
    int getAccumulator();
    void updateCount();
    float getRPM();

private:
    uint8_t address;
    int updateIntervalMillis;
    unsigned long lastUpdateTime;
    std::atomic<int> count{0};
    std::atomic<int> accumulator{0};
    std::atomic<float> rpm{0};
    std::vector<float> rpmFilter;
    int rpmFilterIndex = 0;
    int rpmFilterSize;

    void updateRPMFilter(float newRPM);
    int getCount();
#ifdef ESP32
    static void updateTask(void *pvParameters);
#endif
};
