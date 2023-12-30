#pragma once
#include <atomic>
#include <vector>
#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif
#include <freertos/semphr.h>

class MT6701
{
public:
    static constexpr uint8_t DEFAULT_ADDRESS = 0b0000110; // I2C address of the MT6701
    static constexpr int UPDATE_INTERVAL = 50;            // Update interval in milliseconds
    static constexpr int COUNTS_PER_REVOLUTION = 16384;   // 14 bit encoder
    static constexpr float COUNTS_TO_RADIANS = 2.0 * PI / COUNTS_PER_REVOLUTION;
    static constexpr float COUNTS_TO_DEGREES = 360.0 / COUNTS_PER_REVOLUTION;
    static constexpr float SECONDS_PER_MINUTE = 60.0f;
    static constexpr int RPM_THRESHOLD = 1000; // RPM threshold for filtering
    static constexpr int RPM_FILTER_SIZE = 20; // Size of the RPM moving average filter

    MT6701(uint8_t device_address = DEFAULT_ADDRESS,
           int update_interval = UPDATE_INTERVAL,
           int rpm_threshold = RPM_THRESHOLD,
           int rpm_filter_size = RPM_FILTER_SIZE);
    ~MT6701();
    void begin();
    float getAngleRadians();
    float getAngleDegrees();
    int getFullTurns();
    float getTurns();
    int getAccumulator();
    int getCount();
    float getRPM();
    void updateCount();

private:
    uint8_t address;
    int updateIntervalMillis;
    std::atomic<unsigned long> lastUpdateTime{0};
    std::atomic<int> count{0};
    std::atomic<int> accumulator{0};
    std::atomic<float> rpm{0};
    std::vector<float> rpmFilter;
    int rpmFilterIndex = 0;
    int rpmFilterSize;
    int rpmThreshold;
    SemaphoreHandle_t rpmFilterMutex;

    int readCount();

    void updateRPMFilter(float newRPM);
    static void updateTask(void *pvParameters);
};
