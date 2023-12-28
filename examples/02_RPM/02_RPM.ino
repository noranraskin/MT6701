#include "MT6701.hpp"

// Create an MT6701 object with with very short update interval for accurate RPM measurement.
MT6701 mt6701(MT6701::DEFAULT_ADDRESS, 2);

void setup()
{
    Serial.begin(115200);
    mt6701.begin();
}

void loop()
{
    // On ESP32, the count is updated in a background task.
    // On other platforms, the count is updated here.
    // Make sure to call updateCount() at least once per revolution.
    // mt6701.updateCount();
    Serial.print("Angle (degrees): ");
    Serial.println(mt6701.getAngleDegrees());
    Serial.print("RPM: ");
    Serial.println(mt6701.getRPM());
    Serial.println();
    delay(1000);
}
