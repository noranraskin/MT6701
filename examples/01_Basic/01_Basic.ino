#include "MT6701.hpp"

MT6701 mt6701;

void setup()
{
    Serial.begin(115200);
    mt6701.begin();
}

void loop()
{
    Serial.print("Angle (radians): ");
    Serial.println(mt6701.getAngleRadians());
    Serial.print("Angle (degrees): ");
    Serial.println(mt6701.getAngleDegrees());
    Serial.println();
    delay(1000);
}
