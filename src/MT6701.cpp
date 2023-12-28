#include "MT6701.hpp"

void MT6701::begin()
{
    Wire.begin();
}

float MT6701::getAngleRadians()
{
    int count = getCount();
    return count * COUNTS_TO_RADIANS;
}

float MT6701::getAngleDegrees()
{
    int count = getCount();
    return count * COUNTS_TO_DEGREES;
}

int MT6701::getCount()
{
    uint8_t angle_h, angle_l;
    Wire.beginTransmission(address);
    Wire.write(0x03); // ANGLE_H register
    Wire.endTransmission();
    Wire.requestFrom((int)address, 1);
    if (Wire.available())
        angle_h = Wire.read();

    Wire.beginTransmission(address);
    Wire.write(0x04); // ANGLE_L register
    Wire.endTransmission();
    Wire.requestFrom((int)address, 1);
    if (Wire.available())
        angle_l = Wire.read() >> 2;

    return (int)((angle_h << 6) | angle_l);
}
