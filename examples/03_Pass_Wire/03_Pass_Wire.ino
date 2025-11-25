#include <MT6701.hpp>
#include <Wire.h>

// Pass a TwoWire reference to the MT6701 library.
// This allows you to use a different I2C bus, change the bus frequency, or share the same bus with multiple sensors.

MT6701 mt6701;

void setup()
{
  Serial.begin(115200);

  // You can specify SDA and SCL pins here if needed, e.g., Wire.begin(21, 22);
  Wire.begin();
  Wire.setClock(400000);

  // Pass the initialized Wire object to the MT6701 begin method
  mt6701.begin(&Wire);
}

void loop()
{
  float angle = mt6701.getAngleDegrees();
  Serial.print("Angle: ");
  Serial.println(angle);

  delay(100);
}
