# MT6701 Magnetic Rotary Encoder Library for Arduino and ESP32

The MT6701 library provides a simple and effective way to interface with the MT6701 magnetic rotary encoder using Arduino and ESP32. It supports reading angular positions in radians and degrees, calculating rotational velocity in RPM, and smoothing RPM values with a moving average filter. The sensor uses I2C.

## Features

- Read angular position in radians and degrees.
- Calculate rotational velocity (RPM).
- Smooth RPM readings with a moving average filter.
- Easy-to-use interface for Arduino and ESP32 environments.

## Installation

### Manual Installation
1. Download this library as a ZIP file.
2. Open the Arduino IDE.
3. Go to Sketch > Include Library > Add .ZIP Library....
4. Choose the downloaded ZIP file.
5. Restart the Arduino IDE.

### Library Manager
1. Open the Arduino IDE.
2. Go to Sketch > Include Library > Manage Libraries....
3. Search for "MT6701".

## Usage

To use the library, include it in your Arduino sketch and follow the steps below.

### Basic Setup
```cpp
#include "MT6701.hpp"

MT6701 encoder;

void setup() {
    Serial.begin(115200);
    encoder.begin();
}

void loop() {
    float angleRadians = encoder.getAngleRadians();
    Serial.print("Angle in Radians: ");
    Serial.println(angleRadians);

    delay(1000);
}
```

### Advanced Usage

**RPM Calculation**

The library can calculate the RPM of the rotary encoder. It works best with a short update interval, by using a low update interval for the constructor for ESP32 or manually calling `encoder.updateCount()` frequently for Arduino. Here's how you can get the RPM value:

```cpp
void loop() {
    float rpm = encoder.getRPM();
    Serial.print("RPM: ");
    Serial.println(rpm);

    delay(1000);
}
```

**Changing Update Interval**

You can modify the update interval of the encoder readings (default is 100 ms) by passing the interval in milliseconds to the constructor:

```cpp
MT6701 encoder(MT6701::DEFAULT_ADDRESS, 200); // Update interval set to 200 ms
```

**Changing moving average filter size**

You can modify the number of samples to average for the moving average filter (default is 10) by passing the filter size to the constructor:

```cpp
MT6701 encoder(MT6701::DEFAULT_ADDRESS, MT6701::UPDATE_INTERVAL, 20); //  Filter size set to 20
```

## API Reference

**Methods:**
- `begin()`: Initialize the encoder and start the background task for reading data.
- `getAngleRadians()`: Get the current angle in radians.
- `getAngleDegrees()`: Get the current angle in degrees.
- `getAccumulator()`: Get the accumulated count.
- `getRPM()`: Get the current RPM.
- `getCount()`: Get the raw count value from the sensor.
- `updateCount()`: Update the count value from the sensor. This is only needed for Arduino environments.
- `getTurns()`: Get the number of turns the shaft has made since startup.
- `getFullTurns()`: Get the number of full turns the shaft has made since startup.

**Constants:**
- `DEFAULT_ADDRESS`: Default I2C address of the MT6701 encoder.
- `RPM_FILTER_SIZE`: Number of samples to average for rpm calculation.

## Example Sketches

Example sketches demonstrating basic and advanced usage of the library are included in the /examples folder.

## License

This library is released under the [GPL-3 License](LICENSE).