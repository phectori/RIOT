## About
This is a SAUL test application for the BME280 Pressure, Temperature and
Humidity sensor.

## Usage
The BME280 device is automatically initialized with the default parameters
defined in bme280_params.h. More information about the BME280 device can be
found in datasheet document BST-BME280-DS001-11.

Notice that it is necessary to first read the temperature even if only one
of the other values (humidity or pressure) is needed. This is example in
the above mentioned datasheet.

After initialization, every 2 seconds, the application:
* reads and displays the temperature (d°C);
* reads and displays the pressure (Pa);
* reads and displays the humidity (%rH);
