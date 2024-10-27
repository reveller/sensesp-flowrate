// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

#include <memory>

#include "sensesp.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/transforms/frequency.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/led_blinker.h"
#include "sensesp_app_builder.h"

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>


using namespace sensesp;


// BME280

Adafruit_BME280 bme280;

#define SEALEVELPRESSURE_HPA (1013.25)

float read_temp_callback() {
  float bmeTemp = bme280.readTemperature();
  debugD("BME280 Temperature: %4.2fC", bmeTemp);
  debugD("BME280 Temperature: %4.2fF", 1.8 * bmeTemp + 32);
  return (bmeTemp + 273.15);
}
float read_pressure_callback() {
  float bmePressure = bme280.readPressure() / 100.0F;
  debugD("BME280 Pressure: %4.2fhPa", bmePressure);
  return (bmePressure);
}
float read_humidity_callback() {
  float bmeHumidity = bme280.readHumidity();
  debugD("BME280 Humidity: %4.2f", bmeHumidity);
  return (bmeHumidity);
}
float read_altitude_callback() {
  float bmeAltitude = bme280.readAltitude(SEALEVELPRESSURE_HPA);
  debugD("BME280 Altitude: %4.2fm", bmeAltitude);
  return (bmeAltitude);
}



// The setup function performs one-time application initialization.
void setup() {
  SetupLogging(ESP_LOG_DEBUG);

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("my-sensesp-project")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi_client("My WiFi SSID", "my_wifi_password")
                    //->set_wifi_access_point("My AP SSID", "my_ap_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->get_app();

  // Setup heartbeat LED pin
  // BaseBlinker led_blinker(led_gpio);
  // led_blinker.set_enabled(true);

  // pinMode(SENSOR, INPUT_PULLUP);

  // GPIO number to use for the analog input
  const uint8_t kAnalogInputPin = 36;
  // Define how often (in milliseconds) new samples are acquired
  const unsigned int kAnalogInputReadInterval = 1000;
  // Define the produced value at the maximum input voltage (3.3V).
  // A value of 3.3 gives output equal to the input voltage.
  const float kAnalogInputScale = 3.3;

  // Create a new Analog Input Sensor that reads an analog input pin
  // periodically.
  auto analog_input = std::make_shared<AnalogInput>(
      kAnalogInputPin, kAnalogInputReadInterval, "", kAnalogInputScale);

  // Add an observer that prints out the current value of the analog input
  // every time it changes.
  analog_input->attach([analog_input]() {
    debugD("Analog input value: %f", analog_input->get());
  });

  // Set GPIO pin 15 to output and toggle it every 650 ms

  const uint8_t led_gpio = 25;
  EvenBlinker hbLED(25, 1000);   // Enabled by default
  hbLED.set_enabled(true);

  const uint8_t kDigitalOutputPin = 15;
  const unsigned int kDigitalOutputInterval = 1000;
  pinMode(kDigitalOutputPin, OUTPUT);
  // pinMode(led_gpio, OUTPUT);
  event_loop()->onRepeat(kDigitalOutputInterval, [kDigitalOutputPin]() {
    digitalWrite(kDigitalOutputPin, !digitalRead(kDigitalOutputPin));
    // digitalWrite(led_gpio, !digitalRead(led_gpio));

  });

  // Read GPIO 14 every time it changes

  const uint8_t kDigitalInput1Pin = 14;
  auto digital_input1 = std::make_shared<DigitalInputChange>(
      kDigitalInput1Pin, INPUT_PULLUP, CHANGE);

  // Connect the digital input to a lambda consumer that prints out the
  // value every time it changes.

  // Test this yourself by connecting pin 15 to pin 14 with a jumper wire and
  // see if the value changes!

  auto digital_input1_consumer = std::make_shared<LambdaConsumer<bool>>(
      [](bool input) { debugD("Digital input value changed: %d", input); });

  digital_input1->connect_to(digital_input1_consumer);

  // Create another digital input, this time with RepeatSensor. This approach
  // can be used to connect external sensor library to SensESP!

  const uint8_t kDigitalInput2Pin = 13;
  const unsigned int kDigitalInput2Interval = 1000;

  // Configure the pin. Replace this with your custom library initialization
  // code!
  pinMode(kDigitalInput2Pin, INPUT_PULLUP);

  // Define a new RepeatSensor that reads the pin every 100 ms.
  // Replace the lambda function internals with the input routine of your custom
  // library.

  // Again, test this yourself by connecting pin 15 to pin 13 with a jumper
  // wire and see if the value changes!

  auto digital_input2 = std::make_shared<RepeatSensor<bool>>(
      kDigitalInput2Interval,
      [kDigitalInput2Pin]() { return digitalRead(kDigitalInput2Pin); });

  // Connect the analog input to Signal K output. This will publish the
  // analog input value to the Signal K server every time it changes.
  auto aiv_metadata = std::make_shared<SKMetadata>("V", "Analog input voltage");
  auto aiv_sk_output = std::make_shared<SKOutput<float>>(
      "sensors.analog_input.voltage",   // Signal K path
      "/Sensors/Analog Input/Voltage",  // configuration path, used in the
                                        // web UI and for storing the
                                        // configuration
      aiv_metadata
  );




  const char* flowrate_sk_path = "sensors.main.flowrate";
  const char* flowrate_config_path = "/sensors/flowrate";

  const char* flowrate_config_path_calibrate = "/sensors/flowrate/calibrate";
  const char* flowrate_config_path_skpath = "/sensors/flowrate/sk";

  // The Hall Effect sensor outputs approximately 4.5 pulses per second
  // per liter/min of flow
  const float flowrate_multiplier = 1.0 / 4.5;
  const unsigned int flowrate_read_delay = 1000;

  uint8_t flowrate_pin = 27;

  auto* sensor = new DigitalInputCounter(flowrate_pin, INPUT_PULLUP, FALLING, flowrate_read_delay);

  auto flowrate = new Frequency(flowrate_multiplier, flowrate_config_path);

  auto flowrate_sk_output = new SKOutput<float>(flowrate_sk_path, flowrate_config_path_skpath);

  ConfigItem(flowrate_sk_output)
      ->set_title("Frequency SK Output Path")
      ->set_description("Flow rate of the output of the pump")
      ->set_sort_order(300);

  sensor
      ->connect_to(flowrate)             // connect the output of sensor
                                          // to the input of Frequency()
      ->connect_to(flowrate_sk_output);  // connect the output of Frequency()
                                          // to a Signal K Output as a number




/// BME280 SENSOR CODE - Temp/Humidity/Altitude/Pressure Sensor ////

  // 0x77 is the default address. Some chips use 0x76, which is shown here.
  // If you need to use the TwoWire library instead of the Wire library, there
  // is a different constructor: see bmp280.h

  bme280.begin();
  // Create a RepeatSensor with float output that reads the temperature
  // using the function defined above.
  auto* bme280_temp =
      // new RepeatSensor<float>(5000, read_temp_callback);
      new RepeatSensor<float>(10000, read_temp_callback);

  auto* bme280_pressure =
      // new RepeatSensor<float>(60000, read_pressure_callback);
      new RepeatSensor<float>(10000, read_pressure_callback);

  auto* bme280_humidity =
      // new RepeatSensor<float>(60000, read_humidity_callback);
      new RepeatSensor<float>(10000, read_humidity_callback);

  auto* bme280_altitude =
      // new RepeatSensor<float>(60000, read_humidity_callback);
      new RepeatSensor<float>(10000, read_altitude_callback);

  // Send the temperature to the Signal K server as a Float
  bme280_temp->connect_to(new SKOutputFloat("environment.temperature"));

  bme280_pressure->connect_to(new SKOutputFloat("environment.pressure"));

  bme280_humidity->connect_to(new SKOutputFloat("environment.relativeHumidity"));

  bme280_altitude->connect_to(new SKOutputFloat("environment.altitude"));




  ConfigItem(aiv_sk_output)
      ->set_title("Analog Input Voltage SK Output Path")
      ->set_description("The SK path to publish the analog input voltage")
      ->set_sort_order(100);

  analog_input->connect_to(aiv_sk_output);

  // Connect digital input 2 to Signal K output.
  auto di2_metadata = std::make_shared<SKMetadata>("", "Digital input 2 value");
  auto di2_sk_output = std::make_shared<SKOutput<bool>>(
      "sensors.digital_input2.value",    // Signal K path
      "/Sensors/Digital Input 2/Value",  // configuration path
      di2_metadata
  );

  ConfigItem(di2_sk_output)
      ->set_title("Digital Input 2 SK Output Path")
      ->set_sort_order(200);

  digital_input2->connect_to(di2_sk_output);

  // To avoid garbage collecting all shared pointers created in setup(),
  // loop from here.
  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }
