/*
  DS18B20s and their isolation transistors simple subsystem test.
  For sensor calibrations and troubleshooting.

                                                1284P
                                              +---\/---+
     STBY LED (Yellow)           (D 0)  PB0  1|        |40  PA0  (ADC 0 / D24)     RX Audio Input
     Error LED (Orange)          (D 1)  PB1  2|        |39  PA1  (ADC 1 / D25)     NC
     1-Wire (DQ)           INT 2 (D 2)  PB2  3|        |38  PA2  (ADC 2 / D26)     NC
     NC                      PWM (D 3)  PB3  4|        |37  PA3  (ADC 3 / D27)     NC
     TX LED (Red)         PWM/SS (D 4)  PB4  5|        |36  PA4  (ADC 4 / D28)     NC
     RX LED (Green)         MOSI (D 5)  PB5  6|        |35  PA5  (ADC 5 / D29)     NC
     NC                 PWM/MISO (D 6)  PB6  7|        |34  PA6  (ADC 6 / D30)     NC
     NC                  PWM/SCK (D 7)  PB7  8|        |33  PA7  (ADC 7 / D31)     NC
     [To Prgmr]                         RST  9|        |32  AREF
                                        VCC 10|        |31  GND
                                        GND 11|        |30  AVCC
                                      XTAL2 12|        |29  PC7  (D 23)            Sensor 3 Negative-side Control
                                      XTAL1 13|        |28  PC6  (D 22)            Sensor 3 Positive-side Control
     [To Prgmr]              RX0 (D 8)  PD0 14|        |27  PC5  (D 21) TDI        Sensor 2 Negative-side Control
     [To Prgmr]              RX1 (D 9)  PD1 15|        |26  PC4  (D 20) TDO        Sensor 2 Positive-side Control
     Anemometer-Speed  RX1/INT0 (D 10)  PD2 16|        |25  PC3  (D 19) TMS        Sensor 1 Negative-side Control
     PTT               TX1/INT1 (D 11)  PD3 17|        |24  PC2  (D 18) TCK        Sensor 1 Positive-side Control
     DAC 8.2K               PWM (D 12)  PD4 18|        |23  PC1  (D 17) SDA        [To I2C Devices]
     DAC 3.9K               PWM (D 13)  PD5 19|        |22  PC0  (D 16) SCL        [To I2C Devices]
     DAC 2.2K               PWM (D 14)  PD6 20|        |21  PD7  (D 15) PWM        DAC 1K
                                              +--------+

*/

#include <DallasTemperature.h>  // For DS18B20s
#include <OneWire.h>            // For DS18B20s

#define ONE_WIRE_BUS 2        // Pin 2 is the 1-Wire bus. Don't forget to put a 4.7K pullup on it.

OneWire oneWire(ONE_WIRE_BUS);        // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);  // Pass oneWire reference to Dallas Temperature.

float DS18B20_Air_Temperature, DS18B20_Air_Temperature_C, DS18B20_Air_Calibration = 5.0;              // Let the sensors be read for a few loops before doing calculations.
float DS18B20_Ground_Temperature, DS18B20_Ground_Temperature_C, DS18B20_Ground_Calibration = 5.0;     // *C readings have no calibrations at this time since APRS uses *F.
float DS18B20_Road_Temperature, DS18B20_Road_Temperature_C, DS18B20_Road_Calibration = 4.0;          


void setup() {

  Serial.begin(115200);  // For debug.
   
  int inMin = 18; // Lowest output pin
  int inMax = 23; // Highest output pin
  for (int i = inMin; i <= inMax; i++)  {  // 18- Air sensor positive side control, 19- Air sensor negative side control, 20- Ground sensor positive side control, 21- Ground sensor negative side control, 22- Road sensor positive side control, 23- Road sensor negative side control 
    pinMode(i, OUTPUT);       // Set the pins as output and then default them to off based on pin number:
    digitalWrite(i, !i % 2);  // Odd number pins are N-Ch. MOSFETS, active high. Even number pins are P-Ch. MOSFETs, active low.  Since anything other than 0 is true/ HIGH, invert that to get correct polarity (any odd number modulo by two will give a non-zero remainder). 
  }

    sensors.begin();  // Start library

  Serial.print(F("MCU Programmed: "));  Serial.print(__DATE__); Serial.print(F(" @ "));   Serial.println(__TIME__);  // Date and time of MCU programming/ compile.
  Serial.print(F("From: "));  Serial.println(__FILE__);  // File path of program.

}

void loop() {
  
    DS18B20_Air_Switch(1);  // Turn on sensor
    delay(5);  // Wait for power to stabilize
    sensors.requestTemperatures(); // Send the command to get temperature
    float tempC = sensors.getTempCByIndex(0);  // Get Temperature.

    if (tempC != DEVICE_DISCONNECTED_C)   {  // Check if reading was successful ("DEVICE_DISCONNECTED_C" = -127)
      // Reading was successful
      DS18B20_Air_Temperature = tempC * 1.8 + 32 + DS18B20_Air_Calibration;  // Save temperature data (in F)
      DS18B20_Air_Temperature_C = tempC;
    }
    DS18B20_Air_Switch(0);  // Turn off sensor
    DS18B20_Ground_Switch(1);  // Turn on sensor
    delay(5);
    sensors.requestTemperatures(); // Send the command to get temperatures
    tempC = sensors.getTempCByIndex(0);  // After we got the temperatures, we can print them here. We use the function ByIndex, and as an example get the temperature from the first sensor only.

    if (tempC != DEVICE_DISCONNECTED_C)   {  // Check if reading was successful ("DEVICE_DISCONNECTED_C" = -127)
      // Reading was successful
      DS18B20_Ground_Temperature = tempC * 1.8 + 32 + DS18B20_Ground_Calibration;  // Save temperature data (in F)
      DS18B20_Ground_Temperature_C = tempC;
    }
    DS18B20_Ground_Switch(0);  // Turn off sensor
    DS18B20_Road_Switch(1);  // Turn on sensor
    delay(5);
    sensors.requestTemperatures(); // Send the command to get temperatures
    tempC = sensors.getTempCByIndex(0);  // After we got the temperatures, we can print them here. We use the function ByIndex, and as an example get the temperature from the first sensor only.

    if (tempC != DEVICE_DISCONNECTED_C)   {  // Check if reading was successful ("DEVICE_DISCONNECTED_C" = -127)
      // Reading was successful
      DS18B20_Road_Temperature = tempC * 1.8 + 32 + DS18B20_Road_Calibration;  // Save temperature data (in F)
      DS18B20_Road_Temperature_C = tempC;
    }
    DS18B20_Road_Switch(0);  // Turn off sensor



Serial.print(F("Air temperature: "));
Serial.print(DS18B20_Air_Temperature);
Serial.print(F("*F/ "));
Serial.print(DS18B20_Air_Temperature_C);
Serial.print(F("*C -- Offset:"));
Serial.print(DS18B20_Air_Calibration);
Serial.print(F("F     "));

Serial.print(F("Ground temperature: "));
Serial.print(DS18B20_Ground_Temperature);
Serial.print(F("*F/ "));
Serial.print(DS18B20_Ground_Temperature_C);
Serial.print(F("*C -- Offset:"));
Serial.print(DS18B20_Ground_Calibration);
Serial.print(F("F     "));

Serial.print(F("Road temperature: "));
Serial.print(DS18B20_Road_Temperature);
Serial.print(F("*F/ "));
Serial.print(DS18B20_Road_Temperature_C);
Serial.print(F("*C -- Offset:"));
Serial.print(DS18B20_Road_Calibration);
Serial.println(F("F\n"));

delay(10000);

}

void DS18B20_Air_Switch(bool x) {
    digitalWrite(18, !x);  // Positive side
    digitalWrite(19, x);   // Negative side
}

void DS18B20_Ground_Switch(bool x) {
    digitalWrite(20, !x);  // Positive side
    digitalWrite(21, x);   // Negative side
}

void DS18B20_Road_Switch(bool x) {
    digitalWrite(22, !x);  // Positive side
    digitalWrite(23, x);   // Negative side
}
