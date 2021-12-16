/*
  BME680 simple subsystem test (I2C mode as primary, however SPI capable).
  For sensor calibrations and troubleshooting.

  Wiring:
  (Note: Pin labels are written the same as the board I used. Your labels may differ.)

  +-----------------------+       +-----------------------+
  |                       |       |                       |
  |     BME680 (I2C)      |       |     BME680 (SPI)      |
  |                       |       |                       |
  +_______________________+       +_______________________+
  | V | G | S | S | S |   |       | V | G | S | S | S |   |
  | C | N | C | D | D | C |       | C | N | C | D | D | C |
  | C | D | L | A | O | S |       | C | D | L | A | O | S |
  +-----------------------+       +-----------------------+
    |   |   |   |   |   |           |   |   |   |   |   |
   +5V  |   |   |   NC  NC         +5V  |   |   |   |  CS (To MCU)
       GND  |   |                      GND  |   |  SDO/ MISO (To MCU)
  (To MCU) SCL  |                 (To MCU) SCK  |
      (To MCU) SDA               (To MCU) SDI/ MOSI

  Pinouts deduced from Adafruit guide; confirmed through experiment. Source: https://learn.adafruit.com/adafruit-bme680-humidity-temperature-barometic-pressure-voc-gas?view=all#spi-logic-pins-2957115-4



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

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"


#define SEALEVELPRESSURE_HPA (1013.25)

// Preprocessor directives are used to keep sampling settings the same in setup or after recovery in loop.
#define TemperatureOS BME680_OS_8X
#define HumidityOS BME680_OS_2X
#define PressureOS BME680_OS_4X
#define IIRFS BME680_FILTER_SIZE_3
// BME680_OS_NONE  BME680_OS_1X  BME680_OS_2X  BME680_OS_4X  BME680_OS_8X  BME680_OS_16X
// BME680_FILTER_SIZE_0  BME680_FILTER_SIZE_1  BME680_FILTER_SIZE_3  BME680_FILTER_SIZE_7  BME680_FILTER_SIZE_15  BME680_FILTER_SIZE_31  BME680_FILTER_SIZE_63  BME680_FILTER_SIZE_127

#define Wait_Time 5000  // Delay at end of loop  

Adafruit_BME680 bme; // I2C

// If using SPI don't forget to define pin numbers: #define BME_SCK 13  #define BME_MISO 12  #define BME_MOSI 11  #define BME_CS 10
//Adafruit_BME680 bme(BME_CS); // Hardware SPI
//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);  // Software SPI

bool BME680_Error_Flag = false;           //
//bool BME680_Wait_Flag = false;          // Any bad data would want to be seen so wait flag is not used in this sketch.

// Normally only "BME680_Pressure" and "BME680_Humidity" are used in the WX station sketch, so they are a little different.
float    BME680_Temperature_C;
float    BME680_Temperature_C_Calibration = 0.0;
float    BME680_Temperature_F;
float    BME680_Temperature_F_Calibration = -3.7;
unsigned int BME680_Pressure;  // Different datatype for ease of use with APRS.
byte         BME680_Humidity;  // Different datatype for ease of use with APRS.
float    BME680_Pressure_Float;
float    BME680_Humidity_Float;
float    BME680_VOC;
float    BME680_VOC_Calibration = 0.0;
float    BME680_Altitude;
const int Altitude_Correction = 184;    // Altitude correction for altitude of station in tenths of millibar. General rule is 1 inHg (33.86mb) per 1000ft. So, 528' for me, this works out to 17.878mb (0.03386mb per ft.).
const int BME680_Calibration = 20;      // Estimated offset to BME sensor to make it more accurate; in tenths of millibar
// Testing altitude of station: 544ft (Corrected pressure = 544ft. * 0.03387mb/ft. = 18.42mb @ 544ft.)     https://www.engineeringtoolbox.com/barometers-elevation-compensation-d_1812.html



void setup() {
  Serial.begin(115200);
  //while (!Serial);
  Serial.println(F("\nBME680 sensor calibrations and troubleshooting."));
  Serial.print(F("MCU Programmed: "));  Serial.print(__DATE__); Serial.print(F(" @ "));   Serial.println(__TIME__);  // Date and time of MCU programming/ compile.
  Serial.print(F("From: "));  Serial.println(__FILE__);  // File path of program.
  //Serial.print(F("Free RAM:     ")); Serial.println(freeMemory());


  if (!bme.begin()) {  // If cannot start sensor, raise error flag to try and recover sensor later.
    Serial.println(F("Could not find a valid BME680 sensor, check wiring! (I2C Mode) (Setup)"));
    BME680_Error_Flag = true;
  }

  // Set up oversampling and filter initialization
  // Oversampling is used for signal noise reduction and higher sensor resolution.
  bme.setTemperatureOversampling(TemperatureOS);  // BME680_OS_NONE  BME680_OS_1X  BME680_OS_2X  BME680_OS_4X  BME680_OS_8X  BME680_OS_16X
  bme.setHumidityOversampling(HumidityOS);     //
  bme.setPressureOversampling(PressureOS);     //

  bme.setIIRFilterSize(IIRFS);    // BME680_FILTER_SIZE_0  BME680_FILTER_SIZE_1  BME680_FILTER_SIZE_3  BME680_FILTER_SIZE_7  BME680_FILTER_SIZE_15  BME680_FILTER_SIZE_31  BME680_FILTER_SIZE_63  BME680_FILTER_SIZE_127
  bme.setGasHeater(320, 150);     // 320*C for 150 ms
}

void loop() {
  if (!bme.performReading()) {   // If can't read sensor.
    Serial.println(F("Failed to perform reading :( (I2C Mode) (Loop)  Attempting recovery..."));
    BME680_Error_Flag = true;
    if (!bme.begin()) {  // Try to restart it.
      Serial.println(F("Could not recover, check wiring! (I2C Mode) (Loop)"));
      BME680_Error_Flag = true;
    }
  } else {  // If can read sensor.
    if (BME680_Error_Flag) {  // If error flag, clear it and set sampling rates for post reset.

      // Sample settings are here for post reset setup
      bme.setTemperatureOversampling(TemperatureOS);
      bme.setHumidityOversampling(HumidityOS);
      bme.setPressureOversampling(PressureOS);
      bme.setIIRFilterSize(IIRFS);
      bme.setGasHeater(320, 150); // 320*C for 150 ms

      BME680_Error_Flag = false;
    }

    // Perform the readings and calculate adjusted values.
    BME680_Temperature_C = bme.temperature + BME680_Temperature_C_Calibration;
    BME680_Temperature_F = bme.temperature * 1.8 + 32.0 + BME680_Temperature_F_Calibration;  // Deg. F is not based on "BME680_Temperature_C" due to possible rounding errors.
    BME680_Pressure_Float = bme.pressure / 100.0;
    BME680_Humidity_Float = bme.humidity;
    BME680_Pressure = round(bme.pressure / 10 + Altitude_Correction + BME680_Calibration);  // hPa (hectopascal), Millibar equivalent
    BME680_Humidity = round(bme.humidity);
    BME680_VOC = bme.gas_resistance / 1000.0 + BME680_VOC_Calibration;  // Gas resistance (KOhms)
    BME680_Altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);  // Approx. Altitude (Meters)  Since it is an approximation no calibration is used.


    // Print the readings.
    Serial.print(F("Temperature = "));
    Serial.print(BME680_Temperature_C);
    Serial.print(F(" *C, "));
    Serial.print(BME680_Temperature_F);
    Serial.println(F(" *F"));

    Serial.print(F("Pressure (int) = "));
    Serial.print(BME680_Pressure);
    Serial.print(F(" 1/10th-hPa/ 1/10th-mbar     "));
    Serial.print(F("Pressure (float [Raw]) = "));
    Serial.print(BME680_Pressure_Float);
    Serial.println(F(" hPa/ mbar"));

    Serial.print(F("Humidity (int) = "));
    Serial.print(BME680_Humidity);
    Serial.print(F("%                               "));
    Serial.print(F("Humidity (float [Raw]) = "));
    Serial.print(BME680_Humidity_Float);
    Serial.println(F("% "));

    Serial.print(F("Gas = "));
    Serial.print(BME680_VOC);
    Serial.println(F(" KOhms"));

    Serial.print(F("Approx. Altitude = "));
    Serial.print(BME680_Altitude);
    Serial.print(F("m, "));
    Serial.print(BME680_Altitude * 3.28084);  // Meters to Feet conversion is multiply by 3.28084
    Serial.print(F("ft. "));
  }
  
  Serial.println("\n\n");
  delay(Wait_Time);
}
