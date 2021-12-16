/*
  Davis Anemometer subsystem test using the ADS1015.
  For sensor calibrations and troubleshooting.

  Wiring:

  +------------------------+     +---------------------------------------+
  |                        |     |                                       |
  | Davis Anemometer RJ-11 |     |       ADS1015 (I2C) (Ext. ADC)        |
  |                        |     |                                       |
  +________________________+     |                                       |
  | 1 | 2 | 3 | 4 | 5 | 6 |      +_______________________________________+
  |   | V | D | G | S |   |      |   |   |   |   | A | A |   |   |   |   |
  | N | C | I | N | P | N |      | v | G | S | S | D | L |   |   |   |   |
  | C | C | R | D | D | C |      | C | N | C | D | D | R | A | A | A | A |
  +-----------------------+      | C | D | L | A | R | T | 0 | 1 | 2 | 3 |
        |   |   |   |            +---------------------------------------+
       +5V  |   |   |              |   |   |   |   |   |   |   |   |   |
     (ADC An_WD)|   |      +5V    +5V  |   |   |  GND  |   |   |  VCC  |
               GND  |       |         GND  |   |       NC  |   |      BATT
                    +-\/\/\-+    (To MCU) SCL  |         An_WD |
                    |                (To MCU) SDA             SSV
                  (PB2)


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

#include "ADS1X15.h"            // This library was used instead of the adafruit library, because this library natively supports connection checks. (ADS1015 12-bit ADC)
#include <math.h>               // This library helps with rounding and other math functions.

#define Wind_Sensor_Pin (10)  // The pin location of the anemometer sensor (Be interrupt pin)
#define Wind_Vane_Pin (0)     // On ADS1015 ADC. The pin the wind vane sensor is connected to. e.g. 'A0' is 'Wind_Vane_Pin (0)'.
#define Vane_Offset 0;        // Anemometer offset from magnetic north

ADS1015 ADS(0x48);       // I2C address 0x48  // 12-bit  (For 16-bit version, carefully go throught the code to change any magic numbers.)

const unsigned long interval = 1000;     // Minimum interval at which to check if need update (Milliseconds) (1000 = 1 Second).
unsigned long previousMillis = 0;        // Will store time of last time of needing to increment Timer_Count every interval.
volatile unsigned long Timer_Count = 0;  // Accumulated time/ clock used to determine when to perform certain actions.

byte Anemometer_Refresh_Interval = 15;       // Update/ check anemometer (Default: 15 seconds).
unsigned long Last_Anemometer_Refresh;       // Time of last anemometer update
int Standard_Telemetry_Interval = 300;       // Standard telemetry (a.k.a. minimal data sent) (Default: 300 sec. [5 min.]).
unsigned long Last_Telemetry_Update;

bool ADS1015_Error_Flag = false;          // Flag for any error involving the ADS1015.
bool Anemometer_Warning_Flag = false;     // Is actually a warning. There is no easy way to tell if the anemometer does not work. e.g. Not connected can give floating values or all 0's for wind speed and direction, but those all could still be valid readings.

volatile bool Is_Sample_Required = false;   // This is set true during Timer_Count check. Flag to get wind speed.
volatile unsigned long Rotations = 0;       // Cup rotation counter used in ISR used for wind speed.
volatile unsigned long Contact_Bounce_Time; // Timer to avoid contact bounce in ISR.
int Vane_Value;        // Raw analog value from wind vane.
int Direction;         // Translated 0 - 360 direction.
int Last_Value = 0;    // Last wind direction value.
int Wind_Direction;  // Variable for wind direction to be used in telemetry block.
float Wind_Speed;    // Adjusted wind speed from the reading function (In MPH).
float Average_Wind_Speed;      // Value to be transmitted, derived from accumulated wind speed readings divided by number of wind speed readings.
float Highest_Gust;            // Record highest gust since last update.

byte  MCU_Bus_Voltage_Pin = 3;             // On ADS1015 ADC.  This is MCU bus voltage monitor. Used for calculating wind direction.
float MCU_Bus_Voltage_Calibration = 0.07;  // Final adjustment to get accurate reading
int Raw_MCU_Bus_Voltage_Reading = 0;       // Raw bit data from ADC (ADS1015) for Wind direction reference.



void setup() {
  Serial.begin(115200);

  if (!ADS.begin()) {  // Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");
    Serial.println(F("Failed to initialize ADS1015 (Setup)"));
    ADS1015_Error_Flag = true;
  }
  
  ADS.setGain(0);  // PGA gain set to +/- 6.144v  // If changed, update setup print out below.

  pinMode(Wind_Sensor_Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(Wind_Sensor_Pin), isr_rotation, FALLING);  // Pin to attach anemometer reed switch pin.

  Serial.print(F("MCU Programmed: "));  Serial.print(__DATE__); Serial.print(F(" @ "));   Serial.println(__TIME__);  // Date and time of MCU programming/ compile.
  Serial.print(F("From: "));  Serial.println(__FILE__);  // File path of program.
  Serial.print("\nDavis Anemometer Subsystem Test with ADS1015     ");    Serial.println("ADS PGA gain set to 0 (+/- 6.144v)\n"); 
  Serial.print("Anemometer sampling and preliminary data about every: ");  Serial.print(Anemometer_Refresh_Interval);  Serial.println(" Seconds.");
  Serial.print("Anemometer data products about every: ");  Serial.print(Standard_Telemetry_Interval / 60);  Serial.println(" Minutes."); 
}

void loop() {

  // ----------------------------------------------------------------------------------- Timer Block
  unsigned long currentMillis = millis();  // Create timer that counts in seconds.
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // save the last time of update
    Timer_Count = currentMillis / 1000;
  }

  // -------------------------------------------------------------------------------------------------- Anemometer Interval Data Calculations Block
  if (Is_Sample_Required) {  // If time, calculate data results for refresh interval (Default: 15s).
    // convert to mp/h using the formula V=P(2.25/T)  V = P(2.25/2.5) = P * 0.9
    Wind_Speed = Rotations * (2.25 / Anemometer_Refresh_Interval);  // 2.25 / 15 Seconds
    Rotations = 0; // Reset count for next sample

    if (Wind_Speed > Highest_Gust) {
      Highest_Gust = Wind_Speed;
    }

    Average_Wind_Speed += Wind_Speed;

    getWindDirection();
    
    if (!ADS1015_Error_Flag) { 
      
    Serial.print(F("Preliminary data: Wind Speed: "));  Serial.print(Wind_Speed); 
    Serial.print(F("MPH, Wind Direction: "));  Serial.print(Wind_Direction);
    Serial.print(F("Deg., Highest Gust: "));  Serial.print(Highest_Gust); 
    Serial.print(F("MPH, \"Average Wind Speed\" (Accumulated): "));  Serial.print(Average_Wind_Speed); 
    Serial.print(F("MPH, Next update: "));  Serial.print(Standard_Telemetry_Interval - (Timer_Count - Last_Telemetry_Update));  Serial.println(F("S"));
  }

    Is_Sample_Required = false;
  }

  if (Timer_Count - Last_Telemetry_Update >= Standard_Telemetry_Interval) {  // Time to produce data
  // --------------------------------------------------------------------------------------------------- ADC Health Check Block
    if (!ADS.isConnected()) {  // If ADC is not connected.   Display values anyway for debugging purposes.
      Serial.println(F("Failed to read ADS. (I2C) (Loop)"));
      ADS1015_Error_Flag = true;

    } else if (ADS1015_Error_Flag) { // If not, ADC is connected, so clear flag if there was one.
      ADS1015_Error_Flag = false;
    }

    if (ADS1015_Error_Flag) {  // If error attempt to clear it.
      Serial.println(F("Attempting recovery... (I2C) (Loop)"));
      if (!ADS.begin()) {  // If failed again...
        Serial.println(F("Failed to recover ADS. (I2C) (Loop Recovery)"));
      } else {  // If recovered clear error flag
        ADS1015_Error_Flag = false;
      }
      
  // --------------------------------------------------------------------------------------------------- Telemetry Block
    } else {    // If no error flag...

      getWindDirection();

      if (abs(Wind_Direction - Last_Value) > 2) {  // Only update the display if change greater than 2 degrees.
        Last_Value = Wind_Direction;
      }

      // Calculate average wind speed by finding the number of samples over a given period minus one (because telemetry takes priority over sampling) and dividing them
      // into the accumulated wind speed to get the average wind speed over the telemetry interval.
      Average_Wind_Speed /= (Standard_Telemetry_Interval / Anemometer_Refresh_Interval - 1);   

      Serial.print(F("Average Wind Speed: "));  Serial.print(Average_Wind_Speed); Serial.print(F("MPH, Gust: "));  Serial.print(Highest_Gust); Serial.print(F("MPH  "));
      Serial.print(Wind_Direction);  Serial.println(F("*Deg."));

      Average_Wind_Speed = 0;  // Clear data for next variable.
      Highest_Gust = 0;
      Wind_Speed = 0;
      Rotations = 0;
      
      Last_Telemetry_Update = Timer_Count;
      Last_Anemometer_Refresh = Timer_Count;  // To keep from having bad data added.
    }


  }
  // --------------------------------------------------------------------------------------------------- Sample Time Block
  if (Timer_Count - Last_Anemometer_Refresh >= Anemometer_Refresh_Interval) {  // Time to do sample of anemometer?
    Is_Sample_Required = true;
    Last_Anemometer_Refresh = Timer_Count;
  }
}


void isr_rotation() {  // This is the function that the interrupt calls to increment the rotation count

    if ((millis() - Contact_Bounce_Time) > 15 ) { // Debounce the switch contact.
      Rotations++;                                // Add to rotations
      Contact_Bounce_Time = millis();             // Debounce contingency
    }
  }

void getWindDirection() {  // Get Wind Direction

    if (ADS.isConnected()) {  // Check if ADC is connected to prevent bad data.
      Vane_Value = ADS.readADC(Wind_Vane_Pin);
    } else {  // If ADC is not connected; default to 0.
      ADS1015_Error_Flag = true;
      Vane_Value = 0;
    }

    Raw_MCU_Bus_Voltage_Reading = ADS.readADC(MCU_Bus_Voltage_Pin);  // 359 deg. would be equal to Vcc; anthing else would be inbetween. 

    Direction = map(Vane_Value, 0, Raw_MCU_Bus_Voltage_Reading, 0, 359);  // Raw MCU bus voltage is used here, because 359 deg is Vcc. Since Vcc is not always constant this compensates for fluctuations.
    Wind_Direction = Direction + Vane_Offset;

    if (Wind_Direction > 360)  // Do offset corrections if needed.
      Wind_Direction = Wind_Direction - 360;

    if (Wind_Direction < 0)
      Wind_Direction = Wind_Direction + 360;

  }
