/*
  ADS1015 simple subsystem test.
  For sensor calibrations and troubleshooting.
  ! Note: This program does not use the adafruit ADS1X15 library for sensor recovery reasons.

  Wiring:

  +---------------------------------------+
  |                                       |
  |       ADS1015 (I2C) (Ext. ADC)        |
  |                                       |
  +_______________________________________+
  |   |   |   |   | A | A |   |   |   |   |
  | v | G | S | S | D | L |   |   |   |   |
  | C | N | C | D | D | R | A | A | A | A |
  | C | D | L | A | R | T | 0 | 1 | 2 | 3 |
  +---------------------------------------+
    |   |   |   |   |   |   |   |   |   |
   +5V  |   |   |  GND  |   |   |  VCC  |
       GND  |   |       NC  |   |      BATT
  (To MCU) SCL  |         An_WD |
      (To MCU) SDA             SSV

  An_WD (A0)- Anemometer wind direction; connected to wind direction potentiometer.
  SSV   (A1)- Sensor Supply Voltage; connected to output of current limited supply of DS18B20s.
  VCC   (A2) - VCC of common bus, most importantly the MCU bus; connected directly to positive rail/ bus.
  BATT  (A3) - Battery voltage; connected through resistor divider of 10:1 for up to 10x ADC (ADS1015) input voltage.

  For An_WD (Anemometer Wind Direction Potentiometer) Raw bit values will be given, Volts, non-offset wind direction. 
 (i.e. No North offset correction applied.), then wind direction with offset applied.

  For SSV (Sensor Supply Voltage) Raw bit values will be given, then Volts. This value should never exceed MCU bus  voltage.

  For VCC (Common Bus/ MCU Bus) Raw bit values will be given, then Volts. This is the common voltage for all powered devices in the circuit.

  For Batt (Battery Voltage measured through 10:1 resistor divider) Raw bit values will be given, raw voltage at pin will be given, 
  then adjusted voltage of battery will be given. Use 'Battery_Voltage_Multiplier' to adjust divider to match resistor tolerances.  


                                                1284P
                                              +---\/---+
     STBY LED (Yellow)           (D 0)  PB0  1|        |40  PA0  (ADC 0 / D24)     RX Audio Input
     Error LED (Orange)          (D 1)  PB1  2|        |39  PA1  (ADC 1 / D25)     NC
     1-Wire (DQ)           INT 2 (D 2)  PB2  3|        |38  PA2  (ADC 2 / D26)     NC
     TX LED (Red)            PWM (D 3)  PB3  4|        |37  PA3  (ADC 3 / D27)     NC
     RX LED (Green)       PWM/SS (D 4)  PB4  5|        |36  PA4  (ADC 4 / D28)     NC
     NC                     MOSI (D 5)  PB5  6|        |35  PA5  (ADC 5 / D29)     NC
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
#include "ADS1X15.h"  // This library was used instead of the adafruit library, because this library natively supports connection checks. (ADS1015 12-bit ADC)
#include <math.h>

ADS1015 ADS(0x48);       // I2C address 0x48  // 12-bit  (For 16-bit version, carefully go throught the code to change any magic numbers.)

int Wind_Vane_Pin = 0;        // On ADS1015 ADC. The pin the wind vane sensor is connected to. Anemometer variables are declared a little different in this program, but will still work.
int Vane_Offset = 90;         // Anemometer offset from magnetic north
int Raw_Vane_Value;           // Raw analog value from wind vane.
float Wind_Vane_Voltage = 0;  // Voltage at ADC pin.
int Direction;                // Translated 0 - 360 direction.
int Cal_Direction;            // Converted value with North offset applied.

bool ADS1015_Error_Flag = false;

byte  MCU_Bus_Voltage_Pin = 2;             // On ADS1015 ADC.  This is MCU bus voltage monitor
float MCU_Bus_Voltage_Calibration = 0.07;  // Final adjustment to get accurate reading
float Read_MCU_Bus_Voltage = 0;            //
uint16_t Raw_MCU_Bus_Voltage_Reading = 0;  // Raw bit data from ADC (ADS1015) for Wind direction reference.

byte  Battery_Voltage_Pin = 3;             // On ADS1015 ADC. This is Battery voltage monitor.
float Battery_Voltage_Multiplier = 11.2;   // Needed adjustment (because of resistor tolerances) to get accurate reading. R-Divider is [+]1M to 100K[-] so 11 multiplier is baseline.
float Battery_Voltage_Calibration = 0.03;  // Final adjustment to get accurate reading.
float Read_Battery_Voltage = 0;            //
uint16_t Raw_Battery_Voltage_Reading = 0;  // Raw bit data from ADC (ADS1015) for debug/ serial monitor.

byte  Sensor_Supply_Voltage_Pin = 1;             // On ADS1015 ADC. This is sensors current limited supply voltage monitor.
float Sensor_Supply_Voltage_Calibration = 0.13;  // Final adjustment to get accurate reading.
float Read_Sensor_Supply_Voltage = 0;            // Used to determine if overcurrent of a DS18B20 sensor.
uint16_t Raw_Sensor_Supply_Voltage_Reading = 0;  // Raw bit data from ADC (ADS1015) for debug/ serial monitor.

void setup(void)
{
  Serial.begin(115200);

  Serial.println(F("\nADS1015 sensor calibrations and troubleshooting."));
  Serial.print(F("MCU Programmed: "));  Serial.print(__DATE__); Serial.print(F(" @ "));   Serial.println(__TIME__);  // Date and time of MCU programming/ compile.
  Serial.print(F("From: "));  Serial.println(__FILE__);  // File path of program.

  Serial.println(F("\nDefault ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)\n"));

/*
   The ADC input range (or gain) can be changed via the following
   functions, but be careful never to exceed VDD +0.3V max, or to
   exceed the upper and lower limits if you adjust the input range!
   Setting these values incorrectly may destroy your ADC!
                                                                  ADS1015  ADS1115
                                                                  -------  -------
   ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
   ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
   ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
   ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
   ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
   ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
*/

  if (!ADS.begin()) {
    Serial.println(F("Failed to initialize ADS. (I2C) (Setup)"));
    ADS1015_Error_Flag = true;
  }
}

void loop(void) {

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

  } else {  // If no error flag...

    // Get raw bit data (e.g. 0 - 4095).
    Raw_Vane_Value = ADS.readADC(Wind_Vane_Pin);
    Raw_MCU_Bus_Voltage_Reading = ADS.readADC(MCU_Bus_Voltage_Pin);
    Raw_Sensor_Supply_Voltage_Reading = ADS.readADC(Sensor_Supply_Voltage_Pin);
    Raw_Battery_Voltage_Reading = ADS.readADC(Battery_Voltage_Pin);

    // Covert raw bit data to voltages.
    Wind_Vane_Voltage = ADS.toVoltage(Raw_Vane_Value);
    Read_MCU_Bus_Voltage = ADS.toVoltage(Raw_MCU_Bus_Voltage_Reading);
    Read_Sensor_Supply_Voltage = ADS.toVoltage(Raw_Sensor_Supply_Voltage_Reading);
    Read_Battery_Voltage = ADS.toVoltage(Raw_Battery_Voltage_Reading);


    //  if(ADS1015_Error_Flag){  // For NaN
    //    Raw_Vane_Value = 0;
    //  }


    // Calculate wind direction and apply offset.
    Direction = map(Raw_Vane_Value, 0, Raw_MCU_Bus_Voltage_Reading, 0, 359);  // Raw MCU bus voltage is used here, because 359 deg is Vcc.
    Cal_Direction = Direction + Vane_Offset;

    if (Cal_Direction > 360)  // Do offset corrections if needed.
      Cal_Direction = Cal_Direction - 360;
    if (Cal_Direction < 0)
      Cal_Direction = Cal_Direction + 360;



    Serial.println(F("\n------------------------------------------------------------------------------------"));

    // Anemometer
    Serial.print(F("An_WD_Pot: Raw Bits - "));  Serial.print(Raw_Vane_Value);  Serial.print(F("  ADC Pin - "));  Serial.print(Wind_Vane_Voltage);  Serial.print(F("V  No Offset - "));
    Serial.print(Direction);  Serial.print(F("°  Adjusted - "));  Serial.print(Cal_Direction);  Serial.print(F("°  Applied North Offset - "));
    Serial.print(Vane_Offset);  Serial.println(F("°"));

    // MCU Bus                                  
    Serial.print(F("VCC:       Raw Bits - "));  Serial.print(Raw_MCU_Bus_Voltage_Reading);  Serial.print(F("  "));  Serial.print(Read_MCU_Bus_Voltage);  Serial.println(F("V"));

    // Sensor Supply Voltage
    Serial.print(F("SSV:       Raw Bits - "));  Serial.print(Raw_Sensor_Supply_Voltage_Reading);  Serial.print(F("  ")); Serial.print(Read_Sensor_Supply_Voltage); Serial.println(F("V"));

    // Battery
    Serial.print(F("BATT:      Raw Bits - "));  Serial.print(Raw_Battery_Voltage_Reading);  Serial.print(F("  ADC Pin - "));  Serial.print(Read_Battery_Voltage);
    Serial.print(F("V  Adjusted - "));  Serial.print(Read_Battery_Voltage * Battery_Voltage_Multiplier);  Serial.println(F("V"));


    // Clear variables for if ADS1015 is not connected or unable to update.
    //int Wind_Vane_Pin, Raw_Vane_Value, Wind_Vane_Voltage, Direction,  Cal_Direction = 0;
    //float Read_MCU_Bus_Voltage, Read_Battery_Voltage, Read_Sensor_Supply_Voltage = 0;
    //uint16_t Raw_MCU_Bus_Voltage_Reading, Raw_Battery_Voltage_Reading, Raw_Sensor_Supply_Voltage_Reading = 0;

  }
  
  delay(10000);
}
