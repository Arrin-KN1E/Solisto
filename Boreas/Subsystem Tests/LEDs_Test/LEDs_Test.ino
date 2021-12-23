/*
  Simple test of the testing LEDs
  Creates a 'snake' chasing effect on the LEDs
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

bool polarity; 
byte myPins[] = {0, 1, 3, 4, 19, 21, 23};  // If using different number of LEDs it's just as simple as putting them in sequence in the array e.g.: byte myPins[] = {0, 1, 4, 5, 19, 21, 23, 25, 26};
// 0-(Yellow) STBY/ Reading LED, 1-(Orange) ERROR LED, 3-(Red) TX LED, 4-(Green) RX LED, 19-(Blue) Air Sensor LED, 21-(Green) Ground Sensor LED, 23-(White) Road Sensor LED

void setup() {
  
  for (byte i = 0; i < sizeof(myPins); i++) {
    pinMode(myPins[i], OUTPUT);
  }
}


void loop() {
  
    for (byte i = 0; i < sizeof(myPins); i++) {
    digitalWrite(myPins[i], polarity);
    delay(100);
  }
    polarity ^= true;
}
