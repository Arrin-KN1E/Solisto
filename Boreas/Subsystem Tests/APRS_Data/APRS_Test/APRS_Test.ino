/*
  APRS subsystem test using the LibAPRS circuit and library to test TX and RX capabilities.
  For calibrations and troubleshooting.

  Wiring:
     (Refer to schematic for full circuit layout)

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

#include <LibAPRS.h>

#define ADC_REFERENCE REF_5V
#define OPEN_SQUELCH false    // Define whether modem will be running with an open squelch radio.

boolean gotPacket = false;
AX25Msg incomingPacket;
uint8_t *packetData;

// You always need to include this function. It will get called by the library every time a packet is received, so you can process incoming packets.
// If you are only interested in transmitting, you should just leave this function empty.
// IMPORTANT! This function is called from within an interrupt. That means that you should only do things here that are FAST. Don't print out info directly from this function, instead set a flag and print it from your main loop, like this:
void aprs_msg_callback(struct AX25Msg *msg) {

  if (!gotPacket) {  // If we already have a packet waiting to be processed, we must drop the new one.

    gotPacket = true;  // Set flag to indicate we got a packet

    memcpy(&incomingPacket, msg, sizeof(AX25Msg));  // The memory referenced as *msg is volatile and we need to copy all the data to a local variable for later processing.

    if (freeMemory() > msg->len) {  // We need to allocate a new buffer for the data payload of the packet. First we check if there is enough free RAM.
      packetData = (uint8_t*)malloc(msg->len);
      memcpy(packetData, msg->info, msg->len);
      incomingPacket.info = packetData;
    } else {  // We did not have enough free RAM to receive this packet, so we drop it.
      gotPacket = false;
    }
  }
}

void setup() {
  Serial.begin(115200);

  APRS_init(ADC_REFERENCE, OPEN_SQUELCH);  // Initialise APRS library - This starts the modem
  APRS_setCallsign((char*)"KN1E", 1);      // You must at a minimum configure your callsign and SSID.
  APRS_setDestination((char*)"KN1E", 3);   // You don't need to set the destination identifier, but if you want to, this is how you do it.

  APRS_setPath1((char*)"WIDE1", 1);  // Path parameters are set to sensible values by default.
  APRS_setPath2((char*)"WIDE2", 2);

  APRS_setPreamble(350);  // You can define preamble and tail like this:
  APRS_setTail(100);

  APRS_useAlternateSymbolTable(false);  // You can use the normal or alternate symbol table:
  APRS_setSymbol('n');                  // And set what symbol you want to use. For more info:  http://www.aprs.org/symbols.html or http://wa8lmf.net/aprs/APRS_symbols.htm or https://www.aprsdirect.com/symbol/list

  APRS_setLat((char*)"3849.68N");  // Let's first set our latitude and longtitude. These should be in NMEA format!
  APRS_setLon((char*)"08610.22W");

  // We can optionally set power/height/gain/directivity information. These functions accept ranges from 0 to 10, directivity 0 to 9. LibAPRS will only add PHG info if all four variables are defined!
  // See this site for a calculator: http://www.aprsfl.net/phgr.php
  APRS_setPower(2);
  APRS_setHeight(0);
  APRS_setGain(7);
  APRS_setDirectivity(1);

  APRS_setMessageDestination((char*)"KN1E", 3);  // We first need to set the message recipient


  Serial.print(F("MCU Programmed: "));  Serial.print(__DATE__); Serial.print(F(" @ "));   Serial.println(__TIME__);  // Date and time of MCU programming/ compile.
  Serial.print(F("From: "));  Serial.println(__FILE__);  // File path of program.
  APRS_printSettings();  // We can print out all the settings
  Serial.print(F("Free RAM:     ")); Serial.println(freeMemory());
}

void locationUpdateExample() {

  char *comment = (char*)"LibAPRS location update";  // We'll define a comment string

  APRS_sendLoc(comment, strlen(comment));  // And send the update
}

void messageExample() {  // Send a message directly.
  int Data1 = 3;
  int Data2 = 53;

  char message[100];
  sprintf(message, "The %d burritos are %d degrees F", Data1, Data2);
  
  APRS_sendMsg(message, strlen(message));
}

// Here's a function to process incoming packets. Remember to call this function often, so you won't miss any packets due to one already waiting to be processed
void processPacket() {
  if (gotPacket) {
    gotPacket = false;

    Serial.print(F("Received APRS packet. SRC: "));
    Serial.print(incomingPacket.src.call);
    Serial.print(F("-"));
    Serial.print(incomingPacket.src.ssid);
    Serial.print(F(". DST: "));
    Serial.print(incomingPacket.dst.call);
    Serial.print(F("-"));
    Serial.print(incomingPacket.dst.ssid);
    Serial.print(F(". Data: "));

    for (int i = 0; i < incomingPacket.len; i++) {
      Serial.write(incomingPacket.info[i]);
    }
    Serial.println("");

    free(packetData);  // Remember to free memory for our buffer!

    // You can print out the amount of free RAM to check you don't have any memory leaks
    Serial.print(F("Free RAM: ")); Serial.println(freeMemory());
  }
}

boolean whichExample = false;
void loop() {

  delay(60000);
  if (whichExample) {
    locationUpdateExample();
  } else {
    messageExample();
  }
  whichExample ^= true;

  //delay(500);
  processPacket();
}
