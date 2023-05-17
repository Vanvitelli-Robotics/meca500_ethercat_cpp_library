//IP Address: 192.168.2.120
#define CUSTOM
#define VerNumber 0x10 // firmware version number  V_1.0

#include "EasyCAT_Gateway.h"
#include "EasyCAT/EasyCAT.h" // EasyCAT library for LAN9252

#include <Ethernet.h> // Ethernet library for wiz5500
#include <SPI.h>      // SPI library
#include <Wire.h>     // I2C library
#include <EthernetUdp.h>

#define EEPROM_ADD 0x50  // EEPROM I2C address
#define BLINK_SPEED 1000 // led blink time 1S

#define END 0xC0     // constants for the Slip protocol
#define ESC 0xDB     //      "
#define ESC_END 0xDC //      "
#define ESC_ESC 0xDD //      "
#define HEADER 0x1234
#define BUFFER_SIZE 36

EasyCAT EASYCAT; // EasyCAT istantiation

EthernetClient client;
EthernetUDP Udp;

//---- pins declaration ------------------------------------------------------------------------------

const int Res_Eth = 38; // wiz5500 reset pin
const int Led = 13;     // led

//---- global variables ---------------------------------------------------------------------------

uint8_t Mac[6];

uint8_t EcatState;

uint8_t NumRxByte;
uint8_t PreviousRxByte;

uint8_t RxBuff[TOT_BYTE_NUM_IN];

uint8_t SlipTxBuff[TOT_BYTE_NUM_OUT * 2];
uint8_t WrIdx;

bool IpAssigned;

uint32_t Time;
uint32_t UsbTime;
uint32_t BlinkTime;

byte ip_sensor[] = {192, 168, 2, 120};
//IPAddress ip_sensor(192, 168, 2, 120);
int port_sensor = 49152;
uint8_t *byte_ptr;
byte request[8]; /* The request data sent to the Net F/T. */
/* variables */
static const int _test = 0x1234;
static const char *_ptest = (char *)&_test;

//Struct
// typedef struct
// {
//   uint32 rdt_sequence; /** RDT sequence number of this packet*/
//   uint32 ft_sequence;  /** The record's internal sequence number*/
//   uint32 status;       /** System status code*/
//   int32_t Fx;
//   int32_t Fy;
//   int32_t Fz;
//   int32_t Tx;
//   int32_t Ty;
//   int32_t Tz; /** z-axis torque */
// } copyStruct;

PROCBUFFER_IN copyStruct;
uint8_t buffer[BUFFER_SIZE];
uint32_t last_rdt_sequence = 0;
unsigned long actual_time = 0;
unsigned long last_time = 0;
unsigned long diff_time = 0;
int test=1;
int NumDataRx;
int time_out=0;
unsigned long time1=0;
unsigned long diff=0;

//---- setup ---------------------------------------------------------------------------------------

void setup()
{
  uint8_t i;
  
  NumRxByte = 0;
  IpAssigned = false;
  last_rdt_sequence = 0;

  Serial.begin(9600); // init the debug serial line

  Wire.begin(); // init the I2C interface

  pinMode(Led, OUTPUT);     // init the pins
  pinMode(Res_Eth, OUTPUT); //

  digitalWrite(Res_Eth, LOW);  // two flashes on the led
  digitalWrite(Led, LOW);      // and reset the wiz5500
  delay(300);                  //
  digitalWrite(Res_Eth, HIGH); //
  digitalWrite(Led, HIGH);     //
  delay(300);                  //
  digitalWrite(Led, LOW);      //
  delay(300);                  //
  digitalWrite(Led, HIGH);     //
  delay(1000);                 //
  digitalWrite(Led, LOW);      //

  Serial.println("\nEasyCAT Gateway basic example"); // print the banner
  Serial.print("Firmware version V_");               //
  Serial.print(VerNumber >> 4, HEX);                 // extract Major and Minor from the version number
  Serial.print(".");                                 //
  Serial.println(VerNumber & 0x0F, HEX);             //
  Serial.println("");                                //

  //------ initialize the EtherCAT interface ------

  if (EASYCAT.Init() == true)                           // initialization
  {                                                     // successfully completed
    Serial.println("EtherCAT interface initialized\n"); //
  }                                                     //

  else                                     // initialization failed
  {                                        // the EasyCAT interface was not recognized
    Serial.print("initialization failed"); //
    // stay in loop for ever
    while (1)                  // with the led blinking
    {                          //
      digitalWrite(Led, HIGH); //
      delay(200);              //
      digitalWrite(Led, LOW);  //
      delay(200);              //
    }                          //
  }                            //

  //------ initialize the Ethernet interface ------

  Wire.beginTransmission(EEPROM_ADD); // read the MAC address from the EEPROM
  Wire.write(0xFA);                   //
  Wire.endTransmission(0);            //
  Wire.requestFrom(EEPROM_ADD, 6, 1); //

  Serial.print("Ethernet interface Mac Address\n"); // print it out and store it
  //
  i = 0;

  while (Wire.available())     //
  {                            //
    Mac[i] = Wire.read();      //
    Serial.print(Mac[i], HEX); //
    i++;                       //
    if (i < 6)                 //
    {                          //
      Serial.print(":");       //
    }                          //
  }                            //
  Serial.println("");          //

  if (Ethernet.linkStatus() == LinkOFF)               // nobody is connected to
  {                                                   // our Ethernet interface
    Serial.println("Ethernet cable is disconnected"); //
  }                                                   //

  else if (Ethernet.linkStatus() == LinkON)        // somebody is connected to
  {                                                // our Ethernet interface
    Serial.println("Ethernet cable is connected"); //
    //
    Ethernet.begin(Mac);                             // obtain an IP address from the DHCP
    Serial.println("Ethernet interface IP address"); // print it out
    Serial.println(Ethernet.localIP());              //
    IpAssigned = true;
    Udp.begin(port_sensor); //
  }

  UsbTime = millis();
  BlinkTime = millis();
}

//---- main loop ----------------------------------------------------------------------------------------

void loop()
{
  int nbytes = 0;

  EcatState = EASYCAT.MainTask(); //------ execute the EtherCAT task --------------

  //------ simple WEB server example ---------------
  if (IpAssigned)
  {
    //byte_ptr = EASYCAT.BufferIn.Byte;

    //send request
    int error = Udp.beginPacket(ip_sensor, port_sensor);
    if (error != 0)
    {
      //scrivo nel pacchetto i dati da inviare
      *(uint16_t *)&request[0] = htons(0x1234);                              /* standard header. */
      *(uint16_t *)&request[2] = htons(EASYCAT.BufferOut.Cust.command);      /* see section 9.1 in Net F/T user manual. */
      *(uint32_t *)&request[4] = htonl(EASYCAT.BufferOut.Cust.sample_count); /* per table 9.1 in Net F/T user manual. */

      Udp.write(request, 8);

      //send packet
      Udp.endPacket();

      //request to server
      //Controlla se sono presenti dati da leggere
      //time_out<100
      while(test!=0 && time_out<100)
      {
        time1=micros();
        test=Udp.parsePacket();
        if(test!=0) NumDataRx=test;
        Udp.read(buffer, test);
        time_out++;
        diff=micros() - time1;
        Serial.print("Diff: ");
        Serial.println(diff);
      }
      test=1;
      time_out=0;
      
      
      //int NumDataRx = Udp.parsePacket();
       //Udp.read(buffer, BUFFER_SIZE);
      
/*
      Serial.print("Command: ");
      Serial.println(EASYCAT.BufferOut.Cust.command);
      Serial.print("Sample_count: ");
      Serial.println(EASYCAT.BufferOut.Cust.sample_count);
      Serial.print("\nStatus: ");
      Serial.println(EASYCAT.BufferIn.Cust.status);
*/
      if (NumDataRx == BUFFER_SIZE)
      {

        //Invio al serial monitor il numero di byte presenti nel buffer
        Serial.print("Numero di byte da leggere: ");
        Serial.println(NumDataRx);

        //leggo i dati e li memorizzo in un array
        

        unpack(buffer);

        if (copyStruct.Cust.status != 0)
        {
          Serial.print("Error status: ");
          Serial.println(copyStruct.Cust.status);
        }

        int32_t seqdiff = int32_t(copyStruct.Cust.rdt_sequence - last_rdt_sequence);
        
        if (seqdiff < 1)
        {
          Serial.print("Error invalid packet!\n");
          Serial.print("Actual rdt_sequence: ");
          Serial.println(copyStruct.Cust.rdt_sequence);
          Serial.print("Last rdt_sequence: ");
          Serial.println(last_rdt_sequence);
        }
        else
        {
          last_rdt_sequence = copyStruct.Cust.rdt_sequence;
          actual_time = millis();
          diff_time = actual_time - last_time;
          last_time = actual_time;

          Serial.print("Diff_time: ");
          Serial.println(diff_time);
          
          EASYCAT.BufferIn.Cust = copyStruct.Cust;
        
      
        }
      }
      else
      {
        Serial.print("Receive size of ");
        Serial.print(NumDataRx);
        Serial.print(" bytes does not match expected size of ");
        Serial.println(BUFFER_SIZE);
      }
    }
    else
      Serial.println("Errore connessione UDP..\n");
  }

  Time = millis(); //------ led blink management --------------
  //
  if ((Time - UsbTime) > 500)                 // if there is no communication with
  {                                           // the USB host for more then 500 mS
    digitalWrite(Led, LOW);                   // switch off the led
  }                                           //
  else                                        // otherwise blink
  {                                           // it at 1 S rate
    if ((Time - BlinkTime) > BLINK_SPEED)     //
    {                                         //
      BlinkTime = Time;                       //
      digitalWrite(Led, (!digitalRead(Led))); //
    }                                         //
  }                                           //
}

uint32_t htonl(uint32_t host32)
{
  if (*_ptest == 0x12)
    return host32;
  return ((host32 & 0xff) << 24) | ((host32 & 0xff00) << 8) | ((host32 & 0xff0000) >> 8) | ((host32 & 0xff000000) >> 24);
}

/* htons */
uint16_t htons(uint16_t host16)
{
  if (*_ptest == 0x12)
    return host16;
  return ((host16 & 0xff) << 8) | ((host16 & 0xff00) >> 8);
}

uint32_t unpack32(const uint8_t *buffer)
{
  return (uint32_t(buffer[0]) << 24) |
         (uint32_t(buffer[1]) << 16) |
         (uint32_t(buffer[2]) << 8) |
         (uint32_t(buffer[3]) << 0);
}

void unpack(const uint8_t *buffer)
{
  copyStruct.Cust.rdt_sequence = unpack32(buffer + 0);
  copyStruct.Cust.ft_sequence = unpack32(buffer + 4);
  copyStruct.Cust.status = unpack32(buffer + 8);
  copyStruct.Cust.Fx = unpack32(buffer + 12);
  copyStruct.Cust.Fy = unpack32(buffer + 16);
  copyStruct.Cust.Fz = unpack32(buffer + 20);
  copyStruct.Cust.Tx = unpack32(buffer + 24);
  copyStruct.Cust.Ty = unpack32(buffer + 28);
  copyStruct.Cust.Tz = unpack32(buffer + 32);
}
