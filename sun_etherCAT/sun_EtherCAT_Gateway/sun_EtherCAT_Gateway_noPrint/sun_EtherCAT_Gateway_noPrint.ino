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

byte ip_sensor[] = {192, 168, 2, 120};
int port_sensor = 49152;
byte request[8]; /* The request data sent to the Net F/T. */
static const int _UDP_data = 0x1234;
static const char *_pUDP_data = (char *)&_UDP_data;

PROCBUFFER_IN copyStruct;
uint8_t buffer[BUFFER_SIZE]; //buffer cointains UDP packet
uint32_t last_rdt_sequence = 0;

int NumDataRx;
int time_out = 0;

//command variables
uint16_t command_prev;
uint32_t sample_prev;

//Variables UDP_data
unsigned long actual_time = 0;
unsigned long last_time = 0;
unsigned long diff_time = 0;
unsigned long time1 = 0;
unsigned long diff = 0;
int UDP_data = -1;
unsigned long time_array[500];
unsigned long time_correct_packet[500];
int iteration;

//---- setup ---------------------------------------------------------------------------------------

void setup()
{
    uint8_t i;

    NumRxByte = 0;
    IpAssigned = false;
    last_rdt_sequence = 0;
    command_prev = -1;
    sample_prev = -1;
    iteration = 0;

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

    if (EASYCAT.Init() == true)                             // initialization
    {                                                       // successfully completed
        Serial.println("EtherCAT interface initialized\n"); //
    }                                                       //

    else                                       // initialization failed
    {                                          // the EasyCAT interface was not recognized
        Serial.print("initialization failed"); //
        // stay in loop for ever
        while (1)                    // with the led blinking
        {                            //
            digitalWrite(Led, HIGH); //
            delay(200);              //
            digitalWrite(Led, LOW);  //
            delay(200);              //
        }                            //
    }                                //

    //------ initialize the Ethernet interface ------

    Wire.beginTransmission(EEPROM_ADD); // read the MAC address from the EEPROM
    Wire.write(0xFA);                   //
    Wire.endTransmission(0);            //
    Wire.requestFrom(EEPROM_ADD, 6, 1); //

    Serial.print("Ethernet interface Mac Address\n"); // print it out and store it
    //
    i = 0;

    while (Wire.available())       //
    {                              //
        Mac[i] = Wire.read();      //
        Serial.print(Mac[i], HEX); //
        i++;                       //
        if (i < 6)                 //
        {                          //
            Serial.print(":");     //
        }                          //
    }                              //
    Serial.println("");            //

    if (Ethernet.linkStatus() == LinkOFF)                 // nobody is connected to
    {                                                     // our Ethernet interface
        Serial.println("Ethernet cable is disconnected"); //
    }                                                     //

    else if (Ethernet.linkStatus() == LinkON)          // somebody is connected to
    {                                                  // our Ethernet interface
        Serial.println("Ethernet cable is connected"); //
        //
        Ethernet.begin(Mac);                             // obtain an IP address from the DHCP
        Serial.println("Ethernet interface IP address"); // print it out
        Serial.println(Ethernet.localIP());              //
        IpAssigned = true;
        Udp.begin(port_sensor); //
    }
}

//---- main loop ----------------------------------------------------------------------------------------

void loop()
{
    int nbytes = 0;

    EcatState = EASYCAT.MainTask(); //------ execute the EtherCAT task --------------

    //------ simple WEB server example ---------------
    if (IpAssigned)
    {
        //send request
        if (command_prev != EASYCAT.BufferOut.Cust.command || sample_prev != EASYCAT.BufferOut.Cust.sample_count)
        {
            command_prev = EASYCAT.BufferOut.Cust.command;
            sample_prev = EASYCAT.BufferOut.Cust.sample_count;

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
            }
        }

        //request to server and check data to read
        //time_out<100
        if (command_prev == 2) //start streaming real time communication
        {
            int i = 0;
            while (UDP_data != 0 && time_out < 1)
            {
                time1 = micros();
                UDP_data = Udp.parsePacket();
                if (UDP_data != 0)
                    NumDataRx = UDP_data;
                Udp.read(buffer, UDP_data);
                diff = micros() - time1;
                if (i < 500)
                    time_array[i++] = diff;
                time_out++;
            }
            UDP_data = -1;
            time_out = 0;

            if (NumDataRx == BUFFER_SIZE)
            {
                //leggo i dati e li memorizzo in un array
                unpack(buffer);

                if (copyStruct.Cust.status == 0) //no error
                {
                    int32_t seqdiff = int32_t(copyStruct.Cust.rdt_sequence - last_rdt_sequence);

                    if (seqdiff >= 1)
                    {
                        last_rdt_sequence = copyStruct.Cust.rdt_sequence;
                        EASYCAT.BufferIn.Cust = copyStruct.Cust;
                        actual_time = millis();
                        diff_time = actual_time - last_time;
                        last_time = actual_time;
                        if (iteration < 500)
                        {
                            time_correct_packet[iteration] = diff_time;
                            iteration++;
                        }
                    }
                }
            }
        }
    }
    else
        Serial.println("Errore IP address not assigned..\n");
}

uint32_t htonl(uint32_t host32)
{
    if (*_pUDP_data == 0x12)
        return host32;
    return ((host32 & 0xff) << 24) | ((host32 & 0xff00) << 8) | ((host32 & 0xff0000) >> 8) | ((host32 & 0xff000000) >> 24);
}

/* htons */
uint16_t htons(uint16_t host16)
{
    if (*_pUDP_data == 0x12)
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
