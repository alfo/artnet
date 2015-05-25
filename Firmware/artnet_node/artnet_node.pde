/***************************************************************************
*
* Title          : Arduino ArtNet Node
* Version        : v1.1
* Last updated   : 18.05.2015
* Web            : https://newfangled.me, https://alexforey.com
* Target         : Arduino Mega 2560, Arduino Mega 1280, Arduino Uno

*** Arduino IDE v0023 MUST BE USED ***

* Based on code from
  * Toni Merino, http://www.deskontrol.net/blog  merino.toni@gmail.com
  * Christoph Guillermet, http://www.le-chat-noir-numerique.fr  karistouf@yahoo.fr

* Structures and definitions (common.h and packet.h) from libartnet (c)Simon Newton and Lutz Hillebrand (ilLUTZminator), www.opendmx.net
*
* Art-Net™ Designed by and Copyright Artistic Licence Holdings Ltd.
*
***************************************************************************
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 as published by the Free Software Foundation; either version2 of
 the License, or (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 General Public License for more details.

 If you have no copy of the GNU General Public License, write to the
 Free Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 For other license models, please contact the author.

;***************************************************************************/

#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <Udp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008
#include <lib_dmx.h>     // deskontrol four universes DMX library
#include "artnet_node.h"
#include "common.h"      // definitions from libartnet
#include "packets.h"     // headers from libartnet, striped version
#include <Wire.h>               // Needed for I2C for LCD
#include <LiquidCrystal_I2C.h>  // Control library for LCD
#include "WebServer.h" // To allow settings to be changed over network
#include <EEPROM.h>

// ***************************************************************************************************************
//                             ***        READ THIS        ***
//    Sometimes you need to disconnect the MAX485 on TX/RX (USART) 0 to program the Arduino. Then connect it, and
//    hit the reset button on the Arduino to start it up again.
// ***************************************************************************************************************

#define PREFIX "/artnet"
WebServer webserver(PREFIX, 80);


/************************************
    Webserver Function Definitions
************************************/

void softwareRESET() {
  asm volatile ("  jmp 0");  
}

void webCmd(WebServer &server, WebServer::ConnectionType type, char *, bool)
{
  if (type == WebServer::POST)
  {
    bool repeat;
    char name[16], value[16];
    do
    {
      /* readPOSTparam returns false when there are no more parameters
       * to read from the input.  We pass in buffers for it to store
       * the name and value strings along with the length of those
       * buffers. */
      repeat = server.readPOSTparam(name, 32, value, 32);

      /* this is a standard string comparison function.  It returns 0
       * when there's an exact match.  We're looking for a parameter
       * named "buzz" here. */
      if (strcmp(name, "output_a") == 0)
      {
	/* use the STRing TO Unsigned Long function to turn the string
	 * version of the delay number into our integer buzzDelay
	 * variable */
        EEPROM.write(1, strtoul(value, NULL, 10));
      }
      
      if (strcmp(name, "output_b") == 0)
      {
	/* use the STRing TO Unsigned Long function to turn the string
	 * version of the delay number into our integer buzzDelay
	 * variable */
        EEPROM.write(2, strtoul(value, NULL, 10));
      }
    } while (repeat);
    
    // after procesing the POST data, tell the web browser to reload
    // the page using a GET method. 
    server.httpSeeOther(PREFIX);
    softwareRESET();
  }

  /* for a GET or HEAD, send the standard "it's all OK headers" */
  server.httpSuccess();

  /* we don't output the body for a HEAD request */
  if (type == WebServer::GET)
  {
    /* store the HTML in program memory using the P macro */
    P(message) = 
"<!DOCTYPE html><html> <head> <title>AF ArtNet Settings</title> <meta name='viewport' content='width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no'/> <style>*{margin: 0;}header{width: 100%; padding: 20px; background: rgb(45,45,55);}h1{color: rgb(220, 220, 220);}h2{color: rgb(100, 100, 100);}body{background: rgb(25,25,35); color: white; font-family: Verdana;}form{background: rgb(45,45,55); padding: 20px; width: 80%; margin: 50px auto;}input, select{display: inline; padding: 10px; width: 100%; margin: 20px auto; -webkit-appearance: none; outline: 0; border: none;}label{display: block;}</style> </head> <body> <header> <h1>AF ArtNet Node Settings</h1> <h2>v1.1 2015</h2> </header> <form action='/artnet' method='post'> <label for='output_a'>Output A</label> <select name='output_a'> <option value='1'>Universe 1</option> <option value='2'>Universe 2</option> </select> <label for='output_a'>Output B</label> <select name='output_b'> <option value='1'>Universe 1</option> <option value='2'>Universe 2</option> </select> <input type='submit' value='Save'> </form> </body></html>";    
    server.printP(message);
  }
}

/************************************
          DMX Definitions
************************************/

// Always use at least one universe
#define   USE_UNIVERSE_0

// If using an Arduino Mega
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

  // Use a second universe
  #define   USE_UNIVERSE_1

  // Uncomment these to add a third and fourth universe
  // However, using three universes means having a low refresh rate (so ugly fades)
  // and a fourth universe makes everything unstable, so use it at your own risk.

  //#define   USE_UNIVERSE_2      // remember universe 2 now is USART 2, Arduino output pin 16
  //#define   USE_UNIVERSE_3      // remember universe 3 now is USART 3, Arduino output pin 14
#endif

#define        DMX512            (0)    // (250 kbaud - 2 to 512 channels) Standard USITT DMX-512

/************************************
        ArtNet Definitions
************************************/

artnet_node_t             ArtNode;
artnet_reply_t            ArtPollReply;
artnet_packet_type_t      packet_type;

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  const int MAX_BUFFER_UDP = 1650;  // For Arduino MEGA
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  const int MAX_BUFFER_UDP = 550;  // For Arduino UNO, due to only have 2kb of RAM, and only 1 UART
#endif

uint8_t packetBuffer [MAX_BUFFER_UDP]; // buffer to store incoming UDP data


/************************************
          LCD Definitions
************************************/

// Initialise the LCD
LiquidCrystal_I2C lcd(0x27,16,2);;  // Set the LCD I2C address

/************************************
      Online/Offline Definitions
************************************/

// Set up some variables for online/offline detection
int ArtNetOnline = 5000;
int DMXSendCounter = 0;
int loop_no = 0;
boolean online = false;
boolean previous_state = online;

/************************************
        RGB LED Defintions
************************************/

// Pins for the RGB LED
// Can't use Pin 10, because it's used by the ethernet shield
const int redPin = 11;
const int greenPin = 12;
const int bluePin = 9;
int fadeToState = 0;

/************************************
        Ethernet Definitions
************************************/

// Mac address of your ethernet shield
uint8_t factory_mac          [6] = {   0x90,   0xA2,   0xDA,   0x09,   0x00,  0x03};

// The IP address you want your node to have
int localIp_a = 10;
int localIp_b = 0;
int localIp_c = 1;
int localIp_d = 200;

// The IP address of the gateway/router
int gateway_a = 10;
int gateway_b = 0;
int gateway_c = 1;
int gateway_d = 1;

// The subnet mask
// When using the node on a network with a router, use 255.255.255.0.
// When using the node without a router, use 255.0.0.0
int subnetMask_a = 255;
int subnetMask_b = 255;
int subnetMask_c = 255;
int subnetMask_d = 0;

// Fill the arrays to begin with
uint8_t factory_localIp      [4] = {  localIp_a, localIp_b,  localIp_c, localIp_d};
uint8_t factory_broadcastIp  [4] = {  localIp_a, localIp_b,  localIp_c, localIp_d};
uint8_t factory_gateway      [4] = {  gateway_a, gateway_b, gateway_c, gateway_d};
uint8_t factory_subnetMask  [4] = { subnetMask_a,  subnetMask_b,  subnetMask_c,  subnetMask_d};

int outputA = 1;
int outputB = 2;

/************************************
    Standard Function Definitions
************************************/

void setup()
{

  // Start the LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Booting...");

  // ** This is where code to allow the user to change the IP address using buttons would go **
  // ** That code would change the variables like localIp_a etc, and then they're loaded again below **

  uint8_t factory_localIp [4]     = {localIp_a, localIp_b,  localIp_c, localIp_d};
  uint8_t factory_broadcastIp [4]  = {  localIp_a, localIp_b,  localIp_c, localIp_d};
  uint8_t factory_gateway [4]      = {  gateway_a, gateway_b, gateway_c, gateway_d};
  uint8_t factory_subnetMask [4]  = { subnetMask_a,  subnetMask_b,  subnetMask_c,  subnetMask_d};

  // Tell the ArtNet library what the IP addresses and things are
  fill_art_node (&ArtNode);

  ArtNode.numbports  = 0;

  #if defined(USE_UNIVERSE_0)
    ArduinoDmx0.set_control_pin(2);     // max485 input/output control (connect to 485 pins 2-3)
    ArduinoDmx0.set_tx_address(0);       // set tx0 start address
    ArduinoDmx0.set_tx_channels(512);    // number of TX channels
    ArduinoDmx0.init_tx(DMX512);         // starts universe 0 as tx, standard DMX512   ***new***
    ArtNode.numbports ++;
  #endif

  #if defined(USE_UNIVERSE_1)
    ArduinoDmx1.set_control_pin(3);     // max485 input/output control (connect to 485 pins 2-3)
    ArduinoDmx1.set_tx_address(1);       // set tx1 start address
    ArduinoDmx1.set_tx_channels(512);    // number of TX channels
    ArduinoDmx1.init_tx(DMX512);         // starts universe 1 as tx, standard DMX512   ***new***
    ArtNode.numbports ++;
  #endif

  #if defined(USE_UNIVERSE_2)
    ArduinoDmx1.set_control_pin(4);     // max485 input/output control (connect to 485 pins 2-3)
    ArduinoDmx1.set_tx_address(2);       // set tx1 start address
    ArduinoDmx1.set_tx_channels(512);    // number of TX channels
    ArduinoDmx1.init_tx(DMX512);         // starts universe 1 as tx, standard DMX512   ***new***
    ArtNode.numbports ++;
  #endif

  // More ArtNet stuff
  fill_art_poll_reply    (&ArtPollReply, &ArtNode);

  // Begin the Ethernet connection
  Ethernet.begin(ArtNode.mac, ArtNode.localIp, ArtNode.gateway, ArtNode.subnetMask);

  // Begin UDP connections
  Udp.begin(ArtNode.localPort);
  send_reply(BROADCAST, (uint8_t *)&ArtPollReply, sizeof(ArtPollReply));

  webserver.setDefaultCommand(&webCmd);
  
  outputA = EEPROM.read(1);
  outputB = EEPROM.read(2);
  
  if (outputA < 1 || outputA > 16)
    outputA = 1;
    
  if (outputB < 1 || outputB > 16)
    outputB = 2;

  // Init the LED Pins
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  digitalWrite(redPin, HIGH);

  // Print the IP address to the LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("IP: ");
  lcd.print(localIp_a);
  lcd.print(".");
  lcd.print(localIp_b);
  lcd.print(".");
  lcd.print(localIp_c);
  lcd.print(".");
  lcd.print(localIp_d);
  
  lcd.setCursor(9, 1);
  lcd.print("A");
  lcd.print((int)EEPROM.read(1));
  lcd.print(" B");
  lcd.print((int)EEPROM.read(2));
}

void loop()
{

  checkArtNetOnline();

  displayArtNetStatus();

  handleOverflows();

  if( Udp.available() > ARNET_HEADER_SIZE ) {
    handle_packet();
    ArtNetOnline = 0;
  } else {
    ArtNetOnline++;
  }

  updateLED();
  
  webserver.processConnection();

  previous_state = online;
  loop_no++;

}

/************************************
    Status Function Definitions
************************************/

// Used to fade the LED from red to green when online, or green to red when offline
void updateLED() {

  if (previous_state != online) {
    fadeToState = 255;
  }

  if (fadeToState != 0) {
    if (!online) {
      analogWrite(redPin, 255 - fadeToState);
      analogWrite(greenPin, fadeToState);
    } else {
      analogWrite(redPin, fadeToState);
      analogWrite(greenPin, 255 - fadeToState);
    }
    delay(5);
    fadeToState--;
  }
}

// Checks whether ArtNet UDP packets have been received recently
void checkArtNetOnline() {
  if (ArtNetOnline < 2500) {
    online = true;
  } else {
    online = false;
  }
}

// Refreshes the LCD display with the ArtNet status
void displayArtNetStatus() {
  if (loop_no % 500 == 0) {
    lcd.setCursor(0, 1);
    if (online) {
      lcd.print("Online ");
    } else {
      lcd.print("Offline");
    }
  }
}

// Prevent the counter variables from overflowing
void handleOverflows() {
  if (loop_no > 20000) {
    loop_no = 0;
  }
  if (ArtNetOnline > 20000) {
    ArtNetOnline = 1000;
  }
}

/************************************
    ArtNet Function Definitions
************************************/

void handle_packet()
{
  Udp.readPacket((uint8_t *)&packetBuffer, MAX_BUFFER_UDP, (uint8_t *)&ArtNode.remoteIp, (uint16_t *)&ArtNode.remotePort);

  packet_type = (artnet_packet_type_t) get_packet_type((uint8_t *)&packetBuffer);

  // Ignore bad packets
  if(packet_type == 0) {
    return;
  }

  // Handle DMX data packets
  if(packet_type == ARTNET_DMX) {
    if (sizeof(packetBuffer) < sizeof(artnet_dmx_t)) {
      return;
    } else {
      handle_dmx((artnet_dmx_t *)&packetBuffer);
    }
  }

  else if(packet_type == ARTNET_POLL) {
    if(sizeof(packetBuffer) < sizeof(artnet_poll_t))
      return;
    else
      handle_poll((artnet_poll_t *)&packetBuffer);
  }

  else if(packet_type == ARTNET_ADDRESS)
  {
    if(sizeof(packetBuffer) < sizeof(artnet_address_t))
      return;
    else
      handle_address((artnet_address_t *)&packetBuffer);
  }
}

// Find out the ArtNet packet type
uint16_t get_packet_type(uint8_t *packet) {
  if (!memcmp(packet, ArtNode.id, 8)) {
    return bytes_to_short(packet[9], packet[8]);
  }
  return 0;  // bad packet
}

// Send the DMX data off to one of the universes
int handle_dmx(artnet_dmx_t *packet) {
  
  

  if(packet->universe == ArtNode.swout[outputA - 1]) {
    #if defined(USE_UNIVERSE_0)
      memcpy ((uint8_t *)ArduinoDmx0.TxBuffer, (uint8_t *)packet->data, ARTNET_DMX_LENGTH);
    #endif
  }
  
  if(packet->universe == ArtNode.swout[outputB - 1]) {
    #if defined(USE_UNIVERSE_1)
      memcpy ((uint8_t *)ArduinoDmx1.TxBuffer, (uint8_t *)packet->data, ARTNET_DMX_LENGTH);
    #endif
  }
}

int handle_poll(artnet_poll_t *packet)
{
  if((packet->ttm & 1) == 1) // controller say: send unicast reply
  {
    send_reply(UNICAST, (uint8_t *)&ArtPollReply, sizeof(ArtPollReply));
  }
  else // controller say: send broadcast reply
  {
    send_reply(BROADCAST, (uint8_t *)&ArtPollReply, sizeof(ArtPollReply));
  }
}

int handle_address(artnet_address_t *packet) //not implemented yet
{
  send_reply(UNICAST, (uint8_t *)&ArtPollReply, sizeof(ArtPollReply));
}

void send_reply(uint8_t mode_broadcast, uint8_t *packet, uint16_t size)
{
  if(mode_broadcast == 1) // send broadcast packet
  {
    Udp.sendPacket(packet, size, ArtNode.broadcastIp, ArtNode.remotePort);
  }
  else // send unicast packet to controller
  {
    Udp.sendPacket(packet, size, ArtNode.remoteIp, ArtNode.remotePort);
  }
}

void fill_art_node(artnet_node_t *node)
{
  //fill to 0's
  memset (node, 0, sizeof(node));

  //fill data
  memcpy (node->mac, factory_mac, 6);                   // the mac address of node
  memcpy (node->localIp, factory_localIp, 4);           // the IP address of node
  memcpy (node->broadcastIp, factory_broadcastIp, 4);   // broadcast IP address
  memcpy (node->gateway, factory_gateway, 4);           // gateway IP address
  memcpy (node->subnetMask, factory_subnetMask, 4);     // network mask (art-net use 'A' network type)

  sprintf((char *)node->id, "Art-Net\0"); // *** don't change never ***
  sprintf((char *)node->shortname, "AF ArtNet Node \0");
  sprintf((char *)node->longname, "Alex Forey ArtNet Node v1 2015\0");

  memset (node->porttypes,  0x80, 4);
  memset (node->goodinput,  0x08, 4);

  node->subH           = 0x00;        // high byte of the Node Subnet Address (This field is currently unused and set to zero. It is
                                      // provided to allow future expansion.) (art-net III)
  node->sub            = 0x00;        // low byte of the Node Subnet Address

  // **************************** art-net address of universes **********************************
          // This array defines the 8 bit Universe address of the available output channels.

  
  node->swout[0] = 0x00;
  node->swout[1] = 0x01;
  node->swout[2] = 0x02;
  node->swout[3] = 0x03;

  // not implemented yet
  node->swin       [0] = 0x00;        // This array defines the 8 bit Universe address of the available input channels.
  node->swin       [1] = 0x01;        // values from 0x00 to 0xFF
  node->swin       [2] = 0x02;
  node->swin       [3] = 0x03;

#if defined(USE_UNIVERSE_0)
  node->goodoutput [0] = 0x80;
#endif

#if defined(USE_UNIVERSE_1)
  node->goodoutput [1] = 0x80;
#endif

#if defined(USE_UNIVERSE_2)
  node->goodoutput [2] = 0x80;
#endif

#if defined(USE_UNIVERSE_3)
  node->goodoutput [3] = 0x80;
#endif

  node->etsaman[0] = 0;        // The ESTA manufacturer code.
  node->etsaman[1] = 0;        // The ESTA manufacturer code.
  node->localPort  = 0x1936;   // artnet UDP port is by default 6454 (0x1936)
  node->verH       = 1;        // high byte of Node firmware revision number.
  node->ver        = 1;        // low byte of Node firmware revision number.
  node->ProVerH    = 0;        // high byte of the Art-Net protocol revision number.
  node->ProVer     = 14;       // low byte of the Art-Net protocol revision number.
  node->oemH       = 0;        // high byte of the oem value.
  node->oem        = 0xFF;     // low byte of the oem value. (0x00FF = developer code)
  node->ubea       = 0;        // This field contains the firmware version of the User Bios Extension Area (UBEA). 0 if not used
  node->status     = 0x08;
  node->swvideo    = 0;
  node->swmacro    = 0;
  node->swremote   = 0;
  node->style      = 0;        // StNode style - A DMX to/from Art-Net device
}

void fill_art_poll_reply(artnet_reply_t *poll_reply, artnet_node_t *node)
{
  //fill to 0's
  memset (poll_reply, 0, sizeof(poll_reply));

  //copy data from node
  memcpy (poll_reply->id, node->id, sizeof(poll_reply->id));
  memcpy (poll_reply->ip, node->localIp, sizeof(poll_reply->ip));
  memcpy (poll_reply->mac, node->mac, sizeof(poll_reply->mac));
  memcpy (poll_reply->shortname, node->shortname, sizeof(poll_reply->shortname));
  memcpy (poll_reply->longname, node->longname, sizeof(poll_reply->longname));
  memcpy (poll_reply->nodereport, node->nodereport, sizeof(poll_reply->mac));
  memcpy (poll_reply->porttypes, node->porttypes, sizeof(poll_reply->porttypes));
  memcpy (poll_reply->goodinput, node->goodinput, sizeof(poll_reply->goodinput));
  memcpy (poll_reply->goodoutput, node->goodoutput, sizeof(poll_reply->goodoutput));
  memcpy (poll_reply->swin, node->swin, sizeof(poll_reply->swin));
  memcpy (poll_reply->swout, node->swout, sizeof(poll_reply->swout));
  memcpy (poll_reply->etsaman, node->etsaman, sizeof(poll_reply->etsaman));

  sprintf((char *)poll_reply->nodereport, "%i DMX output universes active.\0", node->numbports);

  poll_reply->opCode          = 0x2100;  // ARTNET_REPLY
  poll_reply->port            = node->localPort;
  poll_reply->verH            = node->verH;
  poll_reply->ver             = node->ver;
  poll_reply->subH            = node->subH;
  poll_reply->sub             = node->sub;
  poll_reply->oemH            = node->oemH;
  poll_reply->oem             = node->oem;
  poll_reply->status          = node->status;
  poll_reply->numbportsH      = node->numbportsH;
  poll_reply->numbports       = node->numbports;
  poll_reply->swvideo         = node->swvideo;
  poll_reply->swmacro         = node->swmacro;
  poll_reply->swremote        = node->swremote;
  poll_reply->style           = node->style;
}

