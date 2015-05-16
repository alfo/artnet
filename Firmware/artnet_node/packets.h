/*
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * packets.h
 * Datagram definitions for libartnet
 * Copyright (C) 2004-2008 Simon Newton, Lutz Hillebrand (ilLUTZminator)
 */

#ifndef _artnet_packets_defined
#define _artnet_packets_defined

#include "common.h"

enum artnet_packet_type_e {
  ARTNET_POLL = 0x2000,
  ARTNET_REPLY = 0x2100,
  ARTNET_DMX = 0x5000,
  ARTNET_ADDRESS = 0x6000,
  ARTNET_INPUT = 0x7000,
  ARTNET_TODREQUEST = 0x8000,
  ARTNET_TODDATA = 0x8100,
  ARTNET_TODCONTROL = 0x8200,
  ARTNET_RDM = 0x8300,
  ARTNET_VIDEOSETUP = 0xa010,
  ARTNET_VIDEOPALETTE = 0xa020,
  ARTNET_VIDEODATA = 0xa040,
  ARTNET_MACMASTER = 0xf000,
  ARTNET_MACSLAVE = 0xf100,
  ARTNET_FIRMWAREMASTER = 0xf200,
  ARTNET_FIRMWAREREPLY = 0xf300,
  ARTNET_IPPROG = 0xf800,
  ARTNET_IPREPLY = 0xf900,
  ARTNET_MEDIA = 0x9000,
  ARTNET_MEDIAPATCH = 0x9200,
  ARTNET_MEDIACONTROLREPLY = 0x9300
}__attribute__((packed));

typedef enum artnet_packet_type_e artnet_packet_type_t;


struct artnet_poll_s {
  uint8_t  id[8];
  uint16_t opCode;
  uint8_t  verH;
  uint8_t  ver;
  uint8_t  ttm;
  uint8_t  pad;
} __attribute__((packed));

typedef struct artnet_poll_s artnet_poll_t;

struct artnet_reply_s {
  uint8_t  id[8];
  uint16_t opCode;
  uint8_t  ip[4];
  uint16_t port;
  uint8_t  verH;
  uint8_t  ver;
  uint8_t  subH;
  uint8_t  sub;
  uint8_t  oemH;
  uint8_t  oem;
  uint8_t  ubea;
  uint8_t  status;
  uint8_t  etsaman[2];
  uint8_t  shortname[ARTNET_SHORT_NAME_LENGTH];
  uint8_t  longname[ARTNET_LONG_NAME_LENGTH];
  uint8_t  nodereport[ARTNET_REPORT_LENGTH];
  uint8_t  numbportsH;
  uint8_t  numbports;
  uint8_t  porttypes[ARTNET_MAX_PORTS];
  uint8_t  goodinput[ARTNET_MAX_PORTS];
  uint8_t  goodoutput[ARTNET_MAX_PORTS];
  uint8_t  swin[ARTNET_MAX_PORTS];
  uint8_t  swout[ARTNET_MAX_PORTS];
  uint8_t  swvideo;
  uint8_t  swmacro;
  uint8_t  swremote;
  uint8_t  sp1;
  uint8_t  sp2;
  uint8_t  sp3;
  uint8_t  style;
  uint8_t  mac[ARTNET_MAC_SIZE];
  uint8_t  filler[32];
} __attribute__((packed));

typedef struct artnet_reply_s artnet_reply_t;

struct artnet_ipprog_s {
  uint8_t  id[8];
  uint16_t OpCode;
  uint8_t  ProVerH;
  uint8_t  ProVer;
  uint8_t  Filler1;
  uint8_t  Filler2;
  uint8_t  Command;
  uint8_t  Filler4;
  uint8_t  ProgIpHi;
  uint8_t  ProgIp2;
  uint8_t  ProgIp1;
  uint8_t  ProgIpLo;
  uint8_t  ProgSmHi;
  uint8_t  ProgSm2;
  uint8_t  ProgSm1;
  uint8_t  ProgSmLo;
  uint8_t  ProgPortHi;
  uint8_t  ProgPortLo;
  uint8_t  Spare1;
  uint8_t  Spare2;
  uint8_t  Spare3;
  uint8_t  Spare4;
  uint8_t  Spare5;
  uint8_t  Spare6;
  uint8_t  Spare7;
  uint8_t  Spare8;

} __attribute__((packed));

typedef struct artnet_ipprog_s artnet_ipprog_t;

struct artnet_ipprog_reply_s {
  uint8_t  id[8];
  uint16_t OpCode;
  uint8_t  ProVerH;
  uint8_t  ProVer;
  uint8_t  Filler1;
  uint8_t  Filler2;
  uint8_t  Filler3;
  uint8_t  Filler4;
  uint8_t  ProgIpHi;
  uint8_t  ProgIp2;
  uint8_t  ProgIp1;
  uint8_t  ProgIpLo;
  uint8_t  ProgSmHi;
  uint8_t  ProgSm2;
  uint8_t  ProgSm1;
  uint8_t  ProgSmLo;
  uint8_t  ProgPortHi;
  uint8_t  ProgPortLo;
  uint8_t  Spare1;
  uint8_t  Spare2;
  uint8_t  Spare3;
  uint8_t  Spare4;
  uint8_t  Spare5;
  uint8_t  Spare6;
  uint8_t  Spare7;
  uint8_t  Spare8;
} __attribute__((packed));

typedef struct artnet_ipprog_reply_s artnet_ipprog_reply_t;


struct artnet_address_s {
  uint8_t  id[8];
  uint16_t opCode;
  uint8_t  verH;
  uint8_t  ver;
  uint8_t  filler1;
  uint8_t  filler2;
  uint8_t  shortname[ARTNET_SHORT_NAME_LENGTH];
  uint8_t  longname[ARTNET_LONG_NAME_LENGTH];
  uint8_t  swin[ARTNET_MAX_PORTS];
  uint8_t  swout[ARTNET_MAX_PORTS];
  uint8_t  subnet;
  uint8_t  swvideo;
  uint8_t  command;
} __attribute__((packed));

typedef struct artnet_address_s artnet_address_t;


struct artnet_dmx_s {
  uint8_t  id[8];
  uint16_t opCode;
  uint8_t  verH;
  uint8_t  ver;
  uint8_t  sequence;
  uint8_t  physical;
  uint16_t universe;
  uint8_t  lengthHi;
  uint8_t  length;
  uint8_t  data[ARTNET_DMX_LENGTH];
} __attribute__((packed));

typedef struct artnet_dmx_s artnet_dmx_t;


struct artnet_input_s {
  uint8_t  id[8];
  uint16_t opCode;
  uint8_t  verH;
  uint8_t  ver;
  uint8_t  filler1;
  uint8_t  filler2;
  uint8_t  numbportsH;
  uint8_t  numbports;
  uint8_t  input[ARTNET_MAX_PORTS];
} __attribute__((packed));

typedef struct artnet_input_s artnet_input_t;

#endif
