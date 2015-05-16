# Arduino-based ArtNet Node v1

![](http://i.imgur.com/85QnOMA.jpg)

This is a device that converts an ArtNet signal to two universes of DMX. I've tested it as working fully with QLC+ and ETC Nomad on both universes, at standard frame rates.

Apologies for the state of the Fritzing file, I'll clean it up when I can be bothered. You need to use Arduino 0023 and the bundled libraries, because some registers were renamed after Arduino 1.0.

## Features
* Two Universe ArtNet to DMX
* Option of both 5-pin and 3-pin output for both universes
* LCD Screen displaying IP address (for now, IP address needs to be manually assigned)
* RGB LED displaying online/offline state (detects being connected to network and receiving valid packets)
* Small form-factor

## Parts
* [Arduino Mega 2560](http://www.ebay.co.uk/sch/i.html?_sacat=0&_nkw=arduino+mega+2560&_frs=1) - £10
* [Arduino Ethernet Shield](http://www.ebay.co.uk/sch/i.html?_odkw=arduino+mega+2560&_osacat=0&_from=R40&_trksid=p2045573.m570.l1313.TR4.TRC2.A0.H0.Xarduino+ethernet+shield.TRS0&_nkw=arduino+ethernet+shield&_sacat=0) - £7
* Custom PCB (PDFs available for home-etching) - Free
* 2x [MAX485](http://www.ebay.co.uk/sch/i.html?_odkw=max485&_osacat=0&_from=R40&_trksid=p2045573.m570.l1313.TR1.TRC0.A0.H0.Xmax485+module.TRS0&_nkw=max485+module&_sacat=0) - £2
* 2x Neutrik XLR Sockets ([3](http://cpc.farnell.com/webapp/wcs/stores/servlet/ProductDisplay?catalogId=15002&langId=69&urlRequestType=Base&partNumber=AV19366&storeId=10180) or [5](http://cpc.farnell.com/webapp/wcs/stores/servlet/ProductDisplay?catalogId=15002&langId=69&urlRequestType=Base&partNumber=AV19367&storeId=10180) pins) - £6-12
* [RGB 5mm Common Cathode LED](http://www.ebay.co.uk/itm/3mm-5mm-LED-LEDs-RED-GREEN-BLUE-WHITE-YELLOW-ORANGE-RGB-UV-1-25-50-100-PACKS-/200921123666?pt=LH_DefaultDomain_3&var=&hash=item2ec7d50b52) - £1
* [Jumper Wires](http://www.ebay.co.uk/sch/i.html?_odkw=jumper+dupont+wires&_osacat=0&_from=R40&_trksid=p2045573.m570.l1313.TR0.TRC0.H0.Xjumper+dupont+wires+female+to+female.TRS0&_nkw=jumper+dupont+wires+female+to+female&_sacat=0) - £2
* [16x2 i2c LCD](http://www.ebay.co.uk/sch/i.html?_from=R40&_trksid=p2050601.m570.l1313.TR0.TRC0.H0.X16x2+i2c+lcd.TRS0&_nkw=16x2+i2c+lcd&_sacat=0) - £5
* Stacking Headers
* 2x 100Ω Resistor, 1x 47Ω Resistor
