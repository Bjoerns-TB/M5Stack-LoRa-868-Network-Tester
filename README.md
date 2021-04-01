# M5Stack-LoRa-868-Network-Tester

A LoRaWAN Network Tester based on the M5Stack for the LoRa 868 module, compatible with TTN (The Things Network)

[Version for LoRaWAN Module]

[Version for COM.LoRaWAN Module]

## Setup
The tester is designed to work with the following hardware:
  - M5Core (Basic, Gray or Fire)
  - M5Go Base (optional)
  - M5Stack GPS Module or Mini GPS/BDS Unit (optional)
  - [M5Stack LoRa 868 Module]

#### Required Libraries!
  - [M5Stack]
  - [TinyGPSPlus]
  - [NeoPixelBus]
  - [M5_UI] - this Fork enables TTN Mapper like colours for RSSI values in the progressbar
  - [MCCI Arduino LMIC] - this Fork enables the LinkCheckRequest to get info on gateway number and margin from the backend

 
#### Installation and Configuration
Upload this sketch to your M5 using the Arduino IDE. M5Stack Fire users have to disable PSRAM, because it will interfer with UART2.
UART2 with GPIO 16 and 17 willbe used for the GPS module.

Edit the lmic_project_config.h

```
  #define CFG_eu868 1
  #define CFG_sx1276_radio 1
```
    
By commenting out #define M5go it is possible to disable the M5GO Base. This will disable all NeoPixel related code and features.
By commenting out #define M5gps ist is possible to disable the M5GPS module. This will disable all GPS related code and features. 
  
Change the your TTN keys under //LoRaWAN ABP and //LoRaWAN OTAA in the networktester.ino file. If you want yo use OTAA mode you have to register a second device for your application. Only the OTAA mode uses OTAA, all other modes use ABP.

In oder to make the LoRa 868 module work with LMIC, you have to connect DIO1 of the Ra-01H with Pin 35 of the M5Stack bus.

**An older picture showed that i connected DIO1 to PIN 35 via two solder pads next to the M5Bus connector. With this i connected DIO0 with DIO1 and Pin 35 and 36,  which was not intended. With the solderpads it is possible to connect DIO0 to PIN 35 or PIN 34 with an 0 Ohm resistor (same for 25/26 and RST), so the right pads are both connected to DIO1. Please use only the left solderpad or better directly the M5Bus connector, as shown in the picture. With this change the #define LMIC_USE_INTERRUPTS in lmic_project_config is obsolete. Furthermore the pin mapping has also changed. Please notice that there are two versions of LoRa 868 modules (before and after december 2020 with different pin mapping). (Thanks to Andreas D. for the appointment).
Sorry for any problems.**

![DIO1 Image](https://github.com/Bjoerns-TB/M5Stack-LoRa-868-Network-Tester/blob/main/images/IMG_2434.jpg "Fig 1. DIO1 solder")

Payload Decoder for TTN (also compatible with TTN Mapper integration):

```
function Decoder(b, port) {

  var lat = (b[0] | b[1]<<8 | b[2]<<16 | (b[2] & 0x80 ? 0xFF<<24 : 0)) / 10000;
  var lon = (b[3] | b[4]<<8 | b[5]<<16 | (b[5] & 0x80 ? 0xFF<<24 : 0)) / 10000;
  var alt = (b[6] | b[7]<<8 | (b[7] & 0x80 ? 0xFF<<16 : 0)) / 100;
  var hdop = b[8] / 10;

  return {
    
      latitude: lat,
      longitude: lon,
      altitude: alt,
      hdop: hdop
    
  };
}
```

To be compliant with the TTN Stack V3, the Frame Counter is now stored on the SD card, so an inserted card is now mandatory. The sent counter displays the sent packet based on the Frame Counter. Thic can be reverted to the previous behaviour by changeing the follwing lines:

```
//txcnt = String("Sent " + String(cnt));
txcnt = String("Sent " + String(LMIC.seqnoUp - 1));
```

## Instructions for Use

#### Menu

On boot you will be presented with the "Boot-Logo" followed by the first working mode. At the moment the tester has 7 modes to select:
  - [NACK](#nack) 
  - [ACK](#ack)  
  - [MAN](#man)  
  - [LCM](#lcm)
  - [SSV](#ssv)
  - [OTAA](#otaa)
  - [SET](#set)
 
You can move between menu items by pushing the button A. 

![Menu Image](https://github.com/Bjoerns-TB/M5Stack-LoRaWAN-Network-Tester/blob/master/images/menu.jpg "Fig 1. Menu")
  
#### NACK 
#### (No Acknowladge)
"NACK" is a mode that utlises the current device [settings](#set) to perform periodic transmissions. "NACK" mode is great for use with TTN Mapper.

![NACK Image](https://github.com/Bjoerns-TB/M5Stack-LoRaWAN-Network-Tester/blob/master/images/nack.jpg "Fig 2. NACK")

Pushing button B will let you cycle through each spreadfactor. By pushig button C the display and LEDs will be turned off. Pushing button C again will turn them on.

#### ACK 
#### (Acknowladge)
"ACK" will perform the same test as NACK but it will request an ACK for every transmission. The RSSI and SNR values of the received packet will be shown on the display. Pushing button B will let you cycle through each spreadfactor. By pushig button C the display and LEDs will be turned off. Pushing button C again will turn them on.

![ACK Image](https://github.com/Bjoerns-TB/M5Stack-LoRaWAN-Network-Tester/blob/master/images/ack.jpg "Fig 3. ACK")

#### MAN 
#### (Manual)
"MAN" will send a LoRaWAN packet with ACK by pushing button C. Pushing button B will let you cycle through each spreadfactor.

![MAN Image](https://github.com/Bjoerns-TB/M5Stack-LoRaWAN-Network-Tester/blob/master/images/man.jpg "Fig 4. MAN")

#### LCM 
#### (LinkCheckMode)
"LCM" is a mode that will trigger a LinkCheckRequest. The TTN backend will report back the number of gateways which received the request. Pushing button B will let you cycle through each spreadfactor. The request is triggered by button C.

![LCM Image](https://github.com/Bjoerns-TB/M5Stack-LoRaWAN-Network-Tester/blob/master/images/lcm.jpg "Fig 5. LCM")

#### SSV 
#### (SiteSurvey)
"SSV" is supposed as mode for testing a location. During SSV mode the DutyCycle check will be disabled an the Node will send a LinkCheckRequest for every spreadfactor from SF7 to SF12. After the test the node will show you on which datarates a ACK was received back. The data is also stored on the SD card in GeoJSON format an could be analyzed with [geojson.io]

![SSV Image](https://github.com/Bjoerns-TB/M5Stack-LoRaWAN-Network-Tester/blob/master/images/ssv-1.jpg "Fig 6. SSV running")
![SSV Image](https://github.com/Bjoerns-TB/M5Stack-LoRaWAN-Network-Tester/blob/master/images/ssv-2.jpg "Fig 7. SSV results")

#### OTAA 
#### (OverTheAirActivation)
"OTAA" enables the tester to perform OTAA-Joins. By selecting Join the tester will try to join the TTN Network. After an successful the the tester will start with periodic transmissions. You have the choice between transmission with or without ACK. The RSSI and SNR values of the received packet will be shown on the display. If there is no valid GPS fix, a packet can by manually send by pushing button B.

![OTAA Image](https://github.com/Bjoerns-TB/M5Stack-LoRaWAN-Network-Tester/blob/master/images/otaa.jpg "Fig 7. OTAA")

#### SET 
#### (Settings)

"SET" allows to change the transmission intervall in NACK or ACK mode. Possible settings are 15/30/45/60/120 seconds. Pressing button C will active the powersaving mode. The node will go to deep sleep and wakes up according to the transmission intervall. Sleep mode can only be stopped by resetting the devicde.

![SET Image](https://github.com/Bjoerns-TB/M5Stack-LoRaWAN-Network-Tester/blob/master/images/set.jpg "Fig 7. SET")

## Notes
  - The DutyCycle check ist activated except for the SSV mode.
  - If you have a valid GPS fix the GPS track will be written to the SD card as a GPX file.
  - Periodic transmission will only work with a valid GPS fix and an GPS age below 2 seconds
  
  
## Known Bugs/Limitations
  - Sometimes, if an ACK is not received, the node tries to resend the last message multiple times. This will interfer with other jobs. Retries are turned of, but sometimes happen.
  - When SSV mode ist left, the Duty Cycle limit is activated. So the next transfer will wait about 150 seconds. Status will display "Queued".
  

## Accesories
  - Michael designed a [Case]. Thank you!
  
  
## Changelog

  - 01.04.2021
    - Corrected the Instructions for connecting DIO1 to M5Bus
    - Updated Pin Mapping
    - Updated lmic_project_config

  - 17.02.2021
    - Replace drawBitmap by pushImage for corrected colours

  - 09.02.2021
    - disabled ADR in ABP mode; this removed unwanted ADR related uplinks 

  - 07.02.2021
    - fix SSV mode on fast retry
    - in SSV mode save yellow lighthouse when only reception of ACK with SF11 and/or SF12
    - in SSV mode save red lighthouse when no ACK is received

  - 05.02.2021
    - fix SSV mode data save and display

  - 03.02.2021
    - fix periodic transmissions in OTAA mode
    - write and read framecounter from sd card; display sent packet based on LMIC framecounter (reversible)

  - 04.01.2021
    - play beep on ACK received in ACK mode
    - reset counters on no ACK received

  - 03.01.2021
    - Allow different regions in ABP mode

  - 01.01.2021
    - change progessbar scale 0: -130 100:-80 RSSI
    - Display information about queued packets in LCM and OTAA mode
    - Reset nGWs and gwMargin after mode Change

  - 30.12.2020
    - Reset RSSI, SNR and No of GWs values on change of mode
    
  - 29.12.2020
    - First commit


## ToDo
  - create tasks for M5UI
  - improve powersave features (GPS module)


[M5Stack]: https://github.com/m5stack/M5Stack
[TinyGPSPlus]: https://github.com/mikalhart/TinyGPSPlus
[NeoPixelBus]: https://github.com/Makuna/NeoPixelBus
[M5_UI]: https://github.com/Bjoerns-TB/M5_UI/tree/TTN-Mapper-Colours-Progressbar
[geojson.io]: http://geojson.io/
[M5Stack LoRa 868 Module]: https://m5stack.com/products/lora-module-868mhz
[MCCI Arduino LMIC]: https://github.com/Bjoerns-TB/arduino-lmic/tree/LMIC_setLinkCheckRequestOnce
[Case]: https://www.thingiverse.com/thing:4706335
[Version for COM.LoRaWAN Module]: https://github.com/Bjoerns-TB/M5Stack-COM-LoRaWAN-Network-Tester
[Version for LoRaWAN Module]: https://github.com/Bjoerns-TB/M5Stack-LoRaWAN-Network-Tester
