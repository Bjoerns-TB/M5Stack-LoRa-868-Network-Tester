#include <M5Stack.h>				//  https://github.com/m5stack/M5Stack
#include <M5_UI.h>					//  https://github.com/dsiberia9s/M5_UI
#include <lmic.h>           //  https://github.com/Bjoerns-TB/arduino-lmic/tree/LMIC_setLinkCheckRequestOnce
#include <hal/hal.h>
#include <SPI.h>
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

#define M5go
#define M5gps

#ifdef M5go
#include <NeoPixelBrightnessBus.h>  //  https://github.com/Makuna/NeoPixelBus
#endif

#ifdef M5gps
#include <TinyGPS++.h>        //  https://github.com/mikalhart/TinyGPSPlus
#endif

//Task
TaskHandle_t TaskGPS;
TaskHandle_t TaskPixel;
TaskHandle_t TaskLMIC;

//Image
extern const unsigned char gImage_logoM5[];

#ifdef M5go
//NeoPixel
const uint16_t PixelCount = 10;
const uint8_t PixelPin = 15;
NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);
RgbColor red(128, 0, 0);
RgbColor green(0, 128, 0);
RgbColor blue(0, 0, 128);
RgbColor lightblue(0, 95, 128);
RgbColor yellow(128, 128, 0);
RgbColor orange(128, 64, 0);
RgbColor off(0, 0, 0);
#endif

#ifdef M5gps
//GPS
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial serialgps(2);
#endif
float latitude, longitude, hdop, alt, hdop2;
int sats;


//LoRa
int isf = 0;
int oldisf = 0;
char *dr[6] = {"DR5", "DR4", "DR3", "DR2", "DR1", "DR0"};
char *sf[6] = {"SF7", "SF8", "SF9", "SF10", "SF11", "SF12"};
RTC_DATA_ATTR int iwm = 0;
char *workmode[7] = {"NACK", "ACK", "MAN", "LCM", "SSV", "OTAA", "SET"};
char buffer[256];
short length;
short rssi;
float snr;
char charsnr[5];
short gwcnt;
byte coords[9];
byte ncoords[1];
long sentMillis = 0;
long currentMillis = 0;
RTC_DATA_ATTR int iiv = 0;
long interval[5] = {15000, 30000, 45000, 60000, 120000};
char *ttext[5] = {"15s", "30s", "45s", "60s", "120s"};
RTC_DATA_ATTR int cnt = -1;
String txcnt;
int otaa = 0;
int otaaack = 0;
static osjob_t sendjob;
bool next = true;
int margin;

//LoRaWAN ABP
// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
static const PROGMEM u1_t NWKSKEY[16] = { 0x00, ..., 0x00 };

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
static const u1_t PROGMEM APPSKEY[16] = { 0x00, ..., 0x00 };

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0x12345678 ; // <-- Change this address for every node!


//LoRaWAN OTAA
// This EUI must be in little-endian format (aka lsb), so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x00, ..., 0x00 };
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format (aka lsb), see above.
static const u1_t PROGMEM DEVEUI[8] = { 0x00, ..., 0x00 };
void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (aka msb) (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x00, ..., 0x00 };
void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}



//Battery
int8_t BattLevel = 0;
#define FULL       (   3)

//SDCard
char filename[] = "/";
bool cardin = false;
bool sdwrite = false;
File dataFile;

//GPX
int year;
byte month, day, hour, minute, second;
char filename1[20];
char date1[22];
char filepath[20];

//SSV
char filename2[20];
char date2[22];
char filepath2[20];
bool firstssv = false;
bool lastssv = false;
String ssvresult = "DR ";

//M5Stack
bool dim = false;
RTC_DATA_ATTR bool powersave = false;

/* RootVar's for UI elements (note: not edit manually) */
String UIInputbox_6nssds = "";        //No GWs for LCR
String UITextbox_vimqus = "SF7";      //SpreadingFactor (B2)
String UITextbox_eq79hh46 = "NACK";   //Workmode  (B1)
String UITextbox_67ofwdh = "Dim";     //Dimming (B3)
String UIProgressbar_eymzer = "0";   //Progressbar RSSI
String UITextbox_859t1hi = "-130";    //RSSI
String UIInputbox_awnh87 = "inactive";//Status
String UITextbox_4t0l0bn = "0";		  //Stattelites
String UITextbox_q7sl3uo = "0";		  //HDOP
String UITextbox_403ohip = "0";		  //Battery Level
String UITextbox_olwwlae = "-20.00";  //SNR
String UITextbox_7mnuudb = "SNR";     //SNR

/* Function for layer default: */
void LayerFunction_default(String* rootVar) {
  /* UI Elements */
  UIInputbox(160, 58, 150, "default", "No of GWs", 0, &UIInputbox_6nssds);
  UITextbox(144, 214, 50, 20, 0x0000, "default", &UITextbox_vimqus);
  UITextbox(44, 215, 50, 20, 0x0000, "default", &UITextbox_eq79hh46);
  UITextbox(227, 215, 50, 20, 0x0000, "default", &UITextbox_67ofwdh);
  UIProgressbar(10, 144, 300, "default", "RSSI, dB", &UIProgressbar_eymzer);
  UITextbox(124, 142, 50, 20, 0x0000, "default", &UITextbox_859t1hi);
  UIInputbox(5, 58, 150, "default", "Status", 0, &UIInputbox_awnh87);
  UITextbox(40, 11, 25, 20, 0x0000, "default", &UITextbox_4t0l0bn);
  UITextbox(100, 11, 60, 20, 0x0000, "default", &UITextbox_q7sl3uo);
  UITextbox(270, 11, 50, 20, 0x0000, "default", &UITextbox_403ohip);
  UITextbox(249, 142, 70, 20, 0x0000, "default", &UITextbox_olwwlae);
  UITextbox(200, 142, 40, 20, 0x0000, "default", &UITextbox_7mnuudb);

  /* To open this layer use: */
  UILayer("default");
}

#ifdef M5gps
static void gpsupdate(void * pcParameters)
{
  for (;;) {
    unsigned long start = millis();
    do {
      while (serialgps.available()) {
        gps.encode(serialgps.read());
      }
    } while (millis() - start < 1000);
    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed = 1;
    TIMERG0.wdt_wprotect = 0;
  }
}
#endif

#ifdef M5go
static void pixelupdate(void * pcParameters)
{
  for (;;) {

#ifdef M5gps
    //Change NeoPixel 4
    if (gps.satellites.value() < 3) {
      strip.SetPixelColor(4, red);
    }
    else if (gps.satellites.value() < 6) {
      strip.SetPixelColor(4, yellow);
    }
    else {
      strip.SetPixelColor(4, green);
    }

    //Change NeoPixel 0
    if (gps.hdop.value() < 500) {
      strip.SetPixelColor(0, green);
    }
    else if (gps.hdop.value() < 1000) {
      strip.SetPixelColor(0, yellow);
    }
    else {
      strip.SetPixelColor(0, red);
    }

    //Change NeoPixel 2
    if (gps.location.isValid() == false) {
      strip.SetPixelColor(2, red);
    }
    else if (gps.location.isValid() && gps.location.age() > 2000) {
      strip.SetPixelColor(2, red);
    }
    else if (gps.location.isValid() == true) {
      strip.SetPixelColor(2, green);
    }
    else {
      strip.SetPixelColor(2, green);
    }
#endif

    //Change NeoPixel 7 for RX status
    if ((iwm == 0) || (iwm == 4) || (iwm == 6)) {
      strip.SetPixelColor(7, off);
    }

    strip.Show();

    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed = 1;
    TIMERG0.wdt_wprotect = 0;
    smartDelay(500);
  }
}
#endif

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 5,                       
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 36,                       
  .dio = {26, 35, LMIC_UNUSED_PIN}, 
};


void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      UISet(&UIInputbox_awnh87, "Joining");
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      UISet(&UIInputbox_awnh87, "Joined");
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_RFU1:
      ||     Serial.println(F("EV_RFU1"));
      ||     break;
    */
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      cnt++;
      if (iwm != 4) {
        txcnt = String("Sent " + String(cnt));
        UISet(&UIInputbox_awnh87, txcnt);
      }
      if (LMIC.txrxFlags & TXRX_ACK) {
        Serial.println(F("Received ack
        M5.Speaker.beep();

        rssi = LMIC.rssi - 64;
        Serial.println(F("RSSI "));
        Serial.println(rssi);

        snr = LMIC.snr * 0.25;
        dtostrf(snr, 5, 1, charsnr);
        Serial.println(F("SNR "));
        Serial.println(snr);
        Serial.println(LMIC.nGws);
        Serial.println(LMIC.gwMargin);
        UISet(&UIProgressbar_eymzer, (rssi + 130) *2);
        UISet(&UITextbox_859t1hi, rssi);
        UISet(&UITextbox_olwwlae, charsnr);

        if ((iwm == 3) || (iwm == 4)) {

          gwcnt = LMIC.nGws;
          margin = LMIC.gwMargin - 64;
          UISet(&UIInputbox_6nssds, gwcnt);
        }

#ifdef M5go
        if (rssi < -120) {
          strip.SetPixelColor(7, blue);
        } else if (rssi < -115) {
          strip.SetPixelColor(7, lightblue);
        } else if (rssi < -110) {
          strip.SetPixelColor(7, green);
        } else if (rssi < -105) {
          strip.SetPixelColor(7, yellow);
        } else if (rssi < -100) {
          strip.SetPixelColor(7, orange);
        } else {
          strip.SetPixelColor(7, red);
        }
#endif

      }
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      next = true;
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_SCAN_FOUND:
      ||    Serial.println(F("EV_SCAN_FOUND"));
      ||    break;
    */
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      UISet(&UIInputbox_awnh87, "Sending");
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
      break;
  }
}

//Delay without delay
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {} while (millis() - start < ms);
}

#ifdef M5gps
//Write GPS-Data into variables
void gpsdata() {
  year = gps.date.year();
  month = gps.date.month();
  day = gps.date.day();
  hour = gps.time.hour();
  minute = gps.time.minute();
  second = gps.time.second();
  latitude = gps.location.lat();
  longitude = gps.location.lng();
  alt = gps.altitude.meters();
  hdop = gps.hdop.value();
}
#endif

#ifdef M5gps
//Initialize GPX-Track to SD-Card
void gpxinit() {
  if (cardin == true && gps.location.isValid() == true) {
    sdwrite = true;
    sprintf(filename1, "/%02d-%02d-%02d", day, month, year - 2000);
    sprintf(filepath, "/%02d-%02d-%02d/%02d-%02d%s", day, month, year - 2000,  hour, minute, ".GPX");

    SD.mkdir(filename1);
    if (!SD.exists(filepath)) {
      dataFile = SD.open(filepath, FILE_WRITE);
      dataFile.print(F(
                       "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n"
                       "<gpx version=\"1.1\" creator=\"Batuev\" xmlns=\"http://www.topografix.com/GPX/1/1\" \r\n"
                       "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\r\n"
                       "xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\">\r\n"
                       "\t<trk>\r\n<trkseg>\r\n"));
      dataFile.print(F("</trkseg>\r\n</trk>\r\n</gpx>\r\n"));
      dataFile.close();
    }
  }
}

//Write data to GPX-File
void writegpx() {
  if (gps.location.isValid() == true) {
    gpsdata();
    sprintf(date1, "%4d-%02d-%02dT%02d:%02d:%02dZ", year, month, day, hour, minute, second);
    dataFile = SD.open(filepath, FILE_WRITE);
    unsigned long filesize = dataFile.size();
    filesize -= 27;
    dataFile.seek(filesize);
    dataFile.print(F("<trkpt lat=\""));
    dataFile.print(latitude, 7);
    dataFile.print(F("\" lon=\""));
    dataFile.print(longitude, 7);
    dataFile.println(F("\">"));
    dataFile.print(F("<time>"));
    dataFile.print(date1);
    dataFile.println(F("</time>"));
    dataFile.print(F("<ele>"));
    dataFile.print(alt, 1);
    dataFile.print(F("</ele>\r\n<hdop>"));
    dataFile.print(hdop2, 1);
    dataFile.println(F("</hdop>\r\n</trkpt>"));
    dataFile.print(F("</trkseg>\r\n</trk>\r\n</gpx>\r\n"));
    dataFile.close();
  }
}
#endif

//Settings for LoRaWAN
void initlora() {

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
  Serial.println("Region eu868");
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set. The LMIC doesn't let you change
  // the three basic settings, but we show them here.
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
#elif defined(CFG_us915) || defined(CFG_au915)
  Serial.println("Region us915/au915");
  // NA-US and AU channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);
#elif defined(CFG_as923)
  Serial.println("Region as923");
  // Set up the channels used in your country. Only two are defined by default,
  // and they cannot be changed.  Use BAND_CENTI to indicate 1% duty cycle.
  // LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  // LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);

  // ... extra definitions for channels 2..n here
#elif defined(CFG_kr920)
  Serial.println("Region kr920");
  // Set up the channels used in your country. Three are defined by default,
  // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
  // BAND_MILLI.
  // LMIC_setupChannel(0, 922100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(1, 922300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(2, 922500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

  // ... extra definitions for channels 3..n here.
#elif defined(CFG_in866)
  Serial.println("Region in866");
  // Set up the channels used in your country. Three are defined by default,
  // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
  // BAND_MILLI.
  // LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

  // ... extra definitions for channels 3..n here.
#else
# error Region not supported
#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink
  LMIC_setDrTxpow(DR_SF7, 14);

  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
}

//Settings for LoRaWAN ABP
void initloraabp() {
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
  Serial.println("Region eu868");
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set. The LMIC doesn't let you change
  // the three basic settings, but we show them here.
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
#elif defined(CFG_us915) || defined(CFG_au915)
  Serial.println("Region us915/au915");
  // NA-US and AU channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);
#elif defined(CFG_as923)
  Serial.println("Region as923");
  // Set up the channels used in your country. Only two are defined by default,
  // and they cannot be changed.  Use BAND_CENTI to indicate 1% duty cycle.
  // LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  // LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);

  // ... extra definitions for channels 2..n here
#elif defined(CFG_kr920)
  Serial.println("Region kr920");
  // Set up the channels used in your country. Three are defined by default,
  // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
  // BAND_MILLI.
  // LMIC_setupChannel(0, 922100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(1, 922300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(2, 922500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

  // ... extra definitions for channels 3..n here.
#elif defined(CFG_in866)
  Serial.println("Region in866");
  // Set up the channels used in your country. Three are defined by default,
  // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
  // BAND_MILLI.
  // LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

  // ... extra definitions for channels 3..n here.
#else
# error Region not supported
#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink
  LMIC_setDrTxpow(DR_SF7, 14);
  otaa = 0;
  cnt = -1;
}

//Settings for LoRaWAN OTAA
void initloraotaa() {
  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  //LMIC specific parameters
  LMIC_setAdrMode(0);
  LMIC_setLinkCheckMode(0);
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  next = false;
  LMIC_startJoining();
  UISet(&UIInputbox_awnh87, "Joining");
  //  UISet(&UIInputbox_awnh87, "Joined");
  otaa = 1;
  cnt = -1;
}

//Send data using LoRaWAN
void sendobject(osjob_t* j) {
  bool result = false;

  int32_t lat = latitude * 10000;
  int32_t lon = longitude * 10000;
  int16_t altitude = alt * 100;
  int8_t hdopGPS = hdop / 10;

  coords[0] = lat;
  coords[1] = lat >> 8;
  coords[2] = lat >> 16;

  coords[3] = lon;
  coords[4] = lon >> 8;
  coords[5] = lon >> 16;

  coords[6] = altitude;
  coords[7] = altitude >> 8;

  coords[8] = hdopGPS;

  sentMillis = millis();

#ifndef M5gps
  if (iwm == 0) {
#else
  if (iwm == 0 && gps.location.isValid() == true && gps.location.age() < 2000) {
#endif

    //UISet(&UIInputbox_awnh87, "Sending");

    if (oldisf != isf) {
      if (isf == 0) {
        LMIC_setDrTxpow(DR_SF7, 14);
        oldisf = isf;
      } else if (isf == 1) {
        LMIC_setDrTxpow(DR_SF8, 14);
        oldisf = isf;
      } else if (isf == 2) {
        LMIC_setDrTxpow(DR_SF9, 14);
        oldisf = isf;
      } else if (isf == 3) {
        LMIC_setDrTxpow(DR_SF10, 14);
        oldisf = isf;
      } else if (isf == 4) {
        LMIC_setDrTxpow(DR_SF11, 14);
        oldisf = isf;
      } else if (isf == 5) {
        LMIC_setDrTxpow(DR_SF12, 14);
        oldisf = isf;
      }
    }

#ifdef M5gps
    if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println(F("OP_TXRXPEND, not sending"));
      UISet(&UIInputbox_awnh87, "TX Pending");
    } else {
      // Prepare upstream data transmission at the next possible time.
      next = false;
      LMIC_setTxData2(1, coords, sizeof(coords) , 0);
      Serial.println(F("Packet queued"));
      UISet(&UIInputbox_awnh87, "Queued");
    }
#else
    if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println(F("OP_TXRXPEND, not sending"));
      UISet(&UIInputbox_awnh87, "TX Pending");
    } else {
      // Prepare upstream data transmission at the next possible time.
      next = false;
      LMIC_setTxData2(1, ncoords, sizeof(ncoords) , 0);
      Serial.println(F("Packet queued"));
      UISet(&UIInputbox_awnh87, "Queued");
    }
#endif

#ifndef M5gps
  } else if ((iwm == 1) || (iwm == 2)) {
#else
  } else if (((iwm == 1) && gps.location.isValid() == true && gps.location.age() < 2000) || (iwm == 2)) {
#endif

    UISet(&UIInputbox_awnh87, "ACK");

    if (oldisf != isf) {
      if (isf == 0) {
        LMIC_setDrTxpow(DR_SF7, 14);
        oldisf = isf;
      } else if (isf == 1) {
        LMIC_setDrTxpow(DR_SF8, 14);
        oldisf = isf;
      } else if (isf == 2) {
        LMIC_setDrTxpow(DR_SF9, 14);
        oldisf = isf;
      } else if (isf == 3) {
        LMIC_setDrTxpow(DR_SF10, 14);
        oldisf = isf;
      } else if (isf == 4) {
        LMIC_setDrTxpow(DR_SF11, 14);
        oldisf = isf;
      } else if (isf == 5) {
        LMIC_setDrTxpow(DR_SF12, 14);
        oldisf = isf;
      }
    }

#ifdef M5gps
    if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println(F("OP_TXRXPEND, not sending"));
      UISet(&UIInputbox_awnh87, "TX Pending");
    } else {
      // Prepare upstream data transmission at the next possible time.
      next = false;
      LMIC_setTxData2(1, coords, sizeof(coords) , 1);
      LMIC.txCnt = TXCONF_ATTEMPTS;
      Serial.println(F("Packet queued"));
      UISet(&UIInputbox_awnh87, "Queued");
    }
#else
    if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println(F("OP_TXRXPEND, not sending"));
      UISet(&UIInputbox_awnh87, "TX Pending");
    } else {
      // Prepare upstream data transmission at the next possible time.
      next = false;
      LMIC_setTxData2(1, ncoords, sizeof(ncoords) , 1);
      LMIC.txCnt = TXCONF_ATTEMPTS;
      Serial.println(F("Packet queued"));
      UISet(&UIInputbox_awnh87, "Queued");
    }
#endif

  } else if (iwm == 3) {

    UISet(&UIInputbox_awnh87, "LCR");

    if (oldisf != isf) {
      if (isf == 0) {
        LMIC_setDrTxpow(DR_SF7, 14);
        oldisf = isf;
        cnt = -1;
      } else if (isf == 1) {
        LMIC_setDrTxpow(DR_SF8, 14);
        oldisf = isf;
        cnt = -1;
      } else if (isf == 2) {
        LMIC_setDrTxpow(DR_SF9, 14);
        oldisf = isf;
        cnt = -1;
      } else if (isf == 3) {
        LMIC_setDrTxpow(DR_SF10, 14);
        oldisf = isf;
        cnt = -1;
      } else if (isf == 4) {
        LMIC_setDrTxpow(DR_SF11, 14);
        oldisf = isf;
        cnt = -1;
      } else if (isf == 5) {
        LMIC_setDrTxpow(DR_SF12, 14);
        oldisf = isf;
        cnt = -1;
      }
    }

#ifdef M5gps
    if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println(F("OP_TXRXPEND, not sending"));
      UISet(&UIInputbox_awnh87, "Error");
    } else {
      // Prepare upstream data transmission at the next possible time.
      next = false;
      LMIC_setLinkCheckRequestOnce(1);
      LMIC_setTxData2(1, coords, sizeof(coords) , 1);
      LMIC.txCnt = TXCONF_ATTEMPTS;
      UISet(&UIInputbox_awnh87, "Queued");
      Serial.println(F("Packet queued"));
    }
#else
    if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println(F("OP_TXRXPEND, not sending"));
      UISet(&UIInputbox_awnh87, "Error");
    } else {
      // Prepare upstream data transmission at the next possible time.
      LMIC_setLinkCheckRequestOnce(1);
      next = false;
      LMIC_setTxData2(1, ncoords, sizeof(ncoords) , 1);
      LMIC.txCnt = TXCONF_ATTEMPTS;
      UISet(&UIInputbox_awnh87, "Queued");
      Serial.println(F("Packet queued"));
    }
#endif
  }
}

void sendobjectotaa(osjob_t* j) {

  bool result = false;

  int32_t lat = latitude * 10000;
  int32_t lon = longitude * 10000;
  int16_t altitude = alt * 100;
  int8_t hdopGPS = hdop / 10;

  coords[0] = lat;
  coords[1] = lat >> 8;
  coords[2] = lat >> 16;

  coords[3] = lon;
  coords[4] = lon >> 8;
  coords[5] = lon >> 16;

  coords[6] = altitude;
  coords[7] = altitude >> 8;

  coords[8] = hdopGPS;

  sentMillis = millis();

  UISet(&UIInputbox_awnh87, "Sending");

#ifdef M5gps
  if (otaaack == 0) {
    if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println(F("OP_TXRXPEND, not sending"));
      UISet(&UIInputbox_awnh87, "Error");
    } else {
      // Prepare upstream data transmission at the next possible time.
      next = false;
      LMIC_setTxData2(1, coords, sizeof(coords) , 0);
      LMIC.txCnt = TXCONF_ATTEMPTS;
      UISet(&UIInputbox_awnh87, "Queued");
      Serial.println(F("Packet queued"));
    }
  } else if (otaaack == 1) {
    if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println(F("OP_TXRXPEND, not sending"));
      UISet(&UIInputbox_awnh87, "Error");
    } else {
      // Prepare upstream data transmission at the next possible time.
      next = false;
      LMIC_setTxData2(1, coords, sizeof(coords) , 1);
      LMIC.txCnt = TXCONF_ATTEMPTS;
      UISet(&UIInputbox_awnh87, "Queued");
      Serial.println(F("Packet queued"));
    }
  }
#else
  if (otaaack == 0) {
    if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println(F("OP_TXRXPEND, not sending"));
      UISet(&UIInputbox_awnh87, "Error");
    } else {
      // Prepare upstream data transmission at the next possible time.
      next = false;
      LMIC_setTxData2(1, ncoords, sizeof(ncoords) , 0);
      UISet(&UIInputbox_awnh87, "Queued");
      Serial.println(F("Packet queued"));
    }
  } else if (otaaack == 1) {
    if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println(F("OP_TXRXPEND, not sending"));
      UISet(&UIInputbox_awnh87, "Error");
    } else {
      // Prepare upstream data transmission at the next possible time.
      next = false;
      LMIC_setTxData2(1, ncoords, sizeof(ncoords) , 1);
      LMIC.txCnt = TXCONF_ATTEMPTS;
      UISet(&UIInputbox_awnh87, "Queued");
      Serial.println(F("Packet queued"));
    }
  }
#endif
}

#ifdef M5gps
//SiteSurvey function
void ssv() {

  UISet(&UIInputbox_awnh87, "SSV running");
  ssvinit();

  bool result = false;
  ssvresult = "DR ";

  LMIC_setDrTxpow(DR_SF7, 14);
  isf = 0;
  LMIC_setLinkCheckRequestOnce(1);
  next = false;
  LMIC_setTxData2(1, ncoords, sizeof(ncoords) , 1);
  LMIC.txCnt = TXCONF_ATTEMPTS;
  smartDelay(5000);
  writessv();

  ssvresult += "5";

  LMIC.bands[BAND_MILLI].avail = os_getTime();
  LMIC.bands[BAND_CENTI].avail = os_getTime();
  LMIC.bands[BAND_DECI].avail = os_getTime();
  LMIC_setDrTxpow(DR_SF8, 14);
  isf = 1;
  LMIC_setLinkCheckRequestOnce(1);
  next = false;
  LMIC_setTxData2(1, ncoords, sizeof(ncoords) , 1);
  LMIC.txCnt = TXCONF_ATTEMPTS;
  smartDelay(5000);
  writessv();

  ssvresult += "4";

  LMIC.bands[BAND_MILLI].avail = os_getTime();
  LMIC.bands[BAND_CENTI].avail = os_getTime();
  LMIC.bands[BAND_DECI].avail = os_getTime();
  LMIC_setDrTxpow(DR_SF9, 14);
  isf = 2;
  LMIC_setLinkCheckRequestOnce(1);
  next = false;
  LMIC_setTxData2(1, ncoords, sizeof(ncoords) , 1);
  LMIC.txCnt = TXCONF_ATTEMPTS;
  smartDelay(5000);
  writessv();
  ssvresult += "3";

  LMIC.bands[BAND_MILLI].avail = os_getTime();
  LMIC.bands[BAND_CENTI].avail = os_getTime();
  LMIC.bands[BAND_DECI].avail = os_getTime();
  LMIC_setDrTxpow(DR_SF10, 14);
  isf = 3;
  LMIC_setLinkCheckRequestOnce(1);
  next = false;
  LMIC_setTxData2(1, ncoords, sizeof(ncoords) , 1);
  LMIC.txCnt = TXCONF_ATTEMPTS;
  smartDelay(5000);
  writessv();
  ssvresult += "2";

  LMIC.bands[BAND_MILLI].avail = os_getTime();
  LMIC.bands[BAND_CENTI].avail = os_getTime();
  LMIC.bands[BAND_DECI].avail = os_getTime();
  LMIC_setDrTxpow(DR_SF11, 14);
  isf = 4;
  LMIC_setLinkCheckRequestOnce(1);
  next = false;
  LMIC_setTxData2(1, ncoords, sizeof(ncoords) , 1);
  LMIC.txCnt = TXCONF_ATTEMPTS;
  smartDelay(5000);
  writessv();
  ssvresult += "1";

  LMIC.bands[BAND_MILLI].avail = os_getTime();
  LMIC.bands[BAND_CENTI].avail = os_getTime();
  LMIC.bands[BAND_DECI].avail = os_getTime();
  LMIC_setDrTxpow(DR_SF12, 14);
  isf = 5;
  LMIC_setLinkCheckRequestOnce(1);
  next = false;
  LMIC_setTxData2(1, ncoords, sizeof(ncoords) , 1);
  LMIC.txCnt = TXCONF_ATTEMPTS;
  smartDelay(5000);
  writessv();
  ssvresult += "0";

  LMIC.bands[BAND_MILLI].avail = os_getTime();
  LMIC.bands[BAND_CENTI].avail = os_getTime();
  LMIC.bands[BAND_DECI].avail = os_getTime();

  lastssv = true;
  writessv();
  lastssv = false;
  firstssv = false;

  UISet(&UIInputbox_awnh87, ssvresult);

  LMIC_setDrTxpow(DR_SF7, 14);
  isf = 0;
}

//Initialize GeoJSON file
void ssvinit() {
  if (cardin == true && gps.location.isValid() == true) {
    sprintf(filename2, "/%02d-%02d-%02d", day, month, year - 2000);
    sprintf(filepath2, "/%02d-%02d-%02d/%02d-%02d%s", day, month, year - 2000,  hour, minute, ".json");

    SD.mkdir(filename2);
    if (!SD.exists(filepath2)) {
      dataFile = SD.open(filepath2, FILE_WRITE);
      dataFile.close();
    }
  }
}

//Write data to GeoJSON file
void writessv() {
  if (gps.location.isValid() == true) {
    gpsdata();
    sprintf(date1, "%4d-%02d-%02dT%02d:%02d:%02dZ", year, month, day, hour, minute, second);

    dataFile = SD.open(filepath2, FILE_WRITE);
    unsigned long filesize = dataFile.size();
    dataFile.seek(filesize);

    if (lastssv == false) {
      if (firstssv == false) {
        firstssv = true;
        dataFile.println(F("{"));
        dataFile.println(F("\"type\": \"FeatureCollection\","));
        dataFile.println(F("\"features\": [{"));
      } else {
        dataFile.println(F(",{"));
      }
      dataFile.println(F("\"type\": \"Feature\","));
      dataFile.println(F("\"properties\": {"));
      dataFile.print(F("\"sf\": \""));
      dataFile.print(sf[isf]);
      dataFile.print(F("\",\r\n"));
      dataFile.print(F("\"rssi\": \""));
      dataFile.print(rssi);
      dataFile.print(F("\",\r\n"));
      dataFile.print(F("\"snr\": \""));
      dataFile.print(snr);
      dataFile.print(F("\",\r\n"));
      dataFile.print(F("\"gwcnt\": \""));
      dataFile.print(gwcnt);
      dataFile.print(F("\",\r\n"));
      dataFile.print(F("\"margin\": \""));
      dataFile.print(margin);
      dataFile.print(F("\",\r\n"));
      dataFile.println(F("\"marker-color\": \"#008800\","));
      dataFile.println(F("\"marker-symbol\": \"lighthouse\""));
      dataFile.println(F("},"));
      dataFile.println(F("\"geometry\": {"));
      dataFile.println(F("\"type\": \"Point\","));
      dataFile.print(F("\"coordinates\": ["));
      dataFile.print(longitude, 7);
      dataFile.print(F(", "));
      dataFile.print(latitude, 7);
      dataFile.print(F("]\r\n"));
      dataFile.println(F("}"));
      dataFile.println(F("}"));
      dataFile.close();
    } else {
      dataFile.println(F("]}"));
      dataFile.close();
    }
  }
}
#endif

// Manage LoRa communication
void lmictask( void * parameter ) {
  for (;;) {
    // Execute the LMIC scheduler
    os_runloop_once();

    // Let other tasks run
    vTaskDelay(1);
  }
}


//initial setup
void setup() {
  /* Prepare M5STACK */
  M5.begin();
  M5.Power.begin();
  Wire.begin();
#ifdef M5gps
  serialgps.begin(9600, SERIAL_8N1, 16, 17);
#endif
  M5.Lcd.setBrightness(50);
  //M5.Lcd.drawBitmap(0, 0, 320, 240, (uint16_t *)imgName);
  M5.Lcd.drawBitmap(0, 0, 320, 240, (uint16_t *)gImage_logoM5);
  initlora();

  xTaskCreatePinnedToCore(
    lmictask,
    "LMIC-Task",
    2048,
    NULL,
    1,
    &TaskLMIC,
    1);

#ifdef M5gps
  xTaskCreatePinnedToCore(
    gpsupdate,
    "TaskGPS",
    10000,
    NULL,
    1,
    &TaskGPS,
    0);
#endif

#ifdef M5go
  strip.Begin();
  strip.Show();

  if (powersave == false) {
    strip.SetBrightness(50);
  } else {
    strip.SetBrightness(0);
  }

  xTaskCreatePinnedToCore(
    pixelupdate,
    "TaskPixel",
    10000,
    NULL,
    1,
    &TaskPixel,
    0);
#endif

  /* Prepare UI */
  UIBegin();
  LayerFunction_default(0);

#ifdef M5gps
  M5.Lcd.drawBitmap(5, 2, 24, 24, (uint16_t *)ICON_10_24);
  M5.Lcd.drawBitmap(65, 5, 24, 24, (uint16_t *)ICON_23_24);
#endif

  if (SD.exists(filename)) {
    M5.Lcd.drawBitmap(200, 5, 24, 24, (uint16_t *)ICON_22_24);
    cardin = true;
  }

  //Prepare UI for iwm = 0
  UISet(&UITextbox_vimqus, sf[isf]);
  UIDisable(true, &UIProgressbar_eymzer);
  UIDisable(true, &UITextbox_859t1hi);
  UIDisable(true, &UITextbox_olwwlae);
  UIDisable(true, &UIInputbox_6nssds);
  UIDisable(true, &UITextbox_7mnuudb);
  UIDisable(false, &UIInputbox_awnh87);

#ifndef M5gps
  UIDisable(true, &UITextbox_4t0l0bn);
  UIDisable(true, &UITextbox_q7sl3uo);
#endif

  Serial.println("Started");

  if (powersave == true) {
    smartDelay(1000);
#ifdef M5gps
    gpsdata();
#endif
    sendobject(&sendjob);
    esp_sleep_enable_timer_wakeup(interval[iiv] * 1000);
    esp_deep_sleep_start();
  }
  sendobject(&sendjob);
}

void loop() {

  //update button status
  if (M5.BtnA.wasPressed()) {
    if (iwm == 6) {
      iwm = 0;
      UISet(&UITextbox_eq79hh46, workmode[iwm]);
      if (otaa == 1) {
        initloraabp();
      }
    } else if (iwm == 3) {
      iwm++;
#ifndef M5gps
      iwm++;
#endif
      UISet(&UITextbox_eq79hh46, workmode[iwm]);
    } else {
      iwm++;
      UISet(&UITextbox_eq79hh46, workmode[iwm]);
    }

    if (iwm == 0) {
      UISet(&UITextbox_vimqus, sf[isf]);
      UIDisable(false, &UIInputbox_awnh87);
      UISet(&UITextbox_67ofwdh, "Dim");
    } else if (iwm == 1) {
      UIDisable(false, &UIProgressbar_eymzer);
      UIDisable(false, &UITextbox_859t1hi);
      UIDisable(false, &UITextbox_olwwlae);
      UIDisable(false, &UITextbox_7mnuudb);
      UISet(&UITextbox_859t1hi, "-130");
      UISet(&UITextbox_olwwlae, "-20.0");
      UISet(&UIProgressbar_eymzer, 0);
    } else if (iwm == 2) {
      UISet(&UITextbox_67ofwdh, "Send");
      UISet(&UITextbox_859t1hi, "-130");
      UISet(&UITextbox_olwwlae, "-20.0");
      UISet(&UIProgressbar_eymzer, 0);
    } else if (iwm == 3) {
      UIDisable(false, &UIInputbox_6nssds);
      UISet(&UITextbox_859t1hi, "-130");
      UISet(&UITextbox_olwwlae, "-20.0");
      UISet(&UIProgressbar_eymzer, 0);
      UISet(&UIInputbox_6nssds, "");
    } else if (iwm == 4) {
      LMIC.nGws = 0;
      LMIC.gwMargin = 0;
      UIDisable(true, &UIProgressbar_eymzer);
      UIDisable(true, &UITextbox_859t1hi);
      UIDisable(true, &UITextbox_olwwlae);
      UIDisable(true, &UIInputbox_6nssds);
      UIDisable(true, &UITextbox_7mnuudb);
    } else if (iwm == 5) {
#ifndef M5gps
      UIDisable(true, &UIInputbox_6nssds);
      UIDisable(true, &UITextbox_7mnuudb);
#endif
      UISet(&UITextbox_vimqus, "Join");
      UISet(&UITextbox_67ofwdh, "NACK");
      UIDisable(false, &UIProgressbar_eymzer);
      UIDisable(false, &UITextbox_859t1hi);
      UIDisable(false, &UITextbox_olwwlae);
      UISet(&UITextbox_859t1hi, "-130");
      UISet(&UITextbox_olwwlae, "-20.0");
      UISet(&UIProgressbar_eymzer, 0);
      //strip.SetPixelColor(7, off);
    } else if (iwm == 6) {
      UISet(&UITextbox_vimqus, ttext[iiv]);
      UIDisable(true, &UIInputbox_awnh87);
      UIDisable(true, &UIProgressbar_eymzer);
      UIDisable(true, &UITextbox_859t1hi);
      UIDisable(true, &UITextbox_olwwlae);
      UISet(&UITextbox_67ofwdh, "PS");
    }
  }

  if (M5.BtnB.wasPressed()) {
    if (isf == 5) {
      isf = 0;
      UISet(&UITextbox_vimqus, sf[isf]);
    } else if (iwm == 5 && otaa == 0) {
      initloraotaa(); //OTAA Join
      UISet(&UITextbox_vimqus, "Send");
    } else if (iwm == 5 && otaa == 1) {
      next = false;
      sendobjectotaa(&sendjob); //Manual send
    } else if (iwm == 6) {
      if (iiv == 4) {
        iiv = 0;
        UISet(&UITextbox_vimqus, ttext[iiv]);
      } else {
        iiv++;
        UISet(&UITextbox_vimqus, ttext[iiv]);
      }
    } else {
      isf++;
      UISet(&UITextbox_vimqus, sf[isf]);
    }
  }

  if (M5.BtnC.wasPressed()) {
    if (iwm < 2 && dim == false) {
      dim = true;
      M5.Lcd.setBrightness(0);
#ifdef M5go
      strip.SetBrightness(0);
#endif
    } else if (iwm < 2 && dim == true) {
      dim = false;
      M5.Lcd.setBrightness(50);
#ifdef M5go
      strip.SetBrightness(50);
#endif
    } else if (iwm == 4) {
#ifdef M5gps
      ssv();
#endif
    } else if (iwm == 5 && otaaack == 0) {
      otaaack = 1;
      UISet(&UITextbox_67ofwdh, "ACK");
    } else if (iwm == 5 && otaaack == 1) {
      otaaack = 0;
      UISet(&UITextbox_67ofwdh, "NACK");
    } else if (iwm == 6 && powersave == false) {
      powersave = true;
      iwm = 0;
    } else if (iwm == 6 && powersave == true) {
      powersave = false;
    } else if (iwm > 1) {
      sendobject(&sendjob);
    }
  }

#ifdef M5gps
  //Update GPS Data
  gpsdata();
#endif

#ifdef M5gps
  //Print satellites and change NeoPixel 4
  sats = gps.satellites.value();
  UISet(&UITextbox_4t0l0bn, sats);

  //Print HDOP and change NeoPixel 0
  hdop = gps.hdop.value();
  hdop2 = hdop / 100.0;
  String stringhdop = String(hdop2);
  UISet(&UITextbox_q7sl3uo, stringhdop);

  //Print GPS fix status und change NeoPixel 2
  if (gps.location.isValid() == false) {
    M5.Lcd.drawBitmap(160, 5, 24, 24, (uint16_t *)ICON_25_24);
  }
  else if (gps.location.isValid() && gps.location.age() > 2000) {
    M5.Lcd.drawBitmap(160, 5, 24, 24, (uint16_t *)ICON_25_24);
  }
  else if (gps.location.isValid() == true) {
    M5.Lcd.drawBitmap(160, 5, 24, 24, (uint16_t *)ICON_20_24);
  }
  else {
    M5.Lcd.drawBitmap(160, 5, 24, 24, (uint16_t *)ICON_20_24);
  }
#endif

  //Battery Status
  if (M5.Power.isCharging() == true) {
    M5.Lcd.drawBitmap(240, 5, 24, 24, (uint16_t *)ICON_40_24);
  }

  if (M5.Power.isChargeFull() == true) {
    UISet(&UITextbox_403ohip, "Full");
  }
  else {
    BattLevel = M5.Power.getBatteryLevel();
    String strbattlevel = String(BattLevel);
    strbattlevel = String(strbattlevel + "%");
    UISet(&UITextbox_403ohip, strbattlevel);
  }

#ifdef M5go
  strip.Show();
#endif

#ifdef M5gps
  //Init of SD Card for GPX-file
  if (sdwrite == false) {
    gpxinit();
  }

  //Write GPS-Track
  if (sdwrite == true) {
    writegpx();
  }
#endif

  if (next == false) {

  } else {

    //Sending intervall
    currentMillis = millis();
    if ((currentMillis - sentMillis > interval[iiv]) && iwm < 2) {
      sendobject(&sendjob);
    }

#ifdef M5gps
    if ((currentMillis - sentMillis > interval[iiv]) && iwm == 5 && otaa == 1 && gps.location.isValid() == true && gps.location.age() < 2000) {
      sendobjectotaa(&sendjob);
    }
#else
    if ((currentMillis - sentMillis > interval[iiv]) && iwm == 5 && otaa == 1) {
      sendobjectotaa(&sendjob);
    }
#endif
  }
  //sleep timer
  if (powersave == true) {
#ifdef M5go
    strip.SetBrightness(0);
#endif
    esp_sleep_enable_timer_wakeup(interval[iiv] * 1000);
    esp_deep_sleep_start();
  }

  //used to deflicker the display, more possible, but with less reactive buttons
  smartDelay(200);

  M5.update();

}
