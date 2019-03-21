

const String Version = "RA_03";
//#define USE_DHT
/*
  2018.07.12 add Wifilocation to indoor location service
			TODO: GPS needs some debugging
				offline datasaving
				time from GPS
* */
//wifimanager
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include "WiFiManager.h"          //https://github.com/tzapu/WiFiManager

//DHT a d3on
//A3-jel,A2-5v

//gcript:
#include "HTTPSRedirect.h"
#include "DebugMacros.h"

// for stack analytics
extern "C" {
#include <cont.h>
  extern cont_t g_cont;
}

const char* host = "script.google.com";
// Replace with your own script id to make server side changes
const char *GScriptId = "FFFFFFFFFFFFFFF";

const int httpsPort = 443;

// echo | openssl s_client -connect script.google.com:443 |& openssl x509 -fingerprint -noout
const char* fingerprint = "";

// Write to Google Spreadsheet
const char *sheet__name = "nyers adatok";                                                                  //a céltábla neve pontosan(lokalizációra figyelni kell)
String url = String("/macros/s/") + GScriptId + "/exec?value=" + Version;
// Fetch Google Calendar events for 1 week ahead
String url2 = String("/macros/s/") + GScriptId + "/exec?cal";
// Read from Google Spreadsheet
String olvasando_cella = "A1";
String url3 = String("/macros/s/") + GScriptId + "/exec?read=";

String payload_base = String("{\"command\": \"appendRow\", \  \"sheet_name\": \"") + sheet__name + "\", \ \"values\": ";
String payload = "";

const char *debug_sheet__name = "debug adatok";
String debug_payload_base = String("{\"command\": \"appendRow\", \  \"sheet_name\": \"") + debug_sheet__name + "\", \ \"values\": ";
String debug_payload = "";


#include "twi.h"
#include <ProcessScheduler.h>
Scheduler sched;
#include <Wire.h>
//scl-d1
//sda-d2
#include <TimeLib.h>



#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads0(0x48); //sensoroknál levő jószág


#define mintavetelezes 1

#include "SparkFunBME280.h"
BME280 myBMESensor;

float  MQ135akt;
float  TEMPakt, HUMakt, BMPakt;


float  MQ135integralt;
float  TEMPintegralt, HUMintegralt, BMPintegralt;

int MQ135db;
int  TEMPdb, HUMdb, BMPdb;

float  MQ135atlag;
float  TEMPatlag, HUMatlag, BMPatlag;

float readVcc() {

  int16_t reading_integration = 0;

  for (int i = 0; i < mintavetelezes; i++) {
    reading_integration = reading_integration + ads0.readADC_SingleEnded(2);
    delay(5);
  }
  reading_integration = reading_integration / mintavetelezes;
  float Vin = reading_integration * 0.0001875;
  // Vin = Vin - 0.02;
  if (Vin < 4.0) {
    Serial.print("i2c-reset:"); Serial.println(I2C_ClearBus());
    Serial.print("alacsony merofeszültseg!"); Serial.println(Vin);
  }
  return Vin;

}

/////////////////////////////////////////////////////////////

//udp
#include <ESP8266WiFi.h>

#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti wifiMulti;

//time udp szerverről
#include <WiFiUdp.h>
WiFiUDP udp;

IPAddress timeServerIP; // time.nist.gov NTP server address
const char* ntpServerName = "time.nist.gov";
unsigned int localPort = 2390;      // local port to listen for UDP packets

//////////////////
//httpupdate:
#include <ESP8266httpUpdate.h>
#include <ESP8266HTTPClient.h>

//gps and location
#include <TinyGPS++.h>
TinyGPSPlus gps;
#include <SoftwareSerial.h>
SoftwareSerial ss(D6, D7, false);
#define A7_enable D8
#define A7_pwr_key D5

double lat, lon;

#include <WifiLocation.h>
const char* googleApiKey = "FFFFFF";
WifiLocation location(googleApiKey);

////////////////////////*********************************//////////////////////////////////
/**
  This routine turns off the I2C bus and clears it
  on return SCA and SCL pins are tri-state inputs.
  You need to call Wire.begin() after this to re-enable I2C
  This routine does NOT use the Wire library at all.

  returns 0 if bus cleared
  1 if SCL held low.
  2 if SDA held low by slave clock stretch for > 2sec
  3 if SDA held low after 20 clocks.
*/

int I2C_ClearBus() {
#if defined(TWCR) && defined(TWEN)
  TWCR &= ~(_BV(TWEN)); //Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
#endif

  pinMode(SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(SCL, INPUT_PULLUP);

  // delay(2500);  // Wait 2.5 secs. This is strictly only necessary on the first power
  // up of the DS3231 module to allow it to initialize properly,
  // but is also assists in reliable programming of FioV3 boards as it gives the
  // IDE a chance to start uploaded the program
  // before existing sketch confuses the IDE by sending Serial data.

  boolean SCL_LOW = (digitalRead(SCL) == LOW); // Check is SCL is Low.
  if (SCL_LOW) { //If it is held low Arduno cannot become the I2C master.
    return 1; //I2C bus error. Could not clear SCL clock line held low
  }

  boolean SDA_LOW = (digitalRead(SDA) == LOW);  // vi. Check SDA input.
  int clockCount = 20; // > 2x9 clock

  while (SDA_LOW && (clockCount > 0)) { //  vii. If SDA is Low,
    clockCount--;
    // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(SCL, INPUT); // release SCL pullup so that when made output it will be LOW
    pinMode(SCL, OUTPUT); // then clock SCL Low
    delayMicroseconds(10); //  for >5uS
    pinMode(SCL, INPUT); // release SCL LOW
    pinMode(SCL, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5uS
    // The >5uS is so that even the slowest I2C devices are handled.
    SCL_LOW = (digitalRead(SCL) == LOW); // Check if SCL is Low.
    int counter = 20;
    while (SCL_LOW && (counter > 0)) {  //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      SCL_LOW = (digitalRead(SCL) == LOW);
    }
    if (SCL_LOW) { // still low after 2 sec error
      return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    SDA_LOW = (digitalRead(SDA) == LOW); //   and check SDA input again and loop
  }
  if (SDA_LOW) { // still low
    return 3; // I2C bus error. Could not clear. SDA data line held low
  }

  // else pull SDA line low for Start or Repeated Start
  pinMode(SDA, INPUT); // remove pullup.
  pinMode(SDA, OUTPUT);  // and then make it LOW i.e. send an I2C Start or Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
  delayMicroseconds(10); // wait >5uS
  pinMode(SDA, INPUT); // remove output low
  pinMode(SDA, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10); // x. wait >5uS
  pinMode(SDA, INPUT); // and reset pins as tri-state inputs which is the default state on reset
  pinMode(SCL, INPUT);
  return 0; // all ok
}




/////////////// time beállítása bármilyen rendelkezésre álló forrásból naponta
class TgetProcess : public Process
{
  public:
    // Call the Process constructor
    TgetProcess(Scheduler &manager, ProcPriority pr, unsigned int period)
      : Process(manager, pr, period)
    {

    }

  protected:


    //LEDs should be off when disabled
    virtual void onDisable()
    {
      digitalClockDisplay();
    }

    //Start the LEDs on
    virtual void onEnable()
    {
      udp.begin(localPort);

    }

    // Create our service routine
    virtual void service()
    {
      int i;
      Serial.print(F("______timeget"));
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        WiFi.hostByName(ntpServerName, timeServerIP);

        sendNTPpacket(timeServerIP); // send an NTP packet to a time server
        // wait to see if a reply is available
        delay(1000);

        int cb = udp.parsePacket();
        if (!cb) {
          Serial.print(F("no packet yet: "));
          i++;
          Serial.println(i);
          if (i > 25) {
            Serial.println(F("reboot"));
            ESP.deepSleep(1);
          }
        }
        else {
          Serial.print(F("packet received, length="));
          Serial.println(cb);
          // We've received a packet, read the data from it
          udp.read(packetBuffer, 48); // read the packet into the buffer

          //the timestamp starts at byte 40 of the received packet and is four bytes,
          // or two words, long. First, esxtract the two words:

          unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
          unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
          // combine the four bytes (two words) into a long integer
          // this is NTP time (seconds since Jan 1 1900):
          unsigned long secsSince1900 = highWord << 16 | lowWord;
          //   Serial.print("Seconds since Jan 1 1900 = " );
          //  Serial.println(secsSince1900);

          // now convert NTP time into everyday time:

          // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
          const unsigned long seventyYears = 2208988800UL;
          // subtract seventy years:
          unsigned long epoch = secsSince1900 - seventyYears + 3600; //utc+1
          // print Unix time:
          //  Serial.print("Unix time = ");Serial.println(epoch);
          if (epoch >= 1509354020) { // check the integer is a valid time (greater than 217,10,30)
            setTime(epoch); // Sync Arduino clock to the time received on the serial port
          }
        }
      }
      else {
        while (wifiMulti.run() != WL_CONNECTED) {
          q++;
          Serial.print(".");
          yield(); delay(1000);

          if (q > 30) {
            WiFi.disconnect(true);
            delay(2000);
            ESP.deepSleep(1);
            break;

          }
        }

      }
      if (Serial.available()) {
        processSyncMessage();
      }


    }

  private:
    int q;
    byte packetBuffer[48]; //buffer to hold incoming and outgoing packets
    void processSyncMessage() {
      unsigned long pctime;

      if (Serial.find("T")) {
        pctime = Serial.parseInt();
        if (pctime >= 1509354020) { // check the integer is a valid time (greater than 217,10,30)
          setTime(pctime); // Sync Arduino clock to the time received on the serial port
        }
      }
    }
    void digitalClockDisplay() {
      // digital clock display of the time
      Serial.print(hour());
      printDigits(minute());
      printDigits(second());
      Serial.print(" ");
      Serial.print(day());
      Serial.print(" ");
      Serial.print(month());
      Serial.print(" ");
      Serial.print(year());
      Serial.println();
    }

    void printDigits(int digits) {
      // utility function for digital clock display: prints preceding colon and leading 0
      Serial.print(":");
      if (digits < 10)
        Serial.print('0');
      Serial.print(digits);
    }
    unsigned long sendNTPpacket(IPAddress& address)
    {
      Serial.println(F("sending NTP packet..."));
      // set all bytes in the buffer to 0
      memset(packetBuffer, 0, 48);
      // Initialize values needed to form NTP request
      // (see URL above for details on the packets)
      packetBuffer[0] = 0b11100011;   // LI, Version, Mode
      packetBuffer[1] = 0;     // Stratum, or type of clock
      packetBuffer[2] = 6;     // Polling Interval
      packetBuffer[3] = 0xEC;  // Peer Clock Precision
      // 8 bytes of zero for Root Delay & Root Dispersion
      packetBuffer[12] = 49;
      packetBuffer[13] = 0x4E;
      packetBuffer[14] = 49;
      packetBuffer[15] = 52;

      // all NTP fields have been given values, now
      // you can send a packet requesting a timestamp:
      udp.beginPacket(address, 123); //NTP requests are to port 123
      udp.write(packetBuffer, 48);
      udp.endPacket();
    }

};
TgetProcess Tget(sched, HIGH_PRIORITY, 900);



///////////////////////////************************////////////////////////////////
class MQ135Process : public Process
{
  public:
    // Call the Process constructor
    MQ135Process(Scheduler &manager, ProcPriority pr, unsigned int period)
      : Process(manager, pr, period)
    {

    }

  protected:
    //setup the pins
    virtual void setup()
    {


    }

    // Undo setup()
    virtual void cleanup()
    {

    }


    virtual void onDisable()
    {
      //  MQ135file.close();
    }


    virtual void onEnable()
    {
    }

    // Create our service routine
    virtual void service()
    {
      ads0.begin();
      //  MQ135file.print(now());
      //  MQ135file.print(F(";"));

      int reading_integration = 0;
      int reading = 0;
      int timeout = 5;
      for (int i = 0; i < mintavetelezes; i++) {
        //////////////////////////////////////////////////////////
        reading = ads0.readADC_SingleEnded(3);
        //////////////////////////////////////////////////////////
        float Vproba = reading * 0.0001875;
        if (Vproba<7 & Vproba>0) {
          reading_integration = reading_integration + reading;
        }
        else {
          i--;
          timeout--;
          Serial.println(F("MQ135 dropout:")); Serial.println(Vproba);
          debug_payload = debug_payload + ";" + "MQ135 dropout:" + String(Vproba);
        }
        delay(5);
        if (timeout < 0) break;
      }
      reading_integration = reading_integration / mintavetelezes;

      //   Serial.println("MQ135:");
      //Serial.print("nyers: read_"); Serial.print(reading_integration);Serial.print(" sens5V_");  Serial.print(reading_Vcc);

      //  MQ135file.print(reading_integration);
      //  MQ135file.print(F(";"));
      float Vin = reading_integration * 0.0001875;
      float sens_5V = readVcc();
      Vin = Vin * (5 / sens_5V);
      //Serial.print("   feldolgozva: read_"); Serial.print(Vin);Serial.print(" sens5V_");  Serial.println( sens_5V);
      //   MQ135file.print(Vin);
      //   MQ135file.print(F(";"));

      //  Vin = Vin + (-0.00002 * pow(HUMakt, 3) + 0.0024 * pow(HUMakt, 2) - 0.0918 * HUMakt + 1.0681);

      MQ135akt = Vin;

      MQ135db = MQ135db + 1;
      MQ135integralt = MQ135integralt + MQ135akt;




      //   Serial.println(MQ135akt);
      /*  MQ135file.print(MQ135akt);

        MQ135file.println();
        MQ135file.sync();*/


    }
  private:



};
MQ135Process MQ135(sched, HIGH_PRIORITY, 2000);


///////////////////////////************************////////////////////////////////
class GPSProcess : public Process
{
  public:
    // Call the Process constructor
    GPSProcess(Scheduler &manager, ProcPriority pr, unsigned int period)
      : Process(manager, pr, period)
    {

    }

  protected:
    //setup the pins
    virtual void setup()
    {


    }

    // Undo setup()
    virtual void cleanup()
    {

    }


    virtual void onDisable()
    {

    }


    virtual void onEnable()
    {

      ss.begin(115200);
      pinMode(A7_enable, OUTPUT);
      pinMode(A7_pwr_key, OUTPUT);
      digitalWrite(A7_enable, LOW);
      delay(1000);
      digitalWrite(A7_enable, HIGH);
      delay(500);
      digitalWrite(A7_pwr_key, HIGH);
      delay(1000);
      digitalWrite(A7_pwr_key, LOW);

      for (int i = 0; i < 3; i++) {
        ss.println("AT+GPS=1");
        delay(1000);
        ss.println("AT+GPSRD=1");
        delay(1000);
        while (ss.available() > 0) {
          Serial.write(ss.read());
        }
      }
    }

    // Create our service routine
    virtual void service()
    {
      ss.readStringUntil(':');
      while (ss.available() > 0) { // This sketch displays information every time a new sentence is correctly encoded.

        gps.encode(ss.read());
      }
    }
  private:



};
GPSProcess GPSproc(sched, HIGH_PRIORITY, 500);

///////////////////////////************************////////////////////////////////
class  WIFILOCProcess : public Process
{
  public:
    // Call the Process constructor
    WIFILOCProcess(Scheduler &manager, ProcPriority pr, unsigned int period)
      : Process(manager, pr, period)
    {

    }

  protected:
    //setup the pins
    virtual void setup()
    {


    }

    // Undo setup()
    virtual void cleanup()
    {

    }


    virtual void onDisable()
    {

    }


    virtual void onEnable()
    {

    }

    // Create our service routine
    virtual void service()
    {
      if (gps.location.isValid())
      {

      }
      else
      {
        location_t loc = location.getGeoFromWiFi();

        Serial.println("Location request data");
        Serial.println(location.getSurroundingWiFiJson());
        Serial.println("Latitude: " + String(loc.lat, 7));
        Serial.println("Longitude: " + String(loc.lon, 7));
        Serial.println("Accuracy: " + String(loc.accuracy));
        lat = loc.lat;
        lon = loc.lon;

        Serial.printf("Free heap: %u\n", ESP.getFreeHeap());
        yield();
        tcpCleanup();
        Serial.printf("Free heap: %u\n", ESP.getFreeHeap());

      }
    }
  private:



};
WIFILOCProcess WIFILOCproc(sched, HIGH_PRIORITY, 30 * 1000);
///////////////////////////************************////////////////////////////////
#ifdef USE_DHT
#include "DHT.h"
#define DHTPIN D3
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
#endif
class BMPProcess : public Process
{
  public:
    // Call the Process constructor
    BMPProcess(Scheduler &manager, ProcPriority pr, unsigned int period)
      : Process(manager, pr, period)
    {

    }

  protected:
    //setup the pins
    virtual void setup()
    {


    }

    // Undo setup()
    virtual void cleanup()
    {

    }


    virtual void onDisable()
    {

    }


    virtual void onEnable()
    {
    }


    virtual void service()
    {
      Wire.begin();
     myBMESensor.setI2CAddress(0x76);
     myBMESensor.setMode(MODE_NORMAL); //lehet csomó mindent állítani, majd optimálásnál rá köll nézni!!!
      if (myBMESensor.beginI2C(Wire)) {

        TEMPakt = myBMESensor.readTempC();
        BMPakt = myBMESensor.readFloatPressure();
        HUMakt= myBMESensor.readFloatHumidity();
        if (TEMPakt > -50.0) {

          BMPdb = BMPdb + 1;
          TEMPintegralt = TEMPintegralt + TEMPakt;

          TEMPdb = TEMPdb + 1;
          BMPintegralt = BMPintegralt + BMPakt;

              HUMintegralt = HUMintegralt + HUMakt;
        HUMdb = HUMdb + 1;

        }
        else {
          debug_payload = debug_payload + ";" + "BMP read error: " + String(BMPakt);
          Serial.print(F("BMP read error:"));
          // Serial.print(F(" TEMPakt: "));
          //   Serial.print(TEMPakt);
          //   Serial.print(F(" BMPakt: "));
          //  Serial.println(BMPakt);
        }

      }
      else {
        Serial.println(F("BMP not working?"));
        debug_payload = debug_payload + ";" + "BMP not working! i2c scan:";
        for (byte i = 1; i < 120; i++)
        {
          Wire.beginTransmission(i);
          if (Wire.endTransmission() == 0)
          {
            Serial.print("Found address: ");
            Serial.print(i, DEC);
            Serial.print(" (0x");
            Serial.print(i, HEX);
            Serial.println(")");
            delay(1);
            debug_payload = debug_payload + " 0x" + String(i, HEX);
          }
        }
      }

#ifdef USE_DHT

      dht.begin();


      float t = dht.readTemperature();
      TEMPakt = dht.readTemperature();
      HUMakt = dht.readHumidity();
      if (TEMPakt > -50.0) {
        TEMPintegralt = TEMPintegralt + TEMPakt;
        TEMPdb = TEMPdb + 1;

        HUMintegralt = HUMintegralt + HUMakt;
        HUMdb = HUMdb + 1;
      }
      else {
        debug_payload = debug_payload + ";" + "DHT error: " + String(TEMPakt);
        Serial.print(F("DHT read error:"));
        // Serial.print(F(" TEMPakt: "));
        //   Serial.print(TEMPakt);
        //   Serial.print(F(" BMPakt: "));
        //  Serial.println(BMPakt);
      }
#endif
    }
  private:


};
BMPProcess BMPproc(sched, HIGH_PRIORITY, 2000);

///////////////////////////************************////////////////////////////////
class LOGFILEProcess : public Process
{
  public:
    // Call the Process constructor
    LOGFILEProcess(Scheduler &manager, ProcPriority pr, unsigned int period)
      : Process(manager, pr, period)
    {

    }

  protected:
    //setup the pins
    virtual void setup()
    {


    }

    // Undo setup()
    virtual void cleanup()
    {

    }

    virtual void onDisable()
    {

    }

    virtual void onEnable()
    {

    }

    // Create our service routine
    virtual void service() {

      //készítsük el az átlagokat


      MQ135atlag = MQ135integralt / MQ135db;
      MQ135integralt = 0;
      MQ135db = 0;

      TEMPatlag = TEMPintegralt / TEMPdb;
      TEMPintegralt = 0;
      TEMPdb = 0;

      HUMatlag = HUMintegralt / HUMdb;
      HUMintegralt = 0;
      HUMdb = 0;

      BMPatlag = BMPintegralt / BMPdb;
      BMPintegralt = 0;
      BMPdb = 0;



      //mentsük az átlagokat
      if (gps.location.isValid())
      {
        lat = gps.location.lat();
        lon = gps.location.lng();
      }
      else
      {
       /* GPSproc.disable();
        debug_payload = debug_payload + ";" + " Invalid GPS data";
        GPSproc.enable();*/
      }

      Serial.print("T:");
      Serial.print(now());
      Serial.print(" _ ");
      Serial.print(hour());
      Serial.print(":");
      Serial.print(minute());
      Serial.print(" lat:");
      Serial.print(lat);
      Serial.print(" lon:");
      Serial.print(lon);
      Serial.print(" MQ135:");
      Serial.print(MQ135atlag);
      Serial.print(" BMP:");
      Serial.print(BMPatlag);
      Serial.print(" TEMP:");
      Serial.print(TEMPatlag);
      Serial.print(" HUM:");
      Serial.print(HUMatlag);

      Serial.println();


      //***feltöltjük a gsheetre***
      //minden alkalommal újranyitjuk a kapcsolatot majd töröljük a cuccokat mert különben teleszemeteli a ramot
      // Use HTTPSRedirect class to create a new TLS connection
      Serial.println("POST to spreadsheet:");
      HTTPSRedirect* clientHTTPS = nullptr;
      clientHTTPS = new HTTPSRedirect(httpsPort);
      clientHTTPS->setPrintResponseBody(true);
      clientHTTPS->setContentTypeHeader("application/json");

      clientHTTPS->connect(host, httpsPort);
      String datestring = String(year()) + "." + String(month()) + "." + String(day());
      String timestring = String(hour() + 1) + ":" + String(minute()) + ":" + String(second());
      String datastring = String(MQ135atlag) + ";" + String(TEMPatlag) + ";" + String(HUMatlag) + ";" + String(BMPatlag) + ";" + String(readVcc()) + ";" + String(lat, 6) + ";" + String(lon, 6);
      datastring.replace(".", ",");
      datastring.replace("nan", "-99");

      payload = payload_base + "\"" + now() + ";" + datestring + " " + timestring + ";" + datastring + "\"}";
      Serial.println(payload);
      clientHTTPS->POST(url2, host, payload);
      payload = "";

      if (debug_payload != "") {
        debug_payload = debug_payload_base + "\"" + now() + ";" + datestring + " " + timestring + ";" + String(ESP.getFreeHeap()) + ";" + debug_payload + "\"}";
        Serial.println(debug_payload);
        clientHTTPS->POST(url2, host, debug_payload);
      }
      debug_payload = "";
      // delete HTTPSRedirect object


      clientHTTPS->stopAll();
      delete clientHTTPS;
      clientHTTPS = nullptr;
      //Serial.printf("Free heap: %u\n", ESP.getFreeHeap());
      yield();
      tcpCleanup();
      //Serial.printf("Free heap: %u\n", ESP.getFreeHeap());

    }
  private:

};
LOGFILEProcess LOGFILE(sched, HIGH_PRIORITY, 15000);



///////////////////////////************************////////////////////////////////

struct tcp_pcb;
extern struct tcp_pcb* tcp_tw_pcbs;
extern "C" void tcp_abort(struct tcp_pcb* pcb);

void tcpCleanup()
{
  while (tcp_tw_pcbs != NULL)
  {
    tcp_abort(tcp_tw_pcbs);
  }
}
String updateFingerprint = "";
String updateUrl = "";
uint8_t fp[20] = { 0x42, 0xe9, 0xf5, 0xf9, 0x30, 0x21, 0x14, 0xcd, 0x75, 0xa1, 0x41, 0xef, 0x39, 0x33, 0xe8, 0xd4, 0xc7, 0x97, 0xb9, 0x7c };



class UPDATEProcess : public Process
{
  public:
    // Call the Process constructor
    UPDATEProcess(Scheduler &manager, ProcPriority pr, unsigned int period)
      : Process(manager, pr, period)
    {

    }

  protected:
    //setup the pins
    virtual void setup()
    {


    }

    // Undo setup()
    virtual void cleanup()
    {

    }

    virtual void onDisable()
    {

    }

    virtual void onEnable()
    {

    }

    // Create our service routine
    virtual void service() {


      //***feltöltjük a gsheetre***
      //minden alkalommal újranyitjuk a kapcsolatot majd töröljük a cuccokat mert különben teleszemeteli a ramot
      // Use HTTPSRedirect class to create a new TLS connection

      Serial.println("UPDATE check");
      HTTPSRedirect* clientHTTPS = nullptr;
      clientHTTPS = new HTTPSRedirect(httpsPort);
      clientHTTPS->setPrintResponseBody(true);
      clientHTTPS->setContentTypeHeader("application/json");

      clientHTTPS->connect(host, httpsPort);


      clientHTTPS->GET(url, host); //feltöltjük az aktuális verziót

      String update_version;
      String kero_url;
      //////////////////////////////////////////////////
      olvasando_cella = "H3";
      kero_url = url3 + olvasando_cella;
      Serial.print("  GET Data from cell: ");
      Serial.println(olvasando_cella);
      clientHTTPS->GET(kero_url, host);
      update_version = clientHTTPS->getResponseBody();
      update_version.trim();
      Serial.print("  updateVersion:");
      Serial.print(update_version);
      Serial.print("  my version:");
      Serial.print(Version);
      if (Version != update_version) {
        Serial.print("update needed");
        Serial.println();


        olvasando_cella = "H1";
        kero_url = url3 + olvasando_cella;
        Serial.print(" GET Data from cell: ");
        Serial.println(olvasando_cella);
        clientHTTPS->GET(kero_url, host);
        updateUrl = clientHTTPS->getResponseBody();
        updateUrl.trim();
        Serial.print("updateUrl: ");
        Serial.print(updateUrl);
        Serial.println();


        Serial.print("updateFingerprint: ");
        for (int i = 0; i < 20; i++) {
          olvasando_cella = "I" + String(2 + i);
          kero_url = url3 + olvasando_cella;
          //Serial.print("GET Data from cell: ");
          //Serial.println(olvasando_cella);
          clientHTTPS->GET(kero_url, host);
          updateFingerprint = clientHTTPS->getResponseBody();
          updateFingerprint.trim();
          //Serial.print(updateFingerprint);
          fp[i] = updateFingerprint.toInt();
          //Serial.print(" > ");
          //Serial.print(fp[i]);
          //Serial.print("; ");
        }
        Serial.println();
        Serial.print("kulcs:");
        for (int i = 0; i < 20; i++) {

          Serial.print(fp[i], HEX);
          Serial.print(", ");
        }
        clientHTTPS->stopAll();// delete HTTPSRedirect object
        delete clientHTTPS;
        clientHTTPS = nullptr;
        Serial.printf("Free heap: %u\n", ESP.getFreeHeap());
        yield();
        tcpCleanup();
        Serial.printf("Free heap: %u\n", ESP.getFreeHeap());

        BearSSL::WiFiClientSecure client;
        for (int i = 0; i < 3; i++) {
          t_httpUpdate_return  ret = ESPhttpUpdate.update(updateUrl, "", fp);
          switch (ret) {
            case HTTP_UPDATE_FAILED:
              Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
              break;

            case HTTP_UPDATE_NO_UPDATES:
              Serial.println("HTTP_UPDATE_NO_UPDATES");
              break;

            case HTTP_UPDATE_OK:
              Serial.println("HTTP_UPDATE_OK");
              break;
          }
        }
        client.stopAll();
      }
      else Serial.println("update not needed");
      clientHTTPS->stopAll();// delete HTTPSRedirect object
      delete clientHTTPS;
      clientHTTPS = nullptr;
      Serial.printf("Free heap: %u\n", ESP.getFreeHeap());
      yield();
      tcpCleanup();
      Serial.printf("Free heap: %u\n", ESP.getFreeHeap());


    }
  private:

};
UPDATEProcess UPDATEproc(sched, HIGH_PRIORITY, 120000);



void setup() {
  delay(5000); //a normális kódfeltöltéshez
  Serial.begin(115200);
  Serial.println(Version);
  Wire.begin();
  //diagnosztikai eszközök:
  Serial.println("I2C scanner. Scanning ...");

  for (byte i = 1; i < 120; i++)
  {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0)
    {
      Serial.print("Found address: ");
      Serial.print(i, DEC);
      Serial.print(" (0x");
      Serial.print(i, HEX);
      Serial.println(")");
      delay(1);
    }
  }


  //netcsatlakozás
  delay(200);
  // WiFi.mode(WIFI_STA);
  wifiMulti.addAP("ALE-L21", "fff");
  // wifiMulti.addAP("ssid_from_AP_3", "your_password_for_AP_3");

  Serial.println("Connecting Wifi...");
  delay(1000);
  int q = 0;
  while (wifiMulti.run() != WL_CONNECTED) {
    q++;
    Serial.print(".");
    yield(); delay(1000);

    if (q > 30) {
      Serial.println("Starting AP");
      WiFiManager wifiManager;
      //and goes into a blocking loop awaiting configuration
      wifiManager.autoConnect("RingAirAP");

      //if you get here you have connected to the WiFi
      Serial.println("connected...yeey :)");

      break;
      // ESP.deepSleep(3 * 1e6);
    }

  }
  //elsőnek az órát kell beállítani, ha az elindult a loopban engedélyezzük a többi funkciót is
  Serial.println("connected");





  delay(200);
  Tget.add(true);
}

void loop() {

  if (timeStatus() == timeSet) {
    UPDATEproc.add(true);
    LOGFILE.add(true);
    MQ135.add(true);
    BMPproc.add(true);
    Tget.disable();
   // GPSproc.add(true);
    WIFILOCproc.add(true);
  }

  sched.run();


}



