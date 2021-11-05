#ifndef button_transmitter_h
#define button_transmitter_h
#include <Adafruit_SH1106.h>   // oled 1.3 cala
#include <Adafruit_SSD1306.h>  // oled 0.9 cala
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <LightDimmerESP32.h>
#include <NTPClient.h>
#include <OneButton.h>
#include <OneWire.h>
#include <SPI.h>
#include <Sim800l.h>
#include <SimpleTimer.h>
#include <TimeLib.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <esp32_can.h>

#include "RF24.h"
#include "SPIFFS.h"
#include "nRF24L01.h"

void EnableOTA();
void readNRF();
bool sendNRF(uint8_t fnID, uint16_t fndata1, uint16_t fndata2, uint16_t fndata3);
void readCAN();
void sendCAN(uint8_t toID, uint8_t fnID, uint32_t fndata, uint32_t fndata2);
void wifiapstart();
void konfiguracja();
void handle_NotFound();
void index_html();
void css_css();
void create_json();
void handleRoot();
void handleLogin();
// void zmiana_poziomu_jasnosci(uint16_t nrLED);

void oneClick0();
void DoubleClick0();
void LongPressStart0();
void LongPressStop0();

void oneClick1();
void DoubleClick1();
void LongPressStart1();
void LongPressStop1();

void oneClick2();
void DoubleClick2();
void LongPressStart2();
void LongPressStop2();

// void jasnoscLED();
void jasnoscLED(uint8_t nrLED, uint16_t jas);
void DimmWriteEEPROM();
void writeEEPROM(unsigned int adres, uint16_t dane, uint8_t dlugosc);
void SettingWriteEEPROM();
void FactoryWriteEEPROM();

void rysuj_jasnosc_na_lcd(uint16_t nrLED);
void rysujemy_na_lcd(void);
void aktualizuj_timestr();
void InicjacjaOdczytTemperatury();
void OdczytTemperatury();
void sprawdzSW1Touch();
void PokazSW1NaDisplay(boolean rundisplay);
uint8_t pobierzTouchVal(uint8_t TouchPin, uint8_t czymin, boolean isTouchPressed);
byte readEEPROM(unsigned int eeaddress);
void InicjacjaSW1Touch();
void readSMS();
void serverON();
void relays(uint8_t n);
String getsmstel();
bool sendSMS(String number, String text);
void wificlose();
void wifiSTAstart();
void wifiSTAcheck();
void writeStringToEEPROM(int adres, const String& ToWrite);
String readStringFromEEPROM(int addrOffset);
void wifiSTAconnecting();
void buttonWIFIenable();
void buttonWIFIdisable();
void realTimeUpdate();
String mcADR();
int FirmwareVersionCheck();
void firmwareupdate();
void update_started();
void update_finished();
void update_progress(int cur, int total);
void update_error(int err);
class Qtimers {
    uint8_t active, function, hourStart, minutesStart, hourEnd, minutesEnd, dayStart, monthStart, dayEnd, monthEnd;
    int lastDay, march, october, dayMarch, dayOctober, a, b, c, lastSundayMarch, lastSundayOctober;
    bool GE = 0;

   public:
    //active, function, hourStart, minutesStart, hourEnd, minutesEnd, dayStart, monthStart
    Qtimers(int = 0, int = 0, int = 0, int = 0, int = 0, int = 0, int = 0, int = 0, int = 0, int = 0);  // konstruktor
    ~Qtimers();
    void winterSummerLastSunday();
    void winterSummerTime();
    void tdcomp(void);
    void set(int ac, int fu, int hrs, int mins, int hre, int mine, int ds, int ms, int de, int me);
};
class Dimlevel {
   private:
    // uint64_t zapamietanyCzas1 = 0;
    uint64_t zapamietanyCzas2 = 0;
    bool DLbutton_is_long_pressed = 0;
    int nrLED = 0;
    bool dimming_up = 0;
    uint16_t TMPjasnosc = 0;

   public:
    Dimlevel(int = 0);  // konstruktor
    ~Dimlevel();        // destruktor
    void oneClick();
    void DoubleClick();
    void start();
    void stop();
    void change();
    void saveeeprom(uint16_t jas);
};
class Delayrelay {
   private:
    uint64_t zapczas = 0;
    bool roff = 1;
    int nrrelay;
    uint16_t relayToff;
    int dim0, dim1, dim2;

   public:
    Delayrelay(int = 10, int = 0, int = 1, int = 1, int = 1);
    ~Delayrelay();

    void delaydim();
    void set(int nrr, int rtoff, int d0, int d1, int d2);
};

void disableoled();
void displaydelayon();
void dimmodulestat();

const char* rootCACertificate =
    "-----BEGIN CERTIFICATE-----\n"
    "MIIDxTCCAq2gAwIBAgIQAqxcJmoLQJuPC3nyrkYldzANBgkqhkiG9w0BAQUFADBs\n"
    "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"
    "d3cuZGlnaWNlcnQuY29tMSswKQYDVQQDEyJEaWdpQ2VydCBIaWdoIEFzc3VyYW5j\n"
    "ZSBFViBSb290IENBMB4XDTA2MTExMDAwMDAwMFoXDTMxMTExMDAwMDAwMFowbDEL\n"
    "MAkGA1UEBhMCVVMxFTATBgNVBAoTDERpZ2lDZXJ0IEluYzEZMBcGA1UECxMQd3d3\n"
    "LmRpZ2ljZXJ0LmNvbTErMCkGA1UEAxMiRGlnaUNlcnQgSGlnaCBBc3N1cmFuY2Ug\n"
    "RVYgUm9vdCBDQTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAMbM5XPm\n"
    "+9S75S0tMqbf5YE/yc0lSbZxKsPVlDRnogocsF9ppkCxxLeyj9CYpKlBWTrT3JTW\n"
    "PNt0OKRKzE0lgvdKpVMSOO7zSW1xkX5jtqumX8OkhPhPYlG++MXs2ziS4wblCJEM\n"
    "xChBVfvLWokVfnHoNb9Ncgk9vjo4UFt3MRuNs8ckRZqnrG0AFFoEt7oT61EKmEFB\n"
    "Ik5lYYeBQVCmeVyJ3hlKV9Uu5l0cUyx+mM0aBhakaHPQNAQTXKFx01p8VdteZOE3\n"
    "hzBWBOURtCmAEvF5OYiiAhF8J2a3iLd48soKqDirCmTCv2ZdlYTBoSUeh10aUAsg\n"
    "EsxBu24LUTi4S8sCAwEAAaNjMGEwDgYDVR0PAQH/BAQDAgGGMA8GA1UdEwEB/wQF\n"
    "MAMBAf8wHQYDVR0OBBYEFLE+w2kD+L9HAdSYJhoIAu9jZCvDMB8GA1UdIwQYMBaA\n"
    "FLE+w2kD+L9HAdSYJhoIAu9jZCvDMA0GCSqGSIb3DQEBBQUAA4IBAQAcGgaX3Nec\n"
    "nzyIZgYIVyHbIUf4KmeqvxgydkAQV8GK83rZEWWONfqe/EW1ntlMMUu4kehDLI6z\n"
    "eM7b41N5cdblIZQB2lWHmiRk9opmzN6cN82oNLFpmyPInngiK3BD41VHMWEZ71jF\n"
    "hS9OMPagMRYjyOfiZRYzy78aG6A9+MpeizGLYAiJLQwGXFK3xPkKmNEVX58Svnw2\n"
    "Yzi9RKR/5CYrCsSXaQ3pjOLAEFe4yHYSkVXySGnYvCoCWw9E1CAx2/S6cCZdkGCe\n"
    "vEsXCS+0yx5DaMkHJ8HSXPfqIbloEpw8nL+e/IBcm2PN7EeqJSdnoDfzAIJ9VNep\n"
    "+OkuE6N36B9K\n"
    "-----END CERTIFICATE-----\n";
#endif