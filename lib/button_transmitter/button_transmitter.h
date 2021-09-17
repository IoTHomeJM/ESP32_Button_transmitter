#ifndef button_transmitter_h
#define button_transmitter_h
//#include <Adafruit_SH1106.h>   // oled 1.3 cala
#include <Adafruit_SSD1306.h>  // oled 0.9 cala
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>
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
#include <WiFiUdp.h>
#include <Wire.h>
#include <esp32_can.h>

#include "RF24.h"
#include "SPIFFS.h"
#include "nRF24L01.h"

void EnableOTA();
void readNRF();
void sendNRF(uint8_t fnID, uint16_t fndata);
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
void zmiana_poziomu_jasnosci(uint16_t nrLED);

void oneClick0();
void DoubleClick0();
void LongPressStart0();

void oneClick1();
void DoubleClick1();
void LongPressStart1();

void oneClick2();
void DoubleClick2();
void LongPressStart2();

void LongPressStop();
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
void relaydimonoff();
void wifiSTAstart();
void wifiSTAcheck();
void writeStringToEEPROM(int adres, const String& ToWrite);
String readStringFromEEPROM(int addrOffset);
void wifiSTAconnecting();
void buttonWIFIenable();
void buttonWIFIdisable();
void timeNetUpdate();
String mcADR();

class Qtimers {
    int active, function, hourStart, minutesStart, hourEnd, minutesEnd, dayStart, monthStart,dayEnd, monthEnd;

   public:
           //active, function, hourStart, minutesStart, hourEnd, minutesEnd, dayStart, monthStart
    Qtimers(int = 0, int = 0, int = 0, int = 0, int = 0, int = 0, int = 0, int = 0,int = 0, int = 0);  // konstruktor
    
    ~Qtimers();  // destruktor

    void tdcomp(void);  //
};
#endif