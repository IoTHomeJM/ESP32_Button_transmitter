
//#include <Adafruit_SH1106.h>   // oled 1.3 cala
#include <Adafruit_SSD1306.h>  // oled 0.9 cala
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include <LightDimmerESP32.h>
#include <OneButton.h>
#include <OneWire.h>
#include <SPI.h>
#include <Sim800l.h>
#include <SimpleTimer.h>
#include <TimeLib.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp32_can.h>
#include <void.h>

#include "RF24.h"
#include "SPIFFS.h"
#include "nRF24L01.h"

unsigned int czasnazapisweeprom = 6000;  //w milisekundach, 6 sekund, w przypadku brak zmiany jasnosci przez ten czas powoduje zapis w eeprom

//const byte portLED = 19, portLED2 = 18, portLED3 = 17;  // dostepne: DIM1/R - 19 | DIM2/G - 18 | DIM3/B - 17

uint16_t FreqLED = 25000;     // czestotliwosc LED
uint16_t ResolutionLED = 10;  // Rozdzielczosc w bitach.

// zmiennych ponizej nie edytujemy
unsigned int jasnosc = 0, jasnosc2 = 0, jasnosc3 = 0;
unsigned int jasnosc_last = 0;  // !! chyba nigdzie nie jest wykorzystywane
unsigned int jasnosc_max = 0;   // !! chyba nigdzie nie jest wykorzystywane
byte eepromBUF[64];             // bufor dla zapisu/odczytu eeprom
bool zapiszweeprom = 0;
uint64_t aktualnyCzas = 0;
uint64_t zapamietanyCzas1 = 0;
uint64_t zapamietanyCzasEEPROM = 0;
uint64_t zapamietanyCzasOLED = 0;
uint64_t zapczas = 0, zapczas2 = 0;  // dla relaydimonoff();

// LightDimmerESP32 DimLED1;
LightDimmerESP32 led[3];
bool dim1 = 0, dim2 = 0, dim3 = 0, LightOnBoot = 0;
uint8_t DimUpDownResolution = 20;  // szybkosc regulacji jasnosci led za pomoca przycisku

// dla przycisku, regulacja jasnosci
int16_t dimmset_now = 0, dimmset_now2 = 0, dimmset_now3 = 0;
int16_t dimmset_last = 0, dimmset_last2 = 0, dimmset_last3 = 0;
int16_t dimmset_min = 2;
int16_t dimmset_max = 999, dimm2set_max = 999, dimm3set_max = 999;

//<--Dimmer
//--> server
boolean ServerActive = 1;

char* ssidAP = strdup("SWModule");        // Enter SSID here
char* passwordAP = strdup("1234567890");  // Enter Password here
// char* ssid = strdup("NET2");
// char* password = strdup("Janek1Ewa2-s7");
// char* ssid = strdup("JLU");
//char* password = strdup("1234567890");
char* ssid = strdup("NET");
char* password = strdup("Janek1@$Eva");

String hostname = "SWModule";
WebServer server(80);
bool wifiSTAon = 0, WIFIconfigured = 0;
uint8_t wifiSTAonCN = 0;
//<--server

//--> CAN-BUS

uint8_t Flag = 0;      // 1 bit
uint8_t SubDevID = 0;  // 4 bity od 0 do 15. wykorzystywane np po to aby zazadzac drugim modulem po CAN z tym zamym DevID
uint8_t DevID = 2;     // 8bit ID tego urzadzenia, ID 1 to master.
uint32_t frameID = 0;
bool allFrame = 0;         // jesli 0 to przyjmuje ramki z filtrem
bool OnDevices[25] = {0};  // lista wlaczonych urzadzen - tylko dla mastera. test
//<-- CAN-BUS

#define eeprom 0x50  // Address of 24LC256 eeprom chip

SimpleTimer timer;

const uint8_t SW1pin = 19;
//const uint8_t SW1pin = 0;
const uint8_t SW1TouchPin = 33;

//--> NRF settings
byte NRFbuf[32] = {0};  // bufor dla nrf
uint32_t RF24_rxAddr = 2138439680;
uint8_t NRFchannel = 1;
uint8_t PALevel = 1;             // 0=MIN, 1=LOW, 2=HIGH, 3=MAX
RF24 radio(27, 15, 14, 12, 13);  //JLU //CE CSN SCK MISO MOSI // ta trzeba uzywac do ESP32 https://github.com/nhatuan84/RF24
//<--NRF

//--> temeratura
OneWire ds(23);
byte DallasAddr[8];
bool CzyJestCzujnikTemperaury = 1;
float celsius = -99;
//<--

//--> dla oled 0.9 cala Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1  // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);
//<--

//--> dla oled 1.3 cala
//Adafruit_SH1106 display(21, 22);
//<--

// OneButton button(SW1pin, true, 1);
OneButton button(SW1pin, true);
bool button_is_long_pressed = 0;
bool dimming_up = 0;
bool poRestarcieTestSW1 = 1;

char timestr[10] = "--:--:--";

boolean SW1TouchEnable = 0;  // aktywacja funkcji przycisku dotyku
boolean SW1TouchActive = 0;  // dotyk uruchamia sie dopiero po inicjalizacji
uint8_t SW1TouchVal = 255;
uint8_t SW1TouchMax = 255;  // ustalana po inicjalizacji wartosc dotyku nie nacisnietego
boolean SW1TouchPressed = 0;
boolean SW1TouchPrevious = 0;
uint8_t SW1TouchFilter = 0;
int SW1TouchValMin = 255;  //tylko do testow

boolean OTAActive = 0, FactorySet = 0, SettingSave = 0;
String json;

Sim800l Sim800l;     //to declare the library
int serialmode = 2;  //0-wylaczony, 1-wlaczoy LOG, 2-wlaczony SIM
String SMSbuf = "";  // bufor SMS
char incomingByte;   // zaciaganie bitow z seriala
bool gsminit = 0, GSMmsg = 0;
uint16_t smsoled = 5000;  // czas wyswietlania sms na OLED

int drvsimreset = 33;
bool oleddim = 0;

int relay[2] = {25, 26};                       // piny fizyczne
bool sp[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // status przekaznikow sp[0],sp[1] - fizyczne
uint16_t relayToff1 = 5;                       // czas opoznienia wylaczenia przekaznika1 w sekundach
uint16_t relayToff2 = 5;
bool roff = 1;

int timerid1, timerid2;
void setup() {
    //--> EEPROM
    Wire.begin(21, 22);  // sda, scl
    Wire.setClock(4000000);
    for (int i = 0; i < 64; ++i) {
        eepromBUF[i] = readEEPROM(i);
    }  // read 32 byte eeprom
    jasnosc = eepromBUF[0] * 256 + eepromBUF[1];
    jasnosc2 = eepromBUF[6] * 256 + eepromBUF[7];
    jasnosc3 = eepromBUF[8] * 256 + eepromBUF[9];
    DevID = eepromBUF[4];
    SubDevID = eepromBUF[5];
    LightOnBoot = eepromBUF[10];
    Flag = eepromBUF[11];
    allFrame = eepromBUF[12];
    PALevel = eepromBUF[13];
    NRFchannel = eepromBUF[14];
    SW1TouchEnable = eepromBUF[15];
    OTAActive = eepromBUF[16];
    ServerActive = eepromBUF[17];
    ResolutionLED = eepromBUF[18];
    FreqLED = eepromBUF[19] * 256 + eepromBUF[20];
    czasnazapisweeprom = eepromBUF[21] * 256 + eepromBUF[22];
    serialmode = eepromBUF[23];
    dimmset_max = eepromBUF[24] * 256 + eepromBUF[25];
    DimUpDownResolution = eepromBUF[26];
    dimm2set_max = eepromBUF[27] * 256 + eepromBUF[28];
    dimm3set_max = eepromBUF[29] * 256 + eepromBUF[30];
    RF24_rxAddr = (127 << 24) + (118 << 16) + (eepromBUF[31] << 8) + 0;
    relayToff1 = eepromBUF[32] * 256 + eepromBUF[33];
    relayToff2 = eepromBUF[34] * 256 + eepromBUF[35];

    strcpy(ssid, readStringFromEEPROM(2000).c_str());
    strcpy(password, readStringFromEEPROM(2033).c_str());

    //dimmset_now = jasnosc;
    dimmset_last = jasnosc;
    dimmset_last2 = jasnosc2;
    dimmset_last3 = jasnosc3;
    // konfikuracja poczatkowa przekaznika
    pinMode(relay[0], OUTPUT);
    pinMode(relay[1], OUTPUT);
    digitalWrite(relay[0], sp[0]);
    digitalWrite(relay[1], sp[1]);
    // serialmode = 1;  // !!
    if (serialmode == 1) {
        Serial.begin(115200);      // dla serial monitro
    } else if (serialmode == 2) {  //dla SIM
        Serial.begin(9600);
    } else {
    }

    create_json();

    if (serialmode == 1) {
        Serial.print("json: ");
        Serial.println(json);
        Serial.print("jasnosc: ");
        Serial.println(jasnosc);
        Serial.print("jasnosc2: ");
        Serial.println(jasnosc2);
        Serial.print("jasnosc3: ");
        Serial.println(jasnosc3);
        Serial.print("DevID: ");
        Serial.println(DevID);
        Serial.print("LightOnBoot: ");
        Serial.println(LightOnBoot);
        Serial.print("PALevel: ");
        Serial.println(PALevel);
        Serial.print("NRFchannel: ");
        Serial.println(NRFchannel);
        Serial.print("SW1TouchEnable: ");
        Serial.println(SW1TouchEnable);
        Serial.print("OTAActive: ");
        Serial.println(OTAActive);
        Serial.print("ServerActive: ");
        Serial.println(ServerActive);
        Serial.print("ResolutionLED: ");
        Serial.println(ResolutionLED);
        Serial.print("FreqLED: ");
        Serial.println(FreqLED);
        Serial.print("czasnazapisweeprom: ");
        Serial.println(czasnazapisweeprom);
        Serial.print("serialmode: ");
        Serial.println(serialmode);
        Serial.print("dimmset_max: ");
        Serial.println(dimmset_max);
        Serial.print("dimm2set_max: ");
        Serial.println(dimm2set_max);
        Serial.print("dimm3set_max: ");
        Serial.println(dimm3set_max);
        Serial.print("DimUpDownResolution: ");
        Serial.println(DimUpDownResolution);
        Serial.print("Pelny RF24_rxAddr: ");
        Serial.println(RF24_rxAddr);
        Serial.print("RF24_rxAddr: ");
        Serial.println((RF24_rxAddr >> 8) & 255);
        Serial.print("dimmset_now: ");
        Serial.println(dimmset_now);
        Serial.print("dimmset_now2: ");
        Serial.println(dimmset_now2);
        Serial.print("dimmset_now3: ");
        Serial.println(dimmset_now3);
        Serial.print("ssi: ");
        Serial.println(ssid);
        Serial.print("password: ");
        Serial.println(password);
    }
    //<-- EEPROM
    /*
    //-->  ESP32 Dimmer settings
    ledcSetup(0, FreqLED, ResolutionLED);  // ledChannel, freq, resolution
    ledcSetup(1, FreqLED, ResolutionLED);
    ledcSetup(2, FreqLED, ResolutionLED);
    // for (uint8_t i = 0; i < 3; i++) {ledcAttachPin(i+17, 0);}

    ledcAttachPin(portLED, 0);
    ledcAttachPin(portLED2, 1);
    ledcAttachPin(portLED3, 2);

    // portLED=19 - led[0], portLED2=18 - led[1], portLED3=17 - led[2];
    led[0].setFadingTime(1000);  // czas od min do max
    led[1].setFadingTime(1000);
    led[2].setFadingTime(1000);
    led[0].setBrighteningTime(300);  // czas od max do min
    led[1].setBrighteningTime(300);
    led[2].setBrighteningTime(300);
    led[0].begin(0, HIGH);
    led[1].begin(1, HIGH);
    led[2].begin(2, HIGH);

    if (LightOnBoot == 1) {  //max jasnosc podczas startu procesora
        //led[0].setupMax(jasnosc);
        dimmset_now = jasnosc;  // poprwka przekazanie jasnosci on boot z eeprom tez na wyswietlacz
        dim1 = 1;
        jasnoscLED();
    }

    if (serialmode == 1) {
        Serial.print("Nastawy po wlaczeniu: Jasnosc:");
        Serial.print(jasnosc);
        Serial.print(" Jasnosc max:");
        Serial.println(jasnosc_max);
    }
    //<--
    */
    //--> CAN-BUS
    if (CAN0.begin(500000)) {  // predkosc CAN-BUS
        if (allFrame == 0) {
            CAN0.setRXFilter(DevID, 0x000000FF, true);  // przyjmuje ramki tylko o danym ID
            CAN0.setRXFilter(255, 0x000000FF, true);    // ID 255 dla wszystkich urzadzen - np zgloszenie obecnosci do mastera
        } else {
            CAN0.setRXFilter(0, 0, true);  // przyjmuje wszystkie CAN ID
        }
        if (serialmode == 1) {
            Serial.println("CAN ready ...!");
        }
    } else {
        if (serialmode == 1) {
            Serial.println("CAN init failed…");
        }
    }
    //<-- CAN-BUS

    dimmset_max -= 1;

    button.attachClick(oneClick);
    button.attachDoubleClick(DoubleClick);
    button.attachLongPressStart(LongPressStart);
    button.attachLongPressStop(LongPressStop);

    //--> dla oled 0.9 cala // Address 0x3D for 128x64

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        if (serialmode == 1) {
            Serial.println(F("SSD1306 allocation failed"));
        }
        for (;;)
            ;
    }
    //<--

    //--> dla oled 1.3 cala
    //display.begin(SH1106_SWITCHCAPVCC, 0x3C);
    //<--

    //display.dim(1);// jasnosc wyswietlacza oled jesli 1 to przyciemnione

    display.clearDisplay();
    display.display();
    rysujemy_na_lcd();

    radio.begin();
    radio.setChannel(NRFchannel);
    radio.setPayloadSize(3);
    radio.setCRCLength(RF24_CRC_8);
    radio.setPALevel(PALevel);
    radio.setDataRate(RF24_250KBPS);
    radio.setRetries(15, 15);
    radio.openWritingPipe(RF24_rxAddr + 1);  // !!
    radio.openReadingPipe(1, RF24_rxAddr);
    radio.startListening();
    if (serialmode == 1) radio.printDetails();  // wyswietlamy info o parametrach NRF

    // inicjalizacja czujnika temperatury DS18B20 tj. odczyt jego adresu
    ds.search(DallasAddr);  // NIE WIEM DLACZEGO ALE TRZEBA WYWOLAC 2 RAZY
    if (!ds.search(DallasAddr)) {
        CzyJestCzujnikTemperaury = 0;
    }
    if (OneWire::crc8(DallasAddr, 7) != DallasAddr[7]) {
        CzyJestCzujnikTemperaury = 0;
    }
    if (DallasAddr[0] != 0x28) {
        CzyJestCzujnikTemperaury = 0;
    }

    if (serialmode == 2) {
        int cz = 0;
        while (cz < 20) {
            Serial.println("AT");
            delay(200);
            readSMS();
            if (SMSbuf.indexOf("OK") > -1) {
                SMSbuf = "GSM zainicjowany";
                rysujemy_na_lcd();
                cz = 20;
                gsminit = 1;
            }
            cz++;
        }
        if (gsminit == 1) {
            //delay(1000);
            Serial.println("AT+CMGF=1");  //Ustaw SMS na tryb tekstowy
            delay(1000);
            rysujemy_na_lcd();
            SMSbuf = "";
            Serial.println("AT+CNMI=1,2,0,0,0");  //Procedura obsługi nowo przybyłych wiadomości
            delay(1000);

            rysujemy_na_lcd();
            //Serial.println("AT+CMGL=\"REC UNREAD\"");  // Czytaj Nieprzeczytane wiadomości
        } else {
            SMSbuf = "Nie wykryto GSM";
        }

        rysujemy_na_lcd();
    } else {
        SMSbuf = "GSM wylaczony";
        rysujemy_na_lcd();
    }

    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS init error");
        return;
    }  // inicjowanie systemu plikow SPI

    // setTime(1586365689); // ustawia zegarek

    //--> server www
    if (ServerActive == 1) {
        // wifiapstart();
        wifiSTAstart();
        server.on("/headers", []() {  // wysyla naglowki
            server.sendHeader("Access-Control-Allow-Origin", "*");
            server.sendHeader("access-control-allow-credentials", "true");
            server.sendHeader("access-control-allow-headers", "x-requested-with");
            server.sendHeader("access-control-allow-methods", "GET,OPTIONS");
            server.send(200, "text/plain", "OK");
        });
        server.on("/after_loading", []() { create_json(); server.send(200, "application/json", json); });
        server.on("/favicon.ico", []() { server.send(200, "text/plain", "OK"); });
        server.on("/config", HTTP_POST, konfiguracja);
        server.on("/", index_html);
        server.on("/index.html", index_html);
        server.on("/tst", HTTP_GET, handleRoot);      // Call the 'handleRoot' function when a client requests URI "/"
        server.on("/login", HTTP_POST, handleLogin);  // Call the 'handleLogin' function when a POST request is made to URI "/login"

        server.onNotFound(handle_NotFound);
        // delay(2000);  // !!
        // server.begin();  // !! powinno byc po polaczeniu wifi?
        // if (serialmode == 1) Serial.println("HTTP server started");
        // !!
        // if (strcmp(ssid, "ssid") != 0) {
        //     WIFIconfigured = 1;
        // }
        // if (serialmode == 1) {
        //     Serial.print("WIFIconfigured: ");
        //     Serial.println(WIFIconfigured);
        // }
    }
    //<-- server

    //// TIMERY ////
    ///////////////

    timer.setInterval(1000, rysujemy_na_lcd);
    timer.setInterval(2000, InicjacjaOdczytTemperatury);
    if (ServerActive == 1) {
        timerid1 = timer.setInterval(300000, wifiSTAcheck);  // co 5min 300000
        timer.setInterval(1500, serverON);
        timerid2 = timer.setTimer(6000, wifiSTAcheck, 5);
    }
    if (gsminit == 1) {
        timer.setInterval(500, readSMS);
    }

    if (SW1TouchEnable) {
        timer.setTimer(100, InicjacjaSW1Touch, 10);
        touch_pad_init();
        touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
        touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    }
}
////////////////////////////
// LOOP ////////////////////
////////////////////////////

void loop() {
    aktualnyCzas = millis();
    timer.run();
    LightDimmerESP32::update();
    readCAN();
    readNRF();
    relaydimonoff();
    if (button_is_long_pressed == 1) {
        zmiana_poziomu_jasnosci();
    }

    if (SW1TouchActive == 1) {
        sprawdzSW1Touch();
    }
    if (SW1TouchActive == 1) {
        button.tick(SW1TouchPressed);
    } else {
        button.tick();
    }

    if (OTAActive == 1) {
        ArduinoOTA.handle();
    }
    if (FactorySet == 1) {
        FactoryWriteEEPROM();
    }
    if (SettingSave == 1) {
        SettingWriteEEPROM();
    }
    if (zapiszweeprom == 1) {
        DimmWriteEEPROM();
    }  // zapisywanie z opoznieniem aktualnej nastawy jasnosci LED do EEPROM
    // wifiSTAcheck();
}
////////////////////////////
// KONIEC LOOP ////////////////////
////////////////////////////

void EnableOTA() {
    // ustawienie MAC Address musi byc przed podlaczeniem do WiFi
    // uint8_t NEW_MACAddress[6] = {0x82,0x01,0x01,0x01,0x01,0x01}; //zostaw w pierwszym 82, sa rezerwacje na pewne nr np. 01...
    // esp_base_mac_addr_set(NEW_MACAddress); //Serial.print("ESP Board MAC Address: "); Serial.println(WiFi.macAddress());

    // WiFi.mode(WIFI_STA);
    // WiFi.begin(ssid, password);

    // while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    //     Serial.println("Connection Failed! Rebooting...");
    //     delay(5000);
    //     ESP.restart();
    // }
    // Serial.print("IP address: ");
    // Serial.println(WiFi.localIP());
    ArduinoOTA.begin();
}

void readNRF() {
    if (radio.available()) {
        radio.read(&NRFbuf, sizeof(NRFbuf));
        if (serialmode == 1) {
            Serial.print("Wiadomosc NRF");
            Serial.print(NRFbuf[0]);
            Serial.print(", ");
            Serial.println(NRFbuf[2] * 256 + NRFbuf[1]);
        }
        int readNRFID = NRFbuf[0];
        switch (readNRFID) {
            case 1:
                break;
            case 2:
                break;
            case 3:
                jasnosc = NRFbuf[2] * 256 + NRFbuf[1];
                dimmset_now = jasnosc;
                dim1 = 1;
                jasnoscLED();
                if (serialmode == 1) {
                    Serial.print("Jasnosc odebrana z nadajnika:");
                    Serial.println(jasnosc);
                }
                break;
            case 4:
                jasnosc2 = NRFbuf[2] * 256 + NRFbuf[1];
                dim2 = 1;
                jasnoscLED();
                if (serialmode == 1) {
                    Serial.print("Jasnosc odebrana z nadajnika:");
                    Serial.println(jasnosc2);
                }
                break;
            case 5:
                jasnosc3 = NRFbuf[2] * 256 + NRFbuf[1];
                dim3 = 1;
                jasnoscLED();
                if (serialmode == 1) {
                    Serial.print("Jasnosc odebrana z nadajnika:");
                    Serial.println(jasnosc3);
                }
                break;
            case 6: {  // przekazniek wirtualny
                //sp[NRFbuf[1]] = NRFbuf[2];  // nie wiem czy dobrze
            }

            break;
            case 7:
                break;
            case 8:
                break;
            case 9:
                break;
            case 10:
                break;
            case 11:
                break;
            case 12:
                break;
            case 13:
                break;
            case 14:
                break;
            case 15:
                break;
            case 16:
                break;
            case 17:
                break;
            case 18:
                break;
            case 19:
                break;
            case 20:
                break;
            case 21:
                break;
            case 22:
                break;
            case 23:
                break;
            case 24:
                break;
            case 25:
                break;
            case 26:
                break;
            case 27:
                break;
            case 28:
                break;
            case 29:
                break;
            case 30:
                break;
            case 31:
                break;
            case 32:
                break;
            case 33:
                break;
            case 34:
                break;
            case 35:
                break;
            case 36:
                break;
            case 37:
                break;
            case 38:
                break;
            case 39:
                break;
            case 40:
                break;
            case 41:
                break;
            case 42:
                break;
            case 43:
                break;
            case 44:
                break;
            case 45:
                break;
            case 46:
                break;
            case 47:
                break;
            case 48:
                break;
            case 49:
                break;
            case 50:
                break;
        }

    }  // end if radio.available()
}

void sendNRF(uint8_t fnID, uint16_t fndata) {
    bool rslt = 0;
    if (1 == 1) {
        char msg[3];
        msg[0] = fnID;
        msg[1] = (fndata & 0xFF);
        msg[2] = (fndata >> 8);  //forma przekazania bajtu starszego oraz młodszego
        radio.stopListening();

        rslt = radio.write(msg, 3);
        if (serialmode == 1) {
            if (rslt) {
                Serial.println("Dane NRF odebrane");
            } else {
                Serial.println("Tx failed, dane nie odebrane");
            }
        }
    }
    radio.startListening();
}
//--> CAN-BUS
void readCAN() {
    CAN_FRAME inFrame;
    if (CAN0.read(inFrame)) {
        frameID = inFrame.id;  // ID ramki CAN
        uint8_t fnID = frameID >> 21;
        uint8_t flag = (frameID >> 20) & 0x1;
        uint8_t SubDevID = (frameID >> 16) & 0xF;
        uint8_t fromID = (frameID >> 8) & 0xFF;
        uint8_t toID = (frameID)&0xFF;

        if (serialmode == 1) {
            Serial.print("0x");
            Serial.print(frameID, HEX);
            if (inFrame.extended)
                Serial.print(" EX ");
            else
                Serial.print(" ST ");
            Serial.print(inFrame.length, DEC);
            for (int i = 0; i < inFrame.length; i++) {
                Serial.print(inFrame.data.byte[i], HEX);
                Serial.print(" ");
            }
            Serial.println();
            Serial.print(" fnID: ");
            Serial.print(fnID);
            Serial.print(" flag: ");
            Serial.print(flag);
            Serial.print(" SubDevID: ");
            Serial.print(SubDevID);
            Serial.print(" fromID: ");
            Serial.print(fromID);
            Serial.print(" toID: ");
            Serial.print(toID);
            Serial.print(" fndata: ");
            Serial.print(inFrame.data.uint32[0]);
            Serial.print(" fndata2: ");
            Serial.println(inFrame.data.uint32[1]);
        }
        switch (fnID) {
            case 1:  // odpowiadamy masterowi obecnosc urzadzenia na CANie
                sendCAN(1, 1, 0, 0);
                if (serialmode == 1) {
                    Serial.println("fnID 1 ramka wyslana");
                }
                break;
            case 2:  // synchronizacja czasu
                setTime(inFrame.data.uint32[0] + 7200);
                break;
            case 3:  // dimmer 1 // NIE TESTOWANE
                jasnosc = inFrame.data.uint16[0];
                dim1 = 1;
                jasnoscLED();
                break;
            case 4:  // dimmer 2 // NIE TESTOWANE
                jasnosc2 = inFrame.data.uint16[0];
                dim2 = 1;
                jasnoscLED();
                break;
            case 5:  // dimmer 3 // NIE TESTOWANE
                jasnosc3 = inFrame.data.uint16[0];
                dim3 = 1;
                jasnoscLED();
                break;
            case 6:  // wszystkie !! NIE TESTOWANE
                jasnosc = inFrame.data.uint16[0];
                dim1 = 1;
                jasnosc2 = inFrame.data.uint16[1];
                dim2 = 1;
                jasnosc3 = inFrame.data.uint16[2];
                dim3 = 1;
                if (serialmode == 1) {
                    Serial.print("Jasnosc1: ");
                    Serial.println(jasnosc);
                    Serial.print("Jasnosc2: ");
                    Serial.println(jasnosc2);
                    Serial.print("Jasnosc3: ");
                    Serial.println(jasnosc3);
                }
                jasnoscLED();
                break;
            case 7:
                break;
            case 8:
                break;
            case 9:
                break;
            case 10:
                break;
            case 11:
                break;
            case 12:
                break;
            case 13:
                break;
            case 14:
                break;
            case 15:
                break;
            case 16:
                break;
            case 17:
                break;
            case 18:
                break;
            case 19:
                break;
            case 20:
                break;
            case 21:
                break;
            case 22:
                break;
            case 23:
                break;
            case 24:
                break;
            case 25:
                break;
            case 26:
                break;
            case 27:
                break;
            case 28:
                break;
            case 29:
                break;
            case 30:
                break;
            case 31:
                break;
            case 32:
                break;
            case 33:
                break;
            case 34:
                break;
            case 35:
                break;
            case 36:
                break;
            case 37:
                break;
            case 38:
                break;
            case 39:
                break;
            case 40:
                break;
            case 41:
                break;
            case 42:
                break;
            case 43:
                break;
            case 44:
                break;
            case 45:
                break;
            case 46:
                break;
            case 47:
                break;
            case 48:
                break;
            case 49:
                break;
            case 50:
                break;
        }
    }
}

void sendCAN(uint8_t toID, uint8_t fnID, uint32_t fndata, uint32_t fndata2) {
    // tworzenie ID RAMKI
    //        | 8bit fnID  |   1bit Flag  | 4bit SubDevID    |  8bit DevID  | 8bit toID |
    frameID = (fnID << 21) + (Flag << 20) + (SubDevID << 16) + (DevID << 8) + toID;

    CAN_FRAME outFrame;
    outFrame.rtr = 0;
    outFrame.id = frameID;
    outFrame.extended = true;
    outFrame.length = 8;
    outFrame.data.uint32[0] = fndata;
    outFrame.data.uint32[1] = fndata2;
    CAN0.sendFrame(outFrame);
    if (serialmode == 1) {
        Serial.print("Can send ID: ");
        Serial.print(frameID, HEX);
        Serial.print(" toID: ");
        Serial.print(toID);
        Serial.print(" fnID: ");
        Serial.print(fnID);
        Serial.print(" fndata: ");
        Serial.println(fndata);
    }
    frameID = 0;
}

//<-- CAN-BUS
//--> server

void wificlose() {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);
    ESP.restart();
}
void wifiapstart() {
    if (serialmode == 1) {
        Serial.println("Laczenie wifi AP");
    }
    timer.disable(timerid1);
    WiFi.persistent(false);
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    //WiFi.mode(WIFI_STA);
    delay(100);

    IPAddress local_ip(192, 168, 1, 100);
    IPAddress gateway(192, 168, 1, 1);
    IPAddress subnet(255, 255, 255, 0);

    WiFi.softAPConfig(local_ip, gateway, subnet);
    WiFi.softAP(ssidAP, passwordAP);
    server.begin();
    zapamietanyCzasOLED = aktualnyCzas;
    SMSbuf = "APmode : 192.168.1.100";
}
void wifiSTAstart() {
    WiFi.persistent(false);
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    delay(100);
    WiFi.mode(WIFI_STA);
    hostname += String(DevID);
    WiFi.setHostname(hostname.c_str());
    WiFi.begin(ssid, password);

    if ((wifiSTAonCN >= 5) && (WIFIconfigured == 0)) {
        if (serialmode == 1) {
            Serial.println("brak polaczenia STA mode");
        }
        wifiSTAonCN = 0;
        wifiapstart();
    }
}
void wifiSTAcheck() {
    if (WiFi.status() != WL_CONNECTED) {
        wifiSTAonCN++;
        if (serialmode == 1) {
            Serial.print("Laczenie do WIFI - STA mode. ");
            Serial.println(" Ilosc prob: " + String(wifiSTAonCN) + "");
        }
        if (wifiSTAon == 1) {
            timer.enable(timerid2);  // wifiSTAcheck
            wifiSTAon = 0;
        }
        wifiSTAstart();

    } else {
        wifiSTAonCN = 0;
        if (serialmode == 1) {
            Serial.print("WiFi OK - IP addres: ");
            Serial.println(WiFi.localIP());
        }

        if (wifiSTAon == 0) {
            wifiSTAon = 1;
            timer.disable(timerid2);  // wifiSTAcheck
            zapamietanyCzasOLED = aktualnyCzas;
            SMSbuf = "IP: " + WiFi.localIP().toString() + "";
            server.begin();
            if (serialmode == 1) Serial.println("HTTP server started");
        }
    }
}

void konfiguracja() {
    DevID = server.arg("DevID").toInt();
    SubDevID = server.arg("SubDevID").toInt();
    Flag = server.arg("Flag").toInt();
    allFrame = server.arg("allFrame").toInt();
    PALevel = server.arg("PALevel").toInt();
    NRFchannel = server.arg("NRFchannel").toInt();
    uint8_t RF24_rxAddrTEMP = server.arg("RF24_rxAddr").toInt();
    RF24_rxAddr = (127 << 24) + (118 << 16) + (RF24_rxAddrTEMP << 8) + 0;
    SW1TouchEnable = server.arg("SW1TouchEnable").toInt();
    OTAActive = server.arg("OTAActive").toInt();
    FactorySet = server.arg("FactorySet").toInt();
    ServerActive = server.arg("ServerActive").toInt();
    ResolutionLED = server.arg("ResolutionLED").toInt();
    FreqLED = server.arg("FreqLED").toInt();
    czasnazapisweeprom = server.arg("czasnazapisweeprom").toInt();
    LightOnBoot = server.arg("LightOnBoot").toInt();
    serialmode = server.arg("serialmode").toInt();
    DimUpDownResolution = server.arg("DimUpDownResolution").toInt();
    SettingSave = server.arg("eeprom_save").toInt();
    jasnosc = server.arg("jasnosc1").toInt();
    dimmset_now = jasnosc;
    dim1 = 1;
    jasnoscLED();
    dimmset_max = server.arg("dimmset_max").toInt();
    jasnosc2 = server.arg("jasnosc2").toInt();
    dimmset_now2 = jasnosc2;
    dim2 = 1;
    jasnoscLED();
    dimm2set_max = server.arg("dimm2set_max").toInt();
    jasnosc3 = server.arg("jasnosc3").toInt();
    dimmset_now3 = jasnosc3;
    dim3 = 1;
    jasnoscLED();
    dimm3set_max = server.arg("dimm3set_max").toInt();
    relayToff1 = server.arg("relayToff1").toInt();
    relayToff2 = server.arg("relayToff2").toInt();

    strcpy(ssid, server.arg("SSID").c_str());
    strcpy(password, server.arg("pass").c_str());

    create_json();
    server.send(200, "application/json", json);
}

void handle_NotFound() {
    server.send(404, "text/plain", "Not found");
}
void index_html() {
    File file = SPIFFS.open("/index.html");
    server.streamFile(file, "text/HTML");
}
void css_css() {
}
void create_json() {
    json = "[{\"DevID\":" + String(DevID) +
           ",\"SubDevID\":" + String(SubDevID) +
           ",\"Flag\":" + String(Flag) +
           ",\"allFrame\":" + String(allFrame) +
           ",\"PALevel\":" + String(PALevel) +
           ",\"NRFchannel\":" + String(NRFchannel) +
           ",\"SW1TouchEnable\":" + String(SW1TouchEnable) +
           ",\"OTAActive\":" + String(OTAActive) +
           ",\"ServerActive\":" + String(ServerActive) +
           ",\"ResolutionLED\":" + String(ResolutionLED) +
           ",\"FreqLED\":" + String(FreqLED) +
           ",\"czasnazapisweeprom\":" + String(czasnazapisweeprom) +
           ",\"LightOnBoot\":" + String(LightOnBoot) +
           ",\"serialmode\":" + String(serialmode) +
           ",\"jasnosc1\":" + String(jasnosc) +
           ",\"jasnosc2\":" + String(jasnosc2) +
           ",\"jasnosc3\":" + String(jasnosc3) +
           ",\"dimmset_max\":" + String(dimmset_max) +
           ",\"dimm2set_max\":" + String(dimm2set_max) +
           ",\"dimm3set_max\":" + String(dimm3set_max) +
           ",\"DimUpDownResolution\":" + String(DimUpDownResolution) +
           ",\"RF24_rxAddr\":" + String((RF24_rxAddr >> 8) & 0xFF) +
           ",\"relayToff1\":" + String(relayToff1) +
           ",\"relayToff2\":" + String(relayToff2) +

           ",\"SSID\":\"" + String(ssid) +
           "\",\"pass\":\"" + String(password) +

           "\"},{\"frameID\":" + String(frameID) +
           ",\"celsius\":" + String(celsius) + "}]";
}

void handleRoot() {  // When URI / is requested, send a web page with a button
    // to toggle the LED
    server.send(
        200, "text/html",
        "<form action=\"/login\" method=\"POST\"><input type=\"text\" "
        "name=\"username\" placeholder=\"Username\"></br><input "
        "type=\"password\" name=\"password\" "
        "placeholder=\"Password\"></br><input type=\"submit\" "
        "value=\"Login\"></form><p>Try 'John Doe' and 'password123' ...</p>");
}
void handleLogin() {
    if (!server.hasArg("username") || !server.hasArg("password") || server.arg("username") == NULL || server.arg("password") == NULL) {
        server.send(400, "text/plain", "400: Invalid Request");
    }
    if (server.arg("username") == "John Doe" && server.arg("password") == "password123") {
        server.send(200, "text/html", "<h1>Welcome, " + server.arg("username") + "!</h1><p>Login successful</p>");
    } else {
        server.send(401, "text/plain", "401: Unauthorized");
    }
}

//<-- server

void zmiana_poziomu_jasnosci() {
    static uint64_t zapamietanyCzas1 = 0;
    static uint64_t zapamietanyCzas2 = 0;

    if (aktualnyCzas - zapamietanyCzas2 >= 2000UL) {
        if (aktualnyCzas - zapamietanyCzas1 >= 200UL) {
            zapamietanyCzas1 = aktualnyCzas;

            if (dimming_up) {
                if (dimmset_now < 15) {
                    dimmset_now += 1;
                } else if ((dimmset_now >= 15) && (dimmset_now <= 40)) {
                    dimmset_now += 5;
                } else {
                    dimmset_now += DimUpDownResolution;
                }
            } else {
                if (dimmset_now < 15) {
                    dimmset_now -= 1;
                } else if ((dimmset_now >= 15) && (dimmset_now <= 40)) {
                    dimmset_now -= 5;
                } else {
                    dimmset_now -= DimUpDownResolution;
                }
            }

            if (dimmset_now > dimmset_max) {
                dimmset_now = dimmset_max;
                zapamietanyCzas2 = aktualnyCzas;
            }
            if (dimmset_now < dimmset_min) {
                dimmset_now = dimmset_min;
                zapamietanyCzas2 = aktualnyCzas;
            }
            dimmset_last = dimmset_now;
            jasnosc = dimmset_now;
            dim1 = 1;  // dim2=0; dim3=0;
            jasnoscLED();
        }
        if (dimmset_now >= dimmset_max || dimmset_now <= dimmset_min) {
            dimming_up = !dimming_up;
        }
    }
}

void oneClick() {
    if (serialmode == 1) Serial.println("oneClick");
    //pierwszy raz po reset
    if (dimmset_now == 0 && dimmset_last == 0) {
        dimmset_now = dimmset_max;
        dimmset_last = dimmset_now;
        jasnosc = dimmset_now;
        dim1 = 1;
        jasnoscLED();

    }
    //wlaczamy na poprzedni poziom
    else if (dimmset_now == 0) {
        dimmset_now = dimmset_last;
        jasnosc = dimmset_now;
        dim1 = 1;
        jasnoscLED();

    }
    //wylaczamy
    else if (dimmset_now > 0 && dimmset_last <= dimmset_now) {
        dimmset_now = 0;
        jasnosc = dimmset_now;
        dim1 = 1;
        jasnoscLED();
    }
}
void DoubleClick() {
    if (serialmode == 1) Serial.println("DoubleClick");
    //wlaczamy na MAX
    if (dimmset_now > 0 && dimmset_last < (dimmset_max + 1) && dimmset_now < (dimmset_max + 1)) {
        dimmset_last = dimmset_now;
        dimmset_now = (dimmset_max + 1);
        jasnosc = dimmset_now;
        dim1 = 1;
        jasnoscLED();

    }
    //wlaczamy na poprzedni poziom
    else if (dimmset_now == (dimmset_max + 1) && dimmset_last != (dimmset_max + 1)) {
        dimmset_now = dimmset_last;
        jasnosc = dimmset_now;
        dim1 = 1;
        jasnoscLED();

    }
    //na MAX z wylaczonego
    else if (dimmset_now == 0) {
        dimmset_now = (dimmset_max + 1);
        jasnosc = dimmset_now;
        dim1 = 1;
        jasnoscLED();
    }
}

void LongPressStart() {
    if (serialmode == 1) Serial.println("LongPressStart");
    button_is_long_pressed = 1;
}
void LongPressStop() {
    if (serialmode == 1) Serial.println("LongPressStop");
    button_is_long_pressed = 0;
}

//<-- DLA dimmer
void jasnoscLED() {
    uint16_t TMPjasnosc = 0, TMPjasnosc2 = 0, TMPjasnosc3 = 0;

    if (dim1 == 1) {
        if (SubDevID != 0) {
            sendCAN(DevID, 3, dimmset_now, 0);
        } else {
            sendNRF(3, dimmset_now);
        }

        //led[0].setupMax(jasnosc1);
        eepromBUF[0] = readEEPROM(0);
        eepromBUF[1] = readEEPROM(1);
        TMPjasnosc = eepromBUF[0] * 256 + eepromBUF[1];  // odczyt zapisanej jasnosci
        if ((jasnosc > 0 && jasnosc < dimmset_max - 1 && TMPjasnosc != jasnosc)) {
            dim1 = 1;
        } else {
            dim1 = 0;
        }
    }

    if (dim2 == 1) {
        if (SubDevID != 0) {
            sendCAN(DevID, 4, dimmset_now, 0);
        } else {
            sendNRF(4, dimmset_now);
        }
        //led[1].setupMax(jasnosc2);
        eepromBUF[6] = readEEPROM(6);
        eepromBUF[7] = readEEPROM(7);
        TMPjasnosc2 = eepromBUF[6] * 256 + eepromBUF[7];
        if ((jasnosc2 > 0 && jasnosc2 < dimm2set_max - 1 && TMPjasnosc2 != jasnosc2)) {
            dim2 = 1;
        } else {
            dim2 = 0;
        }
    }

    if (dim3 == 1) {
        if (SubDevID != 0) {
            sendCAN(DevID, 5, dimmset_now, 0);
        } else {
            sendNRF(5, dimmset_now);
        }
        //led[2].setupMax(jasnosc3);
        eepromBUF[8] = readEEPROM(8);
        eepromBUF[9] = readEEPROM(9);
        TMPjasnosc3 = eepromBUF[8] * 256 + eepromBUF[9];
        if ((jasnosc3 > 0 && jasnosc3 < dimm3set_max - 1 && TMPjasnosc3 != jasnosc3)) {
            dim3 = 1;
        } else {
            dim3 = 0;
        }
    }

    rysuj_jasnosc_na_lcd();

    if (dim1 == 1 || dim2 == 1 || dim3 == 1) {
        //jasnosc_last = jasnosc;
        zapamietanyCzas1 = aktualnyCzas;
        zapiszweeprom = 1;
    }
}

//--> eeprom
void DimmWriteEEPROM() {  // zapisywanie z opoznienim aktualnej nastawy jasnosci LED do EEPROM
    if (aktualnyCzas - zapamietanyCzas1 >= czasnazapisweeprom) {
        if (dim1 == 1) {
            if (aktualnyCzas - zapamietanyCzasEEPROM >= 6) {
                dim1 = 0;
                if (serialmode == 1) Serial.println("Zapisywanie w EEPROM DIM 1 ...");
                writeEEPROM(0, jasnosc, 2);
            }
        }
        if (dim2 == 1) {
            if (aktualnyCzas - zapamietanyCzasEEPROM >= 6) {
                dim2 = 0;
                if (serialmode == 1) Serial.println("Zapisywanie w EEPROM DIM 2 ...");
                writeEEPROM(6, jasnosc2, 2);
            }
        }
        if (dim3 == 1) {
            if (aktualnyCzas - zapamietanyCzasEEPROM >= 6) {
                dim3 = 0;
                if (serialmode == 1) Serial.println("Zapisywanie w EEPROM DIM 3 ...");
                writeEEPROM(8, jasnosc3, 2);
            }
        }
        if (dim1 == 0 && dim2 == 0 && dim3 == 0) {
            zapiszweeprom = 0;
        }
    }
}
void writeEEPROM(unsigned int adres, uint16_t dane, uint8_t dlugosc) {
    Wire.beginTransmission(eeprom);
    Wire.write((int)(adres >> 8));    // MSB
    Wire.write((int)(adres & 0xFF));  // LSB
    switch (dlugosc) {
        case 1:
            Wire.write((byte)(dane & 0xFF));
            break;
        case 2:
            Wire.write((byte)(dane >> 8));
            Wire.write((byte)(dane & 0xFF));
            break;
    }
    Wire.endTransmission();
    if (serialmode == 1) {
        Serial.println("Zapis EEPROM ok");
    }
    zapamietanyCzasEEPROM = aktualnyCzas;
}
void writeStringToEEPROM(int adres, const String& ToWrite) {
    byte size = ToWrite.length();
    writeEEPROM(adres, size, 1);
    Serial.println(size);
    delay(6);
    for (int i = 0; i < size; i++) {
        adres++;
        writeEEPROM(adres, ToWrite[i], 1);
        Serial.print(ToWrite[i]);
        Serial.println(adres);
        delay(6);
    }
    // delay(100);
}

String readStringFromEEPROM(int addrOffset) {
    int newStrLen = readEEPROM(addrOffset);
    char data[newStrLen + 1];
    if (newStrLen <= 32) {
        for (int i = 0; i < newStrLen; i++) {
            data[i] = readEEPROM(addrOffset + 1 + i);
        }
        data[newStrLen] = '\0';
        // data[newStrLen] = "\/ 0";  // !!! NOTE !!! Remove the space between the slash "/" and "0" (I've added a space because otherwise there is a display bug)
    } else {
        if (serialmode == 1) Serial.print("Blad odczytu Strina z eeprom-za dlugi");
    }
    return String(data);
}
void SettingWriteEEPROM() {
    if (serialmode == 1) Serial.print("Zapisywanie ustawien...");

    //writeEEPROM(36, , 1);
    // delay(6);
    writeStringToEEPROM(2033, password);
    writeStringToEEPROM(2000, ssid);
    writeEEPROM(34, relayToff2, 2);  //34,35
    delay(6);
    writeEEPROM(32, relayToff1, 2);  // 32, 33
    delay(6);
    writeEEPROM(31, ((RF24_rxAddr >> 8) & 0xFF), 1);
    delay(6);
    writeEEPROM(29, dimm3set_max, 2);  // 29,30 BAJT dimm3set_max MSB
    delay(6);
    writeEEPROM(27, dimm2set_max, 2);  // 27,28 BAJT dimm2set_max MSB
    delay(6);
    writeEEPROM(26, DimUpDownResolution, 1);
    delay(6);
    writeEEPROM(24, dimmset_max, 2);
    delay(6);
    writeEEPROM(23, serialmode, 1);  // 23 BAJT serialmode
    delay(6);
    writeEEPROM(21, czasnazapisweeprom, 2);  // 21,22 BAJT czasnazapisweeprom MSB
    delay(6);
    writeEEPROM(19, FreqLED, 2);  // 19,20 BAJT FreqLED MSB 1388
    delay(6);
    writeEEPROM(18, ResolutionLED, 1);  // 18 BAJT ResolutionLED
    delay(6);
    writeEEPROM(17, ServerActive, 1);  // 17 BAJT ServerActive
    delay(6);
    writeEEPROM(16, OTAActive, 1);  // 16 BAJT OTAActive
    delay(6);
    writeEEPROM(15, SW1TouchEnable, 1);  // 15 BAJT SW1TouchEnable
    delay(6);
    writeEEPROM(14, NRFchannel, 1);  // 14 BAJT NRFchannel
    delay(6);
    writeEEPROM(13, PALevel, 1);  // 13 BAJT PALevel
    delay(6);
    writeEEPROM(12, allFrame, 1);  // 12 BAJT allFrame
    delay(6);
    writeEEPROM(11, Flag, 1);  // 11 BAJT Flag
    delay(6);
    writeEEPROM(10, LightOnBoot, 1);  // 10 BAJT LightOnBoot
    delay(6);
    writeEEPROM(8, jasnosc3, 2);  // 8,9 BAJT DIM3 jasnosc MSB
    delay(6);
    writeEEPROM(6, jasnosc2, 2);  // 6,7 BAJT DIM2 jasnosc MSB
    delay(6);
    writeEEPROM(5, SubDevID, 1);  // 5 BAJT SubDevID
    delay(6);
    writeEEPROM(4, DevID, 1);  // 4 BAJT devID
    delay(6);
    writeEEPROM(0, jasnosc, 2);  // 0,1 BAJT dim1 jasnosc MSB

    if (serialmode == 1) Serial.println("... zakonczone. Reset ESP.");
    SettingSave = 0;
    if (ServerActive == 0) {
        wificlose();
    } else {
        ESP.restart();
    }
}
void FactoryWriteEEPROM() {
    if (serialmode == 1) Serial.print("Ustawienia fabryczne...");
    writeStringToEEPROM(2000, "ssid");
    writeStringToEEPROM(2033, "pass");
    writeEEPROM(34, 5000, 2);  //34,35
    delay(6);
    writeEEPROM(32, 5000, 2);  // 32, 33
    delay(6);
    writeEEPROM(31, 0, 1);  // RF24_rxAddr
    delay(6);
    writeEEPROM(29, 999, 2);  // 29,30 BAJT dimm3set_max MSB
    delay(6);
    writeEEPROM(27, 999, 2);  // 27,28 BAJT dimm2set_max MSB
    delay(6);
    writeEEPROM(26, 20, 1);  // 26 DimUpDownResolution
    delay(6);
    writeEEPROM(24, 999, 2);  // 24, 25 dimmset_max
    delay(6);
    writeEEPROM(23, 1, 1);  // 23 BAJT serialmode
    delay(6);
    writeEEPROM(21, 6000, 2);  // 21,22 BAJT czasnazapisweeprom MSB
    delay(6);
    writeEEPROM(19, 25000, 2);  // 19,20 BAJT FreqLED MSB 1388
    delay(6);
    writeEEPROM(18, 10, 1);  // 18 BAJT ResolutionLED
    delay(6);
    writeEEPROM(17, 1, 1);  // 17 BAJT ServerActive
    delay(6);
    writeEEPROM(16, 0, 1);  // 16 BAJT OTAActive
    delay(6);
    writeEEPROM(15, 0, 1);  // 15 BAJT SW1TouchEnable
    delay(6);
    writeEEPROM(14, 1, 1);  // 14 BAJT NRFchannel
    delay(6);
    writeEEPROM(13, 1, 1);  // 13 BAJT PALevel
    delay(6);
    writeEEPROM(12, 1, 1);  // 12 BAJT allFrame
    delay(6);
    writeEEPROM(11, 1, 1);  // 11 BAJT Flag
    delay(6);
    writeEEPROM(10, 1, 1);  // 10 BAJT LightOnBoot
    delay(6);
    writeEEPROM(8, 6, 2);  // 8,9 BAJT DIM3 jasnosc MSB
    delay(6);
    writeEEPROM(6, 6, 2);  // 6,7 BAJT DIM2 jasnosc MSB
    delay(6);
    writeEEPROM(5, 0, 1);  // 5 BAJT SubDevID
    delay(6);
    writeEEPROM(4, 0, 1);  // 4 BAJT devID
    // delay(6);writeEEPROM(2,998,2);  // wolne
    delay(6);
    writeEEPROM(0, 6, 2);  // 0,1 BAJT dim1 jasnosc MSB
    if (serialmode == 1) Serial.println("... zakonczone. Reset ESP.");
    FactorySet = 0;
    ESP.restart();
}

byte readEEPROM(unsigned int eeaddress) {
    byte rdata = 0xFF;
    Wire.beginTransmission(eeprom);
    Wire.write((int)(eeaddress >> 8));    // MSB
    Wire.write((int)(eeaddress & 0xFF));  // LSB
    Wire.endTransmission();
    Wire.requestFrom(eeprom, 1);
    if (Wire.available()) rdata = Wire.read();
    return rdata;
}

void rysuj_jasnosc_na_lcd() {
    int val = map(dimmset_now, 0, dimmset_max, 0, 95);
    if (val < 1 && dimmset_now > 0) {
        val = 1;
    }
    display.fillRect(16, 2, 95, 11, BLACK);
    display.fillRect(16, 2, val, 11, WHITE);

    // napisy na pasku jasnosci
    if (dimmset_now == 0) {
        display.setCursor(52, 4);
        display.println("O F F");
    }
    if (dimmset_now == 2) {
        display.setCursor(52, 4);
        display.println("M I N");
    }
    if (dimmset_now == (dimmset_max + 1)) {
        display.setTextColor(BLACK);
        display.setCursor(43, 4);
        display.println("F U L L");
        display.setTextColor(WHITE);
    }
    if (dimmset_now == dimmset_max) {
        display.setTextColor(BLACK);
        display.setCursor(50, 4);
        display.println("M A X");
        display.setTextColor(WHITE);
    }

    if (poRestarcieTestSW1) {
        display.setCursor(17, 4);
        display.println("START:");
    }  // wyswietlamy tylko przy pierwszym starcie lub restarcie procka
    if (poRestarcieTestSW1 && dimmset_now > 0) {
        poRestarcieTestSW1 = 0;
    }

    display.display();
}

void rysujemy_na_lcd() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);

    display.drawRoundRect(14, 0, 99, 15, 2, WHITE);
    display.drawCircle(5, 7, 5, WHITE);
    display.drawLine(2, 4, 8, 10, WHITE);
    display.drawLine(2, 10, 8, 4, WHITE);
    display.fillCircle(122, 7, 5, WHITE);
    display.drawLine(119, 4, 125, 10, BLACK);
    display.drawLine(119, 10, 125, 4, BLACK);

    aktualizuj_timestr();
    display.setCursor(0, 16);
    display.print(timestr);
    display.print("   ");

    display.print("R1:");
    display.print(sp[0]);
    display.print("|");
    display.print("R2:");
    display.print(sp[1]);

    if (SMSbuf != "") {
        if (aktualnyCzas - zapamietanyCzasOLED >= smsoled) {
            SMSbuf = "";
        }
        display.setCursor(0, 24);
        display.print(getsmstel());
        display.setCursor(0, 32);
        display.print(SMSbuf);

    } else {
        display.setCursor(0, 24);
        display.print("CAN ID: 0x");
        display.println(frameID, HEX);  // CAN-BUS ID

        display.setCursor(0, 32);
        display.print("NRF: ");
        display.print(NRFbuf[0]);
        display.print(", ");
        display.println(NRFbuf[2] * 256 + NRFbuf[1]);
        display.setCursor(100, 32);
        display.print("PA:");
        display.println(radio.getPALevel());

        display.setFont(&FreeSansBold24pt7b);
        // dwie wersje wyswietlania temperatury, druga z miejscem dziesietnym
        display.setCursor(20, 60);
        char odczyt[4];
        dtostrf(celsius, 3, 0, odczyt);
        display.drawCircle(90, 39, 3, WHITE);
        if (celsius == -99) {
            display.print(" --");
        } else {
            display.print(odczyt);
        }
        // display.setCursor(0,60); char odczyt[5]; dtostrf(celsius,3, 1, odczyt); display.print(odczyt); display.drawCircle(95,39,3, WHITE);
        display.setFont(&FreeSans12pt7b);
        display.print(" C");
        display.setFont();

        PokazSW1NaDisplay(0);
    }
    rysuj_jasnosc_na_lcd();  //w tej funkcji na koncu jest display.display();
}

void aktualizuj_timestr() {
    timestr[0] = '0' + hour() / 10;
    timestr[1] = '0' + hour() % 10;
    timestr[3] = '0' + minute() / 10;
    timestr[4] = '0' + minute() % 10;
    timestr[6] = '0' + second() / 10;
    timestr[7] = '0' + second() % 10;
}

void InicjacjaOdczytTemperatury() {
    if (CzyJestCzujnikTemperaury > 0) {  // trzeba zadbac aby odczyt odbył sie za co najmniej 750ms
        ds.reset();
        ds.select(DallasAddr);
        ds.write(0x44, 1);
        timer.setTimeout(1000, OdczytTemperatury);
    }
}

void OdczytTemperatury() {
    byte data[12];
    ds.reset();
    ds.select(DallasAddr);
    ds.write(0xBE);
    for (byte i = 0; i < 9; i++) {
        data[i] = ds.read();
    }

    if (OneWire::crc8(data, 8) == data[8]) {
        int16_t raw = (data[1] << 8) | data[0];
        byte cfg = (data[4] & 0x60);
        if (cfg == 0x00)
            raw = raw & ~7;  // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20)
            raw = raw & ~3;  // 10 bit res, 187.5 ms
        else if (cfg == 0x40)
            raw = raw & ~1;  // 11 bit res, 375 ms
        //// default is 12 bit resolution, 750 ms conversion time
        celsius = (float)raw / 16.0;
    }
}

void sprawdzSW1Touch() {
    SW1TouchVal = pobierzTouchVal(SW1TouchPin, 1, SW1TouchPressed);  // obsluga przycisku dotyku
    if (SW1TouchVal < (SW1TouchMax - 6 + SW1TouchFilter)) {
        SW1TouchPressed = 1;
    } else {
        SW1TouchPressed = 0;
    }
    if (SW1TouchPressed > SW1TouchPrevious) {
        Serial.println("SW1 Touch : Nacisniety");
        PokazSW1NaDisplay(1);
        SW1TouchFilter = 4;
    }
    if (SW1TouchPressed < SW1TouchPrevious) {
        Serial.println("SW1 Touch : Zwolniony");
        PokazSW1NaDisplay(1);
        SW1TouchFilter = 0;
    }
    SW1TouchPrevious = SW1TouchPressed;
}

void PokazSW1NaDisplay(boolean rundisplay) {
    if (SW1TouchPressed)
        display.fillCircle(122, 54, 5, WHITE);
    else
        display.fillCircle(122, 54, 5, BLACK);
    if (rundisplay) display.display();
}

uint8_t pobierzTouchVal(uint8_t TouchPin, uint8_t czymin, boolean isTouchPressed) {
    uint8_t touchvaltmp = 0;
    uint8_t touchval = 255;
    if (isTouchPressed) {
        touchvaltmp = 255;
        for (int i = 0; i < 4; i++) {
            touchval = touchRead(TouchPin);
            if (touchval < touchvaltmp) {
                touchvaltmp = touchval;
            }
        }
    } else {
        touchvaltmp = 0;
        for (int i = 0; i < 4; i++) {
            touchval = touchRead(TouchPin);

            if (touchval > touchvaltmp) {
                touchvaltmp = touchval;
            }
        }
        if (czymin) {
            if (touchval < SW1TouchValMin) {
                SW1TouchValMin = touchval;
            }
        }  // tylko do testow czy sam nie zalaczy, test tylko w petli kiedy nie dotkniety
    }

    touchval = touchvaltmp;
    return touchval;
}

void InicjacjaSW1Touch() {
    // uruchamiane przy starcie w petli 10 razy, ustalamy poziom MAX dotyku oraz wlaczamy TouchActive
    static uint8_t licz = 0;
    static uint8_t liczval[10];
    liczval[licz++] = pobierzTouchVal(SW1TouchPin, 0, 0);  //Serial.print("licz = "); Serial.print(licz); Serial.print("liczval = "); Serial.print(liczval[(licz-1)]); Serial.println();
    if (licz == 10) {                                      //dwa pomijamy i wyciagamy srednia z kolejnych 8 odczytow
        uint16_t tvsumtmp = 0;
        for (int i = 2; i < 10; i++) {
            tvsumtmp += liczval[i];
        }
        tvsumtmp = tvsumtmp / 8;
        Serial.print("tvsumtmp = ");
        Serial.print(tvsumtmp);
        Serial.println();
        SW1TouchMax = tvsumtmp;
        SW1TouchActive = 1;
    }
}
//odczyt najnowszego sms
void readSMS() {
    if (Serial.available() > 0) {
        SMSbuf = "";
        while (Serial.available() > 0) {
            SMSbuf = Serial.readString();
        }
        SMSbuf.toUpperCase();                // wiadomosc wielkimi literami
        zapamietanyCzasOLED = aktualnyCzas;  // wlacznie czas na wylaczenie wyswietlania sms na OLED
        // tutaj beda sie wykonywaly polecenia z SMS

        if (SMSbuf.indexOf("@ONOFF1") > -1) {
            relays(1);  //pierwszy przekaznik
            GSMmsg = 1;
        }
        if (SMSbuf.indexOf("@ONOFF2") > -1) {
            relays(2);  //drugi przekaznik
            GSMmsg = 1;
        }
        if (SMSbuf.indexOf("@SERVERONOFF") > -1) {
            ServerActive = !ServerActive;
            writeEEPROM(17, ServerActive, 1);
            wificlose();
        }
        rysujemy_na_lcd();  // wyswietl SMS na OLED
        if (GSMmsg == 1) {
            // if (SMSbuf.indexOf("OK") == -1) {
            Serial.println("AT+CMGDA=\"DEL ALL\"");  //usuwamy wszystkie SMS
            // }
            GSMmsg = 0;
        }
    }
}

String getsmstel() {
    if (SMSbuf.length() > 10) {
        uint8_t _idx1 = SMSbuf.indexOf("\"+");
        return SMSbuf.substring(_idx1 + 1, SMSbuf.indexOf("\",\"", _idx1 + 4));
    } else {
        return "";
    }
}
//dla spowolnienia www
void serverON() {
    server.handleClient();
}
// załączenie przekaznika fizycznego i wirtualnego. 1 i 2 to fizyczne wyjscie
void relays(uint8_t n) {
    sp[n - 1] = !sp[n - 1];
    if (n <= 2) {
        digitalWrite(relay[n - 1], sp[n - 1]);
        // Sim800l.sendSms(getsmstel(), String(sp[n - 1]));
    } else {  // NIE TESTOWANE
        uint16_t data = (n << 8) + sp[n - 1];
        sendNRF(6, data);
        sendCAN(255, 6, data, 0);  //toID - 255 dla wszystkich,
    }
}
void relaydimonoff() {
    if ((dimmset_now > 0 || dimmset_now2 > 0 || dimmset_now3 > 0) && sp[0] == 0) {
        // if (dimmset_now > 0 && sp[0] == 0) {
        relays(1);
        roff = 1;
    } else if (roff == 1 && dimmset_now == 0 && dimmset_now2 == 0 && dimmset_now3 == 0 && sp[0] == 1) {
        // } else if (roff == 1 && dimmset_now == 0 && sp[0] == 1) {
        zapczas = aktualnyCzas;
        roff = 0;
    } else if (roff == 0 && (aktualnyCzas - zapczas >= (relayToff1 * 1000))) {  //relayToff
        roff = 1;
        relays(1);
    }
}

// wymyslic kasowanie wiadomosci.
// !! - sprawdzic

// ustawienie dodatkowe aby od godziny np 23 / w nocy wlaczalo swiatlo na minimalny poziom. ale chyba to ustawienie do mastera.
// dokonczyc przetestowac can.
// napisac funkcje do doladowywania/ wysylania specjalnych sms w celu wlaczenia uslugi np dodatkowe sms.
// napisac synchronizacje czasu przez CAN wysylajace do wszystkich modołów z mastera (255).
// napisac restart modulu GSM co x czasu
// napisac obsluge wielu przyciskow. napewno jeszcze 2 do sterowanie dim2 dim3.
// po sprawdzaniu polaczenia wifi gdy nie bedzie polaczone napisac skrypt aby probowal sie polaczyc ponownie
// opcja www Zawsze probuj laczyc do skonfigurowanej sieci wifi, jesli skonfigurowano wifi to zawsze lacz do niej

// WWW - zapamietanie ssid, pass do eeprom, po nieudanej probie logowania do sieci lokalnej wlacza sie AP mode. Sprawdzanie polaczenia wifi co 5min. Jesli bedzie rozlaczene to bedzie probowac laczyc x co x czas.