#include "button_transmitter.h"

int zaok = 5;          // zaokraglenia suwaka jasnosci na oled
int dimActivNumb = 0;  //dla rysowania jasnosci na oled.
bool checkOne1 = 1;

unsigned int czasnazapisweeprom = 6000;  //w milisekundach, 6 sekund, w przypadku brak zmiany jasnosci przez ten czas powoduje zapis w eeprom

//const byte portLED = 19, portLED2 = 18, portLED3 = 17;  // dostepne: DIM1/R - 19 | DIM2/G - 18 | DIM3/B - 17

uint16_t FreqLED = 25000;     // czestotliwosc LED
uint16_t ResolutionLED = 10;  // Rozdzielczosc w bitach.

// zmiennych ponizej nie edytujemy
// unsigned int jasnosc = 0, jasnosc2 = 0, jasnosc3 = 0;  // wykorzystywane do zapisywnie w eeprom
// unsigned int jasnosc_last = 0;  // !! chyba nigdzie nie jest wykorzystywane
// unsigned int jasnosc_max = 0;   // !! chyba nigdzie nie jest wykorzystywane
byte eepromBUF[64];  // bufor dla zapisu/odczytu eeprom
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
int dimmsetNow[4] = {0};
uint8_t numDIM = 0;
int dimmsetLast[4] = {0};
int dimmsetLastNL[4] = {7, 7, 7, 7};  // for night light function in Qtimers class
int dimmsetMax[4] = {999, 999, 999, 999};
int16_t dimmset_now = 0, dimmset_now2 = 0, dimmset_now3 = 0;
// int16_t dimmset_last = 0, dimmset_last2 = 0, dimmset_last3 = 0;
// int16_t dimmset_max = 999, dimm2set_max = 999, dimm3set_max = 999;

int16_t dimmset_min = 2;

//<--Dimmer
//--> server
boolean ServerActive = 1;

char* ssidAP = strdup("SW");              // Enter SSID here
char* passwordAP = strdup("1234567890");  // Enter Password here
// char* ssid = strdup("NET2");
// char* password = strdup("Janek1Ewa2-s7");
// char* ssid = strdup("JLU");
//char* password = strdup("1234567890");
char* ssid = strdup("NET");
char* password = strdup("Janek1@$Eva");
char* ssidF = strdup("JLU");
char* passwordF = strdup("1234567890");
char* hostname = strdup("1234567890");
// String hostname = "SW";
char* nameDev = strdup("nD");
WebServer server(80);
bool wifiSTAon = 0, wifiAPon = 1, buttonWIFIactived = 0, wifiAPconnected = 0, wifiStartConnecting = 0;
uint8_t wifiSTAonCN = 0;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 0, 60000);  // ntp

int TimerS1[4] = {0, 0, 0, 0};  // FOR Qtimer hour, minutes ,day ,month
int TimerE1[4] = {0, 0, 0, 0};
int TimerS2[4] = {0, 0, 0, 0};  // FOR Qtimer hour, minutes ,day ,month
int TimerE2[4] = {0, 0, 0, 0};
//<--server

//--> CAN-BUS

uint8_t Flag = 0;      // 1 bit
uint8_t SubDevID = 0;  // 4 bity od 0 do 15. wykorzystywane np po to, aby zarzadzac drugim modulem po CAN z tym zamym DevID. Gdy nadajnik i dimmer sa  polaczone z CAN-BUS dajemy w dimmer 1-15
uint8_t DevID = 2;     // 8bit ID tego urzadzenia, ID 1 to master.
uint32_t frameID = 0;
bool allFrame = 0;         // jesli 0 to przyjmuje ramki z filtrem
bool OnDevices[25] = {0};  // lista wlaczonych urzadzen - tylko dla mastera. test
//<-- CAN-BUS

#define eeprom 0x50  // Address of 24LC256 eeprom chip

SimpleTimer timer;

const uint8_t SW1pin = 19, SW2pin = 18, SW3pin = 17, SW4pin = 16;
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

const uint8_t logo16_wifi_bmp[] PROGMEM =  //logo wifi 16
    {
        0x00, 0x00, 0x00, 0x00, 0xE0, 0x07, 0x38, 0x1C, 0xC4, 0x23, 0x72, 0x4E,
        0x08, 0x10, 0xE4, 0x27, 0x10, 0x0C, 0x90, 0x09, 0x40, 0x02, 0x60, 0x06,
        0x40, 0x02, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00};
const uint8_t Signal816[16] PROGMEM =  //mobie signal
    {
        0xFE, 0x02, 0x92, 0x0A, 0x54, 0x2A, 0x38, 0xAA, 0x12, 0xAA, 0x12, 0xAA, 0x12, 0xAA, 0x12, 0xAA};

const uint8_t Msg816[16] PROGMEM =  //message
    {
        0x1F, 0xF8, 0x10, 0x08, 0x18, 0x18, 0x14, 0x28, 0x13, 0xC8, 0x10, 0x08, 0x10, 0x08, 0x1F, 0xF8};

// OneButton button(SW1pin, true, 1);
OneButton button(SW1pin, true);   //SW1pin
OneButton button1(SW2pin, true);  //SW2pin
OneButton button2(SW3pin, true);
OneButton button3(SW4pin, true);
bool button_is_long_pressed = 0;
bool dimming_up = 0;
bool poRestarcieTestSW1 = 1;
uint8_t bWIFIenable = 0;
char timestr[10] = "--:--:--";
char* Timestamp = strdup("575420400");

int Qhour, Qminutes, Qsecounds, Qday, Qmonth;
Qtimers tl1(1, 1, 7, 53);  // active, function, hour, minutes, day, montch

boolean SW1TouchEnable = 0;  // aktywacja funkcji przycisku dotyku
boolean SW1TouchActive = 0;  // dotyk uruchamia sie dopiero po inicjalizacji
uint8_t SW1TouchVal = 255;
uint8_t SW1TouchMax = 255;  // ustalana po inicjalizacji wartosc dotyku nie nacisnietego
boolean SW1TouchPressed = 0;
boolean SW1TouchPrevious = 0;
uint8_t SW1TouchFilter = 0;
int SW1TouchValMin = 255;  //tylko do testow

boolean OTAActive = 0, SettingSave = 0;
uint8_t FactorySet = 0;
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

int timerWifiSTAcheck = 0, timerWifiSTACon = 0, timerServerON = 0, timerbuttonWIFIdisable = 0, timertimeNetUpdate = 0;
//adresy eeprom
int FactorySetE = 37;
int nameDevE = 2200;

// active, function, hour, minutes, day, montch
void setup() {
    //--> EEPROM
    Wire.begin(21, 22);  // sda, scl
    Wire.setClock(4000000);

    FactorySet = readEEPROM(FactorySetE);
    if (FactorySet != 123) {
        FactoryWriteEEPROM();
    } else {
        for (int i = 0; i < 64; ++i) {
            eepromBUF[i] = readEEPROM(i);
        }  // read 64 byte eeprom
        // jasnosc = eepromBUF[0] * 256 + eepromBUF[1];
        // jasnosc2 = eepromBUF[6] * 256 + eepromBUF[7];
        // jasnosc3 = eepromBUF[8] * 256 + eepromBUF[9];
        dimmsetLast[0] = eepromBUF[0] * 256 + eepromBUF[1];
        dimmsetLast[1] = eepromBUF[6] * 256 + eepromBUF[7];
        dimmsetLast[2] = eepromBUF[8] * 256 + eepromBUF[9];

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
        dimmsetMax[0] = eepromBUF[24] * 256 + eepromBUF[25];
        DimUpDownResolution = eepromBUF[26];
        dimmsetMax[1] = eepromBUF[27] * 256 + eepromBUF[28];
        dimmsetMax[2] = eepromBUF[29] * 256 + eepromBUF[30];
        RF24_rxAddr = (127 << 24) + (118 << 16) + (eepromBUF[31] << 8) + 0;
        relayToff1 = eepromBUF[32] * 256 + eepromBUF[33];
        relayToff2 = eepromBUF[34] * 256 + eepromBUF[35];
        wifiAPon = eepromBUF[36];

        strcpy(ssid, readStringFromEEPROM(2000).c_str());
        strcpy(password, readStringFromEEPROM(2033).c_str());
        strcpy(Timestamp, readStringFromEEPROM(2066).c_str());
        strcpy(nameDev, readStringFromEEPROM(nameDevE).c_str());
    }
    //dimmset_now = jasnosc;

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
        Serial.println(dimmsetLast[0]);
        Serial.print("jasnosc2: ");
        Serial.println(dimmsetLast[1]);
        Serial.print("jasnosc3: ");
        Serial.println(dimmsetLast[2]);
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
        Serial.print("dimmsetMax[0]: ");
        Serial.println(dimmsetMax[0]);
        Serial.print("dimmsetMax[1]: ");
        Serial.println(dimmsetMax[1]);
        Serial.print("dimmsetMax[2]: ");
        Serial.println(dimmsetMax[2]);
        Serial.print("DimUpDownResolution: ");
        Serial.println(DimUpDownResolution);
        Serial.print("Pelny RF24_rxAddr: ");
        Serial.println(RF24_rxAddr);
        Serial.print("RF24_rxAddr: ");
        Serial.println((RF24_rxAddr >> 8) & 255);
        Serial.print("dimmsetNow[0]: ");
        Serial.println(dimmsetNow[0]);
        Serial.print("dimmsetNow[1]: ");
        Serial.println(dimmsetNow[1]);
        Serial.print("dimmsetNow[2]: ");
        Serial.println(dimmsetNow[2]);
        Serial.print("ssi: ");
        Serial.println(ssid);
        Serial.print("password: ");
        Serial.println(password);
        Serial.print("wifiAPon: ");
        Serial.println(wifiAPon);
        Serial.print("nameDev: ");
        Serial.println(nameDev);
    }

    strcpy(hostname, ("SW_" + String(nameDev) + "-" + String(DevID) + "_" + mcADR() + "").c_str());
    //<-- EEPROM
    /*
    // -->  ESP32 Dimmer settings
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
        dimmsetNow[0] = jasnosc;  // poprwka przekazanie jasnosci on boot z eeprom tez na wyswietlacz
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

    // dimmsetMax[0] -= 1; // !! ??

    button.attachClick(oneClick0);
    button.attachDoubleClick(DoubleClick0);
    button.attachLongPressStart(LongPressStart0);
    button.attachLongPressStop(LongPressStop);

    button1.attachClick(oneClick1);
    button1.attachDoubleClick(DoubleClick1);
    button1.attachLongPressStart(LongPressStart1);
    button1.attachLongPressStop(LongPressStop);

    button2.attachClick(oneClick2);
    button2.attachDoubleClick(DoubleClick2);
    button2.attachLongPressStart(LongPressStart2);
    button2.attachLongPressStop(LongPressStop);

    // button 3 (fizycznie 4) narazie nie obslugiwany
    button3.attachClick(oneClick0);
    button3.attachDoubleClick(DoubleClick0);
    button3.attachLongPressStart(LongPressStart0);
    button3.attachLongPressStop(LongPressStop);
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

    if (serialmode == 2) {
        int cz = 0;
        while (cz < 20) {
            Serial.println("AT");
            delay(200);
            readSMS();
            if (SMSbuf.indexOf("OK") > -1) {
                // SMSbuf = "GSM zainicjowany";
                // rysujemy_na_lcd();
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
    }  // else {
    //     SMSbuf = "GSM wylaczony";
    //     rysujemy_na_lcd();
    // }

    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS init error");
        return;
    }  // inicjowanie systemu plikow SPI

    // setTime((1629961343959 / 1000) + 7200);  // ustawia zegarek

    if (ServerActive == 1) {
        // wifiapstart();
        wifiSTAstart();
    }

    //// TIMERY ////
    ///////////////

    timerbuttonWIFIdisable = timer.setInterval(600000, buttonWIFIdisable);  //za 10 min
    timer.setInterval(1000, rysujemy_na_lcd);
    timer.setInterval(2000, InicjacjaOdczytTemperatury);
    timerWifiSTAcheck = timer.setInterval(60000, wifiSTAcheck);  // co 5min 300000
    timerServerON = timer.setInterval(150, serverON);            //1500
    timerWifiSTACon = timer.setInterval(6000, wifiSTAconnecting);
    timertimeNetUpdate = timer.setInterval(43200000, timeNetUpdate);  // co 12h

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

    timer.disable(timerbuttonWIFIdisable);
    if (ServerActive != 1) {
        timer.disable(timerWifiSTAcheck);
        timer.disable(timerServerON);
        timer.disable(timerWifiSTACon);
    }
    timer.disable(timertimeNetUpdate);

    if (gsminit == 1) {
        timer.setInterval(500, readSMS);
    }

    if (SW1TouchEnable) {
        timer.setTimer(100, InicjacjaSW1Touch, 10);
        touch_pad_init();
        touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
        touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    }
    setTime(String(Timestamp).toInt() + 7200);  //ustawiamy zegarek
}
////////////////////////////
// LOOP ////////////////////
////////////////////////////

void loop() {
    aktualnyCzas = millis();
    timer.run();
    LightDimmerESP32::update();

    if (SW1TouchActive == 1) {
        sprawdzSW1Touch();
        button.tick(SW1TouchPressed);
    } else {
        button.tick();
        button1.tick();
        button2.tick();
        button3.tick();
    }
    readCAN();
    readNRF();
    relaydimonoff();
    if (button_is_long_pressed == 1) {
        zmiana_poziomu_jasnosci(numDIM);
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
}
////////////////////////////
// KONIEC LOOP ////////////////////
////////////////////////////

void EnableOTA() {
    ArduinoOTA.begin();
    if (serialmode == 1) Serial.println("OTA aktywne");
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
                dimmsetNow[0] = NRFbuf[2] * 256 + NRFbuf[1];
                dim1 = 1;
                jasnoscLED(0, dimmsetNow[0]);
                if (serialmode == 1) {
                    Serial.print("Jasnosc odebrana z nadajnika:");
                    Serial.println(dimmsetNow[0]);
                }
                break;
            case 4:
                dimmsetNow[1] = NRFbuf[2] * 256 + NRFbuf[1];
                dim2 = 1;
                jasnoscLED(1, dimmsetNow[1]);
                if (serialmode == 1) {
                    Serial.print("Jasnosc odebrana z nadajnika:");
                    Serial.println(dimmsetNow[1]);
                }
                break;
            case 5:
                dimmsetNow[2] = NRFbuf[2] * 256 + NRFbuf[1];
                dim3 = 1;
                jasnoscLED(2, dimmsetNow[2]);
                if (serialmode == 1) {
                    Serial.print("Jasnosc odebrana z nadajnika:");
                    Serial.println(dimmsetNow[2]);
                }
                break;
            case 6: {  // przekaznik wirtualny
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
                    Serial.println("fnID 1, ramka wyslana");
                }
                break;
            case 2:  // synchronizacja czasu
                setTime(inFrame.data.uint32[0] + 7200);
                break;
            case 3:  // dimmer 1 // NIE TESTOWANE
                dimmsetNow[0] = inFrame.data.uint16[0];
                dim1 = 1;
                jasnoscLED(0, dimmsetNow[0]);
                break;
            case 4:  // dimmer 2 // NIE TESTOWANE
                dimmsetNow[1] = inFrame.data.uint16[0];
                dim2 = 1;
                jasnoscLED(1, dimmsetNow[1]);
                break;
            case 5:  // dimmer 3 // NIE TESTOWANE
                dimmsetNow[2] = inFrame.data.uint16[0];
                dim3 = 1;
                jasnoscLED(2, dimmsetNow[2]);
                break;
            case 6:  // wszystkie !! NIE TESTOWANE
                dimmsetNow[0] = inFrame.data.uint16[0];
                dim1 = 1;
                jasnoscLED(0, dimmsetNow[0]);
                dimmsetNow[1] = inFrame.data.uint16[1];
                dim2 = 1;
                jasnoscLED(1, dimmsetNow[1]);
                dimmsetNow[2] = inFrame.data.uint16[2];
                dim3 = 1;
                jasnoscLED(2, dimmsetNow[2]);
                if (serialmode == 1) {
                    Serial.print("Jasnosc1: ");
                    Serial.println(dimmsetNow[0]);
                    Serial.print("Jasnosc2: ");
                    Serial.println(dimmsetNow[1]);
                    Serial.print("Jasnosc3: ");
                    Serial.println(dimmsetNow[2]);
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
    timer.disable(timerWifiSTAcheck);
    timer.disable(timerWifiSTACon);
    timer.disable(timerServerON);
    timer.disable(timertimeNetUpdate);
    wifiAPconnected = 0;
    wifiSTAonCN = 0;
    wifiSTAon = 0;
    wifiStartConnecting = 0;
    ArduinoOTA.end();
    server.stop();
    timeClient.end();
    if (serialmode == 1) {
        Serial.println("WiFi OTA wylaczone. ");
        Serial.print("wifiAPconnected: ");
        Serial.println(wifiAPconnected);
        Serial.print("wifiSTAonCN: ");
        Serial.println(wifiSTAonCN);
        Serial.print("wifiSTAon: ");
        Serial.println(wifiSTAon);
    }
    // delay(100);
    // ESP.restart();
}
void wifiapstart() {
    if (serialmode == 1) {
        Serial.print("Laczenie wifi AP ");
    }
    timer.disable(timerWifiSTAcheck);
    WiFi.persistent(false);
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    //WiFi.mode(WIFI_STA);
    delay(100);

    IPAddress local_ip(192, 168, 1, 100);
    IPAddress gateway(192, 168, 1, 1);
    IPAddress subnet(255, 255, 255, 0);

    WiFi.softAPConfig(local_ip, gateway, subnet);
    WiFi.softAP(hostname, passwordAP);
    server.begin();
    wifiAPconnected = 1;
}
void wifiSTAstart() {
    if (serialmode == 1) {
        Serial.print("wifiSTAstart ");
    }
    if (ServerActive != 1) {
        timer.enable(timerWifiSTAcheck);
        timer.enable(timerWifiSTACon);
    }
    wifiStartConnecting = 1;
    WiFi.persistent(false);
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    delay(100);
    WiFi.mode(WIFI_STA);

    WiFi.setHostname(hostname);
    if (wifiSTAonCN == 4) {
        WiFi.begin(ssidF, passwordF);
    } else {
        WiFi.begin(ssid, password);
    }

    if ((wifiSTAonCN >= 5) && (wifiAPon == 1)) {
        if (serialmode == 1) {
            Serial.print(" Brak polaczenia STA mode. ");
        }
        wifiSTAonCN = 0;
        wifiapstart();
    }
}
void wifiSTAconnecting() {
    if ((WiFi.status() != WL_CONNECTED) && (wifiAPconnected == 0)) {
        wifiSTAon = 0;
        wifiSTAonCN++;
        if (serialmode == 1) {
            Serial.print("Laczenie do WIFI - STA mode. ");
            Serial.println(" Ilosc prob: " + String(wifiSTAonCN) + "");
        }
        wifiSTAstart();
        if (wifiSTAonCN >= 5) {
            wifiSTAonCN = 0;
            timer.disable(timerWifiSTACon);
            if (serialmode == 1) {
                Serial.println("timerWifiSTACon - disable ");
            }
        }

    } else {
        wifiSTAcheck();
    }
}
void wifiSTAcheck() {
    if ((WiFi.status() != WL_CONNECTED) && (wifiAPconnected == 0)) {
        if (serialmode == 1) {
            Serial.println("Brak polaczenia WIFI - STA mode. ");
        }
        wifiSTAonCN = 0;
        timer.enable(timerWifiSTACon);
        timer.restartTimer(timerWifiSTACon);
        if (serialmode == 1) {
            Serial.println("timerWifiSTACon - enable ");
        }
    } else {
        wifiSTAonCN = 0;
        if ((wifiSTAon == 0) || (wifiAPconnected == 1)) {
            if (wifiAPconnected != 1) {
                wifiSTAon = 1;
                timeClient.begin();                // NTP
                timer.enable(timertimeNetUpdate);  // NTP
                zapamietanyCzasOLED = aktualnyCzas;
                SMSbuf = "IP: " + WiFi.localIP().toString() + "";
                timeNetUpdate();
            }
            timer.disable(timerWifiSTACon);

            if (wifiAPconnected == 1) {
                zapamietanyCzasOLED = aktualnyCzas;
                SMSbuf = "IP: 192.168.1.100";
            } else {
                zapamietanyCzasOLED = aktualnyCzas;
                SMSbuf = "IP: " + WiFi.localIP().toString() + "";
            }

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
            server.begin();
            timer.enable(timerServerON);
            timer.restartTimer(timerServerON);

            if (serialmode == 1) {
                Serial.println("HTTP server started");
            }
            if ((OTAActive == 1) || (buttonWIFIactived == 1)) {
                EnableOTA();
            }
        }
        if (serialmode == 1) {
            if (wifiAPconnected == 1) {
                Serial.println("WiFi AP OK. 192.168.1.100");
            } else {
                Serial.print("WiFi STA OK - IP addres: ");
                Serial.println(WiFi.localIP());
            }
        }
    }
}
void buttonWIFIenable() {
    bWIFIenable = 0;
    if (serialmode == 1) {
        Serial.println("buttonWIFIenable");
    }
    timer.enable(timerbuttonWIFIdisable);
    timer.restartTimer(timerbuttonWIFIdisable);
    wifiSTAstart();
    buttonWIFIactived = 1;
}
void buttonWIFIdisable() {
    timer.disable(timerbuttonWIFIdisable);
    buttonWIFIactived = 0;
    wificlose();
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

    dimmsetNow[0] = server.arg("jasnosc1").toInt();
    jasnoscLED(0, dimmsetNow[0]);
    dimmsetMax[0] = server.arg("dimmset_max").toInt();

    dimmsetNow[1] = server.arg("jasnosc2").toInt();
    jasnoscLED(1, dimmsetNow[1]);
    dimmsetMax[1] = server.arg("dimm2set_max").toInt();

    dimmsetNow[2] = server.arg("jasnosc3").toInt();
    jasnoscLED(2, dimmsetNow[2]);
    dimmsetMax[2] = server.arg("dimm3set_max").toInt();

    relayToff1 = server.arg("relayToff1").toInt();
    relayToff2 = server.arg("relayToff2").toInt();

    strcpy(ssid, server.arg("SSID").c_str());
    strcpy(password, server.arg("pass").c_str());
    strcpy(nameDev, server.arg("nameDev").c_str());
    wifiAPon = server.arg("wifiAPon").toInt();
    strcpy(Timestamp, server.arg("ttime").c_str());
    setTime(String(Timestamp).toInt() + 7200);

    TimerS1[0] = server.arg("timerS1h").toInt();
    TimerS1[1] = server.arg("timerS1m").toInt();
    TimerS1[2] = server.arg("timerS1day").toInt();
    TimerS1[3] = server.arg("timerS1month").toInt();
    TimerE1[0] = server.arg("timerS1h").toInt();
    TimerE1[1] = server.arg("timerS1m").toInt();
    TimerE1[2] = server.arg("timerS1day").toInt();
    TimerE1[3] = server.arg("timerS1month").toInt();

    TimerS2[0] = server.arg("timerS2h").toInt();
    TimerS2[1] = server.arg("timerS2m").toInt();
    TimerS2[2] = server.arg("timerS2day").toInt();
    TimerS2[3] = server.arg("timerS2month").toInt();
    TimerE2[0] = server.arg("timerS2h").toInt();
    TimerE2[1] = server.arg("timerS2m").toInt();
    TimerE2[2] = server.arg("timerS2day").toInt();
    TimerE2[3] = server.arg("timerS2month").toInt();

    create_json();
    server.send(200, "application/json", json);
    SettingSave = server.arg("eeprom_save").toInt();
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
           ",\"jasnosc1\":" + String(readEEPROM(0) * 256 + readEEPROM(1)) +
           ",\"jasnosc2\":" + String(readEEPROM(6) * 256 + readEEPROM(7)) +
           ",\"jasnosc3\":" + String(readEEPROM(8) * 256 + readEEPROM(9)) +
           ",\"dimmset_max\":" + String(dimmsetMax[0]) +
           ",\"dimm2set_max\":" + String(dimmsetMax[1]) +
           ",\"dimm3set_max\":" + String(dimmsetMax[2]) +
           ",\"DimUpDownResolution\":" + String(DimUpDownResolution) +
           ",\"RF24_rxAddr\":" + String((RF24_rxAddr >> 8) & 0xFF) +
           ",\"relayToff1\":" + String(relayToff1) +
           ",\"relayToff2\":" + String(relayToff2) +
           ",\"wifiAPon\":" + String(wifiAPon) +

           ",\"SSID\":\"" + String(ssid) +
           "\",\"pass\":\"" + String(password) +
           "\",\"nameDev\":\"" + String(nameDev) +
           "\"},{\"frameID\":" + String(frameID) +
           ",\"celsius\":" + String(celsius) +
           "},{\"timerS1h\":" + String(TimerS1[0]) +
           ",\"timerS1m\":" + String(TimerS1[1]) +
           ",\"timerS1day\":" + String(TimerS1[2]) +
           ",\"timerS1month\":" + String(TimerS1[3]) +
           ",\"timerE1h\":" + String(TimerE1[0]) +
           ",\"timerE1m\":" + String(TimerE1[1]) +
           ",\"timerE1day\":" + String(TimerE1[2]) +
           ",\"timerE1month\":" + String(TimerE1[3]) +

           ",\"timerS2h\":" + String(TimerS2[0]) +
           ",\"timerS2m\":" + String(TimerS2[1]) +
           ",\"timerS2day\":" + String(TimerS2[2]) +
           ",\"timerS2month\":" + String(TimerS2[3]) +
           ",\"timerE2h\":" + String(TimerE2[0]) +
           ",\"timerE2m\":" + String(TimerE2[1]) +
           ",\"timerE2day\":" + String(TimerE2[2]) +
           ",\"timerE2month\":" + String(TimerE2[3]) +

           ",}]";
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

void zmiana_poziomu_jasnosci(uint16_t nrLED) {
    static uint64_t zapamietanyCzas1 = 0;
    static uint64_t zapamietanyCzas2 = 0;
    // dimmset_now = dimmsetLast[nrLED];
    if (aktualnyCzas - zapamietanyCzas2 >= 2000UL) {
        if (aktualnyCzas - zapamietanyCzas1 >= 200UL) {
            zapamietanyCzas1 = aktualnyCzas;

            if (dimming_up) {
                if (dimmsetNow[nrLED] < 15) {
                    dimmsetNow[nrLED] += 1;
                } else if ((dimmsetNow[nrLED] >= 15) && (dimmsetNow[nrLED] <= 40)) {
                    dimmsetNow[nrLED] += 5;
                } else {
                    dimmsetNow[nrLED] += DimUpDownResolution;
                }
            } else {
                if (dimmsetNow[nrLED] < 15) {
                    dimmsetNow[nrLED] -= 1;
                } else if ((dimmsetNow[nrLED] >= 15) && (dimmsetNow[nrLED] <= 40)) {
                    dimmsetNow[nrLED] -= 5;
                } else {
                    dimmsetNow[nrLED] -= DimUpDownResolution;
                }
            }

            if (dimmsetNow[nrLED] > dimmsetMax[nrLED]) {
                dimmsetNow[nrLED] = dimmsetMax[nrLED];
                zapamietanyCzas2 = aktualnyCzas;
            }
            if (dimmsetNow[nrLED] < dimmset_min) {
                dimmsetNow[nrLED] = dimmset_min;
                zapamietanyCzas2 = aktualnyCzas;
            }
            dimmsetLast[nrLED] = dimmsetNow[nrLED];
            // jasnosc = dimmsetNow[nrLED];

            jasnoscLED(nrLED, dimmsetNow[nrLED]);
        }
        if (dimmsetNow[nrLED] >= dimmsetMax[nrLED] || dimmsetNow[nrLED] <= dimmset_min) {
            dimming_up = !dimming_up;
        }
    }
}

void oneClick0() {
    if (bWIFIenable == 5) {
        buttonWIFIenable();
    } else {
        bWIFIenable = 1;
    }
    if (serialmode == 1) {
        Serial.print("bWIFIenable: ");
        Serial.println(bWIFIenable);
    }
    uint8_t nrLED = 0;
    if (serialmode == 1) Serial.println("oneClick");
    //pierwszy raz po reset
    /* if (dimmsetNow[nrLED] == 0 && dimmsetLast[nrLED] == 0) {
        dimmsetNow[nrLED] = dimmsetMax[nrLED];
        dimmsetLast[nrLED] = dimmsetNow[nrLED];
        // jasnosc = dimmsetNow[nrLED];
        jasnoscLED(nrLED, dimmsetNow[nrLED]);
    }
    //wlaczamy na poprzedni poziom
    else */

    if (dimmsetNow[nrLED] == 0) {
        dimmsetNow[nrLED] = dimmsetLast[nrLED];
        // jasnosc = dimmsetNow[nrLED];
        jasnoscLED(nrLED, dimmsetNow[nrLED]);
    }
    //wylaczamy
    else if (dimmsetNow[nrLED] > 0 && dimmsetLast[nrLED] <= dimmsetNow[nrLED]) {
        dimmsetNow[nrLED] = 0;
        // jasnosc = dimmsetNow[nrLED];
        jasnoscLED(nrLED, dimmsetNow[nrLED]);
    }
}
void DoubleClick0() {
    if (bWIFIenable == 1) {
        bWIFIenable = 3;
    }
    if (serialmode == 1) {
        Serial.print("bWIFIenable: ");
        Serial.println(bWIFIenable);
    }
    uint8_t nrLED = 0;
    if (serialmode == 1) Serial.println("DoubleClick");
    //wlaczamy na MAX
    if (dimmsetNow[nrLED] > 0 && dimmsetLast[0] < (dimmsetMax[nrLED] + 1) && dimmsetNow[nrLED] < (dimmsetMax[nrLED] + 1)) {
        dimmsetLast[nrLED] = dimmsetNow[nrLED];
        dimmsetNow[nrLED] = (dimmsetMax[nrLED] + 1);
        // jasnosc = dimmsetNow[nrLED];
        jasnoscLED(nrLED, dimmsetNow[nrLED]);
    }
    //wlaczamy na poprzedni poziom
    else if (dimmsetNow[nrLED] == (dimmsetMax[nrLED] + 1) && dimmsetLast[nrLED] != (dimmsetMax[nrLED] + 1)) {
        dimmsetNow[nrLED] = dimmsetLast[nrLED];
        // jasnosc = dimmsetNow[nrLED];
        jasnoscLED(nrLED, dimmsetNow[nrLED]);
    }
    //na MAX z wylaczonego
    else if (dimmsetNow[nrLED] == 0) {
        dimmsetNow[nrLED] = (dimmsetMax[nrLED] + 1);
        // jasnosc = dimmsetNow[nrLED];
        jasnoscLED(nrLED, dimmsetNow[nrLED]);
    }
}

void LongPressStart0() {
    if (bWIFIenable == 3) {
        bWIFIenable = 5;
    }
    if (serialmode == 1) {
        Serial.print("bWIFIenable: ");
        Serial.println(bWIFIenable);
    }
    uint8_t nrLED = 0;
    if (serialmode == 1) Serial.println("LongPressStart");
    button_is_long_pressed = 1;
    numDIM = nrLED;
}

void oneClick1() {
    uint8_t nrLED = 1;

    if (serialmode == 1) Serial.println("oneClick 1");
    //pierwszy raz po reset
    // if (dimmsetNow[nrLED] == 0 && dimmsetLast[nrLED] == 0) {
    //     dimmsetNow[nrLED] = dimmsetMax[nrLED];
    //     dimmsetLast[nrLED] = dimmsetNow[nrLED];
    //     // jasnosc = dimmsetNow[nrLED];
    //     jasnoscLED(nrLED, dimmsetNow[nrLED]);

    // }
    // //wlaczamy na poprzedni poziom
    // else

    if (dimmsetNow[nrLED] == 0) {
        dimmsetNow[nrLED] = dimmsetLast[nrLED];
        // jasnosc = dimmsetNow[nrLED];
        jasnoscLED(nrLED, dimmsetNow[nrLED]);

    }
    //wylaczamy
    else if (dimmsetNow[nrLED] > 0 && dimmsetLast[nrLED] <= dimmsetNow[nrLED]) {
        dimmsetNow[nrLED] = 0;
        // jasnosc = dimmsetNow[nrLED];
        jasnoscLED(nrLED, dimmsetNow[nrLED]);
    }
}
void DoubleClick1() {
    uint8_t nrLED = 1;
    if (serialmode == 1) Serial.println("DoubleClick");
    //wlaczamy na MAX
    if (dimmsetNow[nrLED] > 0 && dimmsetLast[0] < (dimmsetMax[nrLED] + 1) && dimmsetNow[nrLED] < (dimmsetMax[nrLED] + 1)) {
        dimmsetLast[nrLED] = dimmsetNow[nrLED];
        dimmsetNow[nrLED] = (dimmsetMax[nrLED] + 1);
        // jasnosc = dimmsetNow[nrLED];
        jasnoscLED(nrLED, dimmsetNow[nrLED]);

    }
    //wlaczamy na poprzedni poziom
    else if (dimmsetNow[nrLED] == (dimmsetMax[nrLED] + 1) && dimmsetLast[nrLED] != (dimmsetMax[nrLED] + 1)) {
        dimmsetNow[nrLED] = dimmsetLast[nrLED];
        // jasnosc = dimmsetNow[nrLED];
        jasnoscLED(nrLED, dimmsetNow[nrLED]);

    }
    //na MAX z wylaczonego
    else if (dimmsetNow[nrLED] == 0) {
        dimmsetNow[nrLED] = (dimmsetMax[nrLED] + 1);
        // jasnosc = dimmsetNow[nrLED];
        jasnoscLED(nrLED, dimmsetNow[nrLED]);
    }
}

void LongPressStart1() {
    uint8_t nrLED = 1;
    if (serialmode == 1) Serial.println("LongPressStart");
    button_is_long_pressed = 1;
    numDIM = nrLED;
}

void oneClick2() {
    uint8_t nrLED = 2;

    if (serialmode == 1) Serial.println("oneClick");
    //pierwszy raz po reset
    /* if (dimmsetNow[nrLED] == 0 && dimmsetLast[nrLED] == 0) {
        dimmsetNow[nrLED] = dimmsetMax[nrLED];
        dimmsetLast[nrLED] = dimmsetNow[nrLED];
        // jasnosc = dimmsetNow[nrLED];
        jasnoscLED(nrLED, dimmsetNow[nrLED]);
    }
    //wlaczamy na poprzedni poziom
    else */

    if (dimmsetNow[nrLED] == 0) {
        dimmsetNow[nrLED] = dimmsetLast[nrLED];
        // jasnosc = dimmsetNow[nrLED];
        jasnoscLED(nrLED, dimmsetNow[nrLED]);
    } else if (dimmsetNow[nrLED] > 0 && dimmsetLast[nrLED] <= dimmsetNow[nrLED]) {  //wylaczamy
        dimmsetNow[nrLED] = 0;
        // jasnosc = dimmsetNow[nrLED];
        jasnoscLED(nrLED, dimmsetNow[nrLED]);
    }
}

void DoubleClick2() {
    uint8_t nrLED = 2;
    if (serialmode == 1) Serial.println("DoubleClick");
    //wlaczamy na MAX
    if (dimmsetNow[nrLED] > 0 && dimmsetLast[0] < (dimmsetMax[nrLED] + 1) && dimmsetNow[nrLED] < (dimmsetMax[nrLED] + 1)) {
        dimmsetLast[nrLED] = dimmsetNow[nrLED];
        dimmsetNow[nrLED] = (dimmsetMax[nrLED] + 1);
        // jasnosc = dimmsetNow[nrLED];
        jasnoscLED(nrLED, dimmsetNow[nrLED]);
    }
    //wlaczamy na poprzedni poziom
    else if (dimmsetNow[nrLED] == (dimmsetMax[nrLED] + 1) && dimmsetLast[nrLED] != (dimmsetMax[nrLED] + 1)) {
        dimmsetNow[nrLED] = dimmsetLast[nrLED];
        // jasnosc = dimmsetNow[nrLED];
        jasnoscLED(nrLED, dimmsetNow[nrLED]);
    }
    //na MAX z wylaczonego
    else if (dimmsetNow[nrLED] == 0) {
        dimmsetNow[nrLED] = (dimmsetMax[nrLED] + 1);
        // jasnosc = dimmsetNow[nrLED];
        jasnoscLED(nrLED, dimmsetNow[nrLED]);
    }
}
void LongPressStart2() {
    uint8_t nrLED = 2;
    if (serialmode == 1) Serial.println("LongPressStart");
    button_is_long_pressed = 1;
    numDIM = nrLED;
}
void LongPressStop() {
    if (serialmode == 1) Serial.println("LongPressStop");
    button_is_long_pressed = 0;
}

//<-- DLA dimmer
void jasnoscLED(uint8_t nrLED, uint16_t jas) {
    uint16_t TMPjasnosc = 0, TMPjasnosc2 = 0, TMPjasnosc3 = 0;

    switch (nrLED) {
        case 0:
            sendNRF(3, jas);
            sendCAN(DevID, 3, jas, 0);

            //led[0].setupMax(jas);
            eepromBUF[0] = readEEPROM(0);
            eepromBUF[1] = readEEPROM(1);
            TMPjasnosc = eepromBUF[0] * 256 + eepromBUF[1];  // odczyt zapisanej jasnosci
            if ((jas > 0 && jas < dimmsetMax[0] - 1 && TMPjasnosc != jas)) {
                dim1 = 1;
            } else {
                dim1 = 0;
            }
            break;
        case 1:
            sendNRF(4, jas);
            sendCAN(DevID, 4, jas, 0);

            //led[1].setupMax(jas);
            eepromBUF[6] = readEEPROM(6);
            eepromBUF[7] = readEEPROM(7);
            TMPjasnosc2 = eepromBUF[6] * 256 + eepromBUF[7];
            if ((jas > 0 && jas < dimmsetMax[1] - 1 && TMPjasnosc2 != jas)) {
                dim2 = 1;
            } else {
                dim2 = 0;
            }
            break;
        case 2:

            sendNRF(5, jas);
            sendCAN(DevID, 5, jas, 0);

            //led[2].setupMax(jas);
            eepromBUF[8] = readEEPROM(8);
            eepromBUF[9] = readEEPROM(9);
            TMPjasnosc3 = eepromBUF[8] * 256 + eepromBUF[9];
            if ((jas > 0 && jas < dimmsetMax[2] - 1 && TMPjasnosc3 != jas)) {
                dim3 = 1;
            } else {
                dim3 = 0;
            }
            break;
        case 4:
            break;
    }
    numDIM = nrLED;
    rysuj_jasnosc_na_lcd(nrLED);

    if (dim1 == 1 || dim2 == 1 || dim3 == 1) {
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
                writeEEPROM(0, dimmsetNow[0], 2);
            }
        }
        if (dim2 == 1) {
            if (aktualnyCzas - zapamietanyCzasEEPROM >= 6) {
                dim2 = 0;
                if (serialmode == 1) Serial.println("Zapisywanie w EEPROM DIM 2 ...");
                writeEEPROM(6, dimmsetNow[1], 2);
            }
        }
        if (dim3 == 1) {
            if (aktualnyCzas - zapamietanyCzasEEPROM >= 6) {
                dim3 = 0;
                if (serialmode == 1) Serial.println("Zapisywanie w EEPROM DIM 3 ...");
                writeEEPROM(8, dimmsetNow[2], 2);
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

    if (serialmode == 1) Serial.println("Zapis EEPROM ok");

    zapamietanyCzasEEPROM = aktualnyCzas;
}
void writeStringToEEPROM(int adres, const String& ToWrite) {
    byte size = ToWrite.length();
    writeEEPROM(adres, size, 1);
    if (serialmode == 1) Serial.println(size);
    delay(6);
    for (int i = 0; i < size; i++) {
        adres++;
        writeEEPROM(adres, ToWrite[i], 1);
        if (serialmode == 1) {
            Serial.print(ToWrite[i]);
            Serial.println(adres);
        }
        delay(6);
    }
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
        if (serialmode == 1) Serial.println("Blad odczytu Stringa z eeprom-za dlugi");
    }
    return String(data);
}
void SettingWriteEEPROM() {
    if (serialmode == 1) Serial.print("Zapisywanie ustawien...");

    // writeEEPROM(37, wifiAPon, 1);
    writeStringToEEPROM(nameDevE, nameDev);
    delay(6);
    writeEEPROM(36, wifiAPon, 1);
    delay(6);
    writeStringToEEPROM(2066, Timestamp);
    delay(6);
    writeStringToEEPROM(2033, password);
    delay(6);
    writeStringToEEPROM(2000, ssid);
    delay(6);
    writeEEPROM(34, relayToff2, 2);  //34,35
    delay(6);
    writeEEPROM(32, relayToff1, 2);  // 32, 33
    delay(6);
    writeEEPROM(31, ((RF24_rxAddr >> 8) & 0xFF), 1);
    delay(6);
    writeEEPROM(29, dimmsetMax[2], 2);  // 29,30 BAJT dimmsetMax[2] MSB
    delay(6);
    writeEEPROM(27, dimmsetMax[1], 2);  // 27,28 BAJT dimmsetMax[1] MSB
    delay(6);
    writeEEPROM(26, DimUpDownResolution, 1);
    delay(6);
    writeEEPROM(24, dimmsetMax[0], 2);
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
    writeEEPROM(8, dimmsetNow[2], 2);  // 8,9 BAJT DIM3 jasnosc MSB
    delay(6);
    writeEEPROM(6, dimmsetNow[1], 2);  // 6,7 BAJT DIM2 jasnosc MSB
    delay(6);
    writeEEPROM(5, SubDevID, 1);  // 5 BAJT SubDevID
    delay(6);
    writeEEPROM(4, DevID, 1);  // 4 BAJT devID
    delay(6);
    writeEEPROM(0, dimmsetNow[0], 2);  // 0,1 BAJT dim1 jasnosc MSB

    if (serialmode == 1) Serial.println("... zakonczone. Reset ESP.");
    SettingSave = 0;
    if (ServerActive == 0) {
        wificlose();
    }
    ESP.restart();
}
void FactoryWriteEEPROM() {
    if (serialmode == 1) Serial.print("Ustawienia fabryczne...");
    writeStringToEEPROM(nameDevE, "nD");
    delay(6);
    writeEEPROM(FactorySetE, 123, 1);  //37
    delay(6);
    writeEEPROM(36, 1, 1);
    delay(6);
    writeStringToEEPROM(2000, "ssid");
    delay(6);
    writeStringToEEPROM(2033, "pass");
    delay(6);
    writeStringToEEPROM(2066, "575420400");
    delay(6);
    writeEEPROM(34, 5, 2);  //34,35 relayToff2
    delay(6);
    writeEEPROM(32, 5, 2);  // 32, 33 relayToff1
    delay(6);
    writeEEPROM(31, 0, 1);  // RF24_rxAddr
    delay(6);
    writeEEPROM(29, 999, 2);  // 29,30 BAJT dimmsetMax[2] MSB
    delay(6);
    writeEEPROM(27, 999, 2);  // 27,28 BAJT dimmsetMax[1] MSB
    delay(6);
    writeEEPROM(26, 20, 1);  // 26 DimUpDownResolution
    delay(6);
    writeEEPROM(24, 999, 2);  // 24, 25 dimmsetMax[0]
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
    writeEEPROM(11, 0, 1);  // 11 BAJT Flag
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

void rysuj_jasnosc_na_lcd(uint16_t nrLED) {
    if (dimActivNumb == 3) {
        int val = map(dimmsetNow[0], 0, dimmsetMax[0], 0, 37);
        if (val < 1 && dimmsetNow[0] > 0) {
            val = 1;
        }
        display.fillRoundRect(2, 2, 37, 11, zaok, BLACK);
        display.fillRoundRect(2, 2, val, 11, zaok, WHITE);
        // napisy na pasku jasnosci
        if (dimmsetNow[0] == 0) {
            display.setCursor(5, 4);
            display.println("O F F");
        }
        if (dimmsetNow[0] == 2) {
            display.setCursor(5, 4);
            display.println("M I N");
        }
        if (dimmsetNow[0] == (dimmsetMax[0] + 1)) {
            display.setTextColor(BLACK);
            display.setCursor(10, 4);
            display.println("FULL");
            display.setTextColor(WHITE);
        }
        if (dimmsetNow[0] == dimmsetMax[0]) {
            display.setTextColor(BLACK);
            display.setCursor(5, 4);
            display.println("M A X");
            display.setTextColor(WHITE);
        }

        int val1 = map(dimmsetNow[1], 0, dimmsetMax[1], 0, 37);
        if (val1 < 1 && dimmsetNow[1] > 0) {
            val1 = 1;
        }
        display.fillRoundRect(45, 2, 37, 11, zaok, BLACK);
        display.fillRoundRect(45, 2, val1, 11, zaok, WHITE);
        // napisy na pasku jasnosci
        if (dimmsetNow[1] == 0) {
            display.setCursor(50, 4);
            display.println("O F F");
        }
        if (dimmsetNow[1] == 2) {
            display.setCursor(50, 4);
            display.println("M I N");
        }
        if (dimmsetNow[1] == (dimmsetMax[1] + 1)) {
            display.setTextColor(BLACK);
            display.setCursor(55, 4);
            display.println("FULL");
            display.setTextColor(WHITE);
        }
        if (dimmsetNow[1] == dimmsetMax[1]) {
            display.setTextColor(BLACK);
            display.setCursor(50, 4);
            display.println("M A X");
            display.setTextColor(WHITE);
        }

        int val2 = map(dimmsetNow[2], 0, dimmsetMax[2], 0, 37);
        if (val2 < 1 && dimmsetNow[2] > 0) {
            val2 = 1;
        }
        display.fillRoundRect(88, 2, 37, 11, zaok, BLACK);
        display.fillRoundRect(88, 2, val2, 11, zaok, WHITE);
        // napisy na pasku jasnosci
        if (dimmsetNow[2] == 0) {
            display.setCursor(93, 4);
            display.println("O F F");
        }
        if (dimmsetNow[2] == 2) {
            display.setCursor(93, 4);
            display.println("M I N");
        }
        if (dimmsetNow[2] == (dimmsetMax[1] + 1)) {
            display.setTextColor(BLACK);
            display.setCursor(98, 4);
            display.println("FULL");
            display.setTextColor(WHITE);
        }
        if (dimmsetNow[2] == dimmsetMax[1]) {
            display.setTextColor(BLACK);
            display.setCursor(93, 4);
            display.println("M A X");
            display.setTextColor(WHITE);
        }
    } else if (dimActivNumb == 1) {
        int val = map(dimmsetNow[nrLED], 0, dimmsetMax[nrLED], 0, 95);
        if (val < 1 && dimmsetNow[nrLED] > 0) {
            val = 1;
        }
        display.fillRect(16, 2, 95, 11, BLACK);
        display.fillRect(16, 2, val, 11, WHITE);

        // napisy na pasku jasnosci
        if (dimmsetNow[nrLED] == 0) {
            display.setCursor(52, 4);
            display.println("O F F");
        }
        if (dimmsetNow[nrLED] == 2) {
            display.setCursor(52, 4);
            display.println("M I N");
        }
        if (dimmsetNow[nrLED] == (dimmsetMax[nrLED] + 1)) {
            display.setTextColor(BLACK);
            display.setCursor(43, 4);
            display.println("F U L L");
            display.setTextColor(WHITE);
        }
        if (dimmsetNow[nrLED] == dimmsetMax[nrLED]) {
            display.setTextColor(BLACK);
            display.setCursor(50, 4);
            display.println("M A X");
            display.setTextColor(WHITE);
        }

        if (poRestarcieTestSW1) {
            display.setCursor(17, 4);
            display.println("START:");
        }  // wyswietlamy tylko przy pierwszym starcie lub restarcie procka
        if (poRestarcieTestSW1 && dimmsetNow[nrLED] > 0) {
            poRestarcieTestSW1 = 0;
        }
    } else if (dimActivNumb == 2) {
        int val = map(dimmsetNow[0], 0, dimmsetMax[0], 0, 58);
        if (val < 1 && dimmsetNow[0] > 0) {
            val = 1;
        }
        display.fillRoundRect(2, 2, 58, 11, zaok, BLACK);
        display.fillRoundRect(2, 2, val, 11, zaok, WHITE);
        // napisy na pasku jasnosci
        if (dimmsetNow[0] == 0) {
            display.setCursor(5, 4);
            display.println("O F F");
        }
        if (dimmsetNow[0] == 2) {
            display.setCursor(5, 4);
            display.println("M I N");
        }
        if (dimmsetNow[0] == (dimmsetMax[0] + 1)) {
            display.setTextColor(BLACK);
            display.setCursor(10, 4);
            display.println("FULL");
            display.setTextColor(WHITE);
        }
        if (dimmsetNow[0] == dimmsetMax[0]) {
            display.setTextColor(BLACK);
            display.setCursor(5, 4);
            display.println("M A X");
            display.setTextColor(WHITE);
        }

        int val1 = map(dimmsetNow[1], 0, dimmsetMax[1], 0, 58);
        if (val1 < 1 && dimmsetNow[1] > 0) {
            val1 = 1;
        }
        display.fillRoundRect(66, 2, 58, 11, zaok, BLACK);
        display.fillRoundRect(66, 2, val1, 11, zaok, WHITE);
        // napisy na pasku jasnosci
        if (dimmsetNow[1] == 0) {
            display.setCursor(71, 4);  //+5
            display.println("O F F");
        }
        if (dimmsetNow[1] == 2) {
            display.setCursor(71, 4);
            display.println("M I N");
        }
        if (dimmsetNow[1] == (dimmsetMax[1] + 1)) {
            display.setTextColor(BLACK);
            display.setCursor(76, 4);
            display.println("FULL");
            display.setTextColor(WHITE);
        }
        if (dimmsetNow[1] == dimmsetMax[1]) {
            display.setTextColor(BLACK);
            display.setCursor(71, 4);
            display.println("M A X");
            display.setTextColor(WHITE);
        }
    }
    if (poRestarcieTestSW1) {
        display.setCursor(5, 4);
        display.println("|");
    }  // wyswietlamy tylko przy pierwszym starcie lub restarcie procka
    if (poRestarcieTestSW1 && dimmsetNow[nrLED] > 0) {
        poRestarcieTestSW1 = 0;
    }
    display.display();
}

void rysujemy_na_lcd() {
    if (checkOne1 == 1) {
        if (dimmsetLast[0] != 0) {
            dimActivNumb++;
        }
        if (dimmsetLast[1] != 0) {
            dimActivNumb++;
        }
        if (dimmsetLast[2] != 0) {
            dimActivNumb++;
        }
        checkOne1 = 0;
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    if (dimActivNumb == 3) {
        display.drawRoundRect(0, 0, 41, 15, zaok, WHITE);
        display.drawRoundRect(43, 0, 41, 15, zaok, WHITE);
        display.drawRoundRect(86, 0, 41, 15, zaok, WHITE);
    } else if (dimActivNumb == 1) {
        display.drawRoundRect(14, 0, 99, 15, 2, WHITE);
        display.drawCircle(5, 7, 5, WHITE);
        display.drawLine(2, 4, 8, 10, WHITE);
        display.drawLine(2, 10, 8, 4, WHITE);
        display.fillCircle(122, 7, 5, WHITE);
        display.drawLine(119, 4, 125, 10, BLACK);
        display.drawLine(119, 10, 125, 4, BLACK);
    } else if (dimActivNumb == 2) {
        display.drawRoundRect(0, 0, 62, 15, zaok, WHITE);
        display.drawRoundRect(64, 0, 62, 15, zaok, WHITE);
    }
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
        display.print("CAN ID:0x");
        display.println(frameID, HEX);  // CAN-BUS ID

        display.setCursor(0, 32);
        display.print("NRF:");
        display.print(NRFbuf[0]);
        display.print(",");
        display.println(NRFbuf[2] * 256 + NRFbuf[1]);
        display.setCursor(50, 32);
        // display.print("PA:");
        // display.println(radio.getPALevel());
        display.println(nameDev);

        display.setFont(&FreeSansBold18pt7b);
        //    display.setFont(&FreeSans12pt7b);

        // dwie wersje wyswietlania temperatury, druga z miejscem dziesietnym

        display.setCursor(20, 62);
        char odczyt[4];
        dtostrf(celsius, 3, 0, odczyt);
        display.drawCircle(75, 39, 3, WHITE);
        if (celsius == -99) {
            display.print("  --");
        } else {
            display.print(odczyt);
        }
        /*
        display.setCursor(20, 60);
        char odczyt[5];
        dtostrf(celsius, 3, 1, odczyt);
        display.print(odczyt);
        display.drawCircle(95, 39, 3, WHITE);
        */

        display.setFont(&FreeSans12pt7b);
        display.print(" C");
        display.setFont();
        if (wifiSTAon == 1 || wifiAPconnected == 1) {
            display.drawXBitmap(0, 50, logo16_wifi_bmp, 16, 16, WHITE);
        } else if (wifiStartConnecting == 1) {
            display.fillCircle(7, 63, 2, WHITE);
        }
        display.drawBitmap(14, 56, Msg816, 16, 8, WHITE);
        if (gsminit == 1) {
            display.drawBitmap(100, 56, Signal816, 16, 8, WHITE);

        } else if (serialmode == 2) {
            display.drawBitmap(100, 56, Signal816, 9, 8, WHITE);
            display.fillCircle(106, 62, 2, WHITE);
        }
        PokazSW1NaDisplay(0);
    }
    rysuj_jasnosc_na_lcd(numDIM);  //w tej funkcji na koncu jest display.display()
}

void aktualizuj_timestr() {
    Qhour = hour();
    Qminutes = minute();
    Qday = day();
    Qmonth = month();
    Qsecounds = second();
    timestr[0] = '0' + Qhour / 10;
    timestr[1] = '0' + Qhour % 10;
    timestr[3] = '0' + Qminutes / 10;
    timestr[4] = '0' + Qminutes % 10;
    timestr[6] = '0' + Qsecounds / 10;
    timestr[7] = '0' + Qsecounds % 10;
    tl1.tdcomp();
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
    int16_t raw = (data[1] << 8) | data[0];

    if (OneWire::crc8(DallasAddr, 7) == DallasAddr[7]) {
        byte cfg = (data[4] & 0x60);
        if (serialmode == 1) {
            Serial.print("cfg: ");
            Serial.print(cfg);
        }
        if (cfg == 0x00) {
            raw = raw & ~7;  // 9 bit resolution, 93.75 ms
        } else if (cfg == 0x20) {
            raw = raw & ~3;  // 10 bit res, 187.5 ms
        } else if (cfg == 0x40) {
            raw = raw & ~1;
        }  // 11 bit res, 375 ms
        //// default is 12 bit resolution, 750 ms conversion time

        celsius = (float)raw / 16.0;
        if (serialmode == 1) {
            Serial.print(" Temp: ");
            Serial.println(celsius);
        }
    } else if (serialmode == 1) {
        Serial.println("Temp-BLAD CRC ");
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
    } else {  // !! NIE TESTOWANE
        uint16_t data = (n << 8) + sp[n - 1];
        sendNRF(6, data);
        sendCAN(255, 6, data, 0);  //toID - 255 dla wszystkich,
    }
}
void relaydimonoff() {
    if ((dimmsetNow[0] > 0 || dimmsetNow[1] > 0 || dimmsetNow[2] > 0) && sp[0] == 0) {
        // if (dimmsetNow[0] > 0 && sp[0] == 0) {
        relays(1);
        roff = 1;
    } else if (roff == 1 && dimmsetNow[0] == 0 && dimmsetNow[1] == 0 && dimmsetNow[2] == 0 && sp[0] == 1) {
        // } else if (roff == 1 && dimmsetNow[nrLED] == 0 && sp[0] == 1) {
        zapczas = aktualnyCzas;
        roff = 0;
    } else if (roff == 0 && (aktualnyCzas - zapczas >= (relayToff1 * 1000))) {
        //relayToff
        roff = 1;
        relays(1);
    }
}

void timeNetUpdate() {
    timeClient.update();
    setTime(timeClient.getEpochTime() + 7200);
    sendCAN(255, 2, timeClient.getEpochTime(), 0);
}
String mcADR() {
    String macADR = WiFi.macAddress();
    macADR.replace(":", "");
    return macADR;
}

Qtimers::Qtimers(int ac, int fu, int hrs, int mins, int hre, int mine, int ds, int ms, int de, int me) {
    active = ac;
    function = fu;

    if (hrs <= 24) {
        hourStart = hrs;
    } else {
        if (serialmode == 1) {
            Serial.println("Litetimer - niewlasciwie podana godzina startu");
        }
    }
    if (hre <= 24) {
        hourEnd = hre;
    } else {
        if (serialmode == 1) {
            Serial.println("Litetimer - niewlasciwie podana godzina konca");
        }
    }
    if (mins <= 59) {
        minutesStart = mins;
    } else {
        if (serialmode == 1) {
            Serial.println("Litetimer - niewlasciwie podana minuta startu");
        }
    }
    if (mine <= 59) {
        minutesEnd = mine;
    } else {
        if (serialmode == 1) {
            Serial.println("Litetimer - niewlasciwie podana minuta konca");
        }
    }
    if (ds <= 31) {
        dayStart = ds;
    } else {
        if (serialmode == 1) {
            Serial.println("Litetimer - niewlasciwie podany dzien startu");
        }
    }
    if (de <= 31) {
        dayEnd = de;
    } else {
        if (serialmode == 1) {
            Serial.println("Litetimer - niewlasciwie podany dzien konca");
        }
    }
    if (ms <= 12) {
        monthStart = ms;
    } else {
        if (serialmode == 1) {
            Serial.println("Litetimer - niewlasciwie podany miesiac startu");
        }
    }
    if (me <= 12) {
        monthEnd = me;
    } else {
        if (serialmode == 1) {
            Serial.println("Litetimer - niewlasciwie podany miesiac konca");
        }
    }
}
Qtimers::~Qtimers() {
}

void Qtimers::tdcomp() {
    bool GE = 0;
    if (active == 1) {
        if (GE == 0 && (monthStart == 0 || monthStart == Qmonth) && (dayStart == 0 || dayStart == Qday) && (hourStart == 0 || hourStart == Qhour) && (minutesStart == 0 || minutesStart == Qminutes)) {
            GE = 1;
            if (serialmode == 1) Serial.print("Qtimer - tdcomp START ");
            switch (function) {
                case 1:  // dim1 night light
                    dimmsetLast[0] = dimmsetLastNL[0];
                    dimmsetLast[1] = dimmsetLastNL[1];
                    dimmsetLast[2] = dimmsetLastNL[2];
                    if (serialmode == 1) Serial.println("night light ON - dim 1,2,3");
                    break;
                case 2:
                    break;
                case 3:
                    break;
            }
        } else if (GE == 1 && (monthEnd == 0 || monthEnd == Qmonth) && (dayEnd == 0 || dayEnd == Qday) && (hourEnd == 0 || hourEnd == Qhour) && (minutesEnd == 0 || minutesEnd == Qminutes)) {
            GE = 0;
            if (serialmode == 1) Serial.print("Qtimer - tdcomp END ");
            switch (function) {
                case 1:
                    dimmsetLast[0] = readEEPROM(0) * 256 + readEEPROM(1);
                    dimmsetLast[1] = readEEPROM(6) * 256 + readEEPROM(7);
                    dimmsetLast[2] = readEEPROM(8) * 256 + readEEPROM(9);
                    if (serialmode == 1) Serial.println("night light OFF - dim 1,2,3");
                    break;
                case 2:
                    break;
                case 3:
                    break;
            }
        }
    }
}

// wymyslic kasowanie wiadomosci sms.
// !! - sprawdzic
// ustawienie dodatkowe aby od godziny np 23 / w nocy wlaczalo swiatlo na minimalny poziom. ale chyba to ustawienie do mastera.
// dokonczyc przetestowac can.
// napisac funkcje do doladowywania/ wysylania specjalnych sms w celu wlaczenia uslugi np dodatkowe sms.
// napisac funkcje sprawdzajaca saldo konta jesli jest to prepaid-opcja zaznaczenia na www, wpisania kodu sprawdzajacego saldo
// napisac synchronizacje czasu przez CAN wysylajace do wszystkich modołów z mastera (255).
// napisac restart modulu GSM co x czasu

// napisac mozliwosc nazwy urzadzenia/pomieszczenia
// napisac synchronizacje aktualnej godziny przez www
// gdy ustawienia fabryczne to jest wlaczone wifi AP

//
// *****
// trzeba byc w katalogu projektu
//projekt  pio run --target upload --upload-port 192.168.43.47
//fs       pio run --target uploadfs --upload-port 192.168.43.47