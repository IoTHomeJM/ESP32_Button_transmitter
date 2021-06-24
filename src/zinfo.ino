/* PRZETESTOWAC funkcje: NIE TESTOWANE
 *  napisac: konfiguracje www
 *  chyba OTA i Server nie moga byc razem wlaczone. W ota inne ssid i haslo?
 *  w konfikuracji www napisac ustawienia lokalnej sieci. Jesli esp nie polaczy sie z routerem to przelaczy sie na softAP
 *  spowolnic serwer www w loop przez setinterval
 *  
 *  
 * Za pomoca przycisku regulujemy jasnosc tylko dimer 1. 
 * Podczas pierwszego uruchomienia zaswieca sie dimer 1 z ostatnia zapamietana nastawa. Dimer 2, 3 jako dodatek...
 * Mozliwa regulacja DIMER 1, 2, 3 za pomoca CAN-BUS i NRF.
 * Funkcja zapisywania ostatniej innej mniejszej niz dimmset_max jasnosci DIM 1,2,3.
 * 
 *   
 ** Zmienne konfiguracyjne:
 *  LightOnBoot        domyslnie:1 - wlaczenie dim1 podczas uruchamiania CPU
 *  czasnazapisweeprom domyslnie:6000 mili sekund
 *  FreqLED            domyslnie:5000 Hz
 *  ResolutionLED      domyslnie:10 bit
 *  ServerActive       domyslnie:0 - serwer konfiguracyjny
 *  CAN-BUS:
 *    Flag             domyslnie:0 - 1 bit 
 *    SubDevID         domyslnie:0 - 4 bity od 0 do 15 
 *    DevID            domyslnie:0 - 8 bit ID tego urzadzenia, ID 1 to master.   
 *    allFrame         domyslnie:0 - jesli 0 to przyjmuje ramki z filtrem
 *  NRF:
 *    RF24_rxAddr[6] = "00001"; byte
 *    NRFchannel       domyslnie:1
 *    PALevel          domyslnie:1  0=MIN, 1=LOW, 2=HIGH, 3=MAX
 *  
 *  SW1TouchEnable     domyslnie:0; //aktywacja funkcji przycisku dotyku; 0 - NIE AKTYWNE
 *  OTAActive          domyslnie:0; programowanie OTA 0-nie aktywne
 *  FactorySet         domyslnie:0; zapisuje ustawienia FABRYCZNE do eeprom; 0 - brak; 1 - zapis, restartESP
 *  SettingSave        domyslnie:0; zapisuje ustawienia do eeprom; 0 - brak; 1 - zapis, restartESP 
 *  show_print_comment domyslnie:1; wyswietlanie komunikatow na Serial.print; 0 - NIE; 1 - TAK;
 *    
 ** PORTY: trzeba poprawic !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
 *  
 *  SW1 - 19    | SW2 - 18    | SW3 - 17    | SW4 - 16
 *  Touch3 - 0
 *  Relay1 - 26 | Relay2 - 25
 *  ACS_AMP - 33
 *  OnWire - 23
 *  step1 - 2  | dir1 - 32
 *  step2 - 16 | dir2 - 26
 *
 *    
 ** NRF:
 *  skladnia: sendNRF(uint8_t fnID, uint16_t fndata) maks 3 bajty
 *  naglowki(fnID:  
 *    3-poziom jasnosci dimera 1
 *    4-poziom jasnosci dimera 2
 *    5-poziom jasnosci dimera 3
 *    48-ustawiamy maksymalna jasnosc
 *    
 ** CAN-BUS:
 *  skladnia: sendCAN(toID,fnID,32fndata,32fndata); wiadomosc maks 2x 32 bajty. toID i fnID sa przesylane w ID ramki CAN-BUS
 *  Gdy allFrame=0 urzadzenie przyjmuje ramki tylko o swoim ID oraz o ID 255 
 *  naglowki(fnID): 
 *    1-odpowiadamy masterowi, obecnosc urzadzenia na CANie
 *    2-synchronizacja czasu (data godzina).
 *    3-poziom jasnosci dimera 1 (to samo jak dala NRF)
 *    4-poziom jasnosci dimera 2 (to samo jak dala NRF)
 *    5-poziom jasnosci dimera 3 (to samo jak dala NRF)
 *    6-poziomy jasnosci dim1 2 3
 *    48-ustawiamy maksymalna jasnosc dimera
 *    
 ** EEPROM:  
 *  zapis:
 *    writeEEPROM(adres, dane, dlugosc 1 lub 2 bajty);
 *  odczyt:  
 *    readEEPROM(adres);
 *  
 *           BYTE |DATE
 *           
 *        
 *           
 *  writeEEPROM(32, );  
 *  writeEEPROM(31, );  
 *  writeEEPROM(30, );  // dimm3set_max LSB
 *  writeEEPROM(29, );  // dimm3set_max MSB
 *  writeEEPROM(28, );  // dimm2set_max LSB
 *  writeEEPROM(27, );  // dimm2set_max MSB
 *  writeEEPROM(26, );  // DimUpDownResolution
 *  writeEEPROM(25, );  // dimmset_max LSB
 *  writeEEPROM(24, );  // dimmset_max MSB
 *  writeEEPROM(23, );  // show_print_comment
 *  writeEEPROM(22, );  // czasnazapisweeprom LSB
 *  writeEEPROM(21, );  // czasnazapisweeprom MSB
 *  writeEEPROM(20, );  // FreqLED LSB
 *  writeEEPROM(19, );  // FreqLED MSB
 *  writeEEPROM(18, );  // ResolutionLED
 *  writeEEPROM(17, );  // ServerActive
 *  writeEEPROM(16, );  // OTAActive
 *  writeEEPROM(15, );  // W1TouchEnable
 *  writeEEPROM(14, );  // NRFchannel
 *  writeEEPROM(13, );  // PALevel
 *  writeEEPROM(12, );  // allFrame
 *  writeEEPROM(11, );  // Flag  
 *  writeEEPROM(10, 1); // LightOnBoot
 *  writeEEPROM(9, 100);//DIM3 jasnosc LSB
 *  writeEEPROM(8, 0);  //DIM3 jasnosc MSB
 *  writeEEPROM(7, 100);//DIM2 jasnosc LSB
 *  writeEEPROM(6, 0);  //DIM2 jasnosc MSB 
 *  writeEEPROM(5, 0);  //SubDevID
 *  writeEEPROM(4, 0);  //devID 
 *  writeEEPROM(3, 0,1);// wolne
 *  writeEEPROM(2, 0,1);  // wolne
 *  writeEEPROM(1,100,1);//DIM1 jasnosc LSB
 *  writeEEPROM(0,0,1);  //DIM1 jasnosc MSB  
 *  
 *  
 *  
 * 
 */
