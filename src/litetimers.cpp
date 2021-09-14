// #include <Arduino.h>

#include "litetimers.h"

#include "main.cpp"

Litetimers::Litetimers(int hr, int min, int d, int m) {
    if (hr <= 24) {
        hour = hr;
    } else {
        if (serialmode == 1) {
            Serial.println("Litetimer - niewlasciwie podana godzina");
        }
    }
    if (min <= 59) {
        minutes = min;
    } else {
        if (serialmode == 1) {
            Serial.println("Litetimer - niewlasciwie podana minuta");
        }
    }
    if (d <= 31) {
        day = d;
    } else {
        if (serialmode == 1) {
            Serial.println("Litetimer - niewlasciwie podany dzien");
        }
    }
    if (m <= 12) {
        month = m;
    } else {
        if (serialmode == 1) {
            Serial.println("Litetimer - niewlasciwie podany miesiac");
        }
    }
}
// Litetimers::~Litetimers() {
// }
// void Litetimers::compareT() {}