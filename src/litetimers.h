#ifndef Litetimers_h
#define Litetimers_h
#include "Arduino.h"

class Litetimers {
    int hour, minutes, day, month;

   public:
    Litetimers(int = 0, int = 0, int = 0, int = 0);  // konstruktor
    ~Litetimers();                                   // destruktor
    // void load();
    // void tcomp();   //timer dzienny - sprawdza codziennie.
    // void tdcomp();  //timer
};

#endif