/**
 * AVR LC/Frequency Meter
 * Kerry Wong
 * http://www.kerrywong.com
 */

#define __AVR_ATmega328P__

#include <avr/interrupt.h>
#include <binary.h>
#include <HardwareSerial.h>
#include <pins_arduino.h>
#include <WConstants.h>
#include <wiring.h>
#include <wiring_private.h>
#include <math.h>
#include <WProgram.h>
#include <EEPROM/EEPROM.h>
#include <LiquidCrystal/LiquidCrystal.h>
#include <FreqCounter/FreqCounter.h>

enum MeterMode {
    L,
    C,
    F
};

//Frequency input is digital pin 5

const int pinRS = 3;
const int pinEn = 4;
const int pinD4 = 16;
const int pinD5 = 17;
const int pinD6 = 8;
const int pinD7 = 9;
const int pinBACKLIGHT = 10;

const int btn1 = 0;
const int btn2 = 1;
const int btn3 = 2;

const int pinBtn1 = 11;
const int pinBtn2 = 12;
const int pinBtn3 = 15;

const int pinRelay = 14;
const int pinLCMode = 2;

float F0 = 348000;
const float Cth = 1000 * 1e-12; //theoretical 1000pF
const float Lth = 221 * 1e-6; //theoretical 221uH

boolean backLightOn = false;
boolean dispFreq = false;

float l0 = Lth;
float c0 = Cth;
MeterMode currentMode;

LiquidCrystal lcd(pinRS, pinEn, pinD4, pinD5, pinD6, pinD7);
unsigned long frq;

const int BTN_PINS[] = {pinBtn1, pinBtn2, pinBtn3};
boolean btnPressed[] = {false, false, false};

void delayMilliseconds(int ms) {
    for (int i = 0; i < ms; i++) {
        delayMicroseconds(1000);
    }
}

boolean buttonPressed(int btnIdx) {
    if (digitalRead(BTN_PINS[btnIdx]) == LOW && !btnPressed[btnIdx]) {
        delayMilliseconds(20);
        if (digitalRead(BTN_PINS[btnIdx]) == LOW) {
            btnPressed[btnIdx] = true;
            return true;
        }
    }

    return false;
}

boolean buttonRleased(int btnIdx) {
    if (digitalRead(BTN_PINS[btnIdx]) == HIGH && btnPressed[btnIdx]) {
        delayMilliseconds(20);
        if (digitalRead(BTN_PINS[btnIdx]) == HIGH) {
            btnPressed[btnIdx] = false;
            return true;
        }
    }

    return false;
}

float calcV(float f, float VRef) {
    float v = 0;

    v = ((F0 * F0) / (f * f) - 1.0) * VRef;

    return v;
}

void checkLCMode() {
    lcd.clear();

    if (digitalRead(pinLCMode) == LOW) {
        currentMode = L;
        lcd.print("Mode: L");
    } else {
        currentMode = C;
        lcd.print("Mode: C");
    }

    if (dispFreq) {
        lcd.setCursor(0, 0);
        lcd.print("Frequency: ");
    }
}

void setup() {
    for (int i = 0; i < 3; i++) {
        pinMode(BTN_PINS[i], INPUT);
        digitalWrite(BTN_PINS[i], HIGH);
    }

    pinMode(pinLCMode, INPUT);
    digitalWrite(pinLCMode, HIGH);

    pinMode(pinBACKLIGHT, OUTPUT);
    digitalWrite(pinBACKLIGHT, LOW);

    pinMode(pinRelay, OUTPUT);
    digitalWrite(pinRelay, LOW);
    lcd.begin(16, 2);
    checkLCMode();
}

void displayFreq(long fin) {
    lcd.setCursor(0, 1);

    float f = 0;
    if (fin < 1000) {
        f = 1.0 * ((float) fin);
        lcd.print(f, 0);
        lcd.print(" Hz");
    } else if (fin >= 1000) {
        f = ((float) fin) / 1000.0;
        lcd.print(f, 3);
        lcd.print(" KHz");
    }
}

void displayV(long fin) {
    float f = (float) fin;
    float v = 0;
    lcd.setCursor(0, 1);

    switch (currentMode) {
        case C:
            v = calcV(f, c0);

            if (v < 1e-9) {
                v = v * 1e12; // pico
                lcd.print(v);
                lcd.print(" ");
                lcd.print("pF");
            } else if (v >= 1e-9 && v < 1e-6) {
                v = v * 1e9; // n
                lcd.print(v);
                lcd.print(" ");
                lcd.print("nF");
            } else {
                lcd.print("---");
            }
            break;
        case L:
            v = calcV(f, l0);

            if (v < 1e-6) {
                v = v * 1e9; //nH
                lcd.print(v);
                lcd.print(" ");
                lcd.print("nH");
            } else if (v >= 1e-6 && v < 1e-3) {
                v = v * 1e6; //uH
                lcd.print(v);
                lcd.print(" ");
                lcd.print("uH");
            } else if (v >= 1e-3 && v < 1) {
                v = v * 1e3;
                lcd.print(v);
                lcd.print(" ");
                lcd.print("mH");
            } else if (v >= 1 && v < 100) {
                lcd.print(v);
                lcd.print(" ");
                lcd.print("H");
            } else {
                lcd.print("---");
            }
            break;
        case F:
            break;
    }

}

void loop() {
    FreqCounter::f_comp = 106;
    FreqCounter::start(1000);
    while (FreqCounter::f_ready == 0)
        frq = FreqCounter::f_freq;

    checkLCMode();

    if (buttonPressed(btn1)) {
        backLightOn = !backLightOn;
        if (backLightOn) {
            digitalWrite(pinBACKLIGHT, HIGH);
        } else {
            digitalWrite(pinBACKLIGHT, LOW);
        }
    } else if (buttonPressed(btn2)) {
        switch (currentMode) {
            case C:
                c0 = calcV(frq, Cth) + Cth;
                F0 = frq;
                break;
            case L:
                l0 = calcV(frq, Lth) + Lth;
                F0 = frq;
                break;
            default:
                break;
        }

        lcd.print("  Cal");
    } else if (buttonPressed(btn3)) {
        dispFreq = !dispFreq;

        if (dispFreq)
            digitalWrite(pinRelay, HIGH);
        else
            digitalWrite(pinRelay, LOW);
    }

    if (dispFreq)
        displayFreq(frq);
    else
        displayV(frq);

    for (int i = 0; i < 3; i++) {
        if (buttonRleased(i)) {
            //the button is released, no action is necessary.
        }
    }
}

