
#ifndef KEYTABLE_H
#define KEYTABLE_H

#include <stdint.h>
#include <stdbool.h>

#define NUMBER_OF_KEYS              (88)
#define NUMBER_OF_VEOCITY_POINTS    (33)
#define NUMBER_OF_INSTRUMENTS       (8)
#define NUMBER_OF_LEDS              (4)
#define NUMBER_OF_BUTTONS           (4)

#define MS_TO_CURVETIME(a)  (a / 2)            //in units of 2ms, rounded down (integer division)


typedef struct {
    uint8_t note;               //midi note index (A4 = 69)
    uint8_t address;            //address on A pins
    uint8_t s1;                 //index of switch S1 (Early on, late off) on S pins
    uint8_t s2;                 //index of switch s2 (late on, early off) on S pins
} KeyDef_t;

typedef enum {
    KeyUnpressed = 3,           //s1 = 1, s2 = 1 (inverted logic)
    KeyHalfPress = 2,
    KeyPressed = 0
} KeyPressState_t;


typedef struct {
    KeyPressState_t pressState;
    uint32_t timeLastState;
    bool updated;
} KeyState_t;

typedef enum {
    VelocityLinear = 0,
    VelocitySquared,
    VelocitySquareRoot,
    
    VelocityCurveCount,
} VelocityCurveType_t;

typedef enum {
    PedalSustain = 0,
    PedalCount,
} PedalType_t;

typedef enum {
    ButtonMetronome,
    ButtonInstrument,
    ButtonDemo,
    ButtonDemo2,
    ButtonCount
} ButtonType_t;

typedef struct {
    bool leds[NUMBER_OF_LEDS];
    bool button[ButtonCount];
    uint32_t buttonTimes[ButtonCount];
} PanelStatus_t;

typedef struct {
    uint8_t instrumentIndex;
} KeyboardStatus_t;

extern const uint8_t InstrumentTable[NUMBER_OF_INSTRUMENTS];    
extern const KeyDef_t KeyTable[NUMBER_OF_KEYS];
extern const uint8_t VelocityTable[NUMBER_OF_VEOCITY_POINTS][VelocityCurveCount];


#endif      //KEYTABLE_H