#include "KeyTable.h"
#include "main.h"
#include "simulation.h"
#include "stm32f4xx.h"

#define KEYSTATE(val,bitS1,bitS2) ((KeyPressState_t)(((val >> bitS1) & 1) | (((val >> bitS2) & 1) << 1)))

#define MIDI_CHANNEL            0               //0-15
#define MIN_ON_DURATION         50           // 50ms

#define MIDI_CMD_NOTEOFF        0x80
#define MIDI_CMD_NOTEON         0x90
#define MIDI_CMD_CONTROLLER     0xB0
#define MIDI_CMD_PROGCHANGE     0xC0

#define MIDI_CONTROLLER_SUSTAIN 0x40

#define MIDI_PEDAL_ON           0x40
#define MIDI_PEDAL_OFF          0x00

/*
 * Connections
 *  5x Address pins (A-Port)
 *  8x Switch pins (S-Port)
 *  1-2x Pedal Inputs
 *  6x Panel connections (2x common (out), 2x led, 2x switch)
 *      LED:
            Ground Common A, Drive Common B, Drive L0, L1
            Drive Common A, Ground Common B, Drive L0, L1
            
        Switch:
            Pullups on S0, S1
            Ground Common A, Drive Common B, read S0, S1
            Drive Common A, Ground Common B, read S0, S1
 *  Headphone in
 *  Mute out
 */

//Ports
#define PORT_A04            (GPIOC->ODR)
#define PORTMASK_A04        (0x1F00)
#define PORTSHIFT_A04       (8)
#define PORT_S07            (GPIOC->IDR)
#define PORTMASK_S07        (0xFF)
#define PORTSHIFT_S07       (0)


//Pins
#define PINS_A0		        PC8
#define PINS_A1		        PC9
#define PINS_A2		        PC10
#define PINS_A3		        PC11
#define PINS_A4		        PC12
#define PINS_Button		    PC13
#define PINS_Headphone		PB7
#define PINS_LED		    PA5
#define PINS_Mute		    PB8
#define PINS_Panel_C0		PB1
#define PINS_Panel_C1		PB2
#define PINS_Panel_L0		PB3
#define PINS_Panel_L1		PB4
#define PINS_Panel_S0		PB5
#define PINS_Panel_S1		PB6
#define PINS_Pedal		    PB0
#define PINS_S0		        PC0
#define PINS_S1	    	    PC1
#define PINS_S2  		    PC2
#define PINS_S3	    	    PC3
#define PINS_S4		        PC4
#define PINS_S5		        PC5
#define PINS_S6	        	PC6
#define PINS_S7		        PC7
#define PINS_Serial_Tx	    PA2
#define PINS_Serial_Rx	    PA3


KeyState_t _KeyStates[NUMBER_OF_KEYS];
bool _PedalStates[PedalCount];
KeyboardStatus_t _KeyboardStatus;
PanelStatus_t _PanelStatus;
VelocityCurveType_t _VelocityCurve = VelocityLinear;

void scanKeys();
void scanInputs();
void updateLeds();
void playDemo();

bool odd = true;

void setup() {
  // put your setup code here, to run once:

  pinMode(PINS_A0, OUTPUT);
  pinMode(PINS_A1, OUTPUT);
  pinMode(PINS_A2, OUTPUT);
  pinMode(PINS_A3, OUTPUT);
  pinMode(PINS_A4, OUTPUT);
  pinMode(PINS_Button, INPUT_PULLUP);
  pinMode(PINS_Headphone, INPUT_PULLUP);
  pinMode(PINS_LED, OUTPUT);
  pinMode(PINS_Mute, OUTPUT);
  pinMode(PINS_Panel_C0, OUTPUT);
  pinMode(PINS_Panel_C1, OUTPUT);
  pinMode(PINS_Panel_L0, OUTPUT);
  pinMode(PINS_Panel_L1, OUTPUT);
  pinMode(PINS_Panel_S0, INPUT_PULLUP);
  pinMode(PINS_Panel_S1, INPUT_PULLUP);
  pinMode(PINS_Pedal, INPUT_PULLUP);
  pinMode(PINS_S0, INPUT_PULLUP);
  pinMode(PINS_S1, INPUT_PULLUP);
  pinMode(PINS_S2, INPUT_PULLUP);
  pinMode(PINS_S3, INPUT_PULLUP);
  pinMode(PINS_S4, INPUT_PULLUP);
  pinMode(PINS_S5, INPUT_PULLUP);
  pinMode(PINS_S6, INPUT_PULLUP);
  pinMode(PINS_S7, INPUT_PULLUP);

  //Serial.begin(31250);
  Serial.begin(115200);
}

void loop() {

  scanKeys();
  updateLeds();
  scanInputs();
  playDemo();

  #ifdef SIMULATE_KEYS
  processSimulate();
  #endif
}

// send a 3-byte midi message
void midiCommand3(byte cmd, byte data1, byte  data2) {
  Serial.write(cmd | MIDI_CHANNEL);     // command byte (should be > 127)
  Serial.write(data1);   // data byte 1 (should be < 128)
  Serial.write(data2);   // data byte 2 (should be < 128)
}

// send a 3-byte midi message
void midiCommand2(byte cmd, byte data1) {
  Serial.write(cmd | MIDI_CHANNEL);     // command byte (should be > 127)
  Serial.write(data1);   // data byte 1 (should be < 128)
}

void midiSendNoteOn(byte note, byte velocity)
{
  midiCommand3(MIDI_CMD_NOTEON, note, velocity);
}

void midiSendPedal(bool on)
{
    midiCommand3(MIDI_CMD_CONTROLLER, MIDI_CONTROLLER_SUSTAIN, on ? MIDI_PEDAL_ON : MIDI_PEDAL_OFF);
}

void midiChangeInstrument(byte instrument)
{
    midiCommand2(MIDI_CMD_PROGCHANGE, instrument);
}

uint8_t MsToVelocity(VelocityCurveType_t curve, uint32_t ms)
{
    uint8_t time = MS_TO_CURVETIME(ms);
    
    uint8_t velocity = VelocityTable[time][curve];
    
    //TODO: linear regression between points in table?
    
    return velocity;
}

#ifndef SIMULATE_KEYS

uint8_t readKey(uint8_t address)
{
    //Write address to a pins
    PORT_A04 &= ~PORTMASK_A04;
    PORT_A04 |= (address << PORTSHIFT_A04) & PORTMASK_A04;

    //delay 225ns
    //at 100MHz, each cycle takes 10ns
    //for (uint8_t i=0; i<22; i++)
    for (uint8_t i=0; i<220; i++)
    {
        __asm("nop");
    }

    //read switches from s pins
    uint8_t s = (PORT_S07 & PORTMASK_S07) >> PORTSHIFT_S07;

    return s;
}

#endif

void scanKeys()
{
    uint8_t a = 0xFF;       //start with an unused address
    uint8_t s = 0;
    
    for (uint32_t i = 0; i<NUMBER_OF_KEYS; i++)
    {
        uint32_t now = millis();            //or put this outside the loop?
        
        if (a != KeyTable[i].address)
        {
            a = KeyTable[i].address;
            s = readKey(a);
        }

        KeyPressState_t lastState = _KeyStates[i].pressState;
        KeyPressState_t thisState = KEYSTATE(s, KeyTable[i].s1, KeyTable[i].s2);
        
        if (thisState != lastState)
        {
            if (thisState == KeyHalfPress)
            {
                if (lastState == KeyUnpressed)
                {
                    //Key is being pressed - Start the timer!
                    _KeyStates[i].timeLastState = now;
                    _KeyStates[i].pressState = thisState;
                }
                
                //Ignore the Pressed -> Half-Pressed transition, and just handle debouncing of both switches in the Unpressed case
            } 
            else if (thisState == KeyPressed)
            {
                uint8_t velocity = 64;      //default velocity, in case we missed the half-press
                
                if (lastState == KeyHalfPress)
                {
                    velocity = MsToVelocity(_VelocityCurve, now - _KeyStates[i].timeLastState);
                }
                
                _KeyStates[i].timeLastState = now;
                _KeyStates[i].pressState = thisState;
                
                midiSendNoteOn(KeyTable[i].note, velocity);    
                _PanelStatus.leds[0] = true;
            }
            else if (thisState == KeyUnpressed)
            {

                //Debounce off-transition by ignoring off-periods of less than eg 50ms
                uint32_t onDuration = now - _KeyStates[i].timeLastState;
                if (onDuration > MIN_ON_DURATION)
                {
                    if (lastState == KeyPressed) 
                    {
                        //Use running status if we can, so send an On with 0 velocity for an Off Transition
                        midiSendNoteOn(KeyTable[i].note, 0);
                        _PanelStatus.leds[0] = false;
                    }
                    _KeyStates[i].pressState = thisState;
                }
            }
        }        
    }
}

void buttonChange(ButtonType_t button, bool value)
{
    uint32_t now = millis();
    bool updated = false;

    //Debounce

    if (value) 
    {
        _PanelStatus.button[button] = value;
        _PanelStatus.buttonTimes[button] = now;
        updated = true;
    }
    else
    {
        //make sure it's been at least 50ms since the last rising edge
        if (now - _PanelStatus.buttonTimes[button] > MIN_ON_DURATION)
        {
            _PanelStatus.button[button] = value;
            _PanelStatus.buttonTimes[button] = now;
            updated = true;
        }
    }

    if (updated)
    {
        switch (button)
        {
            case ButtonMetronome:
            case ButtonDemo:
            case ButtonDemo2:
            //Ignore for now
            break;

            case ButtonInstrument:
            if (value) 
            {
                _KeyboardStatus.instrumentIndex++;
                if (_KeyboardStatus.instrumentIndex >= NUMBER_OF_INSTRUMENTS)
                {
                    _KeyboardStatus.instrumentIndex = 0;
                }
                midiChangeInstrument(InstrumentTable[_KeyboardStatus.instrumentIndex]);
            }
        }
    }
}

/*
 *  1x Pedal Inputs
 *  6x Panel connections (2x common (out), 2x led, 2x switch)
 *      LED:
            Ground Common A, Drive Common B, Drive L0, L1
            Drive Common A, Ground Common B, Drive L0, L1
            
        Switch:
            Pullups on S0, S1
            Ground Common A, Drive Common B, read S0, S1
            Drive Common A, Ground Common B, read S0, S1
 *  Headphone in
 *  Mute out
 */

void scanInputs()
{
    //TODO: Does this need optimising with port writes?

    //Pedal
    bool pedal = digitalRead(PINS_Pedal);
    if (pedal != _PedalStates[PedalSustain])
    {
        _PedalStates[PedalSustain] = pedal;
        midiSendPedal(pedal);
    }

    //Headphone/Mute
    bool headphone = digitalRead(PINS_Headphone);
    digitalWrite(PINS_Mute, !headphone);

    //debug LED -> toggle each cycle
    digitalWrite(PINS_LED, _PanelStatus.leds[0]);

    //Panel
    PanelStatus_t newPanelStatus;
    digitalWrite(PINS_Panel_C0, LOW);
    digitalWrite(PINS_Panel_C1, HIGH);

    newPanelStatus.button[0] = digitalRead(PINS_Panel_S0);
    newPanelStatus.button[1] = digitalRead(PINS_Panel_S1);

    digitalWrite(PINS_Panel_L0, _PanelStatus.leds[0]);
    digitalWrite(PINS_Panel_L1, _PanelStatus.leds[1]);


    digitalWrite(PINS_Panel_C0, HIGH);
    digitalWrite(PINS_Panel_C1, LOW);

    newPanelStatus.button[2] = digitalRead(PINS_Panel_S0);
    newPanelStatus.button[3] = digitalRead(PINS_Panel_S1);

    digitalWrite(PINS_Panel_L0, _PanelStatus.leds[2]);
    digitalWrite(PINS_Panel_L1, _PanelStatus.leds[3]);


    //Check for button changes
    for (uint8_t i=0; i<ButtonCount; i++)
    {
        if (newPanelStatus.button[i] != _PanelStatus.button[i])
        {
            buttonChange((ButtonType_t)i, newPanelStatus.button[i]);
        }
    }

}

void updateLeds()
{
    //update LEDs if required

    //Toggle Led0 each cycle
    //_PanelStatus.leds[0] = !_PanelStatus.leds[0];
}    

void playDemo()
{
    //if a demo is currently playing
    //and it's time for the next note
    //send it
    
}