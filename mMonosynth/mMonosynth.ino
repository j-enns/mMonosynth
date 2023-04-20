#pragma GCC optimize ("-O2")
#pragma GCC push_options

// Fast pin state check

#define portOfPin(P)\
  (((P)>=0&&(P)<8)?&PORTD:(((P)>7&&(P)<14)?&PORTB:&PORTC))
#define ddrOfPin(P)\
  (((P)>=0&&(P)<8)?&DDRD:(((P)>7&&(P)<14)?&DDRB:&DDRC))
#define pinOfPin(P)\
  (((P)>=0&&(P)<8)?&PIND:(((P)>7&&(P)<14)?&PINB:&PINC))
#define pinIndex(P)((uint8_t)(P>13?P-14:P&7))
#define pinMask(P)((uint8_t)(1<<pinIndex(P)))

#define pinAsInput(P) *(ddrOfPin(P))&=~pinMask(P)                         // pinMode( pin, INPUT ); with pinAsInput( pin );
#define pinAsInputPullUp(P) *(ddrOfPin(P))&=~pinMask(P);digitalHigh(P)    // pinMode( pin, INPUT_PULLUP); with pinAsInputPullUp( pin );
#define pinAsOutput(P) *(ddrOfPin(P))|=pinMask(P)                         // pinMode( pin, OUTPUT ); with pinAsOutput( pin );
#define digitalLow(P) *(portOfPin(P))&=~pinMask(P)                        // digitalWrite( pin, LOW ); with digitalLow( pin );
#define digitalHigh(P) *(portOfPin(P))|=pinMask(P)                        // digitalWrite( pin, HIGH ); with digitalHigh( pin );
#define isHigh(P)((*(pinOfPin(P))& pinMask(P))>0)                         // 
#define isLow(P)((*(pinOfPin(P))& pinMask(P))==0)                         // 
#define digitalState(P)((uint8_t)isHigh(P))                               // digitalRead( pin ); with digitalState( pin );


// Interupts and variables for button reading

#include <EnableInterrupt.h>

  uint8_t oscilAtable = 0;
  uint8_t oscilBtable = 0;
  volatile uint8_t altOrder = 0;
  uint8_t lfoFreq = 4;
  volatile bool awave = false;
  volatile bool bwave = false;
  volatile bool order = false;
  volatile bool lfo = false;

void interruptAWAVE() {
  awave = true;
}
void interruptBWAVE() {
  bwave = true;
}
void interruptORDER() {
  order = true;
}
void interruptLFO() {
  lfo = true;
}


// SET UP MOZZI

#include <MozziGuts.h>
#include <Oscil.h>
#include <ADSR.h>
#include <tables/pinknoise8192_int8.h>
#include "square_sine.h"
#include "sawtooth_sine.h"
#include "sine_wave.h"
#include "triangle_sine.h"

#define CONTROL_RATE 256 // faster than usual CONTROL_RATE

Oscil <2048, AUDIO_RATE> oscilA(SINE_WAVE_DATA);
Oscil <2048, AUDIO_RATE> oscilB(SINE_WAVE_DATA);
Oscil <2048, AUDIO_RATE> oscilLFO(SINE_WAVE_DATA);
Oscil <8192, AUDIO_RATE> noise(PINKNOISE8192_DATA);

ADSR <CONTROL_RATE, AUDIO_RATE> envelope;
ADSR <CONTROL_RATE, AUDIO_RATE> envelopeB;

const int8_t * wavetable[4] ={
  SINE_WAVE_DATA,
  TRIANGLE_SINE_DATA,
  SAWTOOTH_SINE_DATA,
  SQUARE_SINE_DATA
};

// slide pots
#define ATTACK      (5)
#define RELEASE        (4)
#define BATTACK      (3)
#define BVOLUME        (2)
#define DETUNE         (1)
#define NOISE         (0)

uint8_t noisePot = 0;
uint8_t detunePot = 0;

// buttons
#define LFO     2
#define AWAVE     3
#define BWAVE     4
#define ORDER     5


// SET UP MIDI

#include <MIDI.h>
#include "midi_freq.h"

MIDI_CREATE_DEFAULT_INSTANCE();

#define NOTEON_BUFFER_SIZE 16    //BUFFER
uint8_t midiNote;      //note
uint8_t noteBuffer[NOTEON_BUFFER_SIZE];   //buff[BUFFER];
uint8_t velBuffer[NOTEON_BUFFER_SIZE];
uint8_t bufferTail = 0;   // buffersize


// handle incoming midi

void handleNoteOn(byte channel, byte note, byte velocity) {
  if (velocity == 0) { handleNoteOff(channel, note, velocity); }
  if (bufferTail < NOTEON_BUFFER_SIZE) {
    if (channel != 20) {
      noteBuffer[bufferTail] = note;
      velBuffer[bufferTail] = velocity;
      ++bufferTail;
    }

    int attackDecay = (mozziAnalogRead( ATTACK ) >> 2) + 1;
    int release = (mozziAnalogRead( RELEASE ) >> 1) + 3;
    int bAttack = (mozziAnalogRead( BATTACK ) >> 3) + 1;
    int bVolume = (velocity * ( (mozziAnalogRead( BVOLUME ) >> 3) + 1 )) >> 7;
    if (bVolume <= 0) bVolume = 1;
    
    envelope.setLevels ( velocity + 1, velocity, velocity, 1 );
    envelopeB.setLevels ( velocity, bVolume, bVolume, 1 );

    envelope.setAttackUpdateSteps( attackDecay );
    envelope.setDecayUpdateSteps( attackDecay );
    envelope.setReleaseUpdateSteps( release );

    envelopeB.setAttackUpdateSteps( bAttack );
    envelopeB.setDecayUpdateSteps( bAttack );
    envelopeB.setReleaseUpdateSteps( release );
    
    unsigned long frequency = pgm_read_dword ( &PHASE_INC[ note ]);

    oscilA.setPhaseInc( frequency );

    if ( altOrder >= 4) {
      oscilB.setPhaseInc( pgm_read_dword ( &PHASE_INC[ note + (detunePot >> 4) ] ) );   // increase second osccilitor by up to 16 semitones
    }
    else {
      oscilB.setPhaseInc( frequency );
    }
    
    envelope.noteOn();
    envelopeB.noteOn();
  }
}

void handleNoteOff(byte channel, byte note, byte velocity) {
  if (bufferTail >= 1) {
    for (int i=0; i < bufferTail; i++) {
      if (noteBuffer[i] == note) {
        for (int j=i; j < bufferTail - 1; j++) {
          noteBuffer[j] = noteBuffer[j+1];
          velBuffer[j] = velBuffer[j+1];
        }
        noteBuffer[bufferTail] = 0;
        velBuffer[bufferTail] = 0;
        --bufferTail;
        break;
      }
    }
    if (bufferTail == 0) {
      envelopeB.noteOff();
      envelope.noteOff();
    } else {
      handleNoteOn(20, noteBuffer[bufferTail-1], velBuffer[bufferTail-1]);
    }
  } else {
    bufferTail = 0;
    noteBuffer[bufferTail] = 0;
    velBuffer[bufferTail] = 0;

    envelopeB.noteOff();
    envelope.noteOff();
  }
}

//void handleStop() {
void __attribute__((optimize("Os"))) handleStop() {
  for (int i=0; i <= bufferTail; i++) {
    noteBuffer[i] = 0;
    velBuffer[i] = 0;
  }
  bufferTail = 0;
  envelopeB.noteOff();
  envelope.noteOff();
}


// arduino setup code

void __attribute__((optimize("Os"))) setup() {
  uint8_t midiChannel = 1;
  pinMode(AWAVE, INPUT_PULLUP);
  pinMode(BWAVE, INPUT_PULLUP);
  pinMode(ORDER, INPUT_PULLUP);
  pinMode(LFO, INPUT_PULLUP);

  if ( digitalState(AWAVE) == LOW ) {
    midiChannel += 1;
  }
  if ( digitalState(BWAVE) == LOW ) {
    midiChannel += 2;
  } 
  if ( digitalState(ORDER) == LOW ) {
    midiChannel += 4;
  } 
  if ( digitalState(LFO) == LOW ) {
    midiChannel += 8;
  }

  MIDI.setHandleNoteOn(handleNoteOn);  // Put only the name of the function
  MIDI.setHandleNoteOff(handleNoteOff);  // Put only the name of the function
  MIDI.setHandleStop(handleStop);  // Put only the name of the function

  MIDI.begin(midiChannel);
  MIDI.turnThruOff();

  envelope.setAllUpdateSteps(5,20,65535,20);
  envelopeB.setAllUpdateSteps(5,20,65535,20);

  noise.setFreq((float)AUDIO_RATE/PINKNOISE8192_SAMPLERATE);
  oscilLFO.setFreq(4.5f);

//button interupt
  enableInterrupt(AWAVE, interruptAWAVE, FALLING);
  enableInterrupt(BWAVE, interruptBWAVE, FALLING);
  enableInterrupt(ORDER, interruptORDER, FALLING);
  enableInterrupt(LFO, interruptLFO, FALLING);

  startMozzi(CONTROL_RATE);
}


// Mozzi code

void updateControl() {

  MIDI.read();          // process midi in serial buffer
  envelope.update();    // update ADSR at control rate
  envelopeB.update();   // update ADSR at control rate

  noisePot = mozziAnalogRead(NOISE) >> 2;       // 0-255
  detunePot = mozziAnalogRead( DETUNE ) >> 2;   // 0-255

  if (awave == true) {
    oscilAtable = ( oscilAtable + 1 ) & 3;      // fast than oscilAtable % 4
    oscilA.setTable(wavetable[oscilAtable]);
    awave = false;                              // turn interupt flag back off
  }
  if (bwave) {
    oscilBtable = ( oscilBtable + 1 ) & 3;
    oscilB.setTable(wavetable[oscilBtable]);
    bwave = false;
  }
  if (order) {
    altOrder = ( altOrder + 1 ) & 7;
    order = false;
  }
  if (lfo) {
    lfoFreq = ( lfoFreq + 1 ) & 15;
    oscilLFO.setPhaseInc( pgm_read_word ( &LFO_INC[ lfoFreq ]) );   // setPhaseInc is faster than setFreq with pre-computed values
    lfo = false;
  }

}


AudioOutput_t updateAudio() {
  /*
  oscilA.next()   -128 - +127
  oscilB.next()   -128 - +127 
  noise.next()    -128 - +127

  envelope.next() 0 - +128
  envelopeB.next() 0 - +127
  */
  int out = 0;
  int fuzz = ((int16_t)noise.next() * noisePot) >> 8;    // +- 128  (8bit * 8 bit - 8 = 8 bits)
  int8_t lfoNext = oscilLFO.next();
  Q15n16 modulation = (Q15n16) detunePot * lfoNext;

/*
Generation Algarithms
0-3 use lfo for sutle detune/vibrato effect
4-7 detune oscilB by up to 16 semitones
1,3,7 phase modulate oscilA
4,5 do NOT phase modulate oscilB
2,3,5 use sepate envelopes for oscilA and oscilB
*/
if (altOrder == 1 ) {
  //Q15n16 antiMod = (Q15n16) detunePot * (-lfoNext);
  //int outA = oscilA.phMod(antiMod) << 1;        // +- 256   (8bit + 1 = 9 bits)
  int outA = oscilA.phMod(-modulation) << 1;        // +- 256   (8bit + 1 = 9 bits)
  int outB =  ((int16_t)envelopeB.next() * (fuzz + oscilB.phMod(modulation)) ) >> 7;  // +- 256  ( (7 bits * (8bits + 8)) >> 7 = 9 bits)
  out = outA + outB;       // 9bit + 9 = 10 bits
  out = ( out >> 1 ) * envelope.next();      // 9bit * 7 = 16 bits
} 

else if (altOrder == 2) {
  int outA = oscilA.next() * envelope.next();        //    15 bits
  int outB =  ((int16_t)envelopeB.next() * (fuzz + oscilB.phMod(modulation)) ) >> 1;  // 15 bits
  out = outA + outB;     //16 bits
}

else if (altOrder == 3) {
  //Q15n16 antiMod = (Q15n16) detunePot * (-lfoNext);
  //int outA = oscilA.phMod(antiMod) * envelope.next();         //    15 bits
  int outA = oscilA.phMod(-modulation) * envelope.next();         //    15 bits
  int outB =  ((int16_t)envelopeB.next() * (fuzz + oscilB.phMod(modulation)) ) >> 1;  // 15 bits
  out = outA + outB;     //16 bits
}

else if (altOrder == 4) {
  int outA = oscilA.next() << 1;        // +- 256   (8bit + 1 = 9 bits)
  int outB =  ((int16_t)envelopeB.next() * (fuzz + oscilB.next()) ) >> 7;  // +- 256  ( (7 bits * (8bits + 8)) >> 7 = 9 bits)
  out = outA + outB;       // 9bit + 9 = 10 bits
  out = ( out >> 1 ) * envelope.next();      // 9bit * 7 = 16 bits
}

else if (altOrder == 5) {
  int outA = oscilA.next() * envelope.next();        //    15 bits
  int outB =  ((int16_t)envelopeB.next() * (fuzz + oscilB.next()) ) >> 1;  // 15 bits
  out = outA + outB;     //16 bits
}

else if (altOrder == 6) {
  int outA = oscilA.next() << 1;        // +- 256   (8bit + 1 = 9 bits)
  int outB =  ((int16_t)envelopeB.next() * (fuzz + oscilB.phMod(modulation)) ) >> 7;  // +- 256  ( (7 bits * (8bits + 8)) >> 7 = 9 bits)
  out = outA + outB;       // 9bit + 9 = 10 bits
  out = ( out >> 1 ) * envelope.next();      // 9bit * 7 = 16 bits
}

else if (altOrder == 7) {
  int outA = oscilA.phMod(-modulation) << 1;        // +- 256   (8bit + 1 = 9 bits)
  int outB =  ((int16_t)envelopeB.next() * (fuzz + oscilB.phMod(modulation)) ) >> 7;  // +- 256  ( (7 bits * (8bits + 8)) >> 7 = 9 bits)
  out = outA + outB;       // 9bit + 9 = 10 bits
  out = ( out >> 1 ) * envelope.next();      // 9bit * 7 = 16 bits
}

else {
  int outA = oscilA.next() << 1;        // +- 256   (8bit + 1 = 9 bits)
  int outB =  ((int16_t)envelopeB.next() * (fuzz + oscilB.phMod(modulation)) ) >> 7;  // +- 256  ( (7 bits * (8bits + 8)) >> 7 = 9 bits)
  out = outA + outB;       // 9bit + 9 = 10 bits
  out = ( out >> 1 ) * envelope.next();      // 9bit * 7 = 16 bits
}

  return MonoOutput::from16Bit(out);
}


// arduino loop block

void loop() {
  audioHook();
}


#pragma GCC pop_options