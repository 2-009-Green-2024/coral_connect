// initialization sketch

// Import default libraries
#include <Arduino.h>
#include <elapsedMillis.h>
#include <cmath>

//Import encoding/decoding libraries
#include <iostream>

//Import audio-related libraries
#include <Audio.h>
#include <Wire.h>

// Import other device libraries
#include <Adafruit_LC709203F.h> // Battery monitor
#include <Adafruit_MCP23X17.h> // IO expander
#include <Adafruit_NeoPixel.h> // LEDs

// Import audio samples
// #include "AudioSampleLow_o2.h"
// #include "AudioSampleSos.h"
// #include "AudioSampleGoing_up.h"
// #include "AudioSampleGoing_down.h"
// #include "AudioSampleLow_oxygen.h"
// #include "AudioSampleCheck_in.h"
// #include "AudioSampleCome_look.h"
#include "AudioSampleNo_msg.h"

/************ MESSAGE PACKETS */
union UnderwaterMessage {
    struct {
        uint8_t msg; // 8 bits for message
        uint8_t id;  // 8 bits for id
    };
    uint16_t data; // 16 bits total (concatenation of msg and id)

    // // // Constructor for easy initialization
    // UnderwaterMessage(uint8_t m, uint8_t i) : msg(m), id(i) {}
};

/************ MESSAGE QUEUEING */
//LIFO queue - last in first out
#define MESSAGE_QUEUE_LEN 10
// UnderwaterMessage transmitMessageQueue[MESSAGE_QUEUE_LEN];
uint8_t transmitMessagePointer = 0;

// UnderwaterMessage receiveMessageQueue[MESSAGE_QUEUE_LEN];
uint8_t receiveMessagePointer = 0;

// IO expander, buttons, LEDs set up
#define BUTTON_PIN1 0
#define BUTTON_PIN2 1
#define BUTTON_PIN3 2
#define BUTTON_PIN4 3
#define BUTTON_PIN5 4
#define BUTTON_PIN6 5
#define BUTTON_PIN7 6

// Pin connected to neopixels strip
#define LED_PIN  14
#define LED_COUNT 12

// Declare NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_RGBW + NEO_KHZ800);

uint32_t red = strip.Color(0, 64, 0, 0);
uint32_t greenishwhite = strip.Color(0, 64, 0, 64); // r g b w
uint32_t bluishwhite = strip.Color(64, 0, 0, 64);

Adafruit_MCP23X17 mcp;
Adafruit_LC709203F lc;

uint8_t row_pins[4] = {BUTTON_PIN2, BUTTON_PIN7, BUTTON_PIN6, BUTTON_PIN4};
uint8_t col_pins[3] = {BUTTON_PIN3, BUTTON_PIN1, BUTTON_PIN5};

char keypad_array[4][3] = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}, {10, 0, 11}};
static const char *message_array[4][3] = {{"SOS", "GOING UP", "GOING DOWN"}, {"LOW OXYGEN", "CHECK-IN", "COME LOOK"}, {"no msg", "no msg", "no msg"}, {"no msg", "no msg", "no msg"}};

// const unsigned int *audio_messages_array[4][3] = {
//   {AudioSampleSos, AudioSampleGoing_up, AudioSampleGoing_down},
//   {AudioSampleLow_oxygen, AudioSampleCheck_in, AudioSampleCome_look},
//   {AudioSampleNo_msg, AudioSampleNo_msg, AudioSampleNo_msg}
// };

const unsigned int *audio_messages_array[4][3] = {
  {AudioSampleNo_msg, AudioSampleNo_msg, AudioSampleNo_msg},
  {AudioSampleNo_msg, AudioSampleNo_msg, AudioSampleNo_msg},
  {AudioSampleNo_msg, AudioSampleNo_msg, AudioSampleNo_msg}
};


UnderwaterMessage low_oxygen = {0x01, 0x01}; // msg = 0x01, id = 0x01
UnderwaterMessage sos = {0x02, 0x02};        // msg = 0x02, id = 0x02
UnderwaterMessage going_up = {0x04, 0x03};   // msg = 0x04, id = 0x03
UnderwaterMessage going_down = {0x08, 0x04}; // msg = 0x08, id = 0x04
UnderwaterMessage check_in = {0x10, 0x05};   // msg = 0x10, id = 0x05
UnderwaterMessage come_look = {0x20, 0x06};  // msg = 0x20, id = 0x06
UnderwaterMessage no_msg = {0x40, 0x07};     // msg = 0x40, id = 0x07

union UnderwaterMessage messages_bits_array[4][3] = {
  {sos, going_up, going_down},
  {low_oxygen, check_in, come_look},
  {no_msg, no_msg, no_msg} 
};

// amplifier set ups

const int micInput = AUDIO_INPUT_MIC;
//const int micInput = AUDIO_INPUT_LINEIN;

const uint32_t sampleRate = 44100;
//const uint32_t sampleRate = 96000;
// const uint32_t sampleRate = 192000;
//const uint32_t sampleRate = 234000;

const int16_t semitones = 12 * -4;    // shift in semitones
const float32_t pitchShiftFactor = std::pow(2., semitones / 12.);

elapsedMillis performanceStatsClock;

AudioControlSGTL5000      audioShield;
AudioInputI2S            audioInput;
// PitchShift<2048>          fft(sampleRate, pitchShiftFactor);
// Counter                    counter;
// Printer                    printer;
AudioOutputI2S           audioOutput;
AudioOutputAnalogStereo    dacs1;            //xy=372,173
AudioSynthWaveformSine   sine;
AudioSynthNoiseWhite      noise;
AudioSynthWaveform       waveform1;        //xy=171,84
AudioSynthWaveform       waveform2;        //xy=178,148
AudioRecordQueue          queue;
// WavFileWriter            wavWriter(queue);
AudioPlayMemory           playMem; 
AudioAnalyzeToneDetect    findTone;
AudioAnalyzeRMS           rms_L;
AudioAnalyzeRMS           rms_R;

int current_waveform=0;
AudioConnection bc_transducer(playMem, 0, audioOutput, 0); // thru hydro - 1, thru bone conduction - 0 for audioOut
// AudioConnection patchCord1(waveform1, 0, audioOutput, 1);
AudioConnection hydro_listener(audioInput, 0, findTone, 0); //for output: hydro - 0, bone conduction - 1    
AudioConnection hydro_listener1(audioInput, queue); 
// AudioConnection patchCord1(audioInput, 0, rms_L, 0); //for output: hydro - 0, bone conduction - 1    
// AudioConnection patchCord(audioInput, 0, rms_R, 1); //for output: hydro - 0, bone conduction - 1    
int dOUT;
int16_t* destination; 
// pinMode(dOUT, OUTPUT);

// setup for testing a whole octave of sine waves
float32_t octaveF10[13] = {22350.6, 23679.6, 25087.7, 26579.5, 28160.0, 29834.5, 31608.5, 33488.1, 35479.4, 37589.1, 39824.3, 42192.3};
// note names                  F 10,   F# 10,    G 10,   G# 10,    A 10,   A# 10,    B 10,    C 11,   C# 11,    D 11,   D# 11,    E 11
// AudioSynthWaveformSine sineBank[12];
// AudioMixer4 sineMixers[4];

void printPerformanceData();

enum OperatingMode { 
    RECEIVE,
    TRANSMIT,
    ERROR
};
enum OperatingMode mode;


void setup() {
  
    delay(500);
    Serial.begin(115200);
    AudioMemory(500);

    /******** INITIALIZATION */
    /*
    Steps:
    1) Initialize LED strip
        1b) Blink blue to show that we are alive!
    2) Initialize battery monitor
    3) Initialize audio shield
    4) Initialize I/O expander
    5) Play a quick tune on the bone conduction to show we are alive
    6) Blink green to show initialization good!
    If anything fails: display red on LED strip 
    */

    // Get LEDs up and running
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
    strip.setBrightness(32);

    // LEDs: show that we are alive
    strip.fill(bluishwhite, 0, 8); // light up entire strip
    strip.show();
    // keep strip on only for 3 seconds, then continue with rest of installation
    delay(3000); 
    strip.clear();

    // Get IO expander up and running
    if (!mcp.begin_I2C()) {
        Serial.println("Error: I2C connection with IO expander.");
        // initialization error
    }
    initializationError(2);

    // Get buttons (keypad right now) up and running    
    for (int r = 0; r < 4; r++) {
        mcp.pinMode(row_pins[r], OUTPUT);
        mcp.digitalWrite(row_pins[r], HIGH);
        }
    for (int c = 0; c < 3; c++) {
        mcp.pinMode(col_pins[c], INPUT_PULLUP);
        }
    // Serial.println("Begin loop to check what buttons are pressed.");

    // Get battery monitor up and running
    Wire.setClock(100000);

    // Get audio shield up and running
    audioShield.enable();
    audioShield.inputSelect(micInput);
    audioShield.micGain(60);    //0-63
    audioShield.volume(1);    //0-1
    initializationError(4);

    queue.begin(); 
    // setI2SFreq(sampleRate);
    Serial.printf("Running at samplerate: %d\n", sampleRate);

    // fft.setHighPassCutoff(20000.f);
    pinMode(17, OUTPUT); //set relay pin as output

    // Confirgure both to use "myWaveform" for WAVEFORM_ARBITRARY
    // waveform1.arbitraryWaveform(myWaveform, 172.0);
    // waveform2.arbitraryWaveform(myWaveform, 172.0);

    // waveform1.frequency(440);
    // // waveform2.frequency(440);
    // waveform1.amplitude(1.0);
    // // waveform2.amplitude(1.0);

    // current_waveform = WAVEFORM_SQUARE;
    // waveform1.begin(current_waveform);

    sine.frequency(440.f * (AUDIO_SAMPLE_RATE_EXACT / sampleRate));
    sine.amplitude(0.2f);
    noise.amplitude(0.2f);

    //Battery check setup
    if (!lc.begin()) {
    Serial.println(F("Couldnt find Adafruit LC709203F?\nMake sure a battery is plugged in!"));
    while (1) delay(10);
    }
    Serial.println(F("Found LC709203F"));
    Serial.print("Version: 0x"); Serial.println(lc.getICversion(), HEX);

    // lc.setThermistorB(3950);
    // Serial.print("Thermistor B = "); Serial.println(lc.getThermistorB());

    lc.setPackSize(LC709203F_APA_500MAH);
    lc.setAlarmVoltage(3.8);

    initializationError(6);

    // mode = RECEIVE; // set it to receiving mode

    Serial.println("Done initializing! Starting now!");
    strip.fill(bluishwhite, 0, 8); // light up entire strip, all set up!
    strip.show();
    // tone(14, 5000);
    // playMem.play(AudioSampleLow_o2);

}

void loop() {
  strip.clear(); // Set all pixel colors to 'off'

  for (int r = 0; r < 4; r++) {
      mcp.digitalWrite(row_pins[r], LOW);
      delay(5);
      for (int c = 0; c < 3; c++) {
          if (mcp.digitalRead(col_pins[c]) == LOW) {
              mode = TRANSMIT;
              Serial.println(keypad_array[r][c]); // returns button pressed
              Serial.println(message_array[r][c]); // returns message pressed

              // transmitMessageQueue[transmitMessagePointer] = message_array[r][c];// add msg to queue  
              transmitMessagePointer++; //increment ptr by one 

              // confirmation of message pressed back to bone conduction
              playMem.play(audio_messages_array[r][c]); // plays message pressed
              // Serial.println(messages_bits_array[r][c]); // returns UnderwaterMessage bits signal pressed

              Serial.print("Message ID: ");
              Serial.print(messages_bits_array[r][c].id, HEX); // prints the `id` in hexadecimal
              Serial.print(" | Message Code: ");
              Serial.println(messages_bits_array[r][c].msg, HEX); // prints the `msg` in hexadecimal

              strip.fill(red, 0, keypad_array[r][c]);
              strip.show();
          }
      }
      mcp.digitalWrite(row_pins[r], HIGH);
      delay(5);
      }

}

// TODO otti moment
void initializationError(int error) {
    // passed in int error represents number of LEDs in strip that we want to light up
    // for initialization, using greenishwhite (whereas showing life is bluishwhite)
    
    strip.fill(greenishwhite, 0, error); // error = number of tiles lit up
    strip.show();
    delay(3000);
    // while(1);
    strip.clear();
}

// IO expander = initializationError(2)
// audio shield = initializationError(4)
// LC battery monitor = initializationError(6)