// initialization sketch + button functionality

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
// NEED TO COMPRESS THESE FILES
#include "AudioAir.h"
#include "AudioCheck_in.h"
#include "AudioFish.h"
#include "AudioLook.h"
#include "AudioAscend.h"
#include "AudioSos.h"

uint8_t user_ID = 1;

/************ MESSAGE PACKETS */
union UnderwaterMessage {
    struct {
        uint8_t msg : 3; // 8 bits for message
        uint8_t id : 4;  // 8 bits for id
    };
    uint8_t data; // 16 bits total (concatenation of msg and id)

    static constexpr uint8_t size = 7;
};

UnderwaterMessage createUnderwaterMessage(uint8_t m, uint8_t i) {
    UnderwaterMessage message;
    message.msg = m & 0x7;  // Mask to 3 bits
    message.id = i & 0xF;   // Mask to 4 bits
    return message;
}

/************ MESSAGE QUEUEING */
//LIFO queue - last in first out
#define MESSAGE_QUEUE_LEN 10
UnderwaterMessage transmitMessageQueue[MESSAGE_QUEUE_LEN];
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

uint8_t buttons[6] = {BUTTON_PIN1, BUTTON_PIN2, BUTTON_PIN3, BUTTON_PIN4, BUTTON_PIN5, BUTTON_PIN6};

static const char *message_array[6] = {"AIR", "ASCEND", "FISH", "LOOK", "CHECK-IN", "SOS"};

const unsigned int *audio_messages_array[6] = {AudioAir, AudioAscend, AudioFish, AudioLook, AudioCheckin, AudioSos};
// const unsigned int *audio_messages_array[6] = {AudioFish, AudioLook, AudioFish, AudioLook, AudioFish, AudioLook};

UnderwaterMessage air_UM = {1, user_ID};
UnderwaterMessage ascend_UM = {2, user_ID};
UnderwaterMessage fish_UM = {3, user_ID};
UnderwaterMessage look_UM = {4, user_ID};
UnderwaterMessage checkin_UM = {5, user_ID};
UnderwaterMessage sos_UM = {6, user_ID};

union UnderwaterMessage UM_array[6] = {air_UM, ascend_UM, fish_UM, look_UM, checkin_UM, sos_UM};

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
    initializationPass(2);

    // Get battery monitor up and running
    Wire.setClock(100000);

    // Get audio shield up and running
    audioShield.enable();
    audioShield.inputSelect(micInput);
    audioShield.micGain(60);    //0-63
    audioShield.volume(1);    //0-1
    initializationPass(4);

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

    // NOT WORKING!!!

    // //Battery check setup
    // if (!lc.begin()) {
    // Serial.println(F("Couldnt find Adafruit LC709203F?\nMake sure a battery is plugged in!"));
    // while (1) delay(10);
    // }
    // Serial.println(F("Found LC709203F"));
    // Serial.print("Version: 0x"); 
    // Serial.println(lc.getICversion(), HEX);

    // // lc.setThermistorB(3950);
    // // Serial.print("Thermistor B = "); Serial.println(lc.getThermistorB());

    // lc.setPackSize(LC709203F_APA_500MAH);
    // lc.setAlarmVoltage(3.8);

    // initializationPass(6);

    // mode = RECEIVE; // set it to receiving mode

    Serial.println("Done initializing! Starting now!");
    strip.fill(bluishwhite, 0, 8); // light up entire strip, all set up!
    strip.show();
    // tone(14, 5000);
    // playMem.play(AudioSampleLow_o2);

}

void loop() {
  strip.clear(); // Set all pixel colors to 'off'

  for (int b = 0; b < 6; b++) {
    if (mcp.digitalRead(buttons[b]) == LOW) {
        // serial monitor
        Serial.println(b+1);
        Serial.println(message_array[b]);

        // bone conduction
        // playMem.play("YOU SENT" AUDIO FILE);
        playMem.play(audio_messages_array[b]); 

        // hydrophone transmit
        transmitMessageQueue[transmitMessagePointer] = UM_array[b]; // add msg to queue  
        transmitMessagePointer++; // increment ptr by one 

        strip.fill(red, 0, b+1);
        strip.show();
        delay(1000);
        strip.clear();
    }
  }
}

// TODO otti moment
void initializationPass(int error) {
    // passed in int error represents number of LEDs in strip that we want to light up
    // for initialization, using greenishwhite (whereas showing life is bluishwhite)
    
    strip.fill(greenishwhite, 0, error); // error = number of tiles lit up
    strip.show();
    delay(2000);
    strip.clear();
}

// IO expander = initializationPass(2)
// audio shield = initializationPass(4)
// LC battery monitor = initializationPass(6)