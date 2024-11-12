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
// #include "AudioAir.h"
// #include "AudioCheck_in.h"
#include "AudioFish.h"
#include "AudioLook.h"
// #include "AudioAscend.h"
// #include "AudioSos.h"

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

/************ MESSAGE QUEUEING */
//LIFO queue - last in first out
#define MESSAGE_QUEUE_LEN 10
UnderwaterMessage transmitMessageQueue[MESSAGE_QUEUE_LEN];
uint8_t transmitMessagePointer = 0;
// UnderwaterMessage receiveMessageQueue[MESSAGE_QUEUE_LEN];
uint8_t receiveMessagePointer = 0;

/******* ADDITIONAL DEVICE SETUP */

// IO expander, buttons, LEDs set up
#define BUTTON_PIN1 0
#define BUTTON_PIN2 1
#define BUTTON_PIN3 2
#define BUTTON_PIN4 3
#define BUTTON_PIN5 4
#define BUTTON_PIN6 5

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

// const unsigned int *audio_messages_array[6] = {AudioAir, AudioAscend, AudioFish, AudioLook, AudioCheckin, AudioSos};
const unsigned int *audio_messages_array[6] = {AudioFish, AudioLook, AudioFish, AudioLook, AudioFish, AudioLook};

UnderwaterMessage air_UM = {1, user_ID};
UnderwaterMessage ascend_UM = {2, user_ID};
UnderwaterMessage fish_UM = {3, user_ID};
UnderwaterMessage look_UM = {4, user_ID};
UnderwaterMessage checkin_UM = {5, user_ID};
UnderwaterMessage sos_UM = {6, user_ID};

union UnderwaterMessage UM_array[6] = {air_UM, ascend_UM, fish_UM, look_UM, checkin_UM, sos_UM};

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
    pinMode(17, OUTPUT);
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
    // Serial.print("Version: 0x"); Serial.println(lc.getICversion(), HEX);

    // lc.setThermistorB(3950);
    // Serial.print("Thermistor B = "); Serial.println(lc.getThermistorB());

    // lc.setPackSize(LC709203F_APA_500MAH);
    // lc.setAlarmVoltage(3.8);
    // initializationPass(6);

    mode = RECEIVE; // set it to receiving mode

    Serial.println("Done initializing! Starting now!");
    strip.fill(bluishwhite, 0, 8); // light up entire strip, all set up!
    strip.show();

    // delay(1000);
    // Serial.println("PRINTING BATTERY INFO");
    // printBatteryData();
    // delay(5000);
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

        mode = TRANSMIT;
        update_relays(mode);
        transmit(); // right now only changes mode back to receive

        strip.fill(red, 0, b+1);
        strip.show();
        delay(1000);
        strip.clear();
        }
    }
  // switch(mode) {
  //       case RECEIVE:
  //           update_relays(mode);
  //           // receive();
  //           Serial.println("NOW RECEIVING");
  //           break;
  //       case TRANSMIT:
  //           update_relays(mode);
  //           // transmit();
  //           Serial.println("NOW TRANSMITTING");
  //           break;
  //   }
  update_relays(mode);
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

void update_relays(OperatingMode newMode) {
    if (newMode == RECEIVE); {
        digitalWrite(17, LOW); // make relays go to listen mode
    }
    if (newMode == TRANSMIT) {
        digitalWrite(17, HIGH); // make relays go into transmit mode
    }
}

void transmit() {
    /* Plays a single message through the hydrophone, then switches back to receiving mode */
    // int[32] outputData; // init 32 bit array of ints
    // UnderwaterMessage msg = transmitMessageQueue[transmitMessagePointer]; // message is current item in the queue
    // msg.id = 3; // arbitrary diver ID 
    // encode(msg, &outputData);  // outputData now has message inside of it
    // play_data(outputData); //play that message 
    // delay(500);
    /* ADD SOMETHING WHERE THE DIVERS CAN BE ASSIGNED THEIR UNIQUE SENDING OFFSET */
    mode = RECEIVE; // switch back to receiving mode 
    // transmitMessagePointer--; // decrement pointer by 1 to get next newest message in queue 

}

void printBatteryData(){
  //colors
  uint32_t green = strip.Color(0, 255, 0);
  uint32_t red = strip.Color(255, 0, 0);
  uint32_t orange = strip.Color(255, 150, 0);
  uint32_t yellow = strip.Color(255, 255, 0);

    // time = millis()
    Serial.print("Batt_Voltage:");
    Serial.print(lc.cellVoltage(), 3);
    Serial.print("\t");
    Serial.print("Batt_Percent:");
    Serial.print(lc.cellPercent(), 1);
    Serial.print("\t");
    Serial.print("Batt_Temp:");
    Serial.println(lc.getCellTemperature(), 1);

    if (lc.cellPercent() > 80){
        strip.fill(green, 0, 7);
    }
    else if (lc.cellPercent() > 60){
        strip.fill(yellow, 0, 5);
    }
    else if (lc.cellPercent() > 20){
        strip.fill(orange, 0, 3);
    }
    else{
        strip.fill(red, 0, 1);
    }
    strip.show();
}