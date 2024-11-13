// Import default libraries
#include <Arduino.h>
#include <elapsedMillis.h>
#include <cmath>

//Import encoding/decoding libraries
#include <iostream>
// dont need?

//Import audio-related libraries
#include <Audio.h>
#include <Wire.h>
// dont need?

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

// /************ MESSAGE QUEUEING */
// //LIFO queue - last in first out
// #define MESSAGE_QUEUE_LEN 10
// UnderwaterMessage transmitMessageQueue[MESSAGE_QUEUE_LEN];
// uint8_t transmitMessagePointer = 0;
// // UnderwaterMessage receiveMessageQueue[MESSAGE_QUEUE_LEN];
// uint8_t receiveMessagePointer = 0;
// // dont need this queue stuff?

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

static const char *user_ids_array[16] = {"USER ONE", "USER TWO", "USER THREE", "USER FOUR", "USER FIVE", "USER SIX", "USER SEVEN", "USER EIGHT", "USER NINE", "USER TEN", "USER ELEVEN", "USER TWELVE", "USER THIRTEEN", "USER FOURTEEN", "USER FIFTEEN", "USER SIXTEEN"};

// NEED TO RECORD AND PUT IN USER ID AUDIO FILES
// const unsigned int *audio_ids_array[16] = {AudioUserOne, AudioUserTwo, AudioUserThree, AudioUserFour, AudioUserFive, AudioUserSix, AudioUserSeven, AudioUserEight, AudioUserNine, AudioUserTen, AudioUserEleven, AudioUserTwelve, AudioUserThirteen, AudioUserFourteen, AudioUserFifteen, AudioUserSixteen};
const unsigned int *audio_ids_array[16] = {AudioFish, AudioLook, AudioFish, AudioLook, AudioFish, AudioLook, AudioFish, AudioLook, AudioFish, AudioLook, AudioFish, AudioLook, AudioFish, AudioLook, AudioFish, AudioLook};

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

// AudioControlSGTL5000      audioShield;
// AudioInputI2S            audioInput;
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

// const int myInput = AUDIO_INPUT_LINEIN;
const int myInput = AUDIO_INPUT_MIC;
// GUItool: begin automatically generated code
AudioControlSGTL5000    audioShield;
AudioInputI2S            i2s1;           //xy=180,111
AudioFilterBiquad        biquad1;
AudioAmplifier           amp1;           //xy=470,93
AudioAnalyzeFFT1024      fft1024_1;      //xy=616,102
AudioConnection          patchCord1(i2s1, 0, amp1, 0);
AudioConnection          patchCord2(amp1, 0, fft1024_1, 0);
// AudioConnection          patchCord3(biquad1, 0, fft1024_1, 0);
// AudioAnalyzePeak        peak;
// AudioConnection         patchCord4(amp1, peak);
// GUItool: end automatically generated code

/* SAMPLE BUFFER LOGIC
Each bin is numbered 0-1023 and has a float with its amplitude
SamplingBuffer is a time-valued array that records bin number
*/
#define MESSAGE_BIT_DELAY 125 // ms
#define NUM_SAMPLES ((int)(((MESSAGE_BIT_DELAY / 10.0) * (86.0 / 100.0)) + 1.0 + 0.9999))
int16_t samplingBuffer[NUM_SAMPLES]; // BIN indices
uint16_t samplingPointer = 0;
#define MESSAGE_LENGTH UnderwaterMessage::size
bool bitBuffer[MESSAGE_LENGTH]; // Message sample (1 or 0)
int bitPointer = 0;
#define MESSAGE_START_FREQ 10000 // Hz (1/sec)
#define MESSAGE_0_FREQ 5000 // Hz
#define MESSAGE_1_FREQ 7500 // Hz
#define MIN_VALID_AMP 0.4
#define MAX_VALID_AMP 1.5
#define BOUNDS_FREQ 1500

typedef enum {
  LISTENING, //Waiting for start frequency
  CHECK_START,
  MESSAGE_ACTIVE //Currently receiving bytes
} RECV_STATE;

RECV_STATE curState = LISTENING;
bool sampling = 0; // Are we currently sampling?
// Tone queue
#define BUZZER_PIN 14 // Which pin is the output?

const byte toneBufferLength = 5*MESSAGE_LENGTH;
int toneFreqQueue[toneBufferLength];
unsigned long toneDelayQueue[toneBufferLength];
unsigned long lastToneStart = 0;
byte toneStackPos = 0;
unsigned long lastBitChange = 0;

enum OperatingMode { 
    RECEIVE,
    TRANSMIT,
    ERROR
};
enum OperatingMode mode;

void setup() {

    // delay(500);
    
    //Serial.begin(115200); // Which? 
    Serial.begin(44120);
    
    AudioMemory(500);
    // AudioMemory(50);
    pinMode(14, OUTPUT);

    /******** INITIALIZATION */
    /*
    Steps:
    1) Initialize:
        - Blink blue to show that we are alive!
        - LED strip, battery monitor, audio shield, I/O expander
        - Blink green to show initialization good!
        - If anything fails: display full red on LED strip
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
        initializationError(2);
    }
    initializationPass(2);

    // Get battery monitor up and running
    Wire.setClock(100000);

    // Get audio shield up and running
    //  + TRANSMIT RECEIVE ENCODE DECODE SET UP
    
    // biquad1.setBandpass(0,20000,5); //Filter to only freq between 18000-22000
    // biquad1.setBandpass(0,10500, .95);
    amp1.gain(2);        // amplify sign to useful range
    audioShield.enable();
    audioShield.inputSelect(myInput);
    audioShield.micGain(90);
    audioShield.volume(1);
    initializationPass(4);
    Serial.println("Setup done");
    
    transitionState(LISTENING);

    queue.begin(); 
    // setI2SFreq(sampleRate);
    Serial.printf("Running at samplerate: %d\n", sampleRate);

    // fft.setHighPassCutoff(20000.f);
    pinMode(17, OUTPUT); //set relay pin as output

    sine.frequency(440.f * (AUDIO_SAMPLE_RATE_EXACT / sampleRate));
    sine.amplitude(0.2f);
    noise.amplitude(0.2f);
    // need this?

    mode = RECEIVE; // set to receiving mode

    Serial.println("Done initializing! Starting now! In receiving default mode!");
    strip.fill(bluishwhite, 0, 8); // light up entire strip, all set up!
    strip.show();

    // NOT WORKING!!!

    // //Battery check setup
    // if (!lc.begin()) {
    // Serial.println(F("Couldnt find Adafruit LC709203F?\nMake sure a battery is plugged in!"));
    // initializationError(6);
    // while (1) delay(10);
    // }
    // Serial.println(F("Found LC709203F"));
    // Serial.print("Version: 0x"); Serial.println(lc.getICversion(), HEX);

    // lc.setThermistorB(3950);
    // Serial.print("Thermistor B = "); Serial.println(lc.getThermistorB());

    // lc.setPackSize(LC709203F_APA_500MAH);
    // lc.setAlarmVoltage(3.8);
    // initializationPass(6);

    // delay(1000);
    // Serial.println("PRINTING BATTERY INFO");
    // printBatteryData();
    // delay(5000);
}


void loop() {

    strip.clear(); // Set all pixel colors to 'off'

    // if (toneStackPos == 0) {
    // int curId = testMessage1.id;
    // curId++;
    // if (curId > 15) {
    //   curId = 0;
    // }

    // testMessage1 = createUnderwaterMessage(2, curId);
    //Requeue message
    // transmitMessageAsync(testMessage1);
    // transmitMessageAsync(testMessage2);
    // Serial.println("Requeueing message; txbuf empty");
//   }

  // FFT has new data! Reads in data 
  if (fft1024_1.available()) {
    // each time new FFT data is available
    float maxBinAmp = 0;
    int binNumber = 0;
    for (int i = 0; i < 1024; i++) {
      float n = fft1024_1.read(i);
      if (n > maxBinAmp) {
        maxBinAmp = n;
        binNumber = i;
      }
    }
    // Serial.print((double)binNumber*(double)43.0);
    // Serial.print("Hz@");
    // Serial.println(maxBinAmp);
    if (sampling) { // Valid start freq was received, message is actively being received 
      if (validAmplitude(maxBinAmp)) {
        samplingBuffer[samplingPointer] = binNumber; // Commit sample (bin number) to memory
        samplingPointer++;
        if (samplingPointer >= NUM_SAMPLES) { // Wrap around end of buffer
          samplingPointer = 0;
        }
        // Serial.print("FFT: ");
        // Serial.print(binNumber);
        // Serial.print(" sampBufDepth: ");
        // Serial.println(samplingPointer);
      }
    }
    // We get valid message start tone!
    if (curState == LISTENING) {
      if (validAmplitude(maxBinAmp) && freqMatchesBounds((double)binNumber * (double)43.0, BOUNDS_FREQ, MESSAGE_START_FREQ)) {
        transitionState(CHECK_START); // Check start is actively checking if we've gotten start frequencies before recording the message 
      }
    } else if (curState == CHECK_START && millis() - lastBitChange >= MESSAGE_BIT_DELAY) { // Gotten all start samples
      if (isSampleBufferValid() && freqMatchesBounds(sampleBufferMax(), BOUNDS_FREQ, MESSAGE_START_FREQ)) {
        Serial.println("SAW VALID MESSAGE START FREQ!");
        transitionState(MESSAGE_ACTIVE); //Currently receiving valid message
      } else { // If buffer is not valid OR freq doesn’t match start
        transitionState(LISTENING);
      }
    } else if (curState == MESSAGE_ACTIVE && millis() - lastBitChange >= MESSAGE_BIT_DELAY) {
      if (isSampleBufferValid()) { // is our sample buffer valid?
        // Check if 1 or 0 (or neither)
        double bufferAvgFreq = sampleBufferMax();
        if (freqMatchesBounds(bufferAvgFreq, BOUNDS_FREQ, MESSAGE_1_FREQ)) {
          // Serial.println("GOT 1");
          bitBuffer[bitPointer] = 1; // WE GOT A 1
        } else if (freqMatchesBounds(bufferAvgFreq, BOUNDS_FREQ, MESSAGE_0_FREQ)) {
          // Serial.println("GOT 0");
          bitBuffer[bitPointer] = 0;
        }
      }
      clearSampleBuffer();
      lastBitChange = millis() - 5; //Reset bit timer, accoutn for delay of computation
      bitPointer++;
      if (bitPointer >= MESSAGE_LENGTH) { // We got all our samples!
        uint8_t data = 0;
        // Combine bits in bitBuffer into a single byte
        for (int i = MESSAGE_LENGTH - 1; i >= 0; i--) {
          data = (data << 1) | bitBuffer[i];
        }
        // Assign the data to an UnderwaterMessage
        UnderwaterMessage message;
        message.data = data;
        Serial.print("Got message!!! MSG = ");
        Serial.print(message.msg);
        Serial.print(", ID = ");
        Serial.println(message.id);
        
        UnderwaterMessage air_UM = {1, user_ID};
        UnderwaterMessage ascend_UM = {2, user_ID};
        UnderwaterMessage fish_UM = {3, user_ID};
        UnderwaterMessage look_UM = {4, user_ID};
        UnderwaterMessage checkin_UM = {5, user_ID};
        UnderwaterMessage sos_UM = {6, user_ID};

        union UnderwaterMessage UM_array[6] = {air_UM, ascend_UM, fish_UM, look_UM, checkin_UM, sos_UM};

        Serial.println("USER:");
        Serial.println(user_ids_array[message.id]);
        playMem.play(audio_ids_array[message.id]);

        for (int c = 0; c < 6; c++) {
            if (message.msg == UM_array[c].msg) {
                Serial.println("MESSAGE:");
                Serial.println(message_array[c]);
                playMem.play(audio_messages_array[c]);
            }
        }

        transitionState(LISTENING); // Return to listening state
      }
    }
  }

    // BUTTONS AND TRANSMITTING
    for (int b = 0; b < 6; b++) {
        if (mcp.digitalRead(buttons[b]) == LOW) {

            strip.clear();
            
            // MESSAGE SERIAL PRINT
            Serial.println(b+1);
            Serial.println(message_array[b]);

             // LED CONFIRMATION
            strip.fill(red, 0, b+1);
            strip.show();

            // BONE CONDUCTION CONFIRMATION
            // playMem.play("YOU SENT" AUDIO FILE);
            playMem.play(audio_messages_array[b]); 
            
            // HYDROPHONE TRANSMIT
            mode = TRANSMIT;
            update_relays(mode);

            transmitMessageAsync(UM_array[b]);

            Serial.println("Queued message, message in binary: ");
            for (int i = UnderwaterMessage::size - 1; i >= 0; i--) {
                // Shift and mask to get each bit
                Serial.print((UM_array[b].data >> i) & 1);
            }
            Serial.println(); // New line after printing bits

            //Deal with tone sending (asynchronous tone)
            if (toneStackPos > 0) {
                if (millis() - lastToneStart > toneDelayQueue[0]) {
                // Serial.print("ToneQueue: ");
                for (int i=1; i<toneBufferLength; i++) { //Left shift all results by 1
                    // Serial.print(toneFreqQueue[i]);
                    // Serial.print("Hz@");
                    // Serial.print(toneDelayQueue[i]);
                    toneFreqQueue[i-1] = toneFreqQueue[i];
                    toneDelayQueue[i-1] = toneDelayQueue[i];
                }
                // Serial.println();
                toneStackPos--; //we’ve removed one from the stack
                if (toneStackPos > 0) { //is there something new to start playing?
                    if (toneFreqQueue[0] > 0) {
                    tone(BUZZER_PIN, toneFreqQueue[0]); //start new tone
                    }
                    lastToneStart = millis();
                } else {
                    noTone(BUZZER_PIN); //otherwise just stop playing
                }
                }
            }

            // // hydrophone transmit
            // transmitMessageQueue[transmitMessagePointer] = UM_array[b]; // add msg to queue  
            // transmitMessagePointer++; // increment ptr by one 
            // mode = TRANSMIT;
            // update_relays(mode);
            // transmit(); // right now only changes mode back to receive

            // LED CONFIRMATION
            // strip.fill(red, 0, b+1);
            // strip.show();
            // delay(1000);
            // strip.clear();

            // BACK TO RECEIVING MODE
            mode = RECEIVE;
            update_relays(mode);
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
  // NEED THIS??
}

void initializationPass(int check) {
    strip.fill(greenishwhite, 0, check); // check = number of tiles lit up
    strip.show();
    delay(2000);
    strip.clear();
}

void initializationError(int error) {
    strip.fill(red, 0, error); // error = number of tiles lit up
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

// void transmit() {
//     /* Plays a single message through the hydrophone, then switches back to receiving mode */
//     // int[32] outputData; // init 32 bit array of ints
//     // UnderwaterMessage msg = transmitMessageQueue[transmitMessagePointer]; // message is current item in the queue
//     // msg.id = 3; // arbitrary diver ID 
//     // encode(msg, &outputData);  // outputData now has message inside of it
//     // play_data(outputData); //play that message 
//     // delay(500);
//     /* ADD SOMETHING WHERE THE DIVERS CAN BE ASSIGNED THEIR UNIQUE SENDING OFFSET */
//     mode = RECEIVE; // switch back to receiving mode 
//     // transmitMessagePointer--; // decrement pointer by 1 to get next newest message in queue 
// }

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

void clearSampleBuffer() {
  // Clear sampling buffer
  for (int i=0; i<NUM_SAMPLES; i++) {
    samplingBuffer[i] = -1; // Invalid sample
  }
  //Reset sampling pointer
  samplingPointer = 0;
}

void clearBitBuffer() {
  // Clear bit (received message) buffer
  for (int i=0; i<MESSAGE_LENGTH; i++) {
    bitBuffer[i] = 0;
  }
  //Reset bit pointer
  bitPointer = 0;
}

// Check whether sample buffer is valid
bool isSampleBufferValid() {
  int validCount = 0;
  for (int i=0; i<NUM_SAMPLES; i++) {
    if (samplingBuffer[i] != -1) {
      validCount++;
    }
  }
  return (validCount >= ((NUM_SAMPLES)/3));
}

// Find the most freqently occuring frequency in the sample buffer and return frequency
double sampleBufferMax() {
  uint16_t frequency_hist[1024] = {0}; // Initialize all values to zero
  uint16_t binNumber;
  // Populate the frequency histogram
  for (uint16_t i = 0; i < NUM_SAMPLES; i++) {
    binNumber = samplingBuffer[i];
    if (binNumber < 1024) { // Ensure binNumber is within the valid range
      frequency_hist[binNumber]++;
    }
  }
  int max_bin_count = 0;
  binNumber = 0;
  // Find the most frequent bin
  for (int16_t i = 0; i < 1024; i++) {
    if (frequency_hist[i] > max_bin_count) {
      max_bin_count = frequency_hist[i];
      binNumber = i;
    }
  }
  // Return frequency in Hz based on bin number
  return (double)binNumber * 43.0; // Adjust factor if needed for your sample rate/FFT size
}

// Function to transmit the UnderwaterMessage asynchronously
void transmitMessageAsync(UnderwaterMessage message) {
  addToneQueue(MESSAGE_START_FREQ, MESSAGE_BIT_DELAY);
  // Iterate over each bit of the message
  for (int i = 0; i < MESSAGE_LENGTH; i++) {
    // Extract the i-th bit from the message data (starting from LSB)
    if (message.data & (1 << i)) { // If the i-th bit is 1
      addToneQueue(MESSAGE_1_FREQ, MESSAGE_BIT_DELAY);
    } else { // If the i-th bit is 0
      addToneQueue(MESSAGE_0_FREQ, MESSAGE_BIT_DELAY);
    }
  }
  addToneQueue(MESSAGE_0_FREQ, (int)(MESSAGE_BIT_DELAY/2)); // End transmission with a stop tone
}

void addToneQueue(int freq, unsigned long delay) {
  if (toneStackPos < toneBufferLength && delay > 0) {
    toneFreqQueue[toneStackPos] = freq;
    toneDelayQueue[toneStackPos] = delay;
    toneStackPos++; //always increase stack pointer
    if (toneStackPos == 1) { //If it’s the first sound, start playing it
      tone(BUZZER_PIN, toneFreqQueue[0]); //start new tone
      lastToneStart = millis();
    }
  }
}

bool validAmplitude(double amp) {
  return (amp >= MIN_VALID_AMP && amp <= MAX_VALID_AMP);
}

// Example usage: freqMatchesBounds(1100, 200, 1000) -> TRUE
// Example usage: freqMatchesBounds(1101, 200, 1000) -> FALSE
bool freqMatchesBounds(double freq, double bounds, double target) {
  return abs(freq-(double)target) <= (bounds/2.0);
}

// Transition state function
void transitionState(RECV_STATE newState) {
  // Serial.print("TRANSITIONSTATE: ");
  // Serial.println(newState);
  if (newState == CHECK_START || newState == MESSAGE_ACTIVE) {
    clearSampleBuffer();
    sampling = 1; // Begin sampling
    lastBitChange = millis(); // Start timer
  } else { //Back to LISTENING
    clearSampleBuffer();
    clearBitBuffer();
    sampling = 0; // NOT sampling
  }
  curState = newState; // Set our current state to the new one
}