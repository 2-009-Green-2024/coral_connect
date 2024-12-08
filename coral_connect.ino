// Import default libraries
#include <Arduino.h>
#include <elapsedMillis.h>
#include <cmath>
#include <iostream>

//Import audio-related libraries
#include <Audio.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

// Import other device libraries
#include "Adafruit_MAX1704X.h" // Battery monitor
#include <Adafruit_MCP23X17.h> // IO expander
#include <Adafruit_NeoPixel.h> // LEDs
#include <Adafruit_TPA2016.h> // BC transducer amp
#include "pindefs.h" // Pin definitions

/************ MESSAGE PACKETS */
union UnderwaterMessage {
    struct {
        uint8_t msg : 3; // 8 bits for message
        uint8_t id : 4;  // 8 bits for id
    };
    uint8_t data; // 16 bits total (concatenation of msg and id)

    static constexpr uint8_t size = 7;
};

/******* ADDITIONAL DEVICE SETUP */

// Declare NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// GRB color order
uint32_t red = strip.Color(255, 0, 0, 0);
uint32_t greenishwhite = strip.Color(0, 255, 0); // g r b w
uint32_t bluishwhite = strip.Color(0, 0, 255, 0);
uint32_t cyan = strip.Color(0, 255, 255, 0);
uint32_t purple = strip.Color(255, 0, 255, 0);

// LED blinky anim stuff
int counter = 0;
bool dir = 1;
long lastLEDUpdateTime = 0;
long lastButtonCheckTime = 0;

Adafruit_MCP23X17 mcp; // I/O expander
Adafruit_MAX17048 maxlipo; // Battery monitor
Adafruit_TPA2016 audioamp = Adafruit_TPA2016(); // Audio amplifier

// Battery monitor stuff
float lastCellVoltage = 0.0;
float lastPercentage = 100.0; // Initialize with a full battery percentage
unsigned long lastBattCheckTime = 0;
int pctBufPointer = -1;
#define BATTERY_SAMPLES 5
float pctBuf[BATTERY_SAMPLES];

// Underwater message array of messages to be sent upon each button press
UnderwaterMessage UM_array[6];

// IDS: how we identify one device from another
// Pointers to user ID as string, and audio ID files
int user_ID = 0;
int other_user_ID = 1;
static const char *audio_ids_array[16] = {"one.wav", "two.wav", "three.wav", "four.wav", "five.wav", "six.wav", "seven.wav", "eight.wav", "nine.wav", "ten.wav", "eleven.wav", "twelve.wav", "thirteen.wav", "fourteen.wav", "fifteen.wav", "sixteen.wav"};
static const char *user_ids_array[16] = {"USER ONE", "USER TWO", "USER THREE", "USER FOUR", "USER FIVE", "USER SIX", "USER SEVEN", "USER EIGHT", "USER NINE", "USER TEN", "USER ELEVEN", "USER TWELVE", "USER THIRTEEN", "USER FOURTEEN", "USER FIFTEEN", "USER SIXTEEN"};

// const unsigned int *audio_ids_array[16] = {AudioAir, AudioLook, AudioAir, AudioLook, AudioAir, AudioLook, AudioAir, AudioLook, AudioAir, AudioLook, AudioAir, AudioLook, AudioAir, AudioLook, AudioAir, AudioLook};

// Message array, pointers to each message and all audio messages

static const char *message_array[6] = {"SOS", "AIR", "LOOK AT ME", "MARINE LIFE", "ASCEND", "SOMETHING'S OFF"};
const char *audio_messages_array[6] = {"sos.wav", "air.wav", "lookatme.wav", "marinelife.wav", "ascend.wav", "somethingoff.wav"};

// Battery percentage pointers
const char *battery_messages_array[11] = {"0_PC.wav", "10_PC.wav", "20_PC.wav", "30_PC.wav", "40_PC.wav", "50_PC.wav", "60_PC.wav", "70_PC.wav", "80_PC.wav", "90_PC.wav", "100_PC.wav"};


/************* AUDIO SHIELD / PIPELINE SETUP */
const int micInput = AUDIO_INPUT_MIC;
const int chipSelect = 10; 

// SELECT SAMPLE RATE
const uint32_t sampleRate = 44100;
// const uint32_t sampleRate = 96000;
// const uint32_t sampleRate = 192000;
//const uint32_t sampleRate = 234000;

/************** AUDIO OUTPUT CHAIN (BONE CONDUCTION OUT) */
AudioPlaySdWav          playBoneconduct;       //xy=87,384
AudioAmplifier           outputAmp;           //xy=309,351
AudioOutputI2S           audioOutput;           //xy=573,377
AudioConnection          patchCord1(playBoneconduct, outputAmp);
AudioConnection          patchCord2(outputAmp, 0, audioOutput, 0);

/*************** AUDIO INPUT CHAIN (HYDROPHONE IN) */
const int myInput = AUDIO_INPUT_MIC;
AudioControlSGTL5000    audioShield;
AudioInputI2S            audioInput;           //xy=180,111
AudioAmplifier           inputAmp;           //xy=470,93
AudioAnalyzeFFT1024      inputFFT;      //xy=616,102
AudioConnection          patchCord3(audioInput, 0, inputAmp, 0);
AudioConnection          patchCord4(inputAmp, 0, inputFFT, 0);

/************** SAMPLE BUFFER LOGIC / RECEIVING STATE MACHINE
Each bin is numbered 0-1023 and has a float with its amplitude
SamplingBuffer is a time-valued array that records bin number
*/
#define MESSAGE_BIT_DELAY 250 // ms between bits
#define NUM_SAMPLES ((int)(((MESSAGE_BIT_DELAY / 10.0) * (86.0 / 100.0)) + 1.0 + 0.9999))
int16_t samplingBuffer[NUM_SAMPLES]; // BIN indices
uint16_t samplingPointer = 0; //How many samples have we seen?
#define MESSAGE_LENGTH UnderwaterMessage::size
bool bitBuffer[MESSAGE_LENGTH]; // Message sample buffer (1 or 0)
int bitPointer = 0;
#define FFT_BIN_WIDTH 43.0664
#define MESSAGE_START_FREQ 17500 // Hz (1/sec)
#define MESSAGE_0_FREQ 15000 // Hz
#define MESSAGE_1_FREQ 12500 // Hz
#define MESSAGE_LOWPASS_CUTOFF_FREQ 10000
#define FFT_BIN_CUTOFF (int)(MESSAGE_LOWPASS_CUTOFF_FREQ/FFT_BIN_WIDTH) //Lowpass cutoff
#define MIN_VALID_AMP 0.05
#define MAX_VALID_AMP 1.5
#define BOUNDS_FREQ 1500
typedef enum {
  LISTENING, //Waiting for start frequency
  CHECK_START,
  MESSAGE_ACTIVE //Currently receiving bytes
} RECEIVING_STATE;

RECEIVING_STATE curReceivingState = LISTENING;
bool sampling = 0; // Are we currently sampling?

/*************** TRANSMITTING LOGIC (QUEUE) */
const byte toneBufferLength = 10*MESSAGE_LENGTH; //Arbitrarily can hold 10 messages at a time
int toneFreqQueue[toneBufferLength];
unsigned long toneDelayQueue[toneBufferLength];
unsigned long lastToneStart = 0;
byte toneStackPos = 0;
unsigned long lastBitChange = 0;

/*************** AUDIO LOGIC (QUEUE) */
// const byte audioBufferLength = 10*MESSAGE_LENGTH; //Arbitrarily can hold 10 messages at a time
// int audioFnameQueue[toneBufferLength];
// unsigned long au[toneBufferLength];
// unsigned long lastToneStart = 0;
// byte toneStackPos = 0;
// unsigned long lastBitChange = 0;


typedef enum {
  RECEIVE,
  TRANSMIT,
  ERROR
} OPERATING_MODE;

OPERATING_MODE mode = RECEIVE;

void setup() {
    Serial.begin(115200);

    /******** INITIALIZATION */
    /*
    Steps:
    1) Initialize:
        - Blink blue to show that we are alive!
        - LED strip, battery monitor, audio shield, I/O expander
        - Blink green to show initialization good!
        - If anything fails: display full red on LED strip
    */

    // Setup pin modes
    pinMode(LED_PIN, OUTPUT);
    pinMode(HYDROPHONE_PIN, OUTPUT);
    pinMode(RELAY_PIN, OUTPUT);

    // Initial pin setup
    digitalWrite(RELAY_PIN, LOW);
   
    // Get LEDs up and running
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
    strip.setBrightness(LED_BRIGHTNESS);

    // LEDs: show that we are alive
    strip.fill(purple, 0, 20); // light up entire strip
    strip.show();
    // keep strip on, then continue with rest of installation
    delay(500); 
    strip.clear();

    // Initialize SD card
    if (!SD.begin(10)) {
      Serial.println("[ERROR] SD card initialization failed!");
      initializationError(1);
    }
    Serial.println("[OK] SD card initialized.");
    for (int i=0; i<6; i++) {
      if (!SD.exists(audio_messages_array[i])) {
        initializationError(2);
      }
    }
    Serial.println("[OK] Found all audio messages for buttons.");
    for (int i=0; i<16; i++) {
      if (!SD.exists(audio_ids_array[i])) {

        initializationError(2);
      }
    }
    // Check for ID.txt and read the user ID
    if (SD.exists("ID.txt")) {
      File idFile = SD.open("ID.txt", FILE_READ);
      if (idFile) {
        String idContent = idFile.readStringUntil('\n'); // Read the number from the file
        idContent.trim(); // Remove any whitespace or newlines
        user_ID = idContent.toInt(); // Convert to an integer
        if (user_ID == 1) {
          other_user_ID = 2;
        } else {
          other_user_ID = 1;
        }
        Serial.print("[OK] User ID set to: ");
        Serial.println(user_ID);
        idFile.close();
      } else {
        Serial.println("[ERROR] Failed to open ID.txt!");
        initializationError(3);
      }
    } else {
      Serial.println("[ERROR] ID.txt file not found!");
      initializationError(3);
    }
    initializationPass(2);
    Serial.println("[OK] Found all audio messages for user IDs.");

    // Assign underwater messages
    for (int i=1; i<=sizeof(UM_array)/sizeof(UM_array[0]); i++) {
      UM_array[i] = createUnderwaterMessage(i, user_ID);
    }
    Serial.println("[OK] Underwater messages initialized successfully");

    // Get IO expander up and running
    if (!mcp.begin_I2C(0x27)) {
        Serial.println("[Error] I2C connection with IO expander.");
        initializationError(3);
    }
    //Pinmode for I/O expander pins
    for (int i=0; i<6; i++) {
      mcp.pinMode(i, INPUT_PULLUP);
    }
    initializationPass(3);

    Serial.println("[OK] I2C expander initialized successfully, pin directions set");
    
    if (!maxlipo.begin()) {
       Serial.println("[Error] Battery monitor");
       initializationError(4);
    }
    Serial.print(F("[OK] Battery monitor MAX17048 initialized"));
    Serial.print(F(" with Chip ID: 0x")); 
    Serial.println(maxlipo.getChipID(), HEX);
    maxlipo.setAlertVoltages(2.0, 4.2);
    initializationPass(4);

    // Get BC transducer amp up and running 
    if (!audioamp.begin()) {
      Serial.println("[Error] TPA2016 audio amp failed to initialize");
      initializationError(5);
    }
    audioamp.setGain(30);
    Serial.println("[OK] TPA2016 initialized successfully, gain set");

    // Get audio shield up and running
    AudioMemory(500);
    inputAmp.gain(2);        // amplify mic to useful range
    outputAmp.gain(2);
    audioShield.enable();
    audioShield.inputSelect(myInput);
    audioShield.micGain(90);
    audioShield.volume(1);
    // setAudioSampleI2SFreq(sampleRate); // Set I2S sampling frequency
    Serial.printf("[OK] SGTL5000 initialized and running at samplerate: %d\n", sampleRate);
    initializationPass(6);

    // Setup receiver state machine, and transition states
    transitionReceivingState(LISTENING);
    transitionOperatingMode(RECEIVE);
    initializationPass(7);
    Serial.println("[OK] State machine initialized");

    Serial.println("Done initializing!! Starting now! In receiving default mode!");
    Serial.println("Welcome to NEPTUNE >:)");

    int dly = 50; // Initial delay
    float speedFactor = 0.95; // Factor to reduce the delay for smooth speeding up

    for (int cycle = 0; cycle < 6; cycle++) { // Number of cycles
      for (int i = 0; i < strip.numPixels(); i++) { // Loop through each pixel
        strip.clear(); // Clear all pixels
        
        // Illuminate the current pixel and the next two pixels
        strip.setPixelColor(i, strip.Color(0, 255, 0));           // Current pixel
        if (i + 1 < strip.numPixels()) strip.setPixelColor(i + 1, strip.Color(0, 255, 0)); // Next pixel
        if (i + 2 < strip.numPixels()) strip.setPixelColor(i + 2, strip.Color(0, 255, 0)); // Next-next pixel

        strip.show(); // Update the strip
        delay(dly);   // Pause for the specified time

        if (i % 6 == 0) {
          // Gradually decrease delay to speed up
          dly = max(5, (int)(dly * speedFactor)); // Ensure delay doesn't drop below 5 ms
        }
      }
    }
    strip.clear();
    strip.show();

    // Play sound effect
    playBoneconduct.play("WELCOME.wav");
    while (playBoneconduct.isPlaying());
    playBoneconduct.play("user.wav");
    while (playBoneconduct.isPlaying());
    playBoneconduct.play(audio_ids_array[user_ID - 1]);
    while (playBoneconduct.isPlaying());
}

unsigned long lastPlayedBatteryTime = 0;

void loop() {
  if (toneStackPos == 0) {
    strip.clear(); // Set all pixel colors to 'off' if queue is empty
    transitionOperatingMode(RECEIVE); // Switch relays to receive mode
  }

   if (millis() - lastBattCheckTime >= 1000) { // Check every 1 second
    lastBattCheckTime = millis();

    float cellVoltage = maxlipo.cellVoltage(); // Get cell voltage
    if (isnan(cellVoltage)) {
      Serial.println("Failed to read cell voltage, check battery is connected!");
    } else {
      float cellPercent = maxlipo.cellPercent(); // Get battery percentage
      Serial.print("Battery V:");
      Serial.println(cellPercent);

      if (pctBufPointer == -1) { //First sample
        for (int i=0; i<BATTERY_SAMPLES; i++) {
          pctBuf[i] = maxlipo.cellVoltage();
        }
        pctBufPointer = 1; //Increment pointer
      } else {
        pctBuf[pctBufPointer] = maxlipo.cellVoltage();
      }
      pctBufPointer++;
      if (pctBufPointer >= BATTERY_SAMPLES) {
        pctBufPointer = 0;
      }

      float totPctBuf = 0.0;
      for (int i = 0; i < BATTERY_SAMPLES; i++) {
        totPctBuf += pctBuf[i];
      }

      float avgPctBuf = totPctBuf / 5.0;
      float bufDifference = maxlipo.cellVoltage() - avgPctBuf;

      if (millis() - lastPlayedBatteryTime > 3100) {
        if (bufDifference > 0.05) {
          Serial.println("Neptune charging activated!");
          playBoneconduct.play("CHARGING_ACTIVATED.wav");
          while (playBoneconduct.isPlaying());
        } else if (bufDifference < -0.05) {
          Serial.println("Neptune charging deactivated!");
          playBoneconduct.play("CHARGING_DEACTIVATED.wav");
          while (playBoneconduct.isPlaying());
        }
        lastPlayedBatteryTime = millis();
      }

      // Check if the battery percentage has dropped by at least 10%
      if ((int(cellPercent) / 10) < (int(lastPercentage) / 10)) {
        int percentRange = (int(cellPercent) / 10) * 10;
        Serial.print("Battery dropped to ");
        Serial.print(percentRange);
        Serial.println("% range.");

        playBoneconduct.play("BATTERY.wav");
        while (playBoneconduct.isPlaying());
        playBoneconduct.play(battery_messages_array[(int)(percentRange/10)]);
        while (playBoneconduct.isPlaying());

        lastPercentage = cellPercent; // Update the last known percentage
      }
    }
  }

  // LED pulsating effect!
  if (millis() > lastLEDUpdateTime) {
    uint32_t color = strip.Color(0, counter, 0, 0); // r g b w
    if (dir) {
      counter++;
    } else {
      counter--;
    }
    if (counter <= 0 || counter >= 255) dir = !dir;
    strip.fill(color);
    strip.show();
    lastLEDUpdateTime = millis() + 1;
  }
  
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
          tone(HYDROPHONE_PIN, toneFreqQueue[0]); //start new tone
          }
          lastToneStart = millis();
      } else {
          noTone(HYDROPHONE_PIN); //otherwise just stop playing
      }
    }
  }

  // FFT has new data! Reads in data 
  if (inputFFT.available()) {
    // each time new FFT data is available
    float maxBinAmp = 0;
    int binNumber = 0;
    for (int i = 0; i < 1024; i++) {
      float n = inputFFT.read(i);
      if (n > maxBinAmp && i >= FFT_BIN_CUTOFF) { // Ensure we only read above the lowpass cutoff
        maxBinAmp = n;
        binNumber = i;
      }
    }
    Serial.print((double)binNumber*(double)FFT_BIN_WIDTH);
    Serial.print("Hz@");
    Serial.println(maxBinAmp);
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
    if (curReceivingState == LISTENING) {
      if (validAmplitude(maxBinAmp) && freqMatchesBounds((double)binNumber * (double)FFT_BIN_WIDTH, BOUNDS_FREQ, MESSAGE_START_FREQ) && toneStackPos == 0) {
        transitionReceivingState(CHECK_START); // Check start is actively checking if we've gotten start frequencies before recording the message 
      }
    } else if (curReceivingState == CHECK_START && millis() - lastBitChange >= MESSAGE_BIT_DELAY) { // Gotten all start samples
      if (isSampleBufferValid() && freqMatchesBounds(sampleBufferMax(), BOUNDS_FREQ, MESSAGE_START_FREQ) && toneStackPos == 0) {
        Serial.println("SAW VALID MESSAGE START FREQ!");
        transitionReceivingState(MESSAGE_ACTIVE); //Currently receiving valid message
      } else { // If buffer is not valid OR freq doesn’t match start
        transitionReceivingState(LISTENING);
      }
    } else if (curReceivingState == MESSAGE_ACTIVE && millis() - lastBitChange >= MESSAGE_BIT_DELAY) {
      if (isSampleBufferValid()) { // is our sample buffer valid?
        // Check if 1 or 0 (or neither)
        double bufferAvgFreq = sampleBufferMax();
        if (freqMatchesBounds(bufferAvgFreq, BOUNDS_FREQ, MESSAGE_1_FREQ)) {
          bitBuffer[bitPointer] = 1; // WE GOT A 1
        } else if (freqMatchesBounds(bufferAvgFreq, BOUNDS_FREQ, MESSAGE_0_FREQ)) {
          bitBuffer[bitPointer] = 0; // WE GOT A 0
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
        UnderwaterMessage recvdMessage;
        recvdMessage.data = data;
        Serial.print("Got message raw!!! MSG = ");
        Serial.print(recvdMessage.msg);
        Serial.print(", ID = ");
        Serial.print(recvdMessage.id);
        Serial.print(" --- ");
        for (int i = UnderwaterMessage::size - 1; i >= 0; i--) {
          // Shift and mask to get each bit
          Serial.print((recvdMessage.data >> i) & 1);
        }

        if (validUnderwaterMessage(recvdMessage)) {
          txrxAnimate(0); //ANIMATE LEDs in receive direction
          lastLEDUpdateTime = millis() + 1000;
          // Play audio corresponding to usert
          Serial.print(" --- USER: ");
          Serial.print(user_ids_array[recvdMessage.id]);

          playBoneconduct.play("user.wav");
          while (playBoneconduct.isPlaying());
          //UNCOMMENT TO USE PACKET ID
          // playBoneconduct.play(audio_ids_array[recvdMessage.id]);
          // HARDCODED ID
          playBoneconduct.play(audio_ids_array[other_user_ID]);
          while (playBoneconduct.isPlaying());
          playBoneconduct.play("said.wav");
          while (playBoneconduct.isPlaying());

          for (int c = 0; c < 6; c++) {
            if (recvdMessage.msg == UM_array[c].msg) {
                Serial.print(" --- MESSAGE:");
                Serial.println(message_array[c]);
                playBoneconduct.play(audio_messages_array[c]);
            }
          }
        }

        transitionReceivingState(LISTENING); // Return to listening state
      }
    }
  }

  // BUTTONS AND TRANSMITTING
  if (millis() - lastButtonCheckTime >= 500) {
    for (int b = 0; b < 6; b++) {
      // Serial.print(mcp.digitalRead(buttons[b]));
      if (mcp.digitalRead(b) == LOW) {
        lastButtonCheckTime = millis(); //Ensure 250ms between reads
        Serial.print("Sending message ID: ");
        Serial.println(b);
        strip.clear();

        txrxAnimate(1); //ANIMATE LEDs in transmit direction
        lastLEDUpdateTime = millis() + 1000;

        // BONE CONDUCTION CONFIRMATION
        playBoneconduct.play("you.wav");
        while (playBoneconduct.isPlaying());
        playBoneconduct.play("said.wav");
        while (playBoneconduct.isPlaying());
        playBoneconduct.play(audio_messages_array[b]); 

        transmitMessageAsync(UM_array[b]); // Add to queue!

        // Serial.println("Queued message! message in binary: ");
        for (int i = UnderwaterMessage::size - 1; i >= 0; i--) {
            // Shift and mask to get each bit
            Serial.print((UM_array[b].data >> i) & 1);
        }
        Serial.println(); // New line after printing bits
      }
    }
  }
}

void txrxAnimate(bool dir) {
  int leftArr[9] = {1, 2, 3, 4, 5, 19, 18, 17, 16};
  int rightArr[9] = {1, 0, 11, 10, 9, 12, 13, 14, 15};

  strip.clear();
  strip.show();

  if (dir) { // Forward direction
    for (int i = 0; i < 9; i++) {
      strip.setPixelColor(leftArr[i], cyan);
      strip.setPixelColor(rightArr[i], cyan);
      strip.show();
      delay(75); // Adjust delay for animation speed
    }
  } else { // Reverse direction
    for (int i = 8; i >= 0; i--) {
      strip.setPixelColor(leftArr[i], cyan);
      strip.setPixelColor(rightArr[i], cyan);
      strip.show();
      delay(75); // Adjust delay for animation speed
    }
  }
}

float fAbs(float n) {
  if (n < 0) return -n;
  return n;
}

void initializationPass(int check) {
    strip.fill(greenishwhite, 0, check); // check = number of tiles lit up
    strip.show();
    delay(100);
    strip.clear();
}

void initializationError(int error) {
    strip.fill(red, 0, error); // error = number of tiles lit up
    strip.show();
    delay(2000);
    strip.clear();

    while(true);
}

void transitionOperatingMode(OPERATING_MODE newMode) {
    if (newMode == RECEIVE) {
        digitalWrite(RELAY_PIN, LOW); // make relays go to listen mode
    } else if (newMode == TRANSMIT) {
        digitalWrite(RELAY_PIN, HIGH); // make relays go into transmit mode
    }
    mode = newMode;
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
  return (double)binNumber * FFT_BIN_WIDTH; // Adjust factor if needed for your sample rate/FFT size
}

// Function to transmit the UnderwaterMessage asynchronously
void transmitMessageAsync(UnderwaterMessage message) {
  transitionOperatingMode(TRANSMIT);
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
  addToneQueue(MESSAGE_0_FREQ, (int)(MESSAGE_BIT_DELAY)); // End transmission with a stop tone
}

void addToneQueue(int freq, unsigned long delay) {
  if (toneStackPos < toneBufferLength && delay > 0) {
    toneFreqQueue[toneStackPos] = freq;
    toneDelayQueue[toneStackPos] = delay;
    toneStackPos++; //always increase stack pointer
    if (toneStackPos == 1) { //If it’s the first sound, start playing it
      tone(HYDROPHONE_PIN, toneFreqQueue[0]); //start new tone
      lastToneStart = millis();
    }
  }
}

bool validAmplitude(double amp) {
  return (amp >= MIN_VALID_AMP && amp <= MAX_VALID_AMP);
}

bool validUnderwaterMessage(UnderwaterMessage message) {
  uint8_t msg = message.msg;
  uint8_t id = message.id;

  return (msg >= 0 && msg < 6 && id >= 0 && id < 16);
}

// Example usage: freqMatchesBounds(1100, 200, 1000) -> TRUE
// Example usage: freqMatchesBounds(1101, 200, 1000) -> FALSE
bool freqMatchesBounds(double freq, double bounds, double target) {
  return abs(freq-(double)target) <= (bounds/2.0);
}

// Transition state function
void transitionReceivingState(RECEIVING_STATE newState) {
  // Serial.print("transitionReceivingState: ");
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
  curReceivingState = newState; // Set our current state to the new one
}

UnderwaterMessage createUnderwaterMessage(uint8_t m, uint8_t i) {
    UnderwaterMessage message;
    message.msg = m & 0x7;  // Mask to 3 bits
    message.id = i & 0xF;   // Mask to 4 bits
    return message;
}

// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for(int a=0; a<30; a++) {  // Repeat 30 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the strip (strip.numPixels() steps):
        int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show();                // Update strip with new contents
      delay(wait);                 // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this loop:
  for(long firstPixelHue = 0; firstPixelHue < 16384; firstPixelHue += 256) {
    // strip.rainbow() can take a single argument (first pixel hue) or
    // optionally a few extras: number of rainbow repetitions (default 1),
    // saturation and value (brightness) (both 0-255, similar to the
    // ColorHSV() function, default 255), and a true/false flag for whether
    // to apply gamma correction to provide 'truer' colors (default true).
    strip.rainbow(firstPixelHue);
    // Above line is equivalent to:
    // strip.rainbow(firstPixelHue, 1, 255, 255, true);
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}

/**************** Support SGTL sampling frequency changing */
#if defined(__IMXRT1062__)  // Teensy 4.x

#include <utility/imxrt_hw.h>

// taken from: https://forum.pjrc.com/threads/57283-Change-sample-rate-for-Teensy-4-(vs-Teensy-3)?p=213007&viewfull=1#post213007
void setAudioSampleI2SFreq(int freq) {
  // PLL between 27*24 = 648MHz und 54*24=1296MHz
  int n1 = 4; //SAI prescaler 4 => (n1*n2) = multiple of 4
  int n2 = 1 + (24000000 * 27) / (freq * 256 * n1);
  double C = ((double)freq * 256 * n1 * n2) / 24000000;
  int c0 = C;
  int c2 = 10000;
  int c1 = C * c2 - (c0 * c2);
  set_audioClock(c0, c1, c2, true);
  CCM_CS1CDR = (CCM_CS1CDR & ~(CCM_CS1CDR_SAI1_CLK_PRED_MASK | CCM_CS1CDR_SAI1_CLK_PODF_MASK))
       | CCM_CS1CDR_SAI1_CLK_PRED(n1-1) // &0x07
       | CCM_CS1CDR_SAI1_CLK_PODF(n2-1); // &0x3f
//Serial.printf("setAudioSampleI2SFreq(%d)\n",freq);
}

#elif defined(KINETISK)  // Teensy 3.x

// samplerate code by Frank Boesing
// https://forum.pjrc.com/threads/38753-Discussion-about-a-simple-way-to-change-the-sample-rate
void setAudioSampleI2SFreq(int freq) {
  typedef struct {
    uint8_t mult;
    uint16_t div;
  } __attribute__((__packed__)) tmclk;
  const int numfreqs = 14;
  const int samplefreqs[numfreqs] = { 8000, 11025, 16000, 22050, 32000, 44100, static_cast<int>(44117.64706) , 48000, 88200, static_cast<int>(44117.64706 * 2), 96000, 176400, static_cast<int>(44117.64706 * 4), 192000};

// F_PLL = phase lock loop output frequency = teensy CPU clock speed in Hz

#if (F_PLL==16000000)
  const tmclk clkArr[numfreqs] = {{16, 125}, {148, 839}, {32, 125}, {145, 411}, {64, 125}, {151, 214}, {12, 17}, {96, 125}, {151, 107}, {24, 17}, {192, 125}, {127, 45}, {48, 17}, {255, 83} };
#elif (F_PLL==72000000)
  const tmclk clkArr[numfreqs] = {{32, 1125}, {49, 1250}, {64, 1125}, {49, 625}, {128, 1125}, {98, 625}, {8, 51}, {64, 375}, {196, 625}, {16, 51}, {128, 375}, {249, 397}, {32, 51}, {185, 271} };
#elif (F_PLL==96000000)
  const tmclk clkArr[numfreqs] = {{8, 375}, {73, 2483}, {16, 375}, {147, 2500}, {32, 375}, {147, 1250}, {2, 17}, {16, 125}, {147, 625}, {4, 17}, {32, 125}, {151, 321}, {8, 17}, {64, 125} };
#elif (F_PLL==120000000)
  const tmclk clkArr[numfreqs] = {{32, 1875}, {89, 3784}, {64, 1875}, {147, 3125}, {128, 1875}, {205, 2179}, {8, 85}, {64, 625}, {89, 473}, {16, 85}, {128, 625}, {178, 473}, {32, 85}, {145, 354} };
#elif (F_PLL==144000000)
  const tmclk clkArr[numfreqs] = {{16, 1125}, {49, 2500}, {32, 1125}, {49, 1250}, {64, 1125}, {49, 625}, {4, 51}, {32, 375}, {98, 625}, {8, 51}, {64, 375}, {196, 625}, {16, 51}, {128, 375} };
#elif (F_PLL==180000000)
  const tmclk clkArr[numfreqs] = {{46, 4043}, {49, 3125}, {73, 3208}, {98, 3125}, {183, 4021}, {196, 3125}, {16, 255}, {128, 1875}, {107, 853}, {32, 255}, {219, 1604}, {214, 853}, {64, 255}, {219, 802} };
#elif (F_PLL==192000000)
  const tmclk clkArr[numfreqs] = {{4, 375}, {37, 2517}, {8, 375}, {73, 2483}, {16, 375}, {147, 2500}, {1, 17}, {8, 125}, {147, 1250}, {2, 17}, {16, 125}, {147, 625}, {4, 17}, {32, 125} };
#elif (F_PLL==216000000)
  const tmclk clkArr[numfreqs] = {{32, 3375}, {49, 3750}, {64, 3375}, {49, 1875}, {128, 3375}, {98, 1875}, {8, 153}, {64, 1125}, {196, 1875}, {16, 153}, {128, 1125}, {226, 1081}, {32, 153}, {147, 646} };
#elif (F_PLL==240000000)
  const tmclk clkArr[numfreqs] = {{16, 1875}, {29, 2466}, {32, 1875}, {89, 3784}, {64, 1875}, {147, 3125}, {4, 85}, {32, 625}, {205, 2179}, {8, 85}, {64, 625}, {89, 473}, {16, 85}, {128, 625} };
#endif

  for (int f = 0; f < numfreqs; f++) {
    if ( freq == samplefreqs[f] ) {
      while (I2S0_MCR & I2S_MCR_DUF) ;
      I2S0_MDR = I2S_MDR_FRACT((clkArr[f].mult - 1)) | I2S_MDR_DIVIDE((clkArr[f].div - 1));
      return;
    }
  }
}

#endif
