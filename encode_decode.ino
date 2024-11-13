// FROM AARON ON SLACK

/* SPH0645 MEMS Microphone Test (Adafruit product #3421)
 *
 * Forum thread with connection details and other info:
 * https://forum.pjrc.com/threads/60599?p=238070&viewfull=1#post238070
 */
#include <Audio.h>
// const int myInput = AUDIO_INPUT_LINEIN;
const int myInput = AUDIO_INPUT_MIC;
// GUItool: begin automatically generated code
AudioControlSGTL5000 audioShield;
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
// Underwater message struct
union UnderwaterMessage {
    struct {
        uint8_t msg : 3; // 3 bits for message (values 0-7)
        uint8_t id : 4;  // 4 bits for id (values 0-15)
    };
    uint8_t data; // 7 bits total (3 bits msg + 4 bits id)
    // Size field indicating the total size of the message in bits
    static constexpr uint8_t size = 7; // 7 bits total
};
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
UnderwaterMessage testMessage1; // Message MSG = 2, ID = 1
// UnderwaterMessage testMessage2; // Message MSG = 2, ID = 1
void setup() {
  pinMode(14, OUTPUT);
  AudioMemory(50);
  Serial.begin(44120);
  // biquad1.setBandpass(0,20000,5); //Filter to only freq between 18000-22000
  // biquad1.setBandpass(0,10500, .95);
  amp1.gain(2);        // amplify sign to useful range
  audioShield.enable();
  audioShield.inputSelect(myInput);
  audioShield.micGain(90);
  audioShield.volume(1);
  Serial.println("Setup done; adding message to queue");
  // Populate message
  // Message MSG = 2, ID = 1
  testMessage1 = createUnderwaterMessage(2, 1);
  // testMessage2 = createUnderwaterMessage(2, 4);
  transmitMessageAsync(testMessage1);
  // transmitMessageAsync(testMessage2);
  Serial.print("Message1 in binary: ");
  for (int i = UnderwaterMessage::size - 1; i >= 0; i--) {
      // Shift and mask to get each bit
      Serial.print((testMessage1.data >> i) & 1);
  }
  Serial.println(); // New line after printing bits
  transitionState(LISTENING);
}
void loop() {
  if (toneStackPos == 0) {
    int curId = testMessage1.id;
    curId++;
    if (curId > 15) {
      curId = 0;
    }
    testMessage1 = createUnderwaterMessage(2, curId);
    //Requeue message
    transmitMessageAsync(testMessage1);
    // transmitMessageAsync(testMessage2);
    Serial.println("Requeueing message; txbuf empty");
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
          tone(BUZZER_PIN, toneFreqQueue[0]); //start new tone
        }
        lastToneStart = millis();
      } else {
        noTone(BUZZER_PIN); //otherwise just stop playing
      }
    }
  }
  // FFT has new data!
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
    if (sampling) {
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
        transitionState(CHECK_START);
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
        transitionState(LISTENING); // Return to listening state
      }
    }
  }
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
// Helper function to create an UnderwaterMessage
UnderwaterMessage createUnderwaterMessage(uint8_t m, uint8_t i) {
    UnderwaterMessage message;
    message.msg = m & 0x7;  // Mask to 3 bits
    message.id = i & 0xF;   // Mask to 4 bits
    return message;
}