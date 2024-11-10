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

/******* ADDITIONAL DEVICE SETUP */

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

uint32_t magenta = strip.Color(255, 0, 255, 0);
uint32_t greenishwhite = strip.Color(0, 64, 0, 64); // r g b w
uint32_t bluishwhite = strip.Color(64, 0, 0, 64);

Adafruit_MCP23X17 mcp;
Adafruit_LC709203F lc;

uint8_t row_pins[4] = {BUTTON_PIN2, BUTTON_PIN7, BUTTON_PIN6, BUTTON_PIN4};
uint8_t col_pins[3] = {BUTTON_PIN3, BUTTON_PIN1, BUTTON_PIN5};

char keypad_array[4][3] = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}, {10, 0, 11}};
static const char *message_array[4][3] = {{"SOS", "GOING UP", "GOING DOWN"}, {"LOW OXYGEN", "CHECK-IN", "COME LOOK"}, {"no msg", "no msg", "no msg"}, {"no msg", "no msg", "no msg"}};

enum OperatingMode { 
    RECEIVE,
    TRANSMIT,
    ERROR
};
enum OperatingMode mode;

void setup() {
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
    // change the delay to while?
    delay(3000); 
    strip.clear();

    // Get IO expander up and running
    if (!mcp.begin_I2C()) {
        Serial.println("Error: I2C connection with IO expander.");
        initializationError(1);
    }

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
    // initializationError(2);

    //Battery check setup
    if (!lc.begin()) {
    Serial.println(F("Couldnt find Adafruit LC709203F?\nMake sure a battery is plugged in!"));
    while (1) delay(10);
    }
    Serial.println(F("Found LC709203F"));
    Serial.print("Version: 0x"); Serial.println(lc.getICversion(), HEX);

    lc.setThermistorB(3950);
    Serial.print("Thermistor B = "); Serial.println(lc.getThermistorB());

    lc.setPackSize(LC709203F_APA_500MAH);

    lc.setAlarmVoltage(3.8);

    delay(1000);
    Serial.println("PRINTING BATTERY INFO");
    printBatteryData();
    delay(5000);
}

void loop() {
  strip.clear(); // Set all pixel colors to 'off'

  for (int r = 0; r < 4; r++) {
        mcp.digitalWrite(row_pins[r], LOW);
        delay(5);
        for (int c = 0; c < 3; c++) {
            if (mcp.digitalRead(col_pins[c]) == LOW) {
                mode = TRANSMIT;
                update_relays(mode);
                transmit();
                Serial.println(keypad_array[r][c]); // returns button pressed
                Serial.println(message_array[r][c]); // returns message pressed
                
                // transmitMessageQueue[transmitMessagePointer] = message_array[r][c];// add msg to queue  
                // transmitMessagePointer++; //increment ptr by one 

                // confirmation of message pressed back to bone conduction
                // playMem.play(audio_messages_array[r][c]); // plays message pressed

                strip.fill(magenta, 0, keypad_array[r][c]);
                strip.show();
            }
        }
        mcp.digitalWrite(row_pins[r], HIGH);
        delay(5);
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
void initializationError(int error) {
    // passed in int error represents number of LEDs in strip that we want to light up
    // for initialization, using greenishwhite (whereas showing life is bluishwhite)
    
    strip.fill(greenishwhite, 0, error); // error = number of tiles lit up
    strip.show();

    while(1); // how long to keep on for?
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
    delay(500);
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