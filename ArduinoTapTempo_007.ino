#include <ArduinoTapTempo.h>

#include <TimerOne.h>

//#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>
#include <EEPROM.h>

/*
 * For all features: if you do not define the pin, the feature will be disabled!
 */

/*
 * FEATURE: TAP BPM INPUT
 */
#define TAP_PIN 4
#define TAP_PIN_POLARITY FALLING

#define MINIMUM_TAPS 3
#define EXIT_MARGIN 100 // If no tap after 150% of last tap interval -> measure and set



/*
 * FEATURE: BLINK TEMPO LED
 */
#define BLINK_OUTPUT_PIN 10
#define BLINK_PIN_POLARITY 0  // 0 = POSITIVE, 255 - NEGATIVE
#define BLINK_TIME 4 // How long to keep LED lit in CLOCK counts (so range is [0,24])

/*
 * FEATURE: SYNC PULSE OUTPUT
 */

/*
 * FEATURE: Send MIDI start/stop
 */
#define START_STOP_INPUT_PIN 5
#define START_STOP_PIN_POLARITY 0 // 0 = POSITIVE, 1024 = NEGATIVE
#define START_STOP_LED 11

#define NUDGE_MINUS 6
#define NUDGE_PLUS 7
#define ONBOARD_LED 13
boolean bLastNudgePlus = false;
boolean bLastNudgeMinus = false;
#define DECODER_CLICK 8
#define STOP_BUTTON 9

#define MIDI_START 0xFA
#define MIDI_STOP 0xFC

#define DEBOUNCE_INTERVAL 500L // Milliseconds

/*
 * FEATURE: EEPROM BPM storage
 */
#define EEPROM_ADDRESS 0 // Where to save BPM




/*
 * GENERAL PARAMETERS
 */
#define MIDI_TIMING_CLOCK 0xF8
#define CLOCKS_PER_BEAT 24
#define MINIMUM_BPM 400 // Used for debouncing
#define MAXIMUM_BPM 3000 // Used for debouncing

long intervalMicroSeconds;
int bpm;  // BPM in tenths of a BPM!!
int bpmCache; //Nudge

boolean initialized = false;
long minimumTapInterval = 60L * 1000 * 1000 * 10 / MAXIMUM_BPM;
long maximumTapInterval = 60L * 1000 * 1000 * 10 / MINIMUM_BPM;

volatile long firstTapTime = 0;
volatile long lastTapTime = 0;
volatile long timesTapped = 0;

volatile int blinkCount = 0;

int lastDimmerValue = 0;

boolean playing = false;
long lastStartStopTime = 0;


ArduinoTapTempo tapTempo;
Encoder myEnc(3, 2);
int encoder = 0;

void setup() {
  //  Set MIDI baud rate:
  Serial1.begin(31250 /* 9600 */);

  // Set pin modes
  pinMode(BLINK_OUTPUT_PIN, OUTPUT);
  pinMode(START_STOP_LED, OUTPUT);


  // Get the saved BPM value from 2 stored bytes: MSB LSB
  bpm = EEPROM.read(EEPROM_ADDRESS) << 8;
  bpm += EEPROM.read(EEPROM_ADDRESS + 1);
  if (bpm < MINIMUM_BPM || bpm > MAXIMUM_BPM) {
    bpm = 1200;
  }


  // Interrupt for catching tap events
  //attachInterrupt(digitalPinToInterrupt(TAP_PIN), tapInput, TAP_PIN_POLARITY);

  // Attach the interrupt to send the MIDI clock and start the timer
  Timer1.initialize(intervalMicroSeconds);
  Timer1.setPeriod(calculateIntervalMicroSecs(bpm));
  Timer1.attachInterrupt(sendClockPulse);


  pinMode(ONBOARD_LED, OUTPUT);
  pinMode(NUDGE_PLUS, INPUT);
  pinMode(NUDGE_MINUS, INPUT);
  pinMode(START_STOP_INPUT_PIN, INPUT);
  pinMode(TAP_PIN, INPUT);
  pinMode(STOP_BUTTON, INPUT);
  pinMode(DECODER_CLICK, INPUT);
  
  //----Pullup
  digitalWrite(NUDGE_PLUS, HIGH);
  digitalWrite(NUDGE_MINUS, HIGH);
  digitalWrite(START_STOP_INPUT_PIN, HIGH);
  digitalWrite(TAP_PIN, HIGH);
  digitalWrite(STOP_BUTTON, HIGH);
  digitalWrite(DECODER_CLICK, HIGH);

  encoder = myEnc.read();
}


void loop() {
  long now = millis();

/*
  boolean bMinus = digitalRead(NUDGE_MINUS);
  if(bMinus){
    digitalWrite(ONBOARD_LED, HIGH);
  }else{
    digitalWrite(ONBOARD_LED, LOW);
  }
*/

  /*
   * Handle tapping of the tap tempo button
   */
  float tmpBPM1 = tapTempo.getBPM();
  boolean buttonDown = !digitalRead(TAP_PIN);

  //digitalWrite(ONBOARD_LED, buttonDown );
  
  
  //----Neu: ANY incoming midi "note on" with a velocity>0 will trigger a tap
  if(Serial1.available()>=3){
    int note = Serial1.read();
    Serial1.read();
    int velocity = Serial1.read();
    if((note>=0x90) &&(note<0x9f)){
      if(velocity>0){
        tapTempo.update(true);
      }
    }
    
    while(Serial1.available()){
      Serial1.read();
    }
    tapTempo.update(false);
    //tapTempo.setBPM( tapTempo.getBPM()/2 );
  }
  
  tapTempo.update(buttonDown);
  float tmpBPM2 = tapTempo.getBPM();

  if(tmpBPM1 != tmpBPM2){  
    //Serial.print("BPM:"); Serial.println(tmpBPM2, DEC);
    bpm = (int)(tmpBPM2*10);
    bpmCache = bpm;
    updateBpm( now );
  }


  
  

  /*
   * Check for start/stop button pressed
   */  
  boolean startStopPressed = !digitalRead(START_STOP_INPUT_PIN);
    
  if (startStopPressed && (lastStartStopTime + (DEBOUNCE_INTERVAL /* 500*/)) < now) {
    //startOrStop();
    restartMidi();
    lastStartStopTime = now;
    
  }

  //----Check fpr Stop-Button
  boolean bStop = !digitalRead(STOP_BUTTON);
  if(bStop){
    stopMidi(); 
  }


  

  //----Check for Nudge
  boolean bNudgeMinus = !digitalRead(NUDGE_MINUS);
  if(bNudgeMinus){
    if(bLastNudgeMinus==false){
      bLastNudgeMinus=true;
      bpmCache = bpm;
      bpm = bpm*0.95;
      updateBpm( now );
    }
    
  }else{
    bLastNudgeMinus = false;
  }


  boolean bNudgePlus = !digitalRead(NUDGE_PLUS);
  if(bNudgePlus){
    if(bLastNudgePlus==false){
      bLastNudgePlus=true;
      bpmCache = bpm;
      bpm = bpm*1.025;
      updateBpm( now );
    }
  }else{
    bLastNudgePlus = false;
  }

  if((bNudgeMinus == false) && (bNudgePlus == false)){
    bpm = bpmCache;
    updateBpm( now );
  }


    

    //----Check rotary Encoder
    boolean bDecoderPressed = !digitalRead(DECODER_CLICK);
    
    int newEncoder =  myEnc.read();
    if(newEncoder > encoder){
      if(bDecoderPressed){
        bpm+=1;
      }else{
        bpm += 5;
      }
      
      bpmCache = bpm;
      updateBpm( now );
      encoder = newEncoder;
    }
    else if(newEncoder < encoder){
      if(bDecoderPressed){
        bpm-=1;
      }else{
        bpm -= 5;
      }
      
      bpmCache = bpm;
      updateBpm( now );
      encoder = newEncoder;
    }  

  
  
}

/*
void tapInput() {
  long now = millis();
  if (now - lastTapTime < minimumTapInterval) {
    return; // Debounce
  }

  if (timesTapped == 0) {
    firstTapTime = now;
  }

  timesTapped++;
  lastTapTime = now;
}
*/

void startOrStop() {
  //Serial.println("StartOrStop()");
  if (!playing) {
    Serial1.write(MIDI_START);
    //Serial.println("Start");
    digitalWrite(START_STOP_LED, LOW);
    blinkCount = 0;
  } else {
    Serial1.write(MIDI_STOP);
    //Serial.println("Stop");
    digitalWrite(START_STOP_LED, HIGH);
  }
  playing = !playing;
}

void stopMidi(){
  Serial1.write(MIDI_STOP);
  blinkCount = 0;
  playing = false;
}


void restartMidi(){
  //Serial.println("Restart()");
  Serial1.write(MIDI_STOP);
  Serial1.write(MIDI_START);
  blinkCount = 0;
  playing = true;
}

void sendClockPulse() {
  // Write the timing clock byte
  Serial1.write(MIDI_TIMING_CLOCK);

  blinkCount = (blinkCount + 1) % CLOCKS_PER_BEAT;
  if (blinkCount == 0) {
    // Turn led on
#ifdef BLINK_OUTPUT_PIN
    analogWrite(BLINK_OUTPUT_PIN, 255 - BLINK_PIN_POLARITY);
#endif

  } else {

#ifdef BLINK_OUTPUT_PIN
    if (blinkCount == BLINK_TIME) {
      // Turn led off
      analogWrite(BLINK_OUTPUT_PIN, 0 + BLINK_PIN_POLARITY);
    }
#endif
  }
}

void updateBpm(long now) {
  // Update the timer
  long interval = calculateIntervalMicroSecs(bpm);
  Timer1.setPeriod(interval);

#ifdef EEPROM_ADDRESS
  // Save the BPM in 2 bytes, MSB LSB
  EEPROM.write(EEPROM_ADDRESS, bpm / 256);
  EEPROM.write(EEPROM_ADDRESS + 1, bpm % 256);
#endif


}

long calculateIntervalMicroSecs(int bpm) {
  // Take care about overflows!
  return 60L * 1000 * 1000 * 10 / bpm / CLOCKS_PER_BEAT;
}
