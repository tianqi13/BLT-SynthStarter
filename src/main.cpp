#define _USE_MATH_DEFINES


#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <math.h>
#include <STM32FreeRTOS.h>
#include <knob.hpp>
#include <ES_CAN.h>
#include <atomic>


#include <array>
#include <cmath>


#define receiver
// #define sender

// #define DISABLE_THREADS
//#define TEST_SCANKEYS
//#define TEST_DECODETASK
// #define TEST_DISPLAY
// #define TEST_ISR
#define TEST_HANDSHAKE // uncommented defines have not been implemented
#define TEST_ENVELOPE
#define TEST_CAN_TX
#define TEST_CAN_RX

enum waveform {
 SAWTOOTH,
 SINE,
 TRIANGLE,
 SQUARE
};

volatile waveform CurrentWaveform = SINE;

#define TABLE_SIZE 256


// Generate a sine lookup table
int sineLUT[TABLE_SIZE];

void generateSineLUT() {
   int minVal = 127, maxVal = -128;
   for (int i = 0; i < TABLE_SIZE; i++) {
       sineLUT[i] = (int)(127 * sinf(2 * M_PI * i / TABLE_SIZE));
   }
}

struct ADSEnvelope { // milliseconds
  uint32_t attackTime = 50;    
  uint32_t decayTime = 100;
  float sustainLevel = 0.7f;   // 0.0 to 1.0
  
  float currentLevel = 0.0f;
  uint32_t startTime = 0;
  bool noteOn = false;
  bool inRelease = false;
};

volatile ADSEnvelope envelope;

void startEnvelope() {
  envelope.startTime = millis();
  envelope.noteOn = true;
  envelope.inRelease = false;
}

float calculateEnvelopeLevel() {
  uint32_t currentTime = millis();
  uint32_t elapsedTime;
  float level = 0.0f;
  
  if (envelope.noteOn) {
    elapsedTime = currentTime - envelope.startTime;
    
    // Attack phase
    if (elapsedTime < envelope.attackTime) {
      level = (float)elapsedTime / envelope.attackTime;
    }
    // Decay phase
    else if (elapsedTime < (envelope.attackTime + envelope.decayTime)) {
      uint32_t decayElapsed = elapsedTime - envelope.attackTime;
      level = 1.0f - ((1.0f - envelope.sustainLevel) * ((float)decayElapsed / envelope.decayTime));
    }
    // Sustain phase
    else {
      level = envelope.sustainLevel;
    }
  }
  
  envelope.currentLevel = level;
  return level;
}

const char* getWaveformName(waveform w) {
  switch (w) {
      case SAWTOOTH: return "SAW";
      case SINE: return "SINE";
      case TRIANGLE: return "TRI";
      case SQUARE: return "SQR";
  }
}

//Constants
 const uint32_t interval = 100; //Display update interval


//Pin definitions
 //Row select and enable
 const int RA0_PIN = D3;
 const int RA1_PIN = D6;
 const int RA2_PIN = D12;
 const int REN_PIN = A5;


 //Matrix input and output
 const int C0_PIN = A2;
 const int C1_PIN = D9;
 const int C2_PIN = A6;
 const int C3_PIN = D1;
 const int OUT_PIN = D11;


 //Audio analogue out
 const int OUTL_PIN = A4;
 const int OUTR_PIN = A3;


 //Joystick analogue in
 const int JOYY_PIN = A0;
 const int JOYX_PIN = A1;


 //Output multiplexer bits
 const int DEN_BIT = 3;
 const int DRST_BIT = 4;
 const int HKOW_BIT = 5;
 const int HKOE_BIT = 6;


//Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);


//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
     digitalWrite(REN_PIN,LOW);
     digitalWrite(RA0_PIN, bitIdx & 0x01);
     digitalWrite(RA1_PIN, bitIdx & 0x02);
     digitalWrite(RA2_PIN, bitIdx & 0x04);
     digitalWrite(OUT_PIN,value);
     digitalWrite(REN_PIN,HIGH);
     delayMicroseconds(2);
     digitalWrite(REN_PIN,LOW);
}


std::bitset<32> readCols() {
 std::bitset<32> result;
  // Set bits 0 to 3 from the digital inputs
 result[0] = digitalRead(C0_PIN);
 result[1] = digitalRead(C1_PIN);
 result[2] = digitalRead(C2_PIN);
 result[3] = digitalRead(C3_PIN);
  return result;
}


void setROW(uint8_t rowIdx){
 digitalWrite(REN_PIN, LOW);
 digitalWrite(RA0_PIN, rowIdx & 0x01);
 digitalWrite(RA1_PIN, rowIdx & 0x02);
 digitalWrite(RA2_PIN, rowIdx & 0x04);
 digitalWrite(REN_PIN, HIGH);
}


constexpr std::array<uint32_t, 13> getArray() {
 double freq_factor = pow(2, 1.0/12.0);
 std::array<uint32_t, 13> result = {0};
 double freq = 0.0;


 for (size_t i = 0; i < 12; i++) {
   if (i >= 9) {
       freq = 440 * pow(freq_factor, (i - 9));
   } else {
       freq = 440 / pow(freq_factor, (9 - i));
   }


   result[i] = (pow(2, 32) * freq) / 22000;


 }


 //To handle case where no notes are pressed
 result[12] = 0x0;
 return result;
}


std::array<uint32_t, 13> StepSizes = getArray();


//Timer
HardwareTimer sampleTimer(TIM1);


//Global variables
systemState sysState;
volatile Knob knob3 = Knob(3, 3, 0, 8, 0); // volume
volatile Knob knob2 = Knob(2, 3, 2, 10, 0); // change to sustain - figure out waveforms in a bit
volatile Knob knob1 = Knob(1, 4, 0, 500, 0); // decay
volatile Knob knob0 = Knob(0, 4, 2, 500, 0); // attack

volatile Knob knobWave = Knob(4, 5, 1, 3, 0); // waveform - when knob3 is pressed
QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;
SemaphoreHandle_t CAN_TX_Semaphore;
volatile uint32_t Octave;
volatile uint32_t currentStepSizes[10] = {0};
std::array<std::string, 12> pianoNotes = {
  "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
};


volatile bool outBits[7] = {0,0,0,1,1,1,1};
std::atomic<bool> handshakeComplete{false};
struct handshakeState{
 std::unordered_map<uint32_t, int> moduleMap;
 SemaphoreHandle_t mutex;
};
handshakeState hsState;




void sampleISR() {
   // static local variable is not re-initialized on each call
   static uint32_t phaseAcc[10] = {0};  // Phase accumulators for each channel
   uint32_t activeStepSizes[10] = {0};  // Stores the currently active step sizes
   int activeCount = 0;  // Number of currently active keys

  int32_t localVolume;
  localVolume = __atomic_load_n(&sysState.rotation[3], __ATOMIC_RELAXED);

  uint32_t localCurrentWaveform;
  localCurrentWaveform = __atomic_load_n(&CurrentWaveform, __ATOMIC_RELAXED);

   // Retrieve up to 4 active key step sizes
   for (int i = 0; i < 10 && activeCount < 10; i++) {
       uint32_t stepSize = __atomic_load_n(&currentStepSizes[i], __ATOMIC_RELAXED);
       if (stepSize > 0) {
           activeStepSizes[activeCount] = stepSize;
           activeCount++;
          
           // Serial.print("Active key");
           // Serial.println(stepSize);
       }
   }

   // Generate the synthesized waveform
   int32_t Vout = 0;
   uint8_t index = 0;
   for (int i = 0; i < activeCount; i++) {
       phaseAcc[i] += activeStepSizes[i];
       index = (phaseAcc[i] >> 24);

       if(localCurrentWaveform==SINE){
           Vout += sineLUT[index];
       } else if (localCurrentWaveform==SAWTOOTH){
           Vout += index - 128;
       } else if (localCurrentWaveform==TRIANGLE){
           if (index < 128) {
           Vout += 2 * index - 128; // Rising part
           } else {
           Vout += 2 * (255 - index) - 128; // Falling part
           }
       } else if (localCurrentWaveform==SQUARE){
           Vout += (index < 128) ? -128 : 127;
       }
   }

   // Normalize volume (prevent overflow)
   if (activeCount > 0) {
       Vout = Vout / activeCount;
   } else {
       Vout = 0;  // Mute
   }
      
  float envelopeLevel = calculateEnvelopeLevel();
  Vout = Vout * envelopeLevel;

   // volume
   Vout = Vout >> (8 - localVolume);
  
   analogWrite(OUTR_PIN, Vout + 128);

}

void CAN_RX_ISR (void) {
 uint8_t RX_Message_ISR[8];
 uint32_t ID;
 CAN_RX(ID, RX_Message_ISR);
 xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}


void CAN_TX_ISR (void) {
 xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}


void scanKeysTask(void * pvParameters) {
 // Serial.println("Scan Keys Task");
 std::bitset<32> colState;
 std::bitset<32> localInputs = std::bitset<32>(0xFFFFFFFF);
 std::bitset<32> previousLocalInputs = std::bitset<32>(0xFFFFFFFF);
 uint8_t TX_Message[8] = {0};
 bool outBit;
 int32_t waveformIndex;


 #ifdef TEST_SCANKEYS
 previousStepIndex = 11;
 previousAction = 0x50;
 #endif


 TX_Message[0] = 0x50;
 TX_Message[1] = 4;
 TX_Message[2] = 12; //initalise to be 12


 #ifndef TEST_SCANKEYS
 //xFrequency is the initiation interval of the task
 const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
 // xLastWakeTime stores the tick count of the last initiation
 TickType_t xLastWakeTime = xTaskGetTickCount();
 #endif


 while (1){
   #ifndef TEST_SCANKEYS
   vTaskDelayUntil(&xLastWakeTime, xFrequency);
   #endif

   localInputs.reset();
  
   // Loop through rows of key matrix and read columns
   for(int i = 0; i<7; i++){
     setROW(i);
     outBit = __atomic_load_n(&outBits[i], __ATOMIC_RELAXED);
     digitalWrite(OUT_PIN, outBit);
     digitalWrite(REN_PIN, 1);
     delayMicroseconds(3);
     colState = readCols();
     localInputs |= (colState << (i * 4));
     digitalWrite(REN_PIN,0);
   }


   if(localInputs != previousLocalInputs){
       //take mutex to update inputs
       xSemaphoreTake(sysState.mutex, portMAX_DELAY);
       sysState.inputs = localInputs;
       xSemaphoreGive(sysState.mutex);
       
      knob3.updateRotation(localInputs);
      knob2.updateRotation(localInputs);
      knob1.updateRotation(localInputs);
      knob0.updateRotation(localInputs);
      knobWave.updateWave(localInputs);
      
      envelope.attackTime = knob0.getRotation();
      envelope.decayTime = knob1.getRotation();
      envelope.sustainLevel = knob2.getRotation() / 10.0f;

      waveformIndex = knobWave.getRotation();
      switch(waveformIndex) {
      case 0: __atomic_store_n(&CurrentWaveform, SAWTOOTH, __ATOMIC_RELAXED); break;
      case 1: __atomic_store_n(&CurrentWaveform, SINE, __ATOMIC_RELAXED); break;
      case 2: __atomic_store_n(&CurrentWaveform, TRIANGLE, __ATOMIC_RELAXED); break;
      case 3: __atomic_store_n(&CurrentWaveform, SQUARE, __ATOMIC_RELAXED); break;
      }
       
       knobWave.updateWave(localInputs);


       for (int i = 0; i < 12; i++){
           //Case 1: a key has been pressed
           if((localInputs[i] == 0) && (previousLocalInputs[i] == 1)){
               TX_Message[0] = 0x50; //'P' Pressed current key
               TX_Message[1] = __atomic_load_n(&Octave, __ATOMIC_RELAXED);
               TX_Message[2] = i;


               #ifdef receiver
               xQueueSend(msgInQ, TX_Message, portMAX_DELAY);
               #endif

               #ifdef sender
               xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
               #endif
           }


           else if ((localInputs[i] == 1) && (previousLocalInputs[i] == 0)){
               TX_Message[0] = 0x52; //'R' Released current key
               TX_Message[1] = __atomic_load_n(&Octave, __ATOMIC_RELAXED);
               TX_Message[2] = i;


               #ifdef receiver
               xQueueSend(msgInQ, TX_Message, portMAX_DELAY);
               #endif


               #ifdef sender
               xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
               #endif
           }
       }


       previousLocalInputs = localInputs;
   }
    // UBaseType_t stackRemaining = uxTaskGetStackHighWaterMark(NULL); 
   // Serial.print("scanKeysTask stack remaining: ");
   // Serial.println(stackRemaining);
  
   #ifdef TEST_SCANKEYS
   break;
   #endif
 }
}


void displayUpdateTask(void * pvParameters) {
  int32_t localRotation[4];
  int8_t localRX_Message[8];
  waveform localCurrentWaveform;
  std::string pressedNote;
  //xFrequency is the initiation interval of the task 
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  // xLastWakeTime stores the tick count of the last initiation
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1){
    #ifndef TEST_DISPLAY
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    #endif

    //Update display
    u8g2.clearBuffer();         // clear the internal memory

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    localRX_Message[0] = sysState.RX_Message[0];
    localRX_Message[1] = sysState.RX_Message[1];
    localRX_Message[2] = sysState.RX_Message[2];
    localRotation[4] = sysState.rotation[4];
    localRotation[3] = sysState.rotation[3];
    localRotation[2] = sysState.rotation[2];
    localRotation[1] = sysState.rotation[1];
    localRotation[0] = sysState.rotation[0];
    xSemaphoreGive(sysState.mutex);

    //if handshake is complete, we display as normal
    if(handshakeComplete.load(std::memory_order_acquire) == true){

      u8g2.setFont(u8g2_font_unifont_t_76); //height 16, width 16
      u8g2.drawGlyph(2, 11, 0x266C);	

      u8g2.setFont(u8g2_font_boutique_bitmap_9x9_tf); // width 8 
      u8g2.setCursor(14, 8);
      u8g2.print("StackSynth"); //10 * 8 = 80 

      #ifdef receiver
      u8g2.setFont(u8g2_font_siji_t_6x10); //height 6, width 10

      //draws the volume button 
      if(localRotation[3] == 0){
        u8g2.drawGlyph(86, 8, 0xE202);
      }
      else if (localRotation[3] < 5){
        u8g2.drawGlyph(86, 8, 0xE204);	
      }
      else {
        u8g2.drawGlyph(86, 8, 0xE203);	
      }

      //draws the note 
      u8g2.drawGlyph(88, 22, 0xE271);

      //volume 
      u8g2.setFont(u8g2_font_boutique_bitmap_9x9_tf);
      u8g2.setCursor(98, 7);
      u8g2.print(":");
      u8g2.print(localRotation[3]);

      //wave
      localCurrentWaveform = __atomic_load_n(&CurrentWaveform, __ATOMIC_RELAXED);
      u8g2.setCursor(2, 20);
      u8g2.print("Wave:");
      u8g2.print(getWaveformName(static_cast<waveform>(localCurrentWaveform)));

      //musical note
      
      u8g2.setCursor(98,20);
      u8g2.print(":");
      if (localRX_Message[0] == 0x50){
        pressedNote = pianoNotes[localRX_Message[2]];
        u8g2.print(localRX_Message[1]);
        u8g2.print(pressedNote.c_str());
      }

      u8g2.setCursor(2, 31);
      u8g2.print("A:");
      u8g2.print(envelope.attackTime);
      u8g2.setCursor(46, 31);
      u8g2.print("D:");
      u8g2.print(envelope.decayTime);
      u8g2.setCursor(90, 31);
      u8g2.print("S:");
      u8g2.print(envelope.sustainLevel);
      #endif

      #ifdef sender  
      u8g2.setCursor(20, 19);
      u8g2.print(" Sending Notes...");
      #endif
    }
    
    else {
      u8g2.setFont(u8g2_font_siji_t_6x10); //height 6, width 10
      u8g2.drawGlyph(20, 20, 0xE14D);
      u8g2.drawGlyph(95, 20, 0xE141);
      
      u8g2.setFont(u8g2_font_boutique_bitmap_9x9_tf); 
      u8g2.setCursor(24, 19);  // Set cursor at the calculated X position
      u8g2.print(" Handshaking ");             // Print the text

    }


    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);

    #ifdef TEST_DISPLAY
    break;
    #endif
    }
}


void decodeTask(void * pvParameters) {
 uint32_t ID;
 uint32_t localCurrentStepSize;
 uint32_t stepIndex;
 uint32_t octave;
 uint8_t localRX_Message[8];

 while(1){
   xQueueReceive(msgInQ, (void *)localRX_Message, portMAX_DELAY); //localRX_Message is an array which holds the returned ITEM from the queue
   // Serial.println("Received from queue:");
   // Serial.print((char)localRX_Message[0]);
   // Serial.print(localRX_Message[1]);
   // Serial.println(localRX_Message[2]);


   //because RX_Message is a global variable, we need to use a mutex to update it
   xSemaphoreTake(sysState.mutex, portMAX_DELAY);
   sysState.RX_Message[0] = localRX_Message[0];
   sysState.RX_Message[1] = localRX_Message[1];
   sysState.RX_Message[2] = localRX_Message[2];
   xSemaphoreGive(sysState.mutex);

   if (localRX_Message[0] == 0x50) {  // pressed
       stepIndex = localRX_Message[2];
       octave = localRX_Message[1];
       localCurrentStepSize = StepSizes[stepIndex];
  
       if (octave >= 4) {
           localCurrentStepSize = localCurrentStepSize << (octave - 4);
       } else {
           localCurrentStepSize = localCurrentStepSize >> (4 - octave);
       }
  
       // Replace a 0 in currentStepSizes vector
       for (int i = 0; i < 10; i++) {
           uint32_t stepSize = __atomic_load_n(&currentStepSizes[i], __ATOMIC_RELAXED);
           if (stepSize == 0) {
               __atomic_store_n(&currentStepSizes[i], localCurrentStepSize, __ATOMIC_RELAXED);
               // Serial.print("Step size added ");
               // Serial.println(i);
               break; 
           }
       }

       startEnvelope();
   }

   else if (localRX_Message[0] == 0x52) {  // released
       stepIndex = localRX_Message[2];
       octave = localRX_Message[1];
       localCurrentStepSize = StepSizes[stepIndex];
  
       if (octave >= 4) {
           localCurrentStepSize = localCurrentStepSize << (octave - 4);
       } else {
           localCurrentStepSize = localCurrentStepSize >> (4 - octave);
       }
  
       for (int i = 0; i < 10; i++) {
           uint32_t stepSize = __atomic_load_n(&currentStepSizes[i], __ATOMIC_RELAXED);
           if (stepSize == localCurrentStepSize) {
               __atomic_store_n(&currentStepSizes[i], 0, __ATOMIC_RELAXED);
               // Serial.print("Step size removed ");
               // Serial.println(i);
               break; 
           }
       }
   }


   else if(localRX_Message[0] == 0x48){ //handshake
    //  Serial.print("Received Handshake:");
    //  Serial.print((char)localRX_Message[0]);
    //  Serial.print(localRX_Message[1]);
    //  Serial.print(localRX_Message[2]);
    //  Serial.print(localRX_Message[3]);
    //  Serial.print(localRX_Message[4]);
    //  Serial.println(localRX_Message[5]);

     if(handshakeComplete.load(std::memory_order_acquire) == true){
       handshakeComplete.store(false, std::memory_order_release);
     }


     uint32_t moduleID = (localRX_Message[1] << 0)  |  // Least significant byte
                         (localRX_Message[2] << 8)  |
                         (localRX_Message[3] << 16) |
                         (localRX_Message[4] << 24);  // Most significant byte
     xSemaphoreTake(hsState.mutex, portMAX_DELAY);
     if (hsState.moduleMap.find(moduleID) == hsState.moduleMap.end()) {
       hsState.moduleMap[moduleID] = localRX_Message[5];
     }
     xSemaphoreGive(hsState.mutex);
   }


   else if(localRX_Message[0] == 0x44){ //Complete 
     if (localRX_Message[1] == 1){
       //Serial.println("Received Handshake Complete from the eastmost module");
       handshakeComplete.store(true, std::memory_order_release);
       __atomic_store_n(&outBits[6], 1, __ATOMIC_RELAXED);
     }


     else if (localRX_Message[1] == 0){
       //Serial.println("Received Handshake Not Complete");
       handshakeComplete.store(false, std::memory_order_release);
       __atomic_store_n(&outBits[6], 1, __ATOMIC_RELAXED);


       xSemaphoreTake(hsState.mutex, portMAX_DELAY);
       hsState.moduleMap.clear();
       xSemaphoreGive(hsState.mutex);
     };
   }


   #ifdef TEST_DECODETASK
   break;
   #endif
 }
}


void CAN_TX_Task(void * pvParameters) {
 uint8_t msgOut[8];

	while (1) {
		xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
		CAN_TX(0x123, msgOut);
	}
}


//HANDSHAKING AND AUTO DETECT
uint32_t getIDHash(){
   uint32_t uid = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();  // Get the first 32 bits of the unique ID
   return uid;
}


void handshakeTask(void * pvParameters){
  uint32_t ID = getIDHash();
  std::bitset<32> localInputs;
  bool westDetect; 
  bool eastDetect;
  bool westMost = false; 
  bool eastMost = false;
  uint32_t localOctave;


 int32_t handshakeSymbol = 0x48; //'H'
 int8_t TX_Message[8] = {0};
 TX_Message[0] = handshakeSymbol;
 TX_Message[1] = ID & 0xFF;        
 TX_Message[2] = (ID >> 8) & 0xFF;
 TX_Message[3] = (ID >> 16) & 0xFF;
 TX_Message[4] = (ID >> 24) & 0xFF;


 //xFrequency is the initiation interval of the task
 const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
 // xLastWakeTime stores the tick count of the last initiation
 TickType_t xLastWakeTime = xTaskGetTickCount();

 while (1){
   vTaskDelayUntil(&xLastWakeTime, xFrequency);
   xSemaphoreTake(sysState.mutex, portMAX_DELAY);
   localInputs = sysState.inputs;
   xSemaphoreGive(sysState.mutex);


   //check for handshake inputs
   //West detect = Row 5 Col 3
   //East detect = Row 6 Col 3
   westDetect = !localInputs[5*4 + 3];
   eastDetect = !localInputs[6*4 + 3];


   TX_Message[0] = handshakeSymbol;
   TX_Message[1] = ID & 0xFF;        
   TX_Message[2] = (ID >> 8) & 0xFF;
   TX_Message[3] = (ID >> 16) & 0xFF;
   TX_Message[4] = (ID >> 24) & 0xFF;
  
   if(handshakeComplete.load(std::memory_order_acquire) == false){
     delayMicroseconds(3);


     if (westDetect){
       //Serial.println("west detected");
       continue;
     }
  
     else{
       xSemaphoreTake(hsState.mutex, portMAX_DELAY);
       int mapSize = hsState.moduleMap.size();
       bool inMap = hsState.moduleMap.find(ID) != hsState.moduleMap.end();
       xSemaphoreGive(hsState.mutex);


       //no west, yes east
       if (eastDetect){
         //Serial.println("east detected");
         //CASE1: WESTMOST MODULE 
         if (mapSize == 0) {
           TX_Message[5] = mapSize; //position is the size of the map
           xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
           //Serial.println("Sent Handshake: westmost module");
           __atomic_store_n(&outBits[6], 0, __ATOMIC_RELAXED);


           //append yourself to your map
           xSemaphoreTake(hsState.mutex, portMAX_DELAY);
           hsState.moduleMap[ID] = mapSize;
           xSemaphoreGive(hsState.mutex);


           //make myself westmost
           westMost = true;
         }
        
         //map not empty
         else {
           if (inMap){
             //Serial.println("Already in map, waiting for handshake complete");
             continue;
           }


           //CASE 2: MIDDLE MODULE 
           else{
             TX_Message[5] = mapSize; //position is the size of the map
             xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
             //Serial.println("Sent Handshake: Middle module");
             __atomic_store_n(&outBits[6], 0, __ATOMIC_RELAXED);


             //append yourself to your map
             xSemaphoreTake(hsState.mutex, portMAX_DELAY);
             hsState.moduleMap[ID] = mapSize;
             xSemaphoreGive(hsState.mutex);
           }
         }
       }
  
       // no west, no east
       // either you are the only module OR the eastmost module
       else{
         //CASE 4: ONLY MODULE
         if (mapSize == 0){
           //Serial.println("We are the only module");
           handshakeComplete.store(true, std::memory_order_release);
           westMost = true;
           eastMost = true;
         }


         //CASE 3: EASTMOST MODULE
         else{
           TX_Message[5] = mapSize; //position is the size of the map
           xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
           //Serial.println("Sent Handshake: eastmost module");


           //append yourself to your map
           xSemaphoreTake(hsState.mutex, portMAX_DELAY);
           hsState.moduleMap[ID] = mapSize;
           xSemaphoreGive(hsState.mutex);


           TX_Message[0] = 0x44; // 'C'
           TX_Message[1] = 1; //handshake complete
           xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
           //Serial.println("Sent Handshake: handshake complete");


           //update your own handshaking signal
           handshakeComplete.store(true, std::memory_order_release);


           eastMost = true;
         }
       }
     }
   }


   // Complete handshake
   // we either plug in on the westmost or eastmost module
   // and we remove from anywhere
   else{


     xSemaphoreTake(hsState.mutex, portMAX_DELAY);
     int mapSize = hsState.moduleMap.size();
     auto it = hsState.moduleMap.find(ID);
     int position = (it != hsState.moduleMap.end()) ? it->second : -99;
     xSemaphoreGive(hsState.mutex);


     // Serial.print("Position:");
     // Serial.println(position);
  
     localOctave = 4;


     if (mapSize > 1 && position >= 0) {
       localOctave = 3 + position;
     }
     __atomic_store_n(&Octave, localOctave, __ATOMIC_RELAXED);


     if (westMost){
       //look out for a west signal, if detected means something has plugged in
       if(westDetect){
         if(position != -99){
           TX_Message[0] = 0x44; // 'C'
           TX_Message[1] = 0; //handshake incomplete
           xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
           //Serial.println("Sent Handshake: handshake incomplete, something plugged in on west");


           //handle your own handshake variables
           xSemaphoreTake(hsState.mutex, portMAX_DELAY);
           hsState.moduleMap.clear();
           xSemaphoreGive(hsState.mutex);
         }
         handshakeComplete.store(false, std::memory_order_release);
         westMost = false;
       }
      
       //if you are the west most AND not the only module, look out for disconnects on your east
       //but once disconnected you are the only module, so no need to send message
       if(position != -99){
         if(!eastDetect){
           //handle your own handshake variables
           xSemaphoreTake(hsState.mutex, portMAX_DELAY);
           hsState.moduleMap.clear();
           xSemaphoreGive(hsState.mutex);
          
           handshakeComplete.store(false, std::memory_order_release);
         }
       }
     }


     if (eastMost){
       //look out for a east signal, if detected means something has plugged in
       if(eastDetect){
         if(position != -99){
           TX_Message[0] = 0x44; // 'C'
           TX_Message[1] = 0; //handshake incomplete
           xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
           //Serial.println("Sent Handshake: handshake incomplete, something plugged in on east");


           //handle your own handshake variables
           xSemaphoreTake(hsState.mutex, portMAX_DELAY);
           hsState.moduleMap.clear();
           xSemaphoreGive(hsState.mutex);
         }
         handshakeComplete.store(false, std::memory_order_release);
         eastMost = false;
       }


       if(position != -99){
         if(!westDetect){
           //handle your own handshake variables
           xSemaphoreTake(hsState.mutex, portMAX_DELAY);
           hsState.moduleMap.clear();
           xSemaphoreGive(hsState.mutex);
          
           handshakeComplete.store(false, std::memory_order_release);
         }
       }
     }

     if (!eastMost && !westMost){
       if (!westDetect || !eastDetect){
         TX_Message[0] = 0x44; // 'C'
         TX_Message[1] = 0; //handshake incomplete
         xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
         //Serial.println("Sent Handshake: handshake incomplete, westmost module disconnected");


         //handle your own handshake variables
         xSemaphoreTake(hsState.mutex, portMAX_DELAY);
         hsState.moduleMap.clear();
         xSemaphoreGive(hsState.mutex);
        
         handshakeComplete.store(false, std::memory_order_release);
       }
     }
   }
 }
}


void setup() {
 // put your setup code here, to run once:


 //Set pin directions
 pinMode(RA0_PIN, OUTPUT);
 pinMode(RA1_PIN, OUTPUT);
 pinMode(RA2_PIN, OUTPUT);
 pinMode(REN_PIN, OUTPUT);
 pinMode(OUT_PIN, OUTPUT);
 pinMode(OUTL_PIN, OUTPUT);
 pinMode(OUTR_PIN, OUTPUT);
 pinMode(LED_BUILTIN, OUTPUT);


 pinMode(C0_PIN, INPUT);
 pinMode(C1_PIN, INPUT);
 pinMode(C2_PIN, INPUT);
 pinMode(C3_PIN, INPUT);
 pinMode(JOYX_PIN, INPUT);
 pinMode(JOYY_PIN, INPUT);


 //Initialise display
 setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
 delayMicroseconds(2);
 setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
 u8g2.begin();
 setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply


 //Initialise UART
 Serial.begin(9600);
 Serial.println("Hello World");
  generateSineLUT();


 #ifdef receiver
 #ifndef DISABLE_THREADS
 //Timer for ISR
 sampleTimer.setOverflow(22000, HERTZ_FORMAT);
 sampleTimer.attachInterrupt(sampleISR);
 sampleTimer.resume();
 #endif
 #endif


 //Initialise CAN
 #ifndef DISABLE_THREADS
 CAN_Init(false);
 setCANFilter(0x123, 0x7FF);
 CAN_RegisterRX_ISR(CAN_RX_ISR);
 CAN_RegisterTX_ISR(CAN_TX_ISR);
 CAN_Start();
 #endif


 msgOutQ = xQueueCreate(36,8); //(number of items, size of each item)
 msgInQ = xQueueCreate(36, 8);


 #ifndef DISABLE_THREADS

 TaskHandle_t scanKeysHandle = NULL;
 xTaskCreate(
   scanKeysTask,   /* Function that implements the task */
   "scanKeys",   /* Text name for the task */
   512,          /* Stack size in words, not bytes */
   NULL,     /* Parameter passed into the task */
   5,      /* Task priority, higher value = higher priority*/
   &scanKeysHandle /* Pointer to store the task handle */
 ); 


 TaskHandle_t  decodeHandle = NULL;
 xTaskCreate(
   decodeTask,   /* Function that implements the task */
   "decode RX",    /* Text name for the task */
   512,          /* Stack size in words, not bytes */
   NULL,     /* Parameter passed into the task */
   4,      /* Task priority */
   &decodeHandle /* Pointer to store the task handle */
 );


 TaskHandle_t  CAN_TXHandle = NULL;
 xTaskCreate(
   CAN_TX_Task,    /* Function that implements the task */
   "CAN_TX",   /* Text name for the task */
   256,          /* Stack size in words, not bytes */
   NULL,     /* Parameter passed into the task */
   3,      /* Task priority */
   &CAN_TXHandle /* Pointer to store the task handle */
 );


 TaskHandle_t  handshakeHandle = NULL;
 xTaskCreate(
   handshakeTask,    /* Function that implements the task */
   "handshake",    /* Text name for the task */
   256,          /* Stack size in words, not bytes */
   NULL,     /* Parameter passed into the task */
   2,      /* Task priority */
   &handshakeHandle /* Pointer to store the task handle */
 );


 TaskHandle_t displayUpdateHandle = NULL;
 xTaskCreate(
   displayUpdateTask,    /* Function that implements the task */
   "displayUpdate",    /* Text name for the task */
   512,          /* Stack size in words, not bytes */
   NULL,     /* Parameter passed into the task */
   1,      /* Task priority */
   &displayUpdateHandle /* Pointer to store the task handle */
 );
  #endif

 sysState.mutex = xSemaphoreCreateMutex();
 hsState.mutex = xSemaphoreCreateMutex();
 CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3); //(Max count, initial count)


 #ifdef TEST_SCANKEYS
 float startTime = micros();
 for (int iter = 0; iter < 32; iter++) {
   scanKeysTask(NULL);
 }
 float final_time = micros() - startTime;
 Serial.print("Worst Case Time for ScanKeys (ms): ");
 Serial.println(final_time/32000);
 while(1);
 #endif

  #ifdef TEST_DECODETASK
  for (int iter = 0; iter < 36; iter++) {
    uint8_t TX_Message[8] = {0};
    
    // Possible values
    uint8_t options[] = {0x44, 0x48, 0x50, 0x52};

    // Randomly select one of the four values
    TX_Message[0] = options[rand() % 4];

    xQueueSend(msgInQ, TX_Message, portMAX_DELAY);
  }

  float startTime = micros();
  for (int iter = 0; iter < 36; iter++) {
    decodeTask(NULL);
  }

  float final_time = micros() - startTime;
  Serial.print("Worst Case Time for DecodeTask (ms): ");
  Serial.println(final_time/36000);

  while(1);
  #endif

  #ifdef TEST_ISR
  float startTime = micros();
  for (int iter = 0; iter < 32; iter++) {
    sampleISR();
  }
  float final_time = micros() - startTime;
  Serial.print("Worst Case Time for ISR: ");
  Serial.println(final_time/32);
  while(1);
  #endif

  #ifdef TEST_DISPLAY
  float startTime = micros();
  for (int iter = 0; iter < 32; iter++) {
    displayUpdateTask(NULL);
  }
  float final_time = micros() - startTime;
  Serial.print("Worst Case Time for Display Update: ");
  Serial.println(final_time/32);
  while(1);
  #endif

  #ifndef DISABLE_THREADS
  vTaskStartScheduler();
  #endif
 }

void loop() {}

