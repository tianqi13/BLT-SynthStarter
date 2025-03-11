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

// #define receiver 
#define sender

#define DISABLE_THREADS
#define TEST_SCANKEYS

enum waveform {
  SAWTOOTH,
  SINE,
  TRIANGLE,
  SQUARE
};

volatile waveform CurrentWaveform = SAWTOOTH;

#define TABLE_SIZE 256

// Generate a sine lookup table
int sineLUT[TABLE_SIZE];

void generateSineLUT() {
    int minVal = 127, maxVal = -128;
    for (int i = 0; i < TABLE_SIZE; i++) {
        sineLUT[i] = (int)(127 * sinf(2 * M_PI * i / TABLE_SIZE));
    }
}

// #define receiver
// const uint32_t Octave = 3;

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

int32_t getStepIndex(std::bitset<32> inputs) {
  // Mask to clear bits 31 to 12
  std::bitset<32> mask = 0x00000FFF;
  std::bitset<32> temp = inputs & mask; 
  temp = temp ^ mask;
  
  // Convert result to a number
  int index = temp.to_ulong();
  
  // Avoid log2(0) since it is undefined
  if (index == 0) {
      return 12;  // Or some other sentinel value indicating no key is pressed
  }
  
  // Use log2 to calculate the step index
  int stepIndex = static_cast<int>(log2(index)); 

  return stepIndex;
}

//Timer
HardwareTimer sampleTimer(TIM1);

//Global variables
volatile uint32_t currentStepSize;
systemState sysState;
volatile Knob knob3 = Knob(3, 0, 8, 0);
volatile Knob knob2 = Knob(3, 2, 3, 0); // 4 waveforms for now
QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;
SemaphoreHandle_t CAN_TX_Semaphore;
volatile uint32_t Octave;

volatile bool outBits[7] = {0,0,0,1,1,1,1};
std::atomic<bool> handshakeComplete{false};
struct handshakeState{
  std::unordered_map<uint32_t, int> moduleMap;
  SemaphoreHandle_t mutex;
};
handshakeState hsState;


void sampleISR() {
  // static local variable is not re-initialized on each call
  static uint32_t phaseAcc = 0;

  int32_t localRotation;
  localRotation = __atomic_load_n(&sysState.rotation, __ATOMIC_RELAXED);

  uint32_t localCurrentStepSize;
  localCurrentStepSize = __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);

  if (localCurrentStepSize == 0) {
    analogWrite(OUTR_PIN, 128);
    return;
  }

  phaseAcc += localCurrentStepSize;
  // int32_t Vout = (phaseAcc >> 24) - 128;

  uint8_t index = phaseAcc >> 24; // Get upper 8 bits
  int32_t Vout;

  // CurrentWaveform = SINE;
  
  if(CurrentWaveform==SINE){
    Vout = sineLUT[index];
    // Vout = Vout << 8; // Scale??
  } else if (CurrentWaveform==SAWTOOTH){
    Vout = index - 128;
    Vout = Vout * 0.5;
  } else if (CurrentWaveform==TRIANGLE){
    if (index < 128) {
      Vout = 2 * index - 128; // Rising part
    } else {
      Vout = 2 * (255 - index) - 128; // Falling part
    }
    // Vout = Vout * 1.2; // Scale??
  } else if (CurrentWaveform==SQUARE){
    Vout = (index < 128) ? -128 : 127;
    Vout = Vout * 0.5;
  }

  // Serial.println(phaseAcc);
  // Serial.print("Vout: ");
  // Serial.println(Vout);

  // volume
  Vout = Vout >> (8 - localRotation);

  
  // Serial.print("Final output: ");
  // Serial.println(Vout + 128);
     
  analogWrite(OUTR_PIN, Vout + 128);

  //Serial.println(Vout + 128);
  
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
  std::bitset<32> colState;
  std::bitset<32> localInputs;
  int32_t currentStepIndex = 12;
  uint32_t localCurrentStepSize = 0;
  uint8_t TX_Message[8] = {0};
  int32_t previousStepIndex = 12;
  int32_t previousAction = 0x52;
  bool outBit;

  TX_Message[0] = 0x50;
  TX_Message[1] = 4;
  TX_Message[2] = 12; //initalise to be 12 

  #ifndef TEST_SCANKEYS 
  //xFrequency is the initiation interval of the task 
  const TickType_t xFrequency = 25.2/portTICK_PERIOD_MS;
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

    #ifdef TEST_SCANKEYS
    for (int i = 0; i < 12; i++) {
      TX_Message[2] = i;
      xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
    }
    break;
    #endif

    //take mutex to update inputs
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    sysState.inputs = localInputs;
    xSemaphoreGive(sysState.mutex);

    knob3.updateRotation(localInputs);
    int32_t waveformIndex = knob2.getWave();
    switch(waveformIndex) {
      case 0: CurrentWaveform = SAWTOOTH; break;
      case 1: CurrentWaveform = SINE; break;
      case 2: CurrentWaveform = TRIANGLE; break;
      case 3: CurrentWaveform = SQUARE; break;
    }

    StepSizes = getArray();
    knob2.updateWave(localInputs);

    currentStepIndex = getStepIndex(localInputs);

    //if the note is not the same, then there has been a change
    if (currentStepIndex != previousStepIndex){
      //no notes are pressed now 
      if (currentStepIndex == 12){
        if (previousAction == 0x50){ // if previous action was pressed, now it means we released 
          TX_Message[0] = 0x52; // released  
          TX_Message[1] = __atomic_load_n(&Octave, __ATOMIC_RELAXED);
          TX_Message[2] = previousStepIndex;

          #ifdef receiver
          xQueueSend(msgInQ, TX_Message, portMAX_DELAY);
          #endif

          #ifdef sender
          xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
          // Serial.println("Sent to queue:");
          // Serial.print(TX_Message[0]);
          // Serial.print(TX_Message[1]);
          // Serial.print(TX_Message[2]);
          #endif
        }

        else if (previousAction == 0x52){ //previous action was release, still no notes are pressed
          //do nothing
        }
      }

      //a note is pressed now
      else{
        TX_Message[0] = 0x50; //'P' Pressed current key 
        TX_Message[1] = __atomic_load_n(&Octave, __ATOMIC_RELAXED);
        TX_Message[2] = currentStepIndex;

        #ifdef receiver
        xQueueSend(msgInQ, TX_Message, portMAX_DELAY);
        #endif

        #ifdef sender
        xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
        // Serial.println("Sent to queue:");
        // Serial.print(TX_Message[0]);
        // Serial.print(TX_Message[1]);
        // Serial.print(TX_Message[2]);
        #endif
      }
    }

    //in the case where they are the same but we continuously press a key, we should keep sending the message 
    else{
      if(currentStepIndex != 12){
        TX_Message[0] = 0x50; //'P' Pressed current key 
        TX_Message[2] = currentStepIndex;

        #ifdef receiver
        xQueueSend(msgInQ, TX_Message, portMAX_DELAY);
        #endif

        #ifdef sender
        xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
        // Serial.println("Sent to queue:");
        // Serial.print(TX_Message[0]);
        // Serial.print(TX_Message[1]);
        // Serial.print(TX_Message[2]);
        #endif
      }
    }

    previousStepIndex = currentStepIndex;
    previousAction = TX_Message[0];
  
    // UBaseType_t stackRemaining = uxTaskGetStackHighWaterMark(NULL);  
    // Serial.print("scanKeysTask stack remaining: ");
    // Serial.println(stackRemaining);
  }
}

void displayUpdateTask(void * pvParameters) {
  int32_t localRotation;
  int8_t localRX_Message[8];
  //xFrequency is the initiation interval of the task 
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  // xLastWakeTime stores the tick count of the last initiation
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1){
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    #ifdef receiver
    u8g2.drawStr(2,10,"Receiver");
    #endif

    #ifdef sender
    u8g2.drawStr(2,10,"Sender");
    #endif

    #ifdef receiver
    u8g2.setCursor(2,20);
    u8g2.print("Step Size: ");
    u8g2.print(currentStepSize);

    u8g2.setCursor(66,10);
    u8g2.print("Wave: ");
    u8g2.print(CurrentWaveform);

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    localRX_Message[0] = sysState.RX_Message[0];
    localRX_Message[1] = sysState.RX_Message[1];
    localRX_Message[2] = sysState.RX_Message[2];
    localRotation = sysState.rotation;
    xSemaphoreGive(sysState.mutex);

    u8g2.setCursor(2,30);
    u8g2.print("Rotation: ");
    u8g2.print(localRotation);

    u8g2.setCursor(66,30);
    u8g2.print((char) localRX_Message[0]);
    u8g2.print(localRX_Message[1]);
    u8g2.print(localRX_Message[2]);
    #endif

    u8g2.setCursor(66,20);
    u8g2.print("Volume: ");
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    u8g2.print(sysState.rotation);

    xSemaphoreGive(sysState.mutex);

    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);
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

    //because RX_Message is a global variable, we need to use a mutex to update it 
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    sysState.RX_Message[0] = localRX_Message[0];
    sysState.RX_Message[1] = localRX_Message[1];
    sysState.RX_Message[2] = localRX_Message[2];
    xSemaphoreGive(sysState.mutex);

    if (localRX_Message[0] == 0x50){ //pressed
      stepIndex = localRX_Message[2];
      octave = localRX_Message[1];
      localCurrentStepSize = StepSizes[stepIndex];
      if (octave >= 4){
        localCurrentStepSize = localCurrentStepSize << (octave - 4);
      }
      else{
        localCurrentStepSize = localCurrentStepSize >> (4 - octave);
      }
      __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
    }

    else if (localRX_Message[0] == 0x52){ //released
      //if key is released, step size is 0
      localCurrentStepSize = 0;
      __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
    }

    else if(localRX_Message[0] == 0x48){ //handshake 
      Serial.print("Received Handshake:");
      Serial.print((char)localRX_Message[0]);
      Serial.print(localRX_Message[1]);
      Serial.print(localRX_Message[2]);
      Serial.print(localRX_Message[3]);
      Serial.print(localRX_Message[4]);
      Serial.println(localRX_Message[5]);

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

    else if(localRX_Message[0] == 0x44){ //handshake 
      if (localRX_Message[1] == 1){
        Serial.println("Received Handshake Complete from the eastmost module");
        handshakeComplete.store(true, std::memory_order_release);
        __atomic_store_n(&outBits[6], 1, __ATOMIC_RELAXED);
      }

      else if (localRX_Message[1] == 0){
        Serial.println("Received Handshake Not Complete");
        handshakeComplete.store(false, std::memory_order_release);
        __atomic_store_n(&outBits[6], 1, __ATOMIC_RELAXED);

        xSemaphoreTake(hsState.mutex, portMAX_DELAY);
        hsState.moduleMap.clear();
        xSemaphoreGive(hsState.mutex);
      };
    }

  }
}

void CAN_TX_Task(void * pvParameters) {
  uint8_t msgOut[8];

	while (1) {
		xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
    // Serial.println("Sent from queue:");
    // Serial.print(msgOut[0]);
    // Serial.print(msgOut[1]);
    // Serial.print(msgOut[2]);
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
  bool secondWest = false;
  bool secondEast = false; 
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
        Serial.println("west detected");
        continue;
      }
    
      else{
        xSemaphoreTake(hsState.mutex, portMAX_DELAY);
        int mapSize = hsState.moduleMap.size();
        bool inMap = hsState.moduleMap.find(ID) != hsState.moduleMap.end();
        xSemaphoreGive(hsState.mutex);

        //no west, yes east
        if (eastDetect){
          Serial.println("east detected");
          //CASE1: WESTMOST MODULE  
          if (mapSize == 0) {
            TX_Message[5] = mapSize; //position is the size of the map 
            xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
            Serial.println("Sent Handshake: westmost module");
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
              Serial.println("Already in map, waiting for handshake complete");
              continue;
            }

            //CASE 2: MIDDLE MODULE  
            else{
              TX_Message[5] = mapSize; //position is the size of the map
              xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
              Serial.println("Sent Handshake: Middle module");
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
            Serial.println("We are the only module");
            handshakeComplete.store(true, std::memory_order_release);
            westMost = true; 
            eastMost = true;
          }

          //CASE 3: EASTMOST MODULE
          else{
            TX_Message[5] = mapSize; //position is the size of the map
            xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
            Serial.println("Sent Handshake: eastmost module");

            //append yourself to your map 
            xSemaphoreTake(hsState.mutex, portMAX_DELAY);
            hsState.moduleMap[ID] = mapSize;
            xSemaphoreGive(hsState.mutex);

            TX_Message[0] = 0x44; // 'C'
            TX_Message[1] = 1; //handshake complete
            xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
            Serial.println("Sent Handshake: handshake complete");

            //update your own handshaking signal 
            handshakeComplete.store(true, std::memory_order_release);

            eastMost = true;
          }
        }
      } 
    }

    // Complete handshake
    // we either plug in on the westmost or eastmost module 
    // and we either remove the westmost or eastmost module 
    else{

      xSemaphoreTake(hsState.mutex, portMAX_DELAY);
      int mapSize = hsState.moduleMap.size();
      auto it = hsState.moduleMap.find(ID);
      int position = (it != hsState.moduleMap.end()) ? it->second : -99;
      xSemaphoreGive(hsState.mutex);

      Serial.print("Position:");
      Serial.println(position);
    
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
            Serial.println("Sent Handshake: handshake incomplete, something plugged in on west");

            //handle your own handshake variables
            xSemaphoreTake(hsState.mutex, portMAX_DELAY);
            hsState.moduleMap.clear();
            xSemaphoreGive(hsState.mutex);
          }
          handshakeComplete.store(false, std::memory_order_release);
          westMost = false;
        }
      }

      if (eastMost){
        //look out for a east signal, if detected means something has plugged in 
        if(eastDetect){
          if(position != -99){
            TX_Message[0] = 0x44; // 'C'
            TX_Message[1] = 0; //handshake incomplete
            xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
            Serial.println("Sent Handshake: handshake incomplete, something plugged in on east");

            //handle your own handshake variables
            xSemaphoreTake(hsState.mutex, portMAX_DELAY);
            hsState.moduleMap.clear();
            xSemaphoreGive(hsState.mutex);
          }
          handshakeComplete.store(false, std::memory_order_release);
          eastMost = false;
        }
      }

      secondWest = false;
      secondEast = false;

      if (position == 1){
        secondWest = true;
        Serial.println("I am Second West");
      }

      else if (position == (mapSize - 2)){
        secondEast = true;
        Serial.println("I am Second East");
      }

      //if you are second west, you are responsible for sending message if the westmost module disconnects
      if(secondWest){
        if(!westDetect){
          TX_Message[0] = 0x44; // 'C'
          TX_Message[1] = 0; //handshake incomplete
          xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
          Serial.println("Sent Handshake: handshake incomplete, westmost module disconnected");

          //handle your own handshake variables
          xSemaphoreTake(hsState.mutex, portMAX_DELAY);
          hsState.moduleMap.clear();
          xSemaphoreGive(hsState.mutex);
          
          handshakeComplete.store(false, std::memory_order_release);
        }
      }

      if(secondEast){
        if(!eastDetect){
          TX_Message[0] = 0x44; // 'C'
          TX_Message[1] = 0; //handshake incomplete
          xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
          Serial.println("Sent Handshake: handshake incomplete, eastmost module disconnected");

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

  #ifdef TEST_SCANKEYS
  msgOutQ = xQueueCreate(384, 8);
  #else
  msgOutQ = xQueueCreate(36,8); //(number of items, size of each item)
  #endif

  #ifndef DISABLE_THREADS

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask,		/* Function that implements the task */
    "scanKeys",		/* Text name for the task */
    512,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    5,			/* Task priority, higher value = higher priority*/
    &scanKeysHandle /* Pointer to store the task handle */
  );	

  TaskHandle_t  decodeHandle = NULL; 
  xTaskCreate(
    decodeTask,		/* Function that implements the task */
    "decode RX",		/* Text name for the task */
    512,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    4,			/* Task priority */
    &decodeHandle /* Pointer to store the task handle */
  );

  TaskHandle_t  CAN_TXHandle = NULL; 
  xTaskCreate(
    CAN_TX_Task,		/* Function that implements the task */
    "CAN_TX",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    3,			/* Task priority */
    &CAN_TXHandle /* Pointer to store the task handle */
  );

  TaskHandle_t  handshakeHandle = NULL; 
  xTaskCreate(
    handshakeTask,		/* Function that implements the task */
    "handshake",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    2,			/* Task priority */
    &handshakeHandle /* Pointer to store the task handle */
  );

  TaskHandle_t displayUpdateHandle = NULL; 
  xTaskCreate(
    displayUpdateTask,		/* Function that implements the task */
    "displayUpdate",		/* Text name for the task */
    512,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &displayUpdateHandle /* Pointer to store the task handle */
  );
  
  #endif

  #ifdef TEST_SCANKEYS
  uint32_t startTime = micros();
  for (int iter = 0; iter < 32; iter++) {
    scanKeysTask(NULL);
  }
  Serial.println(micros() - startTime);
  while(1);
  #endif

  sysState.mutex = xSemaphoreCreateMutex();
  hsState.mutex = xSemaphoreCreateMutex();
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3); //(Max count, initial count)

  vTaskStartScheduler();
  
}

void loop() {}