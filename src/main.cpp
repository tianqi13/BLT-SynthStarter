#define _USE_MATH_DEFINES

#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <math.h>
#include <STM32FreeRTOS.h>
#include <knob.hpp>
#include <ES_CAN.h>

#include <array>
#include <cmath>

#define receiver 
const uint32_t Octave = 3;

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

void sampleISR() {
  // static local variable is not re-initialized on each call
  static uint32_t phaseAcc = 0;

  int32_t localRotation;
  localRotation = __atomic_load_n(&sysState.rotation, __ATOMIC_RELAXED);
  //Serial.println(localRotation);
  //uint32_t localCurrentStepSize;
  //localCurrentStepSize = __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);

  phaseAcc += currentStepSize;
  // int32_t Vout = (phaseAcc >> 24) - 128;

  // int32_t Vout = 128 * sinf(((phaseAcc >> 24) * 2 * M_PI) / 256);

  uint8_t index = phaseAcc >> 24; // Get upper 8 bits
  int32_t Vout;
  
  if(CurrentWaveform==SINE){
    Vout = sineLUT[index];
    // Vout = Vout * 1.2; // Scale for RMS better
  } else if (CurrentWaveform==SAWTOOTH){
    Vout = index - 128;
    Vout = Vout * 0.5;
  } else if (CurrentWaveform==TRIANGLE){
    if (index < 128) {
      Vout = 2 * index - 128; // Rising part
    } else {
      Vout = 2 * (255 - index) - 128; // Falling part
    }
    // Vout = Vout * 1.2; // Scale for RMS better
  } else if (CurrentWaveform==SQUARE){
    Vout = (index < 128) ? -128 : 127;
    Vout = Vout * 0.5;
  }

  // Serial.println(phaseAcc);
  // Serial.print("Vout: ");
  // Serial.println(Vout);

  // volume
  Vout = Vout >> (8 - localRotation);
     
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
  std::bitset<32> colState;
  std::bitset<32> localInputs;
  int32_t currentStepIndex;
  int32_t previousStepIndex = 12;
  uint32_t localCurrentStepSize;
  uint8_t TX_Message[8] = {0};

  //define octave here 
  TX_Message[1] = Octave;

  //xFrequency is the initiation interval of the task 
  const TickType_t xFrequency = 50/portTICK_PERIOD_MS;
  // xLastWakeTime stores the tick count of the last initiation
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1){
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    localInputs.reset();
    
    // Loop through rows of key matrix and read columns
    for(int i = 0; i<4; i++){
      setROW(i);
      delayMicroseconds(3);
      colState = readCols();
      localInputs |= (colState << (i * 4));
    }
    //Serial.println(localInputs.to_string().c_str());

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

    if (currentStepIndex != previousStepIndex) {
      if (currentStepIndex == 12){
        TX_Message[0] = 0x52; //'R' Released previous key 
        TX_Message[2] = previousStepIndex;
      }
      else{
        TX_Message[0] = 0x50; //'P' Pressed current key 
        TX_Message[2] = currentStepIndex;
      }
    }

    #ifdef sender
    xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
    #endif

    #ifdef receiver
    xQueueSend(msgInQ, TX_Message, portMAX_DELAY);
    #endif

    previousStepIndex = currentStepIndex;

    // UBaseType_t stackRemaining = uxTaskGetStackHighWaterMark(NULL);  
    // Serial.print("scanKeysTask stack remaining: ");
    // Serial.println(stackRemaining);
  }

}

void displayUpdateTask(void * pvParameters) {
  //xFrequency is the initiation interval of the task 
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  // xLastWakeTime stores the tick count of the last initiation
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1){
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"Hi World!");

    u8g2.setCursor(66,10);
    u8g2.print("Wave: ");
    u8g2.print(CurrentWaveform);

    u8g2.setCursor(2,20);
    u8g2.print("Step Size: ");
    u8g2.print(currentStepSize);


    u8g2.setCursor(2,30);
    u8g2.print("Volume: ");
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    u8g2.print(sysState.rotation);

    u8g2.setCursor(66,30);
    u8g2.print((char) sysState.RX_Message[0]);
    u8g2.print(sysState.RX_Message[1]);
    u8g2.print(sysState.RX_Message[2]);
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
    // Serial.println("Received:");
    // Serial.println(localRX_Message[0]);
    // Serial.println(localRX_Message[1]);
    // Serial.println(localRX_Message[2]);

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
  }
}

void CAN_TX_Task(void * pvParameters) {
  uint8_t msgOut[8];

	while (1) {
		xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
    // Serial.print("Sent: ");
    // Serial.print(msgOut[0]);
    // Serial.print(msgOut[1]);
    // Serial.println(msgOut[2]);
		CAN_TX(0x123, msgOut);
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
  //Timer for ISR 
  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();
  #endif

  //Initialise CAN
  CAN_Init(true);
  setCANFilter(0x123, 0x7FF);
  #ifdef receiver
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  #endif

  #ifdef sender
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  #endif
  CAN_Start();

  msgInQ = xQueueCreate(36,8); //(number of items, size of each item)
  msgOutQ = xQueueCreate(36,8); 

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask,		/* Function that implements the task */
    "scanKeys",		/* Text name for the task */
    512,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    4,			/* Task priority, higher value = higher priority*/
    &scanKeysHandle /* Pointer to store the task handle */
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

  #ifdef receiver
  TaskHandle_t  decodeHandle = NULL; 
  xTaskCreate(
    decodeTask,		/* Function that implements the task */
    "decode RX",		/* Text name for the task */
    512,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    3,			/* Task priority */
    &decodeHandle /* Pointer to store the task handle */
  );
  #endif

  #ifdef sender
  TaskHandle_t  CAN_TXHandle = NULL; 
  xTaskCreate(
    CAN_TX_Task,		/* Function that implements the task */
    "CAN_TX",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    2,			/* Task priority */
    &CAN_TXHandle /* Pointer to store the task handle */
  );
  #endif

  sysState.mutex = xSemaphoreCreateMutex();
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3); //(Max count, initial count)

  vTaskStartScheduler();
  
}

void loop() {}