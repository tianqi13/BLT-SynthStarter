#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <math.h>
#include <STM32FreeRTOS.h>

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

//Global variables
volatile uint32_t currentStepSize;

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

const std::array<uint32_t, 13> StepSizes = getArray();

int getStepIndex(std::bitset<32> inputs) {
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

struct {
  std::bitset<32> inputs;
  int rotation;
  SemaphoreHandle_t mutex;

} sysState;


void sampleISR() {
  // static local variable is not re-initialized on each call
  static uint32_t phaseAcc = 0;
  int localRotation;
  localRotation = __atomic_load_n(&sysState.rotation, __ATOMIC_RELAXED);
  //uint32_t localCurrentStepSize;
  //localCurrentStepSize = __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);

  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  Vout = Vout >> (8 - localRotation);
  analogWrite(OUTR_PIN, Vout + 128);
  
}

void scanKeysTask(void * pvParameters) {
  std::bitset<32> colState;
  std::bitset<32> localInputs;
  int StepIndex;
  uint32_t localCurrentStepSize;

  bool knob3_a_current = false;
  bool knob3_b_current = false;
  bool knob3_a_previous = false;
  bool knob3_b_previous = false;

  int rotation = 0;

  //xFrequency is the initiation interval of the task 
  const TickType_t xFrequency = 40/portTICK_PERIOD_MS;
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
    Serial.println(localInputs.to_string().c_str());
    
    knob3_a_current = localInputs[14];
    knob3_b_current = localInputs[15];

    if (knob3_a_previous == false && knob3_a_current == true && knob3_b_previous == false && knob3_b_current == false) {
      rotation = 1;  // Transition 00 -> 01
    }
    else if (knob3_a_previous == true && knob3_a_current == false && knob3_b_previous == false && knob3_b_current == false) {
      rotation = -1; // Transition 01 -> 00
    }
    else if (knob3_a_previous == false && knob3_a_current == true && knob3_b_previous == true && knob3_b_current == true) {
      rotation = -1; // Transition 10 -> 11
    }
    else if (knob3_a_previous == true && knob3_a_current == false && knob3_b_previous == true && knob3_b_current == false) {
      rotation = 1;  // Transition 11 -> 10
    }
    else if (knob3_a_previous != knob3_a_current && knob3_b_previous != knob3_b_current){
      // Impossible state, assume same direction as last legal transition
    }
    else {
      rotation = 0;
    }

    knob3_a_previous = knob3_a_current;
    knob3_b_previous = knob3_b_current;

    //Serial.println(rotation);
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    sysState.inputs = localInputs;
    if (sysState.rotation + rotation > 8 || sysState.rotation + rotation < 0){}
    else {
      sysState.rotation += rotation;
    }
    xSemaphoreGive(sysState.mutex);
    // Get the step index and step size
    StepIndex = getStepIndex(localInputs);
    Serial.println(StepIndex);
    localCurrentStepSize = StepSizes[StepIndex];
    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);

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

    u8g2.setCursor(2,20);
    u8g2.print("Step Size: ");
    u8g2.print(currentStepSize);

    u8g2.setCursor(2,30);
    u8g2.print("Rotation: ");
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    u8g2.print(sysState.rotation);
    xSemaphoreGive(sysState.mutex);

    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);
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

  //Timer for ISR 
  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask,		/* Function that implements the task */
    "scanKeys",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    2,			/* Task priority, higher value = higher priority*/
    &scanKeysHandle /* Pointer to store the task handle */
  );	

  TaskHandle_t displayUpdateHandle = NULL; 
  xTaskCreate(
    displayUpdateTask,		/* Function that implements the task */
    "displayUpdate",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &displayUpdateHandle /* Pointer to store the task handle */
  );

  sysState.mutex = xSemaphoreCreateMutex();

  vTaskStartScheduler();
  
}

void loop() {}