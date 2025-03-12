#include <bitset>
#include <STM32FreeRTOS.h>

struct systemState{
    std::bitset<32> inputs;
    int32_t rotation[5] = {50, 100, 7, 8, -1};
    uint8_t RX_Message[8] = {0};
  
    SemaphoreHandle_t mutex;
};

extern systemState sysState;

class Knob {
    private:
      // volatile int rotation3;
      // volatile int rotation2;
      // volatile int rotation1;
      // volatile int rotation0;
      volatile int rotation[5];
      volatile int knobIndex;
      // volatile int waveRotation;
      volatile int offset;
      volatile int row; 
      volatile int column;
      volatile int upperLimit; 
      volatile int lowerLimit;
    
      volatile bool knob_a_current;
      volatile bool knob_b_current;
      volatile bool knob_a_previous;
      volatile bool knob_b_previous;

      int decodeOffset(std::bitset<32> localInputs) volatile {
        knob_a_current = localInputs[row*4 + column];
        knob_b_current = localInputs[row*4 + column + 1];
    
        if (knob_a_previous == false && knob_a_current == true && knob_b_previous == false && knob_b_current == false) {
          offset = 1;  // Transition 00 -> 01
        }
        else if (knob_a_previous == true && knob_a_current == false && knob_b_previous == false && knob_b_current == false) {
          offset = -1; // Transition 01 -> 00
        }
        else if (knob_a_previous == false && knob_a_current == true && knob_b_previous == true && knob_b_current == true) {
          offset = -1; // Transition 10 -> 11
        }
        else if (knob_a_previous == true && knob_a_current == false && knob_b_previous == true && knob_b_current == false) {
          offset = 1;  // Transition 11 -> 10
        }
        else if (knob_a_previous != knob_a_current && knob_b_previous != knob_b_current){
          // Impossible state, assume same direction as last legal transition
        }
        else {
          offset = 0;
        }
    
        knob_a_previous = knob_a_current;
        knob_b_previous = knob_b_current;
        
        return offset;
      }

    public:
      Knob(volatile int _knobIndex, volatile int _row, volatile int _column, volatile int _upperLimit, volatile int _lowerLimit){ 
        knobIndex = _knobIndex;
        // rotation[3] = 5;
        // rotation[2] = 7;
        // rotation[1] = 100;
        // rotation[0] = 50; 
        // waveRotation = 0;
        row = _row;
        column = _column;   
        upperLimit = _upperLimit;
        lowerLimit = _lowerLimit;   
        knob_a_current = false;   
        knob_b_current = false;
        knob_a_previous = false;
        knob_b_previous = false;
      }
  
      void setLimits(int upper, int lower) volatile{
        upperLimit = upper;
        lowerLimit = lower;
      }
  
      void updateRotation(std::bitset<32> localInputs) volatile{
        int offset = decodeOffset(localInputs);

        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        int localRotation = sysState.rotation[knobIndex];
        xSemaphoreGive(sysState.mutex);

        if (localRotation + offset > upperLimit || localRotation + offset < lowerLimit){}
        else if (knobIndex == 0 || knobIndex == 1) {
          localRotation += offset * 10;
        } else{
          localRotation += offset;
        }
        //get mutex to update rotation value 
        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        sysState.rotation[knobIndex] = localRotation;
        xSemaphoreGive(sysState.mutex);

        // if (knobIndex == 4){
        //   Serial.println(localRotation);
        // }
      }

      void updateWave(std::bitset<32> localInputs) volatile{
        knob_b_current = localInputs[row*4 + column];

        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        int localRotation = sysState.rotation[knobIndex];
        xSemaphoreGive(sysState.mutex);
        

        if (knob_b_current == 1 && knob_b_previous == 0 && localRotation < upperLimit){
          localRotation += 1;
        } else if (knob_b_current == 1 && knob_b_previous == 0 && localRotation > lowerLimit){
          localRotation = 0;
        }

        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        sysState.rotation[knobIndex] = localRotation;
        xSemaphoreGive(sysState.mutex);

        knob_b_previous = knob_b_current;


      }

      int getRotation() volatile{
        int localRotation;
        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        localRotation = sysState.rotation[knobIndex];
        xSemaphoreGive(sysState.mutex);
        return localRotation;
      }

      // int getWave() volatile{
      //   int localRotation;
      //   xSemaphoreTake(sysState.mutex, portMAX_DELAY);
      //   localRotation = sysState.waveRotation;
      //   xSemaphoreGive(sysState.mutex);
      //   return localRotation;
      // }
  };