#include <bitset>
#include <STM32FreeRTOS.h>

struct systemState{
    std::bitset<32> inputs;
    int32_t rotation;
    uint8_t RX_Message[8] = {0};
    SemaphoreHandle_t mutex;
};

extern systemState sysState;

class Knob {
    private:
      volatile int rotation;
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
      Knob(volatile int _row, volatile int _column, volatile int _upperLimit, volatile int _lowerLimit){ 
        rotation = 0;  
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
        if (rotation + offset > upperLimit || rotation + offset < lowerLimit){}
        else {
          rotation += offset;
        }
        //get mutex to update rotation value 
        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        sysState.rotation = rotation;
        xSemaphoreGive(sysState.mutex);
      }

      int getRotation() volatile{
        int localRotation;
        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        localRotation = sysState.rotation;
        xSemaphoreGive(sysState.mutex);
        return localRotation;
      }
  };