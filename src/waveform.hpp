#define _USE_MATH_DEFINES

#include <array>
#include <cmath>
#include <bitset>

constexpr size_t WAVETABLE_SIZE = 13;

class WavetableGenerator {
    public:
        constexpr std::array<uint32_t, WAVETABLE_SIZE> generateSawtoothWave() {
            double freq_factor = pow(2, 1.0/12.0);
            std::array<uint32_t, WAVETABLE_SIZE> result = {0};
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

        constexpr std::array<uint32_t, WAVETABLE_SIZE> generateSineWave() {
            double freq_factor = pow(2, 1.0/12.0);
            std::array<uint32_t, WAVETABLE_SIZE> result = {0};
            double freq = 0.0;

            for (size_t i = 0; i < 12; i++) {
                if (i >= 9) {
                    freq = 440 * pow(freq_factor, (i - 9));  // Above A4
                } else {
                    freq = 440 / pow(freq_factor, (9 - i));  // Below A4
                }
                result[i] = static_cast<uint32_t>((pow(2, 31) - 1) * (sin(2 * M_PI < * freq / 22000) + 1) / 2);
            }
            result[12] = 0x0;
            return result;
        }
    
        constexpr std::array<uint32_t, WAVETABLE_SIZE> generateTriangleWave() {
            std::array<int16_t, WAVETABLE_SIZE> wave;
            for (size_t i = 0; i < WAVETABLE_SIZE; ++i) {
                float x = (float)i / WAVETABLE_SIZE;
                float val;
                if (x < 0.25f) 
                    val = 4.0f * x;
                else if (x < 0.75f) 
                    val = 2.0f - 4.0f * x;
                else 
                    val = 4.0f * x - 4.0f;
                
                wave[i] = static_cast<int16_t>(32767.0 * val);
            }
            return wave;
        }
    
        constexpr std::array<uint32_t, WAVETABLE_SIZE> generateSquareWave() {
            std::array<int16_t, WAVETABLE_SIZE> wave;
            for (size_t i = 0; i < WAVETABLE_SIZE; ++i) {
                // Square wave generation
                float val = (i < WAVETABLE_SIZE/2) ? 1.0f : -1.0f;
                
                wave[i] = static_cast<int16_t>(32767.0 * val);
            }
            return wave;
        }
    };