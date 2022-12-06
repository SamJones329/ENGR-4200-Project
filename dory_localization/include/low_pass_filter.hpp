// Source: https://github.com/jimmyberg/LowPassFilter

#ifndef DoryLocLowPassFilter
#define DoryLocLowPassFilter

#include <cmath>

#define ERROR_CHECK (true)

#if ERROR_CHECK
#include <iostream>
#endif

namespace DoryLoc {
    class LowPassFilter {
    public:
        LowPassFilter() : output(0), ePow(0) {};

        LowPassFilter(float iCutOffFrequency, float iDeltaTime) : output(0), ePow(1-exp(-iDeltaTime * 2 * M_PI * iCutOffFrequency)) {
            #if ERROR_CHECK
            if (iDeltaTime <= 0) {
                std::cout << "Warning: A LowPassFilter instance has been configured with 0 s as delta time.";
                ePow = 0;
            }
            if(iCutOffFrequency <= 0) {
                std::cout << "Warning: A LowPassFilter instance has been configured with 0 Hz as cut-off frequency.";
                ePow = 0;
            }
            #endif
        }

        float update(float input) {
            return output += (input - output) * ePow; // TODO use macro
        }

        float update(float input, float deltaTime, float cutOffFrequency) {
            reconfigureFilter(deltaTime, cutOffFrequency);
            return output += (input - output) * ePow;
        }

        float getOutput() const {return output;}

        void reconfigureFilter(float deltaTime, float cutOffFrequency) {
            #if ERROR_CHECK
            if (deltaTime <= 0) {
                std::cout << "Warning: A LowPassFilter instance has been configured with 0 s as delta time.";
                ePow = 0;
            }
            if(cutOffFrequency <= 0) {
                std::cout << "Warning: A LowPassFilter instance has been configured with 0 Hz as cut-off frequency.";
                ePow = 0;
            }
            #endif
            ePow = 1-exp(-deltaTime * 2 * M_PI * cutOffFrequency); // TODO use macro
        }

    private:
        float output;
        float ePow;
    };
}

#endif //DoryLocLowPassFilter