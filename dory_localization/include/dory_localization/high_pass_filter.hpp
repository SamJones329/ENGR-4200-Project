// Source: https://github.com/jimmyberg/DigitalFilters

#ifndef DoryLocHighPassFilter
#define DoryLocHighPassFilter

#include <stdexcept>
#include <cmath>
#include <boost/format.hpp>

namespace DoryLoc {
    /**
     * @brief      Class for high pass filter using bilinear transform.
     */
    class HighPassFilter {
    public:
        /**
         * @brief      Constructor to set sample time and the tau constant
         *
         * @param[in]  idt     Sample time for the low pass filter
         * @param[in]  itua_c  Or
         *             @f$ \tau_c
         *             @f$ The time constant for the filter. Note that
         *             @f$ \tau_c = \frac{1}{2 pi f_c}@f$  where @f$ f_c @f$ is the cutoff frequency
         */
        HighPassFilter(double idt, double omega_c):
            amplFac(1/((idt * omega_c / 2) + 1)),
            y1c((idt * omega_c / 2) - 1),
            dt(idt){
                if(omega_c < idt){
                    auto err_format_str = boost::format("LowPassFilter constructor error: tua_c=%1% is smaller than the sample time dt=%2%. In particular the sampling frequency must be greater than to 2pi times the cutoff frequency.") % omega_c % idt;
                    std::string err_str = err_format_str.str();
                    throw std::domain_error(err_str);
                }
            }
        /**
         * @brief      Update function to push new value into the low pass filter
         *
         * @param[in]  newValue  The new value after dt time
         *
         * @return     The new output value
         */
        double update(double newValue) {
            // Note that output before assignment equals y1 being y[n-1]
            output = amplFac * (newValue - x1 - output * y1c);
            x1 = newValue;
            return output;
        }
        /**
         * @brief      Gets the output.
         *
         * @return     The output.
         */
        double getOutput() {return output;}
        /**X
         * @brief      Force the output to a desired value
         *
         *             This can be useful when the output needs to be forced in case
         *             of extreme inputs or such
         *
         * @param[in]  newOutput  The new output
         */
        void configOutput(double newOutput){output = newOutput;}
        const double* outputPointer(){return &output;}
    private:
        const double amplFac; // one time calculation constant
        const double y1c; // one time calculation constant
        const double dt;
        double x1 = 0;
        double output = 0;
    };
}

#endif // DoryLocHighPassFilter