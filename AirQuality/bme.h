#pragma once
#include <cmath>

// Structure for BME68x sensor data.
// Not to be confused with bme68xData structure.
struct bmeData {
    float iaq{NAN};    // Indoor Air Quality
    float iaqAcc{NAN}; // IAQ Accuracy
    float temp{NAN};   // temperature
    float pres{NAN};   // pressure
    float hum{NAN};    // humidity
    float gas{NAN};    // gas resistance
    float stab{NAN};   // stabilization status
    float runIn{NAN};  // run-in status
};
