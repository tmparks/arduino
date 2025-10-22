#pragma once
#include "bme.h"
#include <Adafruit_PM25AQI.h>

template <typename T>
void printHeader(T& file);

template <typename T>
void printPM25Data(T& file, const PM25_AQI_Data& data);

template <typename T>
void printBMEData(T& file, const bmeData& data);
