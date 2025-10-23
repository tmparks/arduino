#pragma once
#include "bme.h"
#include <Adafruit_PM25AQI.h>
#include <FS.h>
#include <SdFat.h>

void printHeader(Print& file);

void printRow(
        Print& file,
        bool motion,
        float pm25avg,
        float micsVoltage,
        const bmeData& envData,
        const PM25_AQI_Data& aqiData1,
        const PM25_AQI_Data& aqiData2);
