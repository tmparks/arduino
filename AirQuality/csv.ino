// Utility functions to print CSV headers and data

#include "csv.h"

template <typename T>
void printHeader(T& file) {
    file.println(
            "Timestamp,"
            "Motion,"
            "PM2.5_1MinAvg,"
            "PM1.0_std,"
            "PM2.5_std,"
            "PM10_std,"
            "PM1.0_env,"
            "PM2.5_env,"
            "PM10_env,"
            "P>0.3um,"
            "P>0.5um,"
            "P>1.0um,"
            "P>2.5um,"
            "P>5.0um,"
            "P>10um,"
            "AQI_PM2.5,"
            "AQI_PM10,"
            "MiCS5524-Vs,"
            "IAQ,"
            "IAQ_Accuracy,"
            "Temp,"
            "Pressure,"
            "Humidity,"
            "Gas,"
            "Stab_Status,"
            "RunIn_Status,"
            "2-PM1.0_std,"
            "2-PM2.5_std,"
            "2-PM10_std,"
            "2-PM1.0_env,"
            "2-PM2.5_env,"
            "2-PM10_env,"
            "2-P>0.3um,"
            "2-P>0.5um,"
            "2-P>1.0um,"
            "2-P>2.5um,"
            "2-P>5.0um,"
            "2-P>10um,"
            "2-AQI_PM2.5,"
            "2-AQI_PM10");
}

template <typename T>
void printPM25Data(T& file, const PM25_AQI_Data& data) {
    file.print(data.pm10_standard);
    file.print(",");
    file.print(data.pm25_standard);
    file.print(",");
    file.print(data.pm100_standard);
    file.print(",");
    file.print(data.pm10_env);
    file.print(",");
    file.print(data.pm25_env);
    file.print(",");
    file.print(data.pm100_env);
    file.print(",");
    file.print(data.particles_03um);
    file.print(",");
    file.print(data.particles_05um);
    file.print(",");
    file.print(data.particles_10um);
    file.print(",");
    file.print(data.particles_25um);
    file.print(",");
    file.print(data.particles_50um);
    file.print(",");
    file.print(data.particles_100um);
    file.print(",");
    file.print(data.aqi_pm25_us);
    file.print(",");
    file.print(data.aqi_pm100_us);
}
