// Box number (incorporated into file names)
constexpr int BOX_NUMBER = 0;

// PM2.5 high breakpoints
constexpr float PM25_GOOD = 12.0;
constexpr float PM25_MODERATE = 35.4;
constexpr float PM25_UNHEALTHY_SENSITIVE = 55.4;
constexpr float PM25_UNHEALTHY = 150.4;
constexpr float PM25_VERY_UNHEALTHY = 250.4;

// PM2.5 sleep hours (24-hour format)
// Set both to the same value to disable sleep mode
constexpr int PM25_SLEEP_BEGIN_HOUR = 22; // 10 PM
constexpr int PM25_SLEEP_END_HOUR = 5;    // 5 AM
constexpr int PM25_WAKE_MINUTES = 5; // wake for a few minutes each quarter hour

// Enable or disable the motion sensor
constexpr bool ENABLE_MOTION_SENSOR = true;
#include <ADC.h>
#include <Adafruit_PM25AQI.h>
#include <Arduino.h>
#include <DFRobot_C4001.h>
#include <SD.h>
#include <SPI.h>
#include <SdFat.h>
#include <TimeLib.h>
#include <Wire.h>
#include <bsec2.h>

// Re-enable warnings that were supressed for libraries and treat them as errors.
// Affects all .ino files that are part of this sketch.
// See compiler.cpp.extra_flags in platform.txt.
// https://docs.arduino.cc/arduino-cli/sketch-build-process/#pre-processing
#pragma GCC diagnostic error "-Wsign-compare"
#pragma GCC diagnostic error "-Wunused-variable"
// Enable additional warnings and treat them as errors.
// This is done here to avoid warnings in included libraries.
#pragma GCC diagnostic error "-Wall"
#pragma GCC diagnostic error "-Wextra"
#pragma GCC diagnostic error "-Wpedantic"

#include "bme.h"
#include "csv.h"

template <typename T>
void setZero(T& var) {
    static_assert(std::is_trivially_copyable_v<T> == true);
    memset(&var, 0, sizeof(T));
}

// Initialize ADC library for analog gas sensor (MiCS5524)
ADC* adc = new ADC();

// RGB common anode LED pin definitions (active LOW)
const int LED1_R = 10, LED1_G = 11, LED1_B = 12;
const int LED2_R = 16, LED2_G = 8, LED2_B = 9;
const int LED3_R = 33, LED3_G = 14, LED3_B = 15;

// Gas sensor config
const int ANALOG_PIN = A17;
const float ADC_REF_VOLTAGE = 3.3;
const int ADC_RESOLUTION = 1024;

// Integration interval for analog voltage (1 second)
const unsigned long INTEGRATION_INTERVAL_MS = 1000;
float integratedVoltage = 0.0;
unsigned long intervalStartMillis;
unsigned long previousMicros;

//Use hardware Serial7 (RX7 = pin 28, TX7 = pin 29)
DFRobot_C4001_UART radar(&Serial7, 9600);
bool dfMotion = false;

File internalFile;
bool internalOK = false;
bool internalFull = false;

const int externalChipSelect = 0;
SdFat externalSD;
FsFile externalFile;
bool externalOK = false;
bool externalFull = false;

// PM2.5 sensor (Adafruit PM5003 or equivalent)
constexpr auto PM25_SET_PIN_1 = 25; // sensor 1 control pin
constexpr auto PM25_SET_PIN_2 = 24; // sensor 2 control pin
constexpr auto PM25_SLEEP = LOW;
constexpr auto PM25_WAKE = HIGH;
Adafruit_PM25AQI aqi1 = Adafruit_PM25AQI();
Adafruit_PM25AQI aqi2 = Adafruit_PM25AQI();

//PM2.5 running average - Circular buffer for 1-minute (60 samples)
const int PM25_HISTORY_SIZE = 60;
uint16_t pm25History[PM25_HISTORY_SIZE] = {0}; // Holds last 60 PM2.5 values
int pm25Index = 0;                             // Points to next insert location
int pm25Count = 0; // Actual number of samples stored (max 60)

// Initialize PM2.5 sensors
void pm25Setup() {
    pinMode(PM25_SET_PIN_1, OUTPUT);
    pinMode(PM25_SET_PIN_2, OUTPUT);
    pm25SleepWake(); // set initial state

    Serial2.begin(9600); // sensor 1
    Serial5.begin(9600); // sensor 2
    delay(3000);
    if (!aqi1.begin_UART(&Serial2)) {
        errLeds();
    }
    if (!aqi2.begin_UART(&Serial5)) {
        errLeds();
    }
}

// PM2.5 sensor sleep/wake control
int pm25SleepWake() {
    auto val = PM25_WAKE; // wake by default
    auto currentTime = now();
    if (minute(currentTime) % 15 < PM25_WAKE_MINUTES) {
        // wake for a few minutes each quarter hour
        val = PM25_WAKE;
    }
    else if (PM25_SLEEP_BEGIN_HOUR < PM25_SLEEP_END_HOUR) {
        // Sleep period does not cross midnight
        if (PM25_SLEEP_BEGIN_HOUR <= hour(currentTime)
            && hour(currentTime) < PM25_SLEEP_END_HOUR) {
            val = PM25_SLEEP;
        }
    }
    else if (PM25_SLEEP_END_HOUR < PM25_SLEEP_BEGIN_HOUR) {
        // Sleep period crosses midnight
        if (PM25_SLEEP_BEGIN_HOUR <= hour(currentTime)
            || hour(currentTime) < PM25_SLEEP_END_HOUR) {
            val = PM25_SLEEP;
        }
    }
    else {
        // Sleep period disabled
        val = PM25_WAKE;
    }
    digitalWrite(PM25_SET_PIN_1, val);
    digitalWrite(PM25_SET_PIN_2, val);
    return val;
}

//PM2.5 breakpoints
String pm25Category(float avg) {
    if (avg <= PM25_GOOD)
        return "Good";
    else if (avg <= PM25_MODERATE)
        return "Moderate";
    else if (avg <= PM25_UNHEALTHY_SENSITIVE)
        return "Unhealthy_Sensitive";
    else if (avg <= PM25_UNHEALTHY)
        return "Unhealthy";
    else if (avg <= PM25_VERY_UNHEALTHY)
        return "Very_Unhealthy";
    else
        return "Hazardous";
}

void setRGB(int rPin, int gPin, int bPin, int rVal, int gVal, int bVal) {
    digitalWrite(rPin, rVal);
    digitalWrite(gPin, gVal);
    digitalWrite(bPin, bVal);
}

// For checking if date changed (used for daily CSV files)
String lastDate = "";
String lastDatext = "";

// Used for setting system time from serial input
#define TIME_HEADER "T"
const unsigned long DEFAULT_TIME = 1357041600; // Jan 1, 2013 fallback

// BME688 + BSEC2 sensor variables
#define PANIC_LED LED_BUILTIN
#define ERROR_DUR 1000
Bsec2 envSensor; // BSEC2 driver for BME688
// Output values from BSEC2
auto envData = bmeData{};

// Flash error code using LED
void errLeds() {
    while (1) {
        digitalWrite(PANIC_LED, HIGH);
        delay(ERROR_DUR);
        digitalWrite(PANIC_LED, LOW);
        delay(ERROR_DUR);
    }
}

// Check BSEC and BME sensor status and handle errors
void checkBsecStatus(Bsec2 bsec) {
    if (bsec.status < BSEC_OK) {
        Serial.println("BSEC error code : " + String(bsec.status));
        errLeds();
    }
    else if (bsec.status > BSEC_OK) {
        Serial.println("BSEC warning code : " + String(bsec.status));
    }
    if (bsec.sensor.status < BME68X_OK) {
        Serial.println("BME68X error code : " + String(bsec.sensor.status));
        errLeds();
    }
    else if (bsec.sensor.status > BME68X_OK) {
        Serial.println("BME68X warning code : " + String(bsec.sensor.status));
    }
}

// Callback for new data from BSEC2
void newDataCallback(const bme68xData, const bsecOutputs outputs, Bsec2) {
    if (!outputs.nOutputs)
        return;
    for (uint8_t i = 0; i < outputs.nOutputs; i++) {
        const bsecData output = outputs.output[i];
        switch (output.sensor_id) {
        case BSEC_OUTPUT_IAQ:
            envData.iaq = output.signal;
            envData.iaqAcc = output.accuracy;
            break;
        case BSEC_OUTPUT_RAW_TEMPERATURE:
            envData.temp = output.signal;
            break;
        case BSEC_OUTPUT_RAW_PRESSURE:
            envData.pres = output.signal;
            break;
        case BSEC_OUTPUT_RAW_HUMIDITY:
            envData.hum = output.signal;
            break;
        case BSEC_OUTPUT_RAW_GAS:
            envData.gas = output.signal;
            break;
        case BSEC_OUTPUT_STABILIZATION_STATUS:
            envData.stab = output.signal;
            break;
        case BSEC_OUTPUT_RUN_IN_STATUS:
            envData.runIn = output.signal;
            break;
        }
    }
}

// Get RTC time from Teensy
time_t getTeensyTime() { return (time_t)Teensy3Clock.get(); }

// Parse incoming serial time sync message (used in setup)
unsigned long processSyncMessage() {
    unsigned long pctime = 0L;
    if (Serial.find(TIME_HEADER)) {
        pctime = Serial.parseInt();
        if (pctime >= DEFAULT_TIME)
            return pctime;
    }
    return 0L;
}

// Get full timestamp (YYYY-MM-DD HH:MM:SS)
String currentDateTime() {
    char buffer[25];
    sprintf(buffer,
            "%04d-%02d-%02d %02d:%02d:%02d",
            year(),
            month(),
            day(),
            hour(),
            minute(),
            second());
    return String(buffer);
}

// Get date string only (YYYY-MM-DD)
String currentDateString() {
    char buffer[11];
    sprintf(buffer, "%04d-%02d-%02d", year(), month(), day());
    return String(buffer);
}

// Open daily CSV file on internal SD, create and write header if needed
File openDailyPM25Log() {
    String currentDate = currentDateString();
    if (currentDate != lastDate) {
        lastDate = currentDate;
        String filenameStr =
                "/Box" + String(BOX_NUMBER) + "pm25_" + currentDate + ".csv";
        char filename[32];
        filenameStr.toCharArray(filename, sizeof(filename));
        if (!SD.exists(filename)) {
            File f = SD.open(filename, FILE_WRITE);
            if (f) {
                printHeader(f);
                f.close();
            }
        }
    }
    String filenameStr =
            "/Box" + String(BOX_NUMBER) + "pm25_" + lastDate + ".csv";
    char filename[32];
    filenameStr.toCharArray(filename, sizeof(filename));
    return SD.open(filename, FILE_WRITE);
}

// Same as above but for external SD using SdFat
FsFile openDailyPM25ExtLog() {
    String currentDate = currentDateString();
    String filenameStr =
            "/Box" + String(BOX_NUMBER) + "pm25_" + currentDate + ".csv";
    char filename[32];
    filenameStr.toCharArray(filename, sizeof(filename));

    // Only update lastDatext if the date changes
    if (currentDate != lastDatext) {
        lastDatext = currentDate;
    }

    // Check if file exists or is empty (new)
    bool writeHeader = false;

    if (!externalSD.exists(filename)) {
        writeHeader = true;
    }
    else {
        FsFile temp = externalSD.open(filename, O_READ);
        if (temp && temp.size() == 0) {
            writeHeader = true;
        }
        temp.close();
    }

    if (writeHeader) {
        if (externalFile.open(filename, O_WRITE | O_CREAT | O_TRUNC)) {
            printHeader(externalFile);
            externalFile.close();
            Serial.println("External SD: Header written");
        }
        else {
            Serial.println("External SD: Failed to create header file");
        }
    }
    else {
        //Serial.println("External SD: File exists and has content");
    }

    // Now open the file for appending
    externalFile.open(filename, O_RDWR | O_CREAT | O_AT_END);
    return externalFile;
}

// If external SD fails, retry every 5 seconds and blink LED as warning
void checkExternalSD() {
    static unsigned long lastBlink = 0;
    static bool ledState = false;
    static unsigned long lastRetry = 0;

    if (millis() - lastBlink >= 1000) {
        ledState = !ledState;
        digitalWrite(PANIC_LED, ledState);
        lastBlink = millis();
    }

    if (!externalOK && millis() - lastRetry >= 5000) {
        Serial.println("Retrying external SD card initialization...");
        externalOK = externalSD.begin(SdSpiConfig(
                externalChipSelect, SHARED_SPI, SD_SCK_MHZ(8), &SPI1));
        if (externalOK) {
            Serial.println("External SD card reinitialized.");
            externalFull = false;
            digitalWrite(PANIC_LED, LOW);
        }
        else {
            Serial.println("External SD card still not available.");
        }
        lastRetry = millis();
    }
}

void setup() {
    Serial.begin(9600);
    Wire.begin();
    pinMode(PANIC_LED, OUTPUT);
    digitalWrite(PANIC_LED, LOW);

    // Configure ADC for gas sensor input
    adc->adc0->setAveraging(32);  // Better noise performance
    adc->adc0->setResolution(10); // 10-bit resolution
    pinMode(ANALOG_PIN, INPUT);

    // Set RGB LED pins as outputs
    int rgbPins[] = {
            LED1_R, LED1_G, LED1_B, LED2_R, LED2_G, LED2_B, LED3_R, LED3_G, LED3_B};
    for (int i = 0; i < 9; i++) {
        pinMode(rgbPins[i], OUTPUT);
        digitalWrite(rgbPins[i], HIGH); // all off initially (common anode)
    }

    if (ENABLE_MOTION_SENSOR) {
        //Initialize DFRobot Motion Sensor

        radar.begin();
        radar.setSensorMode(eExitMode);

        //Min 30-2000cm; Max 240-2000cm; default trig = max
        radar.setDetectionRange(/*min*/ 30, /*max*/ 500, /*trig*/ 500);

        radar.setTrigSensitivity(0); //range 0-9
        radar.setKeepSensitivity(0); //range 0-9

        //trig 0.1s :unit 0.01s (0-2s); keep 2s :unit 0.5s (1s-1500s)
        radar.setDelay(/*trig*/ 10, /*keep*/ 4);

        radar.setPwm(/*pwm1*/ 100, /*pwm2*/ 0, /*timer*/ 10);
        radar.setIoPolaity(1);
    }

    // Initialize internal SD
    Serial.println("Initializing SD card...");
    internalOK = SD.begin(BUILTIN_SDCARD);
    if (!internalOK) {
        Serial.println("SD card initialization failed!");
        internalFull = true;
    }
    else {
        Serial.println("SD card initialization done.");
    }

    // Initialize external SD
    Serial.println("Initializing external SD card...");
    externalOK = externalSD.begin(
            SdSpiConfig(externalChipSelect, SHARED_SPI, SD_SCK_MHZ(8), &SPI1));
    if (!externalOK) {
        Serial.println("External SD card initialization failed!");
        externalFull = true;
    }
    else {
        Serial.println("External SD card initialization done.");
    }

    // Sync RTC from Teensy hardware clock
    setSyncProvider(getTeensyTime);
    if (timeStatus() != timeSet) {
        Serial.println("Unable to sync with the RTC");
    }
    else {
        Serial.println("RTC has set the system time");
    }

    pm25Setup(); // Initialize PM2.5 sensors

    // Initialize BME688 via BSEC2
    bsecSensor sensorList[] = {
            BSEC_OUTPUT_IAQ,
            BSEC_OUTPUT_RAW_TEMPERATURE,
            BSEC_OUTPUT_RAW_PRESSURE,
            BSEC_OUTPUT_RAW_HUMIDITY,
            BSEC_OUTPUT_RAW_GAS,
            BSEC_OUTPUT_STABILIZATION_STATUS,
            BSEC_OUTPUT_RUN_IN_STATUS};
    if (!envSensor.begin(BME68X_I2C_ADDR_LOW, Wire))
        checkBsecStatus(envSensor);
    if (!envSensor.updateSubscription(
                sensorList, ARRAY_LEN(sensorList), BSEC_SAMPLE_RATE_LP))
        checkBsecStatus(envSensor);
    envSensor.attachCallback(newDataCallback);

    intervalStartMillis = millis();
    previousMicros = micros();
}

void loop() {
    // Optional time sync from PC
    if (Serial.available()) {
        time_t t = processSyncMessage();
        if (t != 0) {
            Teensy3Clock.set(t);
            setTime(t);
            Serial.println("Time updated from serial.");
        }
    }

    // Run BME688 sensor reading (non-blocking)
    if (!envSensor.run())
        checkBsecStatus(envSensor);

    // Retry external SD if failed previously
    if (internalFull || (!externalOK && externalFull)) {
        checkExternalSD();
        return;
    }

    // Integration of analog voltage using trapezoidal method
    unsigned long currentMicros = micros();
    float dt = (currentMicros - previousMicros) / 1e6;
    previousMicros = currentMicros;

    int raw = adc->adc0->analogRead(ANALOG_PIN);
    float voltage = (raw * ADC_REF_VOLTAGE) / (ADC_RESOLUTION - 1);
    integratedVoltage += voltage * dt;

    // Every 1 second, log all sensor values
    if (millis() - intervalStartMillis >= INTEGRATION_INTERVAL_MS) {
        intervalStartMillis = millis();

        auto data1 = PM25_AQI_Data{};
        auto data2 = PM25_AQI_Data{};
        setZero(data1);
        setZero(data2);
        if (pm25SleepWake() == PM25_WAKE) {
            aqi1.read(&data1);
            aqi2.read(&data2);
        }

        // Store PM2.5 reading into circular buffer
        pm25History[pm25Index] = data1.pm25_standard;
        pm25Index = (pm25Index + 1) % PM25_HISTORY_SIZE;
        if (pm25Count < PM25_HISTORY_SIZE)
            pm25Count++;

        // Compute 1-minute average
        uint32_t pm25Sum = 0;
        for (int i = 0; i < pm25Count; i++) {
            pm25Sum += pm25History[i];
        }
        float pm25Avg = (pm25Count > 0) ? (float)pm25Sum / pm25Count : NAN;
        String category = pm25Category(pm25Avg);
        Serial.print("1-min PM2.5 avg: ");
        Serial.print(pm25Avg);
        Serial.print(" µg/m³ — Category: ");
        Serial.println(category);

        // Motion detection
        if (ENABLE_MOTION_SENSOR && radar.motionDetection()) {
            dfMotion = true;
            Serial.println("Motion");
        }
        else {
            dfMotion = false;
        }

        // Turn off all LEDs initially
        setRGB(LED1_R, LED1_G, LED1_B, HIGH, HIGH, HIGH);
        setRGB(LED2_R, LED2_G, LED2_B, HIGH, HIGH, HIGH);
        setRGB(LED3_R, LED3_G, LED3_B, HIGH, HIGH, HIGH);

        // Set LEDs according to category
        if (category == "Good") {
            setRGB(LED1_R, LED1_G, LED1_B, HIGH, LOW, HIGH); // Green
        }
        else if (category == "Moderate") {
            setRGB(LED1_R, LED1_G, LED1_B, LOW, LOW, HIGH); // Yellow
        }
        else if (category == "Unhealthy_Sensitive") {
            setRGB(LED1_R, LED1_G, LED1_B, LOW, HIGH, HIGH); // Orange (red on, green off)
        }
        else if (category == "Unhealthy") {
            setRGB(LED1_R, LED1_G, LED1_B, LOW, HIGH, HIGH); // Red
        }
        else if (category == "Very_Unhealthy") {
            setRGB(LED1_R, LED1_G, LED1_B, LOW, HIGH, HIGH); // Red
            setRGB(LED2_R, LED2_G, LED2_B, LOW, HIGH, HIGH); // Red
        }
        else if (category == "Hazardous") {
            setRGB(LED1_R, LED1_G, LED1_B, LOW, HIGH, HIGH); // Red
            setRGB(LED2_R, LED2_G, LED2_B, LOW, HIGH, HIGH); // Red
            setRGB(LED3_R, LED3_G, LED3_B, LOW, HIGH, HIGH); // Red
        }

        // Log to internal SD
        if (internalOK) {
            internalFile = openDailyPM25Log();
            if (internalFile) {
                printRow(
                        internalFile,
                        dfMotion,
                        pm25Avg,
                        integratedVoltage,
                        envData,
                        data1,
                        data2);
                internalFile.close();
                Serial.println("Data recorded to internal SD card");
            }
            else {
                internalFull = true;
            }
        }

        // Log to external SD
        if (externalOK) {
            externalFile = openDailyPM25ExtLog();
            if (externalFile) {
                printRow(
                        externalFile,
                        dfMotion,
                        pm25Avg,
                        integratedVoltage,
                        envData,
                        data1,
                        data2);
                externalFile.close();
                Serial.println("Data recorded to external SD card");
            }
            else {
                externalFull = true;
                externalOK = false;
            }
        }
    }
    integratedVoltage = 0.0; // Reset integration for next interval
}
