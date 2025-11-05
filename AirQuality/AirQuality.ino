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

#include <ADC.h>
#include <Adafruit_PM25AQI.h>
#include <DFRobot_C4001.h>
#include <SD.h>
#include <SPI.h>
#include <SdFat.h>
#include <TimeLib.h>
#include <Wire.h>
#include <bsec2.h>

template <typename T>
void setZero(T& var) {
    static_assert(std::is_trivially_copyable_v<T> == true);
    memset(&var, 0, sizeof(T));
}

// Initialize ADC library for analog gas sensor (MiCS5524)
ADC *adc = new ADC();

// RGB common anode LED pin definitions (active LOW)
const int LED1_R = 10, LED1_G = 11, LED1_B = 12;
const int LED2_R = 16, LED2_G = 8,  LED2_B = 9;
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

// Internal SD card
File myFile;
FsFile extFile;
bool sdFull = false;
bool sdOK = false;

// External SD card
const int chipSelect_ext = 0;
SdFat SD_ext;
bool sdExtOK = false;
bool sdExtFull = false;
bool triedReinit = false;

// PM2.5 sensor (Adafruit PM5003 or equivalent)
constexpr auto PM25_SET_PIN_1 = 25; // sensor 1 control pin
constexpr auto PM25_SET_PIN_2 = 24; // sensor 2 control pin
constexpr auto PM25_SLEEP = LOW;
constexpr auto PM25_WAKE = HIGH;
Adafruit_PM25AQI aqi1 = Adafruit_PM25AQI();
Adafruit_PM25AQI aqi2 = Adafruit_PM25AQI();

//PM2.5 running average - Circular buffer for 1-minute (60 samples)
const int PM25_HISTORY_SIZE = 60;
uint16_t pm25History[PM25_HISTORY_SIZE] = {0};  // Holds last 60 PM2.5 values
int pm25Index = 0; // Points to next insert location
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
    if (avg <= PM25_GOOD) return "Good";
    else if (avg <= PM25_MODERATE) return "Moderate";
    else if (avg <= PM25_UNHEALTHY_SENSITIVE) return "Unhealthy_Sensitive";
    else if (avg <= PM25_UNHEALTHY) return "Unhealthy";
    else if (avg <= PM25_VERY_UNHEALTHY) return "Very_Unhealthy";
    else return "Hazardous";
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
#define TIME_HEADER  "T"
const unsigned long DEFAULT_TIME = 1357041600; // Jan 1, 2013 fallback

// BME688 + BSEC2 sensor variables
#define PANIC_LED   LED_BUILTIN
#define ERROR_DUR   1000
Bsec2 envSensor; // BSEC2 driver for BME688
// Output values from BSEC2
float bmeIaq = NAN, bmeIaqAcc = NAN, bmeTemp = NAN, bmePres = NAN;
float bmeHum = NAN, bmeGas = NAN, bmeStab = NAN, bmeRunIn = NAN;

// Flash error code using LED
void errLeds() {
  while(1) {
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
  } else if (bsec.status > BSEC_OK) {
    Serial.println("BSEC warning code : " + String(bsec.status));
  }
  if (bsec.sensor.status < BME68X_OK) {
    Serial.println("BME68X error code : " + String(bsec.sensor.status));
    errLeds();
  } else if (bsec.sensor.status > BME68X_OK) {
    Serial.println("BME68X warning code : " + String(bsec.sensor.status));
  }
}

// Callback for new data from BSEC2
void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec) {
  if (!outputs.nOutputs) return;
  for (uint8_t i = 0; i < outputs.nOutputs; i++) {
    const bsecData output  = outputs.output[i];
    switch (output.sensor_id) {
      case BSEC_OUTPUT_IAQ: bmeIaq = output.signal; bmeIaqAcc = output.accuracy; break;
      case BSEC_OUTPUT_RAW_TEMPERATURE: bmeTemp = output.signal; break;
      case BSEC_OUTPUT_RAW_PRESSURE: bmePres = output.signal; break;
      case BSEC_OUTPUT_RAW_HUMIDITY: bmeHum = output.signal; break;
      case BSEC_OUTPUT_RAW_GAS: bmeGas = output.signal; break;
      case BSEC_OUTPUT_STABILIZATION_STATUS: bmeStab = output.signal; break;
      case BSEC_OUTPUT_RUN_IN_STATUS: bmeRunIn = output.signal; break;
    }
  }
}

// Get RTC time from Teensy
time_t getTeensyTime() {
  return (time_t)Teensy3Clock.get();
}

// Parse incoming serial time sync message (used in setup)
unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  if (Serial.find(TIME_HEADER)) {
    pctime = Serial.parseInt();
    if (pctime >= DEFAULT_TIME) return pctime;
  }
  return 0L;
}

// Get full timestamp (YYYY-MM-DD HH:MM:SS)
String currentDateTime() {
  char buffer[25];
  sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d", year(), month(), day(), hour(), minute(), second());
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
    String filenameStr = "/Box" + String(BOX_NUMBER) + "pm25_" + currentDate + ".csv";
    char filename[32];
    filenameStr.toCharArray(filename, sizeof(filename));
    if (!SD.exists(filename)) {
      File f = SD.open(filename, FILE_WRITE);
      if (f) {
        f.println("Timestamp,Motion,PM2.5_1MinAvg,PM1.0_std,PM2.5_std,PM10_std,PM1.0_env,PM2.5_env,PM10_env,"
                      "P>0.3um,P>0.5um,P>1.0um,P>2.5um,P>5.0um,P>10um,AQI_PM2.5,AQI_PM10,MiCS5524-Vs,"
                      "IAQ,IAQ_Accuracy,Temp,Pressure,Humidity,Gas,Stab_Status,RunIn_Status,"
                      "2-PM1.0_std,2-PM2.5_std,2-PM10_std,2-PM1.0_env,2-PM2.5_env,2-PM10_env,"
                      "2-P>0.3um,2-P>0.5um,2-P>1.0um,2-P>2.5um,2-P>5.0um,2-P>10um,2-AQI_PM2.5,2-AQI_PM10");
        f.close();
      }
    }
  }
  String filenameStr =  "/Box" + String(BOX_NUMBER) + "pm25_" + lastDate + ".csv";
  char filename[32];
  filenameStr.toCharArray(filename, sizeof(filename));
  return SD.open(filename, FILE_WRITE);
}

// Same as above but for external SD using SdFat
FsFile openDailyPM25ExtLog() {
  String currentDate = currentDateString();
  String filenameStr =  "/Box" + String(BOX_NUMBER) + "pm25_" + currentDate + ".csv";
  char filename[32];
  filenameStr.toCharArray(filename, sizeof(filename));

  // Only update lastDatext if the date changes
  if (currentDate != lastDatext) {
    lastDatext = currentDate;
  }

  // Check if file exists or is empty (new)
  bool writeHeader = false;

  if (!SD_ext.exists(filename)) {
    writeHeader = true;
  } else {
    FsFile temp = SD_ext.open(filename, O_READ);
    if (temp && temp.size() == 0) {
      writeHeader = true;
    }
    temp.close();
  }

  if (writeHeader) {
    if (extFile.open(filename, O_WRITE | O_CREAT | O_TRUNC)) {
      extFile.println("Timestamp,Motion,PM2.5_1MinAvg,PM1.0_std,PM2.5_std,PM10_std,PM1.0_env,PM2.5_env,PM10_env,"
                      "P>0.3um,P>0.5um,P>1.0um,P>2.5um,P>5.0um,P>10um,AQI_PM2.5,AQI_PM10,MiCS5524-Vs,"
                      "IAQ,IAQ_Accuracy,Temp,Pressure,Humidity,Gas,Stab_Status,RunIn_Status,"
                      "2-PM1.0_std,2-PM2.5_std,2-PM10_std,2-PM1.0_env,2-PM2.5_env,2-PM10_env,"
                      "2-P>0.3um,2-P>0.5um,2-P>1.0um,2-P>2.5um,2-P>5.0um,2-P>10um,2-AQI_PM2.5,2-AQI_PM10");
      extFile.close();
      Serial.println("External SD: Header written");
    } else {
      Serial.println("External SD: Failed to create header file");
    }
  } else {
    //Serial.println("External SD: File exists and has content");
  }

  // Now open the file for appending
  extFile.open(filename, O_RDWR | O_CREAT | O_AT_END);
  return extFile;
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

  if (!sdExtOK && millis() - lastRetry >= 5000) {
    Serial.println("Retrying external SD card initialization...");
    sdExtOK = SD_ext.begin(SdSpiConfig(chipSelect_ext, SHARED_SPI, SD_SCK_MHZ(8), &SPI1));
    if (sdExtOK) {
      Serial.println("External SD card reinitialized.");
      sdExtFull = false;
      digitalWrite(PANIC_LED, LOW);
    } else {
      Serial.println("External SD card still not available.");
    }
    lastRetry = millis();
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  radar.begin();
  pinMode(PANIC_LED, OUTPUT);
  digitalWrite(PANIC_LED, LOW);

  // Configure ADC for gas sensor input
  adc->adc0->setAveraging(32);   // Better noise performance
  adc->adc0->setResolution(10);  // 10-bit resolution
  pinMode(ANALOG_PIN, INPUT);

  // Set RGB LED pins as outputs
  int rgbPins[] = {LED1_R, LED1_G, LED1_B, LED2_R, LED2_G, LED2_B, LED3_R, LED3_G, LED3_B};
  for (int i = 0; i < 9; i++) {
  pinMode(rgbPins[i], OUTPUT);
  digitalWrite(rgbPins[i], HIGH); // all off initially (common anode)
}

  //Initialize DFRobot Motion Sensor
  radar.setSensorMode(eExitMode);
  radar.setDetectionRange(/*min*/30, /*max*/500, /*trig*/500); //Min 30-2000cm; Max 240-2000cm; default trig = max
  radar.setTrigSensitivity(0); //range 0-9
  radar.setKeepSensitivity(0);  //range 0-9
  radar.setDelay(/*trig*/10, /*keep*/4); //trig 0.1s :unit 0.01s (0-2s); keep 2s :unit 0.5s (1s-1500s)
  radar.setPwm(/*pwm1*/100, /*pwm2*/0, /*timer*/10);
  radar.setIoPolaity(1);

  // Initialize internal SD
  Serial.println("Initializing SD card...");
  sdOK = SD.begin(BUILTIN_SDCARD);
  if (!sdOK) {
    Serial.println("SD card initialization failed!");
    sdFull = true;
  } else {
    Serial.println("SD card initialization done.");
  }

  // Initialize external SD
  Serial.println("Initializing external SD card...");
  sdExtOK = SD_ext.begin(SdSpiConfig(chipSelect_ext, SHARED_SPI, SD_SCK_MHZ(8), &SPI1));
  if (!sdExtOK) {
    Serial.println("External SD card initialization failed!");
    sdExtFull = true;
  } else {
    Serial.println("External SD card initialization done.");
  }

  // Sync RTC from Teensy hardware clock
  setSyncProvider(getTeensyTime);
  if (timeStatus() != timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
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
    BSEC_OUTPUT_RUN_IN_STATUS
  };
  if (!envSensor.begin(BME68X_I2C_ADDR_LOW, Wire)) checkBsecStatus(envSensor);
  if (!envSensor.updateSubscription(sensorList, ARRAY_LEN(sensorList), BSEC_SAMPLE_RATE_LP)) checkBsecStatus(envSensor);
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
  if (!envSensor.run()) checkBsecStatus(envSensor);

  // Retry external SD if failed previously
  if (sdFull || (!sdExtOK && sdExtFull)) {
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
    if (pm25Count < PM25_HISTORY_SIZE) pm25Count++;

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
    if(radar.motionDetection()){
      dfMotion = true;
      Serial.println("Motion");
    } else {
      dfMotion = false;
    }

    // Turn off all LEDs initially
    setRGB(LED1_R, LED1_G, LED1_B, HIGH, HIGH, HIGH);
    setRGB(LED2_R, LED2_G, LED2_B, HIGH, HIGH, HIGH);
    setRGB(LED3_R, LED3_G, LED3_B, HIGH, HIGH, HIGH);

    // Set LEDs according to category
    if (category == "Good") {
      setRGB(LED1_R, LED1_G, LED1_B, HIGH, LOW, HIGH);       // Green
    } else if (category == "Moderate") {
      setRGB(LED1_R, LED1_G, LED1_B, LOW, LOW, HIGH);        // Yellow
    } else if (category == "Unhealthy_Sensitive") {
      setRGB(LED1_R, LED1_G, LED1_B, LOW, HIGH, HIGH);       // Orange (red on, green off)
    } else if (category == "Unhealthy") {
      setRGB(LED1_R, LED1_G, LED1_B, LOW, HIGH, HIGH);       // Red
    } else if (category == "Very_Unhealthy") {
      setRGB(LED1_R, LED1_G, LED1_B, LOW, HIGH, HIGH);       // Red
      setRGB(LED2_R, LED2_G, LED2_B, LOW, HIGH, HIGH);       // Red
    } else if (category == "Hazardous") {
      setRGB(LED1_R, LED1_G, LED1_B, LOW, HIGH, HIGH);       // Red
      setRGB(LED2_R, LED2_G, LED2_B, LOW, HIGH, HIGH);       // Red
      setRGB(LED3_R, LED3_G, LED3_B, LOW, HIGH, HIGH);       // Red
    }


    // Log to internal SD
    if (sdOK) {
      myFile = openDailyPM25Log();
      if (myFile) {
        myFile.print(currentDateTime()); myFile.print(",");
        myFile.print(dfMotion); myFile.print(",");
        myFile.print(pm25Avg,2); myFile.print(",");
        //Write 1st pm2.5 sensor data
        myFile.print(data1.pm10_standard); myFile.print(",");
        myFile.print(data1.pm25_standard); myFile.print(",");
        myFile.print(data1.pm100_standard); myFile.print(",");
        myFile.print(data1.pm10_env); myFile.print(",");
        myFile.print(data1.pm25_env); myFile.print(",");
        myFile.print(data1.pm100_env); myFile.print(",");
        myFile.print(data1.particles_03um); myFile.print(",");
        myFile.print(data1.particles_05um); myFile.print(",");
        myFile.print(data1.particles_10um); myFile.print(",");
        myFile.print(data1.particles_25um); myFile.print(",");
        myFile.print(data1.particles_50um); myFile.print(",");
        myFile.print(data1.particles_100um); myFile.print(",");
        myFile.print(data1.aqi_pm25_us); myFile.print(",");
        myFile.print(data1.aqi_pm100_us); myFile.print(",");
        //Write Mics analog data
        myFile.print(integratedVoltage, 6); myFile.print(",");
        //Write BME688 data
        myFile.print(bmeIaq); myFile.print(",");
        myFile.print(bmeIaqAcc); myFile.print(",");
        myFile.print(bmeTemp); myFile.print(",");
        myFile.print(bmePres); myFile.print(",");
        myFile.print(bmeHum); myFile.print(",");
        myFile.print(bmeGas); myFile.print(",");
        myFile.print(bmeStab); myFile.print(",");
        myFile.print(bmeRunIn); myFile.print(",");
        //Write 2nd pm2.5 sensor data
        myFile.print(data2.pm10_standard); myFile.print(",");
        myFile.print(data2.pm25_standard); myFile.print(",");
        myFile.print(data2.pm100_standard); myFile.print(",");
        myFile.print(data2.pm10_env); myFile.print(",");
        myFile.print(data2.pm25_env); myFile.print(",");
        myFile.print(data2.pm100_env); myFile.print(",");
        myFile.print(data2.particles_03um); myFile.print(",");
        myFile.print(data2.particles_05um); myFile.print(",");
        myFile.print(data2.particles_10um); myFile.print(",");
        myFile.print(data2.particles_25um); myFile.print(",");
        myFile.print(data2.particles_50um); myFile.print(",");
        myFile.print(data2.particles_100um); myFile.print(",");
        myFile.print(data2.aqi_pm25_us); myFile.print(",");
        myFile.println(data2.aqi_pm100_us); 
        myFile.close();
        Serial.println("Data recorded to internal SD card");
      } else {
        sdFull = true;
      }
    }

    // Log to external SD
    if (sdExtOK) {
      extFile = openDailyPM25ExtLog();
      if (extFile) {
        extFile.print(currentDateTime()); extFile.print(",");
        extFile.print(dfMotion); extFile.print(",");
        extFile.print(pm25Avg,2); extFile.print(",");
        //Write 1st pm2.5 sensor data
        extFile.print(data1.pm10_standard); extFile.print(",");
        extFile.print(data1.pm25_standard); extFile.print(",");
        extFile.print(data1.pm100_standard); extFile.print(",");
        extFile.print(data1.pm10_env); extFile.print(",");
        extFile.print(data1.pm25_env); extFile.print(",");
        extFile.print(data1.pm100_env); extFile.print(",");
        extFile.print(data1.particles_03um); extFile.print(",");
        extFile.print(data1.particles_05um); extFile.print(",");
        extFile.print(data1.particles_10um); extFile.print(",");
        extFile.print(data1.particles_25um); extFile.print(",");
        extFile.print(data1.particles_50um); extFile.print(",");
        extFile.print(data1.particles_100um); extFile.print(",");
        extFile.print(data1.aqi_pm25_us); extFile.print(",");
        extFile.print(data1.aqi_pm100_us); extFile.print(",");
        //Write Mics analog data
        extFile.print(integratedVoltage, 6); extFile.print(",");
        //Write BME688 data
        extFile.print(bmeIaq); extFile.print(",");
        extFile.print(bmeIaqAcc); extFile.print(",");
        extFile.print(bmeTemp); extFile.print(",");
        extFile.print(bmePres); extFile.print(",");
        extFile.print(bmeHum); extFile.print(",");
        extFile.print(bmeGas); extFile.print(",");
        extFile.print(bmeStab); extFile.print(",");
        extFile.print(bmeRunIn); extFile.print(",");
        //Write 2nd pm2.5 sensor data
        extFile.print(data2.pm10_standard); extFile.print(",");
        extFile.print(data2.pm25_standard); extFile.print(",");
        extFile.print(data2.pm100_standard); extFile.print(",");
        extFile.print(data2.pm10_env); extFile.print(",");
        extFile.print(data2.pm25_env); extFile.print(",");
        extFile.print(data2.pm100_env); extFile.print(",");
        extFile.print(data2.particles_03um); extFile.print(",");
        extFile.print(data2.particles_05um); extFile.print(",");
        extFile.print(data2.particles_10um); extFile.print(",");
        extFile.print(data2.particles_25um); extFile.print(",");
        extFile.print(data2.particles_50um); extFile.print(",");
        extFile.print(data2.particles_100um); extFile.print(",");
        extFile.print(data2.aqi_pm25_us); extFile.print(",");
        extFile.println(data2.aqi_pm100_us);
        extFile.close();
        Serial.println("Data recorded to external SD card");
      } else {
        sdExtFull = true;
        sdExtOK = false;
      }
    }
    integratedVoltage = 0.0;  // Reset integration for next interval
  }
}
