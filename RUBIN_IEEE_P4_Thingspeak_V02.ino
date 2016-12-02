/*
   Mike Rubin Project 4 GA

   ThingSpeak Weather Station 
   Adapted from IEEE_IOT_Sketch04_Thingspeak_V02


   05 October 2016

   Hardware connections NodeMCU to Level Converter & LCD:
     Signal   NodeMCU       Lvl Conv      LCD
     ------   -------       --------      ---
     5V ----- VIN --------------- HV ---- VCC
     3.3V --- 3V3 --------- LV  |
     GND ---- GND --------- GND | GND --- GND
     SCL ---- D1 (GPIO5) -- LV1 | HV1 --- SCL
     SDA ---- D2 (GPIO4) -- LV2 | HV2 --- SDA

   Hardware connections from Level Converter to Sensors:
     Signal   Lvl Conv      BME280        BH1750
     -----    --------      ------        ------
     3.3V --- LV ---------- 3.3 --------- VCC
     GND ---- GND --------- GND --------- GND
     SCL ---- LV1 --------- SCL --------- SCL
     SDA ---- LV2 --------- SDA --------- SDA

   Units Switch connected between NodeMCU pin D7 (GPIO13) and GND

   Set serial monitor to 115,200 baud
*/

/* Libraries that can be installed from menu Sketch | Include Libraries | Manage Libraries...
      ArduinoThread
      Liquidcrystal_I2C by Frank de Brabander

    Libraries that must be downloaded are:
      BH1750
      RunningAverage
*/

#include <Wire.h>                // [built-in] I2C bus (part of IDE)
#include <LiquidCrystal_I2C.h>   // https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library
#include <BME280.h>              // [manager] https://github.com/finitespace/BME280
#include <BH1750.h>              // https://github.com/claws/BH1750
#include <ESP8266WiFi.h>         // [built-in] ESP8266 WiFi
#include <RunningAverage.h>      // https://github.com/RobTillaart/Arduino/tree/master/libraries/RunningAverage
#include <Thread.h>              // [manager] https://github.com/ivanseidel/ArduinoThread
#include <ThreadController.h>    // [manager] part of ArduinoThread library

// ************** LOGON INFORMATION **********************

// Logon info for your wireless access point
//const char ssid[] = "RUBIN1";      // enter your WiFi SSID
//const char password[] = "MICHAELnELLEN";  // enter your WiFi password
const char ssid[] = "GA-Guest";      // enter your WiFi SSID
const char password[] = "yellowpencil";  // enter your WiFi password
// API write key for your ThingSpeak channel
String apiWriteKey = "E89T2QNU9DYFPM8J";  // ENTER YOUR API KEY
const char IOT_SERVER[] = "api.thingspeak.com";

// *******************************************************
// **************** PROGRAM PARAMETERS *******************
// *******************************************************
const float STATION_ELEV = 125.0;               // Altitude of 12504 Chasbarb Terrace Herndon VA RUBIN HOME
const int LCD_ADDR = 0x27;                      // backlight: yellow = 0x27, blue = 0x3F
const int UNITS_PIN = 13;                       // GPIO13 = NodeMCU pin D7
const long UPDATE_INTERVAL_SENSORS = 2000;
const long UPDATE_LCD_INTERVAL = 3000;
const long UPDATE_INTERVAL_THINGSPEAK = 30000;  //  Set to every 30 seconds ThingSpeak update must be > 15 seconds
const long UPDATE_INTERVAL_BAROMETER_TREND = 5 * 60 * 1000;                   // 5 minute average for barometer trend
const int CB_SIZE = (3 * 60 * 60 * 1000) / UPDATE_INTERVAL_BAROMETER_TREND;   // one measurement every 5 minutes for 3 hours
const int SENSOR_SAMPLES = 30;  // arbitrary number of readings to average
//const int SENSOR_SAMPLES = (5 * 60 * 1000) / UPDATE_INTERVAL_SENSORS;         // average measurements over 5 minutes
const long SPLASHSCREEN_DELAY = 4000;

// *******************************************************
// ******************* GLOBALS ***************************
// *******************************************************
const int ARROW_FALLING_VERY_RAPIDLY = 0;
const int ARROW_FALLING_QUICKLY      = 1;
const int ARROW_FALLING              = 2;
const int ARROW_FALLING_SLOWLY       = 3;
const int ARROW_STEADY               = 0x7E; // built in right arrow
const int ARROW_RISING_SLOWLY        = 4;
const int ARROW_RISING               = 5;
const int ARROW_RISING_QUICKLY       = 6;
const int ARROW_RISING_VERY_RAPIDLY  = 7;
// LEDs are connected but not used in this sketch
const int LED_D5 = 14;
const int LED_D6 = 12;


const float HPA_TO_INHG = 0.0295299830714;  // hPa to inHg pressure

// structure to hold sensor measurements
struct sensorData
{
  float stationPressure;
  float seaLevelPressure;
  float temperature;
  float humidity;
  unsigned int lightIntensity;
  float voltage;
} rawData, smoothData; // declare struct variables

// structure to hold barometric trend
struct trend
{
  float delta;        // 3-hour difference in pressure (hPa)
  int symbol;         // index to corresponding trend arrow
} barometricTrend;    // declare struct variable

// ******************* CUSTOM LCD CHARACTERS *************
// these arrays define graphic arrows for barometric tendency
byte fallingVeryRapidly[8] = {0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b10101, 0b01110, 0b00100};
byte fallingQuickly[8]     = {0b10000, 0b10000, 0b10000, 0b10000, 0b01000, 0b00101, 0b00011, 0b00111};
byte falling[8]            = {0b10000, 0b10000, 0b01000, 0b01000, 0b00100, 0b00101, 0b00011, 0b00111};
byte fallingSlowly[8]      = {0b00000, 0b00000, 0b00000, 0b10000, 0b01000, 0b00101, 0b00011, 0b00111};
//   steady                  arrow is built-in character 0x7E
byte risingSlowly[8]       = {0b00111, 0b00011, 0b00101, 0b01000, 0b10000, 0b00000, 0b00000, 0b00000};
byte rising[8]             = {0b00111, 0b00011, 0b00101, 0b00100, 0b01000, 0b01000, 0b10000, 0b10000};
byte risingQuickly[8]      = {0b00111, 0b00011, 0b00101, 0b01000, 0b10000, 0b10000, 0b10000, 0b10000};
byte risingVeryRapidly[8]  = {0b00100, 0b01110, 0b10101, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100};

// ********* INSTANTIATE OBJECTS *************************
// usage: LiquidCrystal_I2C your_object_name(I2C_address, columns, rows);
// run IEEE_IoT_I2C_scanner if address is not known
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

// Create instances of the sensor objects
BME280 myBME280;        // barometric pressure / temperature / humidity sensor
BH1750 myBH1750;           // light intensity sensor

// instantiate the WiFi connection
WiFiClient client;

// instantiate Threads for various tasks
Thread sensorsThread = Thread();
Thread lcdThread = Thread();
Thread thingspeakThread = Thread();
Thread barometricTrendThread = Thread();

// instantiate a container to hold all threads
ThreadController myThreads = ThreadController();

// instantiate objects to hold smoothed values
RunningAverage avgStationPressure(SENSOR_SAMPLES);
RunningAverage avgSeaLevelPressure(SENSOR_SAMPLES);
RunningAverage avgTemperature(SENSOR_SAMPLES);
RunningAverage avgHumidity(SENSOR_SAMPLES);

// *******************************************************
// ********************* SETUP ***************************
// *******************************************************
void setup()
{
  Serial.begin(115200);             // initialize the serial port
  lcd.begin();                      // initialize the lcd
  createArrows();                   // create graphic arrows for barom. tendency
  splashScreen();                   // show the splash screen
  logonToRouter();                  // logon to local Wi-Fi
  myBME280.begin();                 // initialize BME280 pressure/temperature/humidity
  myBH1750.begin();                 // initialize BH1750 light sensor
  pinMode(UNITS_PIN, INPUT_PULLUP); // pull NodeMCU pin HIGH for units selection switch
  rawData = readSensors();          // do a single sensor read to initialize baraometric trend
  barometricTrend = baroTrend(rawData.seaLevelPressure);

// fix to keep LEDs dark - they are not used in this sketch
  pinMode(LED_D5, OUTPUT);
  pinMode(LED_D6, OUTPUT);
  digitalWrite(LED_D5, LOW);
  digitalWrite(LED_D6, LOW);



  // associate each Thread with the callback function and set the interval
  sensorsThread.onRun(updateSensors);
  sensorsThread.setInterval(UPDATE_INTERVAL_SENSORS);

  lcdThread.onRun(updateLCD);
  lcdThread.setInterval(UPDATE_LCD_INTERVAL);

  thingspeakThread.onRun(updateThingspeak);
  thingspeakThread.setInterval(UPDATE_INTERVAL_THINGSPEAK);

  barometricTrendThread.onRun(updateBarometricTrend);
  barometricTrendThread.setInterval(UPDATE_INTERVAL_BAROMETER_TREND);

  // add the Threads to the Thread Controller
  // the '&' operator provides the address of the callback function
  // as required by the ArduinoThread library
  myThreads.add(&sensorsThread);
  myThreads.add(&lcdThread);
  myThreads.add(&thingspeakThread);
  myThreads.add(&barometricTrendThread);
} //setup()

// *******************************************************
// ******************** LOOP *****************************
// *******************************************************
void loop()
{
  // the ThreadController handles all the scheduled tasks
  myThreads.run();
} // loop()

// *******************************************************
// ******** CALLBACK FUNCTIONS (for threads) *************
// *******************************************************

// read sensors and smooth the data
// this is the callback function for sensorsThread
void updateSensors()
{
  rawData = readSensors();          // load all sensor data into rawData struct

  // average the sensor data
  avgStationPressure.addValue(rawData.stationPressure);
  avgSeaLevelPressure.addValue(rawData.seaLevelPressure);
  avgTemperature.addValue(rawData.temperature);
  avgHumidity.addValue(rawData.humidity);

  // fill the data struct with the averaged sensor data
  smoothData.stationPressure = avgStationPressure.getAverage();
  smoothData.seaLevelPressure = avgSeaLevelPressure.getAverage();
  smoothData.temperature = avgTemperature.getAverage();
  smoothData.humidity = avgHumidity.getAverage();
  smoothData.lightIntensity = rawData.lightIntensity;  // don't average the light intensity
} //updateSensors

// *******************************************************
// read sensors and print raw and smoothed
// data to the LCD and the serial port
// this is the callback function for lcdThread
void updateLCD() 
{
  // read the units switch
  bool units = readUnits(UNITS_PIN);

  printToLCD(smoothData, barometricTrend, units);
  printToSerialPort(rawData, smoothData, units);
} // updateLCD

// *******************************************************
// post data to ThingSpeak and print to the LCD
// this is the callback function for thingspeakThread
void updateThingspeak() 
{
  // read the units switch
  bool units = readUnits(UNITS_PIN);

  printToLCD(smoothData, barometricTrend, units);
  postToThingSpeak(smoothData);
} // updateThinkspeak

// *******************************************************
// determine the 3-hour trend in barometric pressure
// this is the callback function for barometricTrendThread
void updateBarometricTrend() 
{
  barometricTrend = baroTrend(smoothData.seaLevelPressure);
} //updateBarometricTrend()

// *******************************************************
// ************** OTHER FUNCTIONS ************************
// *******************************************************

// show the splash screen for a set number of seconds
void splashScreen() 
{
  lcd.backlight();                     // turn on backlight
  lcd.setCursor(0, 0);                 //
  lcd.print("WDI-12 Project 4");
  lcd.setCursor(1, 1);                 // indent to center text
  lcd.print("IoT Wx Station");
  delay(SPLASHSCREEN_DELAY);          // display for x seconds
  lcd.clear();                         // clear screen
} // splashScreen()

// *******************************************************
// read all sensors into a sensors struct
sensorData readSensors() 
{
  sensorData sensorData;  // temporary variable to hold readings

  // Pressure Parameters: (float& pressure, float& temp, float& humidity, bool metric, int pressureUnit)
  byte pressureUnit = 1; // unit: 0 = Pa, 1 = hPa, 2 = Hg, 3 = atm, 4 = bar, 5 = torr, 6 = N/m^2, 7 = psi

  // read each BME280 value individually for clarity of code
  // or read them all at once with the following line
  // myBME280.ReadData(sensorData.stationPressure, sensorData.temperature, sensorData.humidity, true, pressureUnit);

  sensorData.stationPressure = myBME280.ReadPressure(pressureUnit);
  sensorData.temperature = myBME280.ReadTemperature();
  sensorData.humidity = myBME280.ReadHumidity();

  // calculate the sealevel (relative) pressure
  sensorData.seaLevelPressure = calculateSeaLevelPressure(sensorData.temperature, sensorData.stationPressure, STATION_ELEV);

  // read light level in lux
  sensorData.lightIntensity = myBH1750.readLightLevel();

  // read analog voltage from the Analog to Digital Converter
  // on NodeMCU this is 0 - 1023 for voltages 0 to 3.3V
  sensorData.voltage = 3.3 * analogRead(A0) / 1023.0;

  return sensorData;
} // readSensors()

// *******************************************************
// print raw and smoothed data to the serial port
void printToSerialPort(sensorData dataRaw, sensorData dataSmooth, bool metricUnits) 
{
  // convert display data to Imperial if requested by user
  if (metricUnits == false) {
    dataRaw.temperature = dataRaw.temperature * 1.8 + 32;
    dataRaw.stationPressure = dataRaw.stationPressure * HPA_TO_INHG;
    dataRaw.seaLevelPressure = dataRaw.seaLevelPressure * HPA_TO_INHG;
    dataSmooth.temperature = dataSmooth.temperature * 1.8 + 32;
    dataSmooth.stationPressure = dataSmooth.stationPressure * HPA_TO_INHG;
    dataSmooth.seaLevelPressure = dataSmooth.seaLevelPressure * HPA_TO_INHG;
  }

  // '\t' is the C++ escape sequence for tab
  // header line
  Serial.println("\tTemp:\tHumd:\tPsta:\tPsea:\tLux:\tVolt:");

  // first line - raw data
  Serial.print("RAW\t");
  Serial.print(dataRaw.temperature, 2);
  Serial.print("\t");
  Serial.print(dataRaw.humidity, 2);
  Serial.print("\t");
  Serial.print(dataRaw.stationPressure, 2);
  Serial.print("\t");
  Serial.print(dataRaw.seaLevelPressure, 2);
  Serial.print("\t");
  Serial.print(dataRaw.lightIntensity);
  Serial.print("\t");
  Serial.println(dataRaw.voltage, 2);

  // second line - smoothed data
  Serial.print("SMTH\t");
  Serial.print(dataSmooth.temperature, 2);
  Serial.print("\t");
  Serial.print(dataSmooth.humidity, 2);
  Serial.print("\t");
  Serial.print(dataSmooth.stationPressure, 2);
  Serial.print("\t");
  Serial.print(dataSmooth.seaLevelPressure, 2);
  Serial.println("");
  Serial.println("----------------------------------------------------");
} // printToSerialPort()

// *******************************************************
// print selected data to LCD display
void printToLCD(sensorData data, trend baroTrend, bool metricUnits) 
{
  char temperatureSymbol = 'C';       // default is Celsius
  char pressureUnit[5] = "hPa";
  if (metricUnits == false)           // user wants Imperial units
  {
    data.temperature = data.temperature * 1.8 + 32;
    data.seaLevelPressure = data.seaLevelPressure * HPA_TO_INHG;
    temperatureSymbol = 'F';
    strncpy(pressureUnit, "inHg", 5);
  }

  // clears screen faster than lcd.clear() with less blink
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 0);
  lcd.print(data.temperature, 1);
  lcd.print((char)0xDF);              // prints degree symbol on LCD
  lcd.print(temperatureSymbol);
  lcd.print(' ');
  lcd.print(data.humidity, 0);
  lcd.print("% ");
  lcd.print(data.lightIntensity);

  // move to next line
  lcd.setCursor(0, 1);
  lcd.print(data.seaLevelPressure, 1);
  lcd.print(pressureUnit);
  lcd.print(' ');

  // show barometric trend arrow
  // use the write() command for graphic values
  lcd.write(baroTrend.symbol);
  lcd.print(' ');
  lcd.print(baroTrend.delta, 1);
} // printToLCD()

// *******************************************************
// logon to your Wi-Fi router
void logonToRouter()
{
  int count = 0;
  // log on to router
  lcd.clear();
  lcd.print("Connecting to ");
  lcd.setCursor(0, 1);
  lcd.print(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    count++;
    delay(500);
    lcd.print(".");
    if (count > 15) {
      lcd.setCursor(0, 0);
      lcd.print("Exit code:      ");
      lcd.setCursor(11, 0);
      lcd.print(WiFi.status());
      delay(5000);
      ESP.reset();
    }
  }
  lcd.clear();
  lcd.print("Connected w/IP:");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP().toString());
  delay(SPLASHSCREEN_DELAY);
} // logonToRouter()

// *******************************************************
// post data to your ThingSpeak account
// only metric values are posted
void postToThingSpeak(sensorData data) 
{
  // assemble and post the data
  if (client.connect(IOT_SERVER, 80) == true)
  {
    // declare dataString as a String and initialize with the apiWriteKey
    String dataString = apiWriteKey;

    // cocatenate each field onto the end of dataString
    dataString += "&field1=";
    dataString += String(data.temperature);
    dataString += "&field2=";
    dataString += String(data.humidity);
    dataString += "&field3=";
    dataString += String(data.stationPressure);
    dataString += "&field4=";
    dataString += String(data.seaLevelPressure);
    dataString += "&field5=";
    dataString += String(data.lightIntensity);
    //    dataString += "&field6=";   // spare field
    //    dataString += String();
    //    dataString += "&field7=";   // spare field
    //    dataString += String();
    //    dataString += "&field8=";
    //    dataString += String();
    //    dataString += "&status=";   // spare field
    //    dataString += sensorStatus;

    // find the number of characters in dataString
    String dataStringLength = String(dataString.length());

    // post the data to ThingSpeak
    client.println("POST /update HTTP/1.1");
    client.println("Host: api.thingspeak.com");
    client.println("Connection: close");
    client.println("X-THINGSPEAKAPIKEY: " + apiWriteKey);
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.println("Content-Length: " + dataStringLength);
    client.println("");
    client.print(dataString);
  }
  client.stop();
} // postToThingSpeak()

// *******************************************************
// send the graphic arrow definitions to lcd display memory
void createArrows() 
{
  // (character index, character binary definition)
  lcd.createChar(ARROW_FALLING_VERY_RAPIDLY, fallingVeryRapidly);
  lcd.createChar(ARROW_FALLING_QUICKLY, fallingQuickly);
  lcd.createChar(ARROW_FALLING, falling);
  lcd.createChar(ARROW_FALLING_SLOWLY, fallingSlowly);
  //            (ARROW_STEADY is builtin character 0x7E)
  lcd.createChar(ARROW_RISING_SLOWLY, risingSlowly);
  lcd.createChar(ARROW_RISING, rising);
  lcd.createChar(ARROW_RISING_QUICKLY, risingQuickly);
  lcd.createChar(ARROW_RISING_VERY_RAPIDLY, risingVeryRapidly);
} // createArrows()

// *******************************************************
// calculate dewpoint temperature in Celsius
// reference: http://en.wikipedia.org/wiki/Dew_point
float dewPoint(float celsius, float humidity) {
  float b = 17.271;
  float c = 237.7;
  // log(x) =  conventional ln(x), log10(x) = conventional log(x)
  float gamma = (b * celsius) / (c + celsius) + log(humidity * 0.01);
  float Td = (c * gamma) / (b - gamma);
  return Td;
} // dewPoint()

// *******************************************************
// calculate relative sealevel pressure from absolute station pressure in hPa
// temperature in °C, elevation in m
// http://www.sandhurstweather.org.uk/barometric.pdf
// http://keisan.casio.com/exec/system/1224575267
float calculateSeaLevelPressure(float celsius, float stationPressure, float elevation) 
{
  float slP = stationPressure / pow(2.718281828, -(elevation / ((273.15 + celsius) * 29.263)));
  return slP;
} // calculateSeaLevelPressure()

// *******************************************************
// report new pressure minus pressure 3 hours ago
// use relative (sea level) pressure in hPa
trend baroTrend(float newPressure) 
{
  trend newTrend;                // temporary variable
  static float CB[CB_SIZE];      // Circular Buffer
  static byte index = 0;
  static bool firstPass = true;  // flag for first time called

  // fill buffer with current pressure if this is first pass
  if (firstPass == true)
  {
    for (int j = 0; j < CB_SIZE; j++) {
      CB[j] = newPressure;
    }
    newTrend.delta = 0;
    newTrend.symbol = ARROW_STEADY;
    firstPass = false;    // mark first pass as done
    return newTrend;
  }
  else                    // update trend
  {
    float oldPressure = CB[index];
    CB[index] = newPressure;
    index++;              // increment the index
    if (index == CB_SIZE) {
      index = 0;          // reset index to 0 when buffer is full
    }
    newTrend.delta = newPressure - oldPressure;

    if (newTrend.delta < -6.0) {
      newTrend.symbol = ARROW_FALLING_VERY_RAPIDLY;
      return newTrend;
    }
    if (newTrend.delta < -3.5) {
      newTrend.symbol = ARROW_FALLING_QUICKLY;
      return newTrend;
    }
    if (newTrend.delta < -1.5) {
      newTrend.symbol = ARROW_FALLING;
      return newTrend;
    }
    if (newTrend.delta < -0.1) {
      newTrend.symbol = ARROW_FALLING_SLOWLY;
      return newTrend;
    }
    if (newTrend.delta < 0.1)  {
      newTrend.symbol = ARROW_STEADY;
      return newTrend;
    }
    if (newTrend.delta < 1.5)  {
      newTrend.symbol = ARROW_RISING_SLOWLY;
      return newTrend;
    }
    if (newTrend.delta < 3.5)  {
      newTrend.symbol = ARROW_RISING;
      return newTrend;
    }
    if (newTrend.delta < 6.0)  {
      newTrend.symbol = ARROW_RISING_QUICKLY;
      return newTrend;
    }
    // delta >= 6.0
    newTrend.symbol = ARROW_RISING_VERY_RAPIDLY;
    return newTrend;
  }
} // barometricTrend()

// *******************************************************
// read the GPIO pin used to set metric/Imperial
// leave pin open for metric, short to ground for Imperial
bool readUnits(int pin) 
{
  if (digitalRead(pin) == HIGH)
  {
    return true;   // metric units
  }
  else
  {
    return false; // Imperial units
  }
} // readUnits()

// *******************************************************
// *********************** END ***************************
// *******************************************************

