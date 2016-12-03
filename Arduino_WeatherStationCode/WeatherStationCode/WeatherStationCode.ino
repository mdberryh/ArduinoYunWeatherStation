/*
Pin Assignments

Any Arduino pins labeled:  SDA  SCL
Uno, Redboard, Pro:        A4   A5
YUN              2 3
WindDirection A0
SolarPanel A1

DOCUMENTATION:
https://learn.sparkfun.com/tutorials/bmp180-barometric-pressure-sensor-hookup-?_ga=1.44355940.1518985383.1445783773
https://www.sparkfun.com/products/11824
https://www.sparkfun.com/datasheets/Sensors/Weather/Weather%20Sensor%20Assembly..pdf
https://tushev.org/articles/arduino/5
https://learn.sparkfun.com/tutorials/weather-station-wirelessly-connected-to-wunderground?_ga=1.257332043.1518985383.1445783773

https://www.arduino.cc/en/Tutorial/YunDatalogger
https://www.arduino.cc/en/Tutorial/Process

http://www.postgresql.org/docs/9.1/static/datatype-datetime.html


http://blog.atmel.com/2014/03/12/building-a-yun-powered-weather-station/
http://forum.arduino.cc/index.php?topic=191974.0
http://forum.arduino.cc/index.php?topic=191974.0

https://www.arduino.cc/en/Main/ArduinoBoardYun
"External Interrupts: 3 (interrupt 0), 2 (interrupt 1), 0 (interrupt 2), 1 (interrupt 3) and 7 (interrupt 4)."

Pins 2 and 3 are taken by the I2C, so that leaves 1 and 7



Digital IO Pins
WindSpeed = 7
Rain = 1


Power adapter
http://www.amazon.com/gp/product/B00KH87U30?keywords=samsung%20tablet%20car%20charger&pebp=1447020282068&perid=XGPMXB58K2C838SRNV66&qid=1447020259&ref_=sr_1_2&sr=8-2

battery connection
http://www.amazon.com/NOCO-GC018-Socket-Eyelet-Terminal/dp/B00G8WLW2Y/ref=sr_1_2?ie=UTF8&qid=1447020382&sr=8-2-spons&keywords=12V+socket&psc=1

*/

#include <avr/wdt.h> //We need watch dog for this program
#include <SFE_BMP180.h>
#include <Wire.h>
#include <FileIO.h>
#include <OneWire.h>
SFE_BMP180 pressure;

#define ALTITUDE 240.585


// digital I/O pins
const byte WSPEED = 7;
const byte RAIN = 1;
const byte GROUND_TEMP  =4;


OneWire ds(GROUND_TEMP);

// analog I/O pins
const byte WDIR = A0;

const byte SOLAR = A1;

long lastWindCheck = 0;

long firstWindIRQ = 0;

volatile long lastWindIRQ = 0;
volatile long windClicks = 0;

// volatiles are subject to modification by IRQs
volatile unsigned long lastRainTime;
volatile unsigned long rainClicks = 0;

long lastMilliSecond = 0; //we will use this to help keep the reporting to every X.
String lastDate = "";
long timeSinceLastSolarSample = 0;
long timeSinceLastWindSample = 0;
long timeLastGroundTemp = 0;
void setup()
{
  wdt_reset(); //Pet the dog
  wdt_disable(); //We don't want the watchdog during init

  // put your setup code here, to run once:
  Serial.begin(9600);
  Bridge.begin();
  FileSystem.begin();
  pinMode(WSPEED, INPUT_PULLUP); // input from wind meters windspeed sensor
  // pinMode(RAIN, INPUT_PULLUP);
  pinMode(WDIR, INPUT);
  pinMode(SOLAR, INPUT);

  //I had to look up the below function https://www.arduino.cc/en/Reference/AttachInterrupt
  // attachInterrupt(digitalPinToInterrupt(RAIN), rainIRQ, FALLING);
  attachInterrupt(digitalPinToInterrupt(WSPEED), windSpeedIRQ, FALLING);

  // turn on interrupts
  interrupts();

  //Enable the watch dog for 8 seconds
  wdt_enable(WDTO_8S);

  lastMilliSecond = millis(); //Reset variable used to track minutes
  lastWindCheck = millis(); //we must set when was the last time we checked wind speed.
  lastRainTime = millis();
  timeSinceLastSolarSample = millis();
  timeSinceLastWindSample = millis();
  firstWindIRQ = lastWindCheck; //at boot we want to set this as our first check.
timeLastGroundTemp=millis();
  bool pressureReady = pressure.begin();

  // Initialize the sensor (it is important to get calibration values stored on the device).
  if (pressureReady)
  {
    //Serial.println("BMP180 init fail\n\n");
  }
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.

    Serial.println("BMP180 init fail\n\n");
    while (1) ; // Pause forever.
  }

}

void loop()
{
  wdt_reset();
  //DATA FORMAT:
  //TIMESTAMP, WINDDIRECTION, WIND SPEED, RAIN, TEMPURATURE, BAROMETRIC PRESSURE, ALTITUDE, Volts ,mA,Watts

  //We should take samples and average solar
  //Take wind speed and average every 30 seconds
  if (millis() - timeSinceLastWindSample >= 30000) {
    //WriteToLogFile("Read wind speed: " + String(millis()/1000));
    //Serial.print("Reading wind");
    ReadSolarValues();
    sample_wind_speed();
    timeSinceLastWindSample = millis();
    wdt_reset();
  }

  //Take Temp and Baro samples and average every minute.

  //every 3 minutes write out weather and solar data
  if (millis() - lastMilliSecond >= 300000)//180000)
  {
    sampleTempAndBaroData();
    wdt_reset();
    String windDir = get_wind_direction();
    String windSpeed = String(get_wind_current_speed());
    String windGust = String(get_wind_gust_speed());
    String rain = String(get_rain_amount_inInches());
    String volts = get_solar_volts();
    String mAmps = get_solar_mAmps();
    String Watts = get_solar_watts();
    wdt_reset();
    //WriteToLogFile("Temp: " + temp);
    //RESET COUNTERS
    reset_solar_values();
    reset_wind_values();
    
    //we have to do a hack to get the string to work...arduino doesn't like long strings...
    //we have a buffer with 225 characters...we will have to insert string then shift then instert other part.

    postValueToServer(1, 1, getTempInC()); //temp
     wdt_reset();
    postValueToServer(1, 12, windSpeed); //windSpeed
     wdt_reset();
    postValueToServer(1, 4, windDir); //windDirection
     wdt_reset();
    postValueToServer(1, 13, windGust); //windGust
     wdt_reset();
    postValueToServer(1, 8, volts); //Volts
     wdt_reset();
    postValueToServer(1, 11, mAmps); // mAmps
     wdt_reset();
    postValueToServer(1, 10, Watts); //watts
     wdt_reset();
    postValueToServer(1, 7, getBaro()); //baro
    wdt_reset();

   
    String dataRecord = "";
    dataRecord = getTimeStamp() + " , " + windDir + " , " +
                 windSpeed + " , " + windGust + " , " +
                 get_rain_amount_inInches() + " , " +  getTempInC() + " , " +
                 getBaro() + " , " + getAltimeter() + " , " +
                 volts + " , " + mAmps + " , " + Watts;
    WriteLocalLogfile(dataRecord);
    //Serial.print(dataRecord);
    //when we are done collecting data reset our timer.
    lastMilliSecond = millis();
  }

//every 30 minutes write out the ground temp
 if (millis() - timeLastGroundTemp >= 1800000)//180000)
  {
    postValueToServer(1, 15, String(GetGroundTemp()));

    timeLastGroundTemp = millis();
  }
  //Reset the watchdog timer.
  wdt_reset();
}

float GetGroundTemp(){
//returns the temperature from one DS18S20 in DEG Celsius

 byte data[12];
 byte addr[8];

 if ( !ds.search(addr)) {
   //no more sensors on chain, reset search
   ds.reset_search();
   return -1000;
 }

 if ( OneWire::crc8( addr, 7) != addr[7]) {
   Serial.println("CRC is not valid!");
   return -1000;
 }

 if ( addr[0] != 0x10 && addr[0] != 0x28) {
   Serial.print("Device is not recognized");
   return -1000;
 }

 ds.reset();
 ds.select(addr);
 ds.write(0x44,1); // start conversion, with parasite power on at the end
 
 delay(750); // Wait for temperature conversion to complete

 byte present = ds.reset();
 ds.select(addr);  
 ds.write(0xBE); // Read Scratchpad

 
 for (int i = 0; i < 9; i++) { // we need 9 bytes
  data[i] = ds.read();
 }
 
 ds.reset_search();
 
 byte MSB = data[1];
 byte LSB = data[0];

 float tempRead = ((MSB << 8) | LSB); //using two's compliment
 float TemperatureSum = tempRead / 16;
 
 return TemperatureSum;
 
}

void WriteLocalLogfile(String dataString) {
  //
  //We need to prepare the filename we are going to write data into.
  String fileName = getDate(); // get the day we are currently running

  String filePath = "/mnt/sd/datalogs/" + fileName + ".csv\0";
  char chFilePath[128];

  filePath.toCharArray(chFilePath, 128);

  //if the file already exists we need to use it or else we need to prepend the data with a header.
  if (!FileSystem.exists(chFilePath) )
  {
    //
    //open file and print the header
    File dataFile = FileSystem.open(chFilePath, FILE_APPEND);
    if (dataFile)
    {
      //
      dataFile.print("DATE, WindDirection, WindSpeed (km/h), WindGustSpeed(km/hr),");
      dataFile.println("Rain (mm), Temp (C), BARO (hPa), Altitude (M), Volts, mAmps, Watts");
      dataFile.close();
    }

  }

  File dataFile = FileSystem.open(chFilePath, FILE_APPEND);
  if (dataFile)
  {
    dataFile.println(dataString);
    //we are done so close the file.
    dataFile.close();
  }
}



void rainIRQ()
// Count rain gauge bucket tips as they occur
// Activated by the magnet and reed switch in the rain gauge, attached to input D2
{

  long currentRaintime = millis(); // grab current time
  long raininterval = currentRaintime - lastRainTime; // calculate interval between this and last event

  if (raininterval > 10) // ignore switch-bounce glitches less than 10mS after initial edge
  {
    rainClicks += 1; //+= 0.011 in or 0.2794 mm ; //Each dump is 0.011" of water
    lastRainTime = currentRaintime; // set up for next event
  }
}

float get_rain_amount_inInches()
{
  return 0.0; //RAIN SENSOR ISN'T WORKING RIGHT, and we dont need it now anyways
  // 0.011 inches per click
  //0.2794 mm per click
  if (rainClicks == 0) return 0.0;

  float rainAmount = rainClicks * 0.2794; //0.011;
  rainClicks = 0;
  return rainAmount;
}

/// <summary>
/// a function to build the baro metric sensor's string of values
/// </summary>
/// <returns>TEMP, PRESSURE, ALTITUDE</returns>
String tempC = "";
String baro = "";
String altimeter = "";
void sampleTempAndBaroData() {

  //NOTE: I discovered that the yun seems to have troubles when I use the STRING for return value of the function.
  //Everytime I called this function with a STRING...it would lock up.
  char status;
  double T, P, p0, a;

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {

      tempC = String(T);

      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P, T);
        if (status != 0)
        {

          p0 = pressure.sealevel(P, ALTITUDE); // we're at 1655 meters (Boulder, CO)
          baro = String(p0);

          a = pressure.altitude(P, p0);
          altimeter = String(a);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}
String getTempInC()
{
  return tempC;
}
String getBaro()
{
  return baro;
}
String getAltimeter()
{
  return altimeter;
}


String get_wind_direction()
// read the wind direction sensor, return heading in degrees
{
  unsigned int adc;

  adc = averageAnalogRead(WDIR); // get the current reading from the sensor

  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order! See Weather Meters datasheet for more information.
  int range = 2;
  if (adc < 66 + range) return ("115");//ESE 115
  if (adc < 79 + range) return ("65");//ENE 65
  if (adc < 88 + range) return ("90");//E 90
  if (adc < 121 + range) return ("155"); //SSE 155
  if (adc < 178 + range) return ("135"); //SE
  if (adc < 236 + range) return ("205"); //SSW
  if (adc < 279 + range) return ("180"); //s
  if (adc < 394 + range) return ("20"); //NNE
  if (adc < 449 + range) return ("45"); //NE
  if (adc < 585 + range) return ("250"); //WSW
  if (adc < 616 + range) return ("225"); //SW
  if (adc < 687 + range) return ("340"); //NNW 340
  if (adc < 769 + range) return ("360"); //N 360
  if (adc < 812 + range) return ("290"); //WNW 290
  if (adc < 872 + range) return ("315"); //NW 315
  if (adc < 932 + range) return ("270");//W 270
  return ("-1" ); // error, disconnected?
}

float windSpeed_sum = 0;
float windSamples = 0;
float windGustinMph = 0;




void windSpeedIRQ()
// Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3
{
  float deltaTime = millis() - lastWindIRQ;
  if (deltaTime > 10) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
  {
    lastWindIRQ = millis(); //Grab the current time
    if (firstWindIRQ <= 0) {
      //if this is the first time we counted this time
      // we need to save it.
      firstWindIRQ = lastWindIRQ;
    }
    windClicks++; //There is 1.492MPH for each click per second.
  }
}
void sample_wind_speed() {
  //
  long now = millis();
  float deltaTime = now - firstWindIRQ; //750ms //diff between first irq and now.
  long tempWindIRQ= firstWindIRQ;
  firstWindIRQ = 0; //we have to reset the first wind IRQ ASAP...the IRQ could activate
  
  deltaTime /= 1000.0; //Covert to seconds
  float windSpeed = (float)windClicks / deltaTime; //3 / 0.750s = 4
  windClicks =0; //after I take the sample I do need to reset the number of clicks!
  windSpeed *= 2.4;//IN 2.4 km/h //1.492mph; //4 * 1.492 = 5.968MPH
  
  //check if we need to set this to the gust speed before we average it in forever.

  //we need to add this as a sample  before we try to divide by it...first time we sample, we can't get an average.
  // without adding in the sample first.
  windSpeed_sum += windSpeed;
  windSamples++;
  
  float averageWindSpeed = windSpeed_sum / windSamples;

  if (windSpeed >= averageWindSpeed) {
    //set gust
    windGustinMph = windSpeed;
  }
}

float get_wind_current_speed()
{
  return (windSpeed_sum / windSamples);
}
float get_wind_gust_speed()
{
  return (windGustinMph);
}
void reset_wind_values() {
  windSpeed_sum = 0;
  windSamples = 0;
  windGustinMph = 0;
  //reset our wind counter. We should call this after we write out the averages.
  lastWindIRQ=millis();
  windClicks=0;
}




//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0;

  for (int x = 0; x < numberOfReadings; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return (runningValue);
}

String getTimeStamp()
{
  String result;
  Process time;
  time.begin("date");
  time.addParameter("+%D %T");
  time.run();

  while (time.available() > 0)
  {
    char c = time.read();
    if (c != '\n')
      result += c;
  }

  return result;
}
String getDate()
{
  String result;
  Process time;
  time.begin("date");
  time.addParameter("+%F"); // date in: 2015-11-15
  time.run();

  while (time.available() > 0)
  {
    char c = time.read();
    if (c != '\n')
      result += c;
  }

  return result;
}

//we will store the sums in the numerator then average when we request the data.
float voltage_sums = 0;
float mAmps_sums = 0;
int solarSamples = 0;

void ReadSolarValues() {
  unsigned int adc;

  adc = averageAnalogRead(SOLAR); // get the current reading from the sensor
  //Serial.println(adc);
  //update our solar sensors;
  //convert adc to voltage
  float volts = adc * (4.62 / 1023.0);  //(arduino 5V voltage line/ 1023)
  //calculate amps  4.7k resistor
  float mAmps = ((volts) / 4.68) * 1000; //calculate the current accross R1

  voltage_sums += volts * 2; //I am using a voltage divider, so volts are really 2* the value read.
  mAmps_sums += mAmps;
  //WriteToLogFile("Volts: " + String(voltage_sums/solarSamples));

  //WriteToLogFile("Amps: " + String(mAmps_sums/solarSamples));
  solarSamples++;
}

String get_solar_volts() {
  //calculate averages, from the last time we called get_solar()
  float volts = voltage_sums / solarSamples;
  String voltageAverage = String(volts);
  return voltageAverage;
}
String get_solar_mAmps() {
  //calculate averages, from the last time we called get_solar()
  float mAmps = mAmps_sums / solarSamples;
  String mAmpsAverage = String(mAmps);
  return mAmpsAverage;
}
String get_solar_watts() {
  //calculate averages, from the last time we called get_solar()

  float volts = voltage_sums / solarSamples;
  float mAmps = mAmps_sums / solarSamples;
  float watts = volts * (mAmps / 1000);

  String voltageAverage = String(volts);
  String mAmpsAverage = String(mAmps);
  return String(watts);
}
void reset_solar_values() {
  voltage_sums = 0;
  mAmps_sums = 0;
  solarSamples = 0;
}

void postValueToServer(int snsorPk, int catPk, String value) {
  Process p;
  p.begin("curl");
  
  p.addParameter("-d sensorPk=" + String(snsorPk));
  p.addParameter("-d catPk=" + String(catPk));
  p.addParameter("-d value=" + value);
  p.addParameter("http://192.168.1.3/api/postSensorData.php");
  p.run();

}
