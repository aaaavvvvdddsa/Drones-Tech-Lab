#include <ArduinoJson.h>
#include <EEPROM.h>
#include <OneWire.h>
#include "DFRobot_PH.h"
#include "DFRobot_EC10.h"
#include <Arduino.h>
//float TemperatureSum_Global;
float phValue;
float ecValue;
#include "DFRobot_AirQualitySensor.h"
#include "DFRobot_MICS.h"
#include <DFRobot_ENS160.h>
#define I2C_COMMUNICATION
#ifdef  I2C_COMMUNICATION
/**
     For Fermion version, the default I2C address is 0x53, connect SDO pin to GND and I2C address will be 0x52
*/
DFRobot_ENS160_I2C ENS160(&Wire, /*I2CAddr*/ 0x53);
#else
/**
   Set up digital pin according to the on-board pin connected with SPI chip-select pin
   csPin Available Pins. For example: ESP32&ESP8266(D3), m0(6)
*/
uint8_t csPin = D3;
DFRobot_ENS160_SPI ENS160(&SPI, csPin);
#endif

#define CALIBRATION_TIME   .5   //calibration time for mics sensor

#define Mics_I2C_ADDRESS 0x75
DFRobot_MICS_I2C mics(&Wire, Mics_I2C_ADDRESS);


#define I2C_ADDRESS    0x19
DFRobot_AirQualitySensor particle(&Wire , I2C_ADDRESS);

//water

#define VOLTAGE 5.00  //system voltage
#define OFFSET -530   //zero drift voltage
#define LED 13        //operating instructions

#define EC_PIN A4
#define ArrayLenth 40  //times of collection
#define orpPin A0      //orp meter output,connect to Arduino controller ADC pin
#define PH_PIN A1
#define Turbidity_PIN A2
#define TdsSensorPin A3
#define DO_PIN A5

#define VREF 5000     //VREF (mv) for DO Sensor
#define ADC_RES 1024  //ADC Resolution for DO Sensor

double orpValue;
float voltage_ec,TemperatureSum_Global = 25.0;
float voltage_pH;
//float voltage, phValue, temperature = 25.0, ecValue = 25;
DFRobot_PH ph;

int DS18S20_Pin = 8;
OneWire ds(DS18S20_Pin);
DFRobot_EC10 ec;


#define VREF 5.0           // analog reference voltage(Volt) of the ADC
#define SCOUNT 30          // sum of sample point
int analogBuffer[SCOUNT];  // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0;

int orpArray[ArrayLenth];
int orpArrayIndex = 0;

double avergearray(int* arr, int number) {
  int i;
  int max, min;
  double avg;
  long amount = 0;
  if (number <= 0) {
    printf("Error number for the array to avraging!/n");
    return 0;
  }
  if (number < 5) {  //less than 5, calculated directly statistics
    for (i = 0; i < number; i++) {
      amount += arr[i];
    }
    avg = amount / number;
    return avg;
  } else {
    if (arr[0] < arr[1]) {
      min = arr[0];
      max = arr[1];
    } else {
      min = arr[1];
      max = arr[0];
    }
    for (i = 2; i < number; i++) {
      if (arr[i] < min) {
        amount += min;  //arr<min
        min = arr[i];
      } else {
        if (arr[i] > max) {
          amount += max;  //arr>max
          max = arr[i];
        } else {
          amount += arr[i];  //min<=arr<=max
        }
      }  //if
    }    //for
    avg = (double)amount / (number - 2);
  }  //if
  return avg;
}
// for orp
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}


//For DO Sensor_Start
//Single-point calibration Mode=0
//Two-point calibration Mode=1
#define TWO_POINT_CALIBRATION 0

#define READ_TEMP (TemperatureSum_Global)  //Current water temperature ℃, Or temperature sensor function

//Single point calibration needs to be filled CAL1_V and CAL1_T
#define CAL1_V (1425)  //mv
#define CAL1_T (TemperatureSum_Global)    //℃
//Two-point calibration needs to be filled CAL2_V and CAL2_T
//CAL1 High temperature point, CAL2 Low temperature point
#define CAL2_V (1300)  //mv
#define CAL2_T (15)    //℃

const uint16_t DO_Table[41] = {
  14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
  11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
  9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
  7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410
};

uint8_t Temperaturet;
uint16_t ADC_Raw;
uint16_t ADC_Voltage;
uint16_t DO;

int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c) {
#if TWO_POINT_CALIBRATION == 0
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
  uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}




void setup() {
  pinMode(LED, OUTPUT);
  pinMode(TdsSensorPin, INPUT);
  ph.begin();
  ec.begin();

  //serial begining
  Serial.begin(115200);

  while (!Serial)continue;
  Serial2.begin(9600);
  /**
    Sensor initialization is used to initialize IIC, which is determined by the communication mode used at this time.
  */
  //mics code
  // Serial.begin(115200);
  //  while(!Serial);
  //  while(!mics.begin()){
  //    Serial.println("NO Deivces !");
  //    delay(1000);
  //  } Serial.println("Device connected successfully !");
  //  while(!particle.begin())
  //  {
  //    Serial.println("NO Deivces !");
  //    delay(1000);
  //  }
  Serial.println("sensor begin success!");

  /**
    Get sensor version number
  */
  uint8_t version = particle.gainVersion();
  Serial.print("version is : ");
  Serial.println(version);
  //mics

  while (!mics.begin()) {
    Serial.println("NO Deivces !");
    delay(1000);
  } Serial.println("Device connected successfully !");


  uint8_t mode = mics.getPowerState();
  if (mode == SLEEP_MODE) {
    mics.wakeUpMode();
    Serial.println("wake up sensor success!");
  } else {
    Serial.println("The sensor is wake up mode");
  }
  while (!mics.warmUpTime(CALIBRATION_TIME)) {
    Serial.println("Please wait until the warm-up time is over!");
    delay(1000);
  }
  while ( NO_ERR != ENS160.begin() ) {
    Serial.println("Communication with device failed, please check connection");
    delay(3000);
  }
  Serial.println("Begin ok!");

  ENS160.setPWRMode(ENS160_STANDARD_MODE);

  /**
     Users write ambient temperature and relative humidity into ENS160 for calibration and compensation of the measured gas data.
     ambientTemp Compensate the current ambient temperature, float type, unit: C
     relativeHumidity Compensate the current ambient temperature, float type, unit: %rH
  */
  ENS160.setTempAndHum(/*temperature=*/25.0, /*humidity=*/50.0);


  //  delay(1000);
}

void loop() {
  //mq4
  int mq4;
  mq4 = analogRead(0); //Read Gas value from analog 0
  Serial.print("methane");
  Serial.println(mq4, DEC); //Print the value to serial port
  delay(100);

  //  ens
  uint8_t Status = ENS160.getENS160Status();
  //Serial.print("Sensor operating status : ");
  //Serial.println(Status);

  uint8_t AQI = ENS160.getAQI();
  Serial.print("Air quality index : ");
  Serial.println(AQI);
  //ens  VOC data
  uint16_t TVOC = ENS160.getTVOC();
  Serial.print("Concentration of total volatile organic compounds : ");
  Serial.print(TVOC);
  Serial.println(" ppb");

  uint16_t ECO2 = ENS160.getECO2();
  Serial.print("Carbon dioxide equivalent concentration : ");
  Serial.print(ECO2);
  Serial.println(" ppm");
  Serial.println();


  /**
    @brief : Get concentration of PM1.0
    @param :PARTICLE_PM1_0_STANDARD  Standard particle
            PARTICLE_PM2_5_STANDARD  Standard particle
            PARTICLE_PM10_STANDARD   Standard particle
            PARTICLE_PM1_0_ATMOSPHERE  In atmospheric environment
            PARTICLE_PM2_5_ATMOSPHERE  In atmospheric environment
            PARTICLE_PM10_ATMOSPHERE   In atmospheric environment
  */
  //pm2.5
  uint16_t PM2_5 = particle.gainParticleConcentration_ugm3(PARTICLE_PM2_5_ATMOSPHERE );
  uint16_t PM1_0 = particle.gainParticleConcentration_ugm3(PARTICLE_PM1_0_ATMOSPHERE );
  uint16_t PM10 = particle.gainParticleConcentration_ugm3(PARTICLE_PM10_ATMOSPHERE);
  Serial.print("PM2.5 concentration:");
  Serial.print(PM2_5);
  Serial.println(" ug/m3");
  Serial.print("PM1.0 concentration:");
  Serial.print(PM1_0);
  Serial.println(" ug/m3");
  Serial.print("PM10 concentration:");
  Serial.print(PM10);
  Serial.println(" ug/m3");
  Serial.println();
  //mics
  float Ethanol = mics.getGasData(C2H5OH);
  float Methane = mics.getGasData(CH4);
  float Hydrogen = mics.getGasData(H2);
  float Ammonia = mics.getGasData(NH3);
  float Carbon_Monoxide = mics.getGasData(CO);
  float Nitrogen_Dioxide = mics.getGasData(NO2);

  Serial.print("Ethanol: ");
  Serial.print(Ethanol, 2);
  Serial.println(" PPM");
  Serial.print("Methane: ");
  Serial.print(Methane, 2);
  Serial.println(" PPM");
  Serial.print("Hydrogen: ");
  Serial.print(Hydrogen, 2);
  Serial.println(" PPM");
  Serial.print("Ammonia: ");
  Serial.print(Ammonia, 2);
  Serial.println(" PPM");
  Serial.print("Carbon_Monoxide: ");
  Serial.print(Carbon_Monoxide, 2);
  Serial.println(" PPM");
  Serial.print("Nitrogen_Dioxide: ");
  Serial.print(Nitrogen_Dioxide, 2);
  Serial.println(" PPM");
  Serial.println(".....................");
  Serial.print("ec value:");
  Serial.println(ecValue);


  //water
  //orp
  static unsigned long orpTimer = millis();  //analog sampling interval
  static unsigned long printTime = millis();
  if (millis() >= orpTimer) {
    orpTimer = millis() + 20;
    orpArray[orpArrayIndex++] = analogRead(orpPin);  //read an analog value every 20ms
    if (orpArrayIndex == ArrayLenth) {
      orpArrayIndex = 0;
    }
    orpValue = ((30 * (double)VOLTAGE * 1000) - (75 * avergearray(orpArray, ArrayLenth) * VOLTAGE * 1000 / 1024)) / 75 - OFFSET;

    //convert the analog value to orp according the circuit
  }
  if (millis() >= printTime)  //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
    printTime = millis() + 800;
    //    orpValue_Global = orpValue;
  }
  //pH sensor
  static unsigned long xtimepoint = millis();
  if (millis() - xtimepoint > 10U) {  //time interval: 1s
    xtimepoint = millis();
    //temperature = readTemperature();         // read your temperature sensor to execute temperature compensation
    voltage_pH = analogRead(PH_PIN) / 1024.0 * 5000;  // read the voltage
    // Serial.print(voltage);
    phValue = ph.readPH(voltage_pH, TemperatureSum_Global);  // convert voltage to pH with temperature compensation
    //    phValue_Global = phValue;
  }

  //turbidity
  int sensorValue = analogRead(Turbidity_PIN);   // read the input on analog pin 0:
  float voltage_turbidity = sensorValue * (5.0 / 1024.0);  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  //    voltage_turbidity_Global = voltage_turbidity;

  //electrical conductivity
   static unsigned long timepoint = millis();
  if (millis() - timepoint > 1000U)  //time interval: 1s
  {
    timepoint = millis();
    voltage_ec = analogRead(EC_PIN) / 1024.0 * 5000;  // read the voltage
    //      temperature = TemperatureSum_Global;            // read your temperature sensor to execute temperature compensation
    ecValue = ec.readEC(voltage_ec, TemperatureSum_Global);  // convert voltage to EC with temperature compensation
    //      ecValue_Global = ecValue;

  }
  ec.calibration(voltage_ec, TemperatureSum_Global);  // calibration process by Serail CMD

  //temperature
  byte data[12];
  byte addr[8];

  if (!ds.search(addr)) {
    //no more sensors on chain, reset search
    ds.reset_search();
    return -1000;
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return -1000;
  }

  if (addr[0] != 0x10 && addr[0] != 0x28) {
    Serial.print("Device is not recognized");
    return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);  // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);  // Read Scratchpad


  for (int i = 0; i < 9; i++) {  // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB);  //using two's compliment
  float TemperatureSum = tempRead / 16;
  TemperatureSum_Global = TemperatureSum;


  //TDS sensor
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U)  //every 40 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);  //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U) {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;                                                                                                   // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (TemperatureSum_Global - 25.0);                                                                                                                //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient;                                                                                                             //temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5;  //convert voltage value to tds value
    //      tdsValue_Global = tdsValue;
  }
  //
  //    int getMedianNum(int bArray[], int iFilterLen) {
  //      int bTab[iFilterLen];
  //      for (byte i = 0; i < iFilterLen; i++)
  //        bTab[i] = bArray[i];
  //      int i, j, bTemp;
  //      for (j = 0; j < iFilterLen - 1; j++) {
  //        for (i = 0; i < iFilterLen - j - 1; i++) {
  //          if (bTab[i] > bTab[i + 1]) {
  //            bTemp = bTab[i];
  //            bTab[i] = bTab[i + 1];
  //            bTab[i + 1] = bTemp;
  //          }
  //        }
  //      }
  //      if ((iFilterLen & 1) > 0)
  //        bTemp = bTab[(iFilterLen - 1) / 2];
  //      else
  //        bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  //      return bTemp;
  //    }

  //dissolved oxygen
  float final_DO_value = 0.0;
  Temperaturet = (uint8_t)READ_TEMP;
  ADC_Raw = analogRead(DO_PIN);
  ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;
  final_DO_value = Temperaturet;
  //    final_DO_value_Global = final_DO_value;



  //JSON document
  StaticJsonDocument<2000> doc;
  doc["methane"] = mq4;
  doc["aqi"] = AQI;
  doc["tvoc"] = TVOC;
  doc["co2"] = ECO2;
  doc["pm2_5"] = PM2_5;
  doc["pm1"] = PM1_0;
  doc["pm10"] = PM10;
  doc["ethanol"] = Ethanol;
  doc["methane2"] = Methane;
  doc["hydrogen"] = Hydrogen;
  doc["ammonia"] = Ammonia;
  doc["co"] = Carbon_Monoxide;
  doc["no2"] = Nitrogen_Dioxide;
  //water
  //    doc["Orp_value"] = orpValue;
  //    doc["pH_value"] = phValue ;
  //    doc["Turbidity_value"] = voltage_turbidity;
  doc["ec"] = ecValue;
  //    doc["TDS_value"] = tdsValue;
  doc["temp"] = TemperatureSum;
  //    doc["DO_value"] = final_DO_value;


  serializeJson(doc, Serial2);

  delay(1000);

}
