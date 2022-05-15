#include <max6675.h>
#include <ModbusRtu.h>
/*
 This sample code demonstrates how to use the SimpleKalmanFilter object. 
 Use a potentiometer in Analog input A0 as a source for the reference real value.
 Some random noise will be generated over this value and used as a measured value.
 The estimated value obtained from SimpleKalmanFilter should match the real
 reference value.

 SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty 
 q: Process Noise
 */

#include <SimpleKalmanFilter.h>
SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);
SimpleKalmanFilter simpleKalmanFilter0(2, 2, 0.01);
float u_kalman,kal_adc0; // giá được lọc nhiễu
float measured_value,adc0;


// which analog pin to connect
#define THERMISTORPIN A1         
// resistance at 25 degrees C
#define THERMISTORNOMINAL 100000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 82000 
// data array for modbus network sharing
uint16_t au16data[16] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1 };

/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  u8serno : serial port (use 0 for Serial)
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
Modbus slave(1,0,0); // this is slave @1 and RS-232 or USB-FTDI

int thermoDO = 4;//SO
int thermoCS = 5;
int thermoCLK = 6;//SCK

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

int relay = 13;  
  
void setup() {
  slave.begin( 19200); // 19200 baud, 8-bits, even, 1-bit stop
  // use Arduino pins 
  pinMode(relay, OUTPUT);
 delay(500);
  
}

void loop() {
   //write current thermocouple value
   au16data[2] = ((uint16_t) (thermocouple.readCelsius()*100));
   //au16data[2] = (uint16_t)random(15, 150)*100;
   //au16data[3] = (uint16_t)random(15, 150)*100;
   //poll modbus registers
   slave.poll( au16data, 16 );
   
   measured_value = analogRead(THERMISTORPIN);
   u_kalman = simpleKalmanFilter.updateEstimate(measured_value); // tầng 1
   u_kalman = (1023/u_kalman -1) * SERIESRESISTOR;

   adc0 = analogRead(A0);
   kal_adc0 = simpleKalmanFilter0.updateEstimate(adc0);
   kal_adc0 = (1023/kal_adc0 -1) * SERIESRESISTOR;
   
   float steinhart;
  steinhart = u_kalman / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert absolute temp to C

  float steinhart0;
  steinhart0 = kal_adc0 / THERMISTORNOMINAL;     // (R/Ro)
  steinhart0 = log(steinhart0);                  // ln(R/Ro)
  steinhart0 /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart0 += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart0 = 1.0 / steinhart0;                 // Invert
  steinhart0 -= 273.15;                         // convert absolute temp to C
  //Serial.print("Temperature "); 
  //Serial.println(steinhart);
   
   //au16data[2] = (uint16_t)random(15, 150)*100;
   //au16data[3] = (uint16_t)random(15, 150)*100;
   au16data[0] = (uint16_t)(steinhart0*100);
   au16data[1] = (uint16_t)(steinhart*100);
   

   //write relay value using pwm
   analogWrite(relay, (au16data[4]/100.0)*255);
   delay(500);
}
