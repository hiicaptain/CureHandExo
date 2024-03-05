/*
  Copyright (c) [2023] [Edinburgh Centre for Robotics]

  This code is licensed under the [license name] [license version] [license URL]

  Description: Send a packet with force sensor values from a hand exoskeleton 

  Author: [Hao Yu]
  Date: [Created v0.1 on 2023/06/29; Update v0.2 on 2023/07/13;]
*/

#include "HX711.h"
#include "math.h" 

// set four force sensors
HX711 MCPL;
HX711 MCPR;
HX711 PIP;
HX711 DIP;

// setting HX711 and sensor data conversion
int bit2mv = 3355; // 0~8388607 binary = 0~2500mv 3355
int B_Gain = 128;
long Error = 0;
float N_ratio = 0.32; // data = Newton*N_ratio, so newton = data/N_ratio
// old: 0.32; new: DIP 0.38, PIP: 0.33, MCPL: 0.33, MCPR: 0.32
float reading = 0;

// demo variables
unsigned long int starttime = 0; // the time stamp
int mcp1, mcp2, dip, pip; 
int mcp1_old, mcp2_old, dip_old, pip_old;
//  the formatted output is "%d,%d,%d,%d", 
//  where %d may occupy 5 characters, "," occupy 1 character and there is a "null char".
//  so the output should have 5*4+3+1 = 24 characters.
char All_data[24]; 

void setup() {
  // maximuise the baud rate
  Serial.begin(115200);        

  // initialise four force sensors
  // HX711 circuit wiring, larger pin number --> yellow cable --> clock, smaller pin number --> green cable --> data
  // Serial.println("Initialising and Calibrating all force sensors...");
  MCPL = SensorCalibration(MCPL, 22, 23, "ulnar MCP");
  MCPR = SensorCalibration(MCPR, 24, 25, "radial MCP");
  PIP = SensorCalibration(PIP, 26, 27, "PIP");
  DIP = SensorCalibration(DIP, 28, 29, "DIP");
  delay(100);

  // Serial.println("Done!");
  mcp1_old =  ReadSensor(MCPL, "ulnar MCP");
  mcp2_old = ReadSensor(MCPR, "radial MCP");
  dip_old = ReadSensor(PIP, "PIP");
  pip_old = ReadSensor(DIP, "DIP");
  delay(100);
}

void loop() {
  starttime = millis();
  // Serial.println(starttime);
  mcp1 = ReadSensor(MCPL, "ulnar MCP");
  mcp2 = ReadSensor(MCPR, "radial MCP");
  pip = ReadSensor(PIP, "PIP");
  dip = ReadSensor(DIP, "DIP");

  if (mcp1 == 0){mcp1 = mcp1_old;} 
  else {mcp1_old = mcp1;}
  if (mcp2 == 0){mcp2 = mcp2_old;} 
  else {mcp2_old = mcp2;}
  if (pip == 0){pip = pip_old;} 
  else {pip_old = pip;}
  if (dip == 0){dip = dip_old;} 
  else {dip_old = dip;}

    
  sprintf(All_data, "%d,%d,%d,%d", mcp1, mcp2, pip, dip);
  Serial.println(All_data);
  while ((millis() - starttime) < 20){}  
}

HX711 SensorCalibration(HX711 sensor, int DataPin, int ClkPin, String name)
{
  // initialise four force sensors
  sensor.begin(DataPin, ClkPin);
  sensor.set_gain(B_Gain);
  sensor.set_scale(bit2mv*B_Gain); // this value is obtained by calibrating the scale with known weights; see the README for details
  // Serial.println("Calibrating the " +  name + " force sensor");
  delay(1000); // wait for steady and clean data
  Error = sensor.get_offset(100); 
  sensor.set_offset(Error); // reset the scale to 0
  reading = sensor.get_units()/N_ratio;
  // Serial.print("Current baseline data: ");
  // Serial.println(reading,1);
  return sensor;
}

int ReadSensor(HX711 sensor, String name)
{
  if (sensor.is_ready()) {
    // old: 0.32; new: DIP 0.38, PIP: 0.33, MCPL: 0.33, MCPR: 0.32
    if (name == "ulnar MCP"){
      N_ratio = 0.33;
    } else if (name == "radial MCP") {
      N_ratio = 0.32;
    } else if (name == "PIP") {
      N_ratio = 0.33;
    } else if (name == "DIP") {
      N_ratio = 0.38;
    } else {
      N_ratio = 0.32;      
    }
    reading = sensor.get_units()/N_ratio;
    reading =sensor.get_units();
  } else {
    reading = 0;
  }
  // Serial.print(name + " force sensor reading: "); Serial.println(reading,2);
  int value = reading*100; // the force sensor values 'reading' are from 0.0 N to about 20.0 N
  return value;
}
