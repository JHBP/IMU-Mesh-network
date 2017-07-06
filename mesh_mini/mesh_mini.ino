/** 
  *Name of file:mesh_mini.ino
  *Purpose: Research. Baxter robot IMU. Center for Automation Technologies and Systems (CATS) at-
  *         Rensselaer Polytechnic Institute
  *Credit:Jihoo Park
  *Used: RF24Mesh by TMRh20
  *
  **/
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU9150 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU9150.h"


#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include <SPI.h>
#include <EEPROM.h>
//#include <printf.h>

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9150 accelgyro;

int ax, ay, az;
int gx, gy, gz;

int IMU[7] = {0,0,0,0,0,0,0};

/**** Configure the nrf24l01 CE and CS pins ****/
RF24 radio(8,7);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

/**
 * User Configuration: nodeID - A unique identifier for each radio. Allows addressing
 * to change dynamically with physical changes to the mesh.
 *
 * In this example, configuration takes place below, prior to uploading the sketch to the device
 * A unique value from 1-255 must be configured for each node.
 * This will be stored in EEPROM on AVR devices, so remains persistent between further uploads, loss of power, etc.
 *
 *nodeID 1 = Left
 *nodeID 2 = Right
 **/
#define nodeID 1
#define DEBUG 0

void setup() {
  Serial.begin(57600);

  // initialize device
  if(DEBUG) Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  accelgyro.setFullScaleGyroRange(0);
  // verify connection
  if(DEBUG){
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU9150 connection successful" : "MPU9150 connection failed");
  }
  // Set the nodeID manually
  mesh.setNodeID(nodeID);
  // Connect to the mesh
  if(DEBUG) Serial.println(F("Connecting to the mesh..."));
  mesh.begin(91);//set channel: baxter 1==90/ Baxter2==91 /Baxter3==92
}

void loop() {
  //fetch data from IMU
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  IMU[0] = ax; IMU[1] = ay; IMU[2] = az; IMU[3] = gx; IMU[4] = gy; IMU[5] = gz; IMU[6] = ax;
  //Need to frequently updat network
  mesh.update();
  
  //Left IMU
  if(nodeID ==1){
    // Send an 'L' type message containing the current IMU()
    if (!mesh.write((byte *)&IMU, 'L', sizeof((byte *)&IMU)*7)) {

      // If a write fails, check connectivity to the mesh network
      if ( ! mesh.checkConnection() ) {
        //refresh the network address
        if(DEBUG) Serial.println("Renewing Address");
        mesh.renewAddress();
      } else {
        if(DEBUG) Serial.println("Send fail, Test OK");
      }
    } else {
      if(DEBUG){
        Serial.println("Send OK: ");
        Serial.println(sizeof((byte *)&IMU)*7);      
        Serial.print(IMU[0]);Serial.print(" ");Serial.print(IMU[1]);Serial.print(" ");Serial.print(IMU[2]);Serial.print(" ");Serial.println(IMU[3]);
      }   
    }
  }
  
  //Right IMU
  else if(nodeID ==2){
    // Send an 'R' type message containing the current IMU()
    if (!mesh.write((byte *)&IMU, 'R', sizeof((byte *)&IMU)*7)) {

      // If a write fails, check connectivity to the mesh network
      if ( ! mesh.checkConnection() ) {
        //refresh the network address
        if(DEBUG) Serial.println("Renewing Address");
        mesh.renewAddress();
      } else {
        if(DEBUG) Serial.println("Send fail, Test OK");
      }
    } else {
      if(DEBUG){
        Serial.println("Send OK: ");
        Serial.println(sizeof((byte *)&IMU)*7);      
        Serial.print(IMU[0]);Serial.print(" ");Serial.print(IMU[1]);Serial.print(" ");Serial.print(IMU[2]);Serial.print(" ");Serial.println(IMU[3]);   
      }
    }
  }
}



