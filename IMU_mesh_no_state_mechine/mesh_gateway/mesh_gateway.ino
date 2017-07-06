/** 
 *Name of file:mesh_gateway.ino
 *Purpose: Research. Baxter robot IMU. Center for Automation Technologies and Systems (CATS) at-
 *         Rensselaer Polytechnic Institute
 *Credit: Jihoo Park
 *Used: RF24Mesh by TMRh20
 *
 **/

#include "RF24Network.h"
#include "RF24.h"
#include "RF24Mesh.h"
#include <SPI.h>
//Include eeprom.h for AVR (Uno, Nano) etc. except ATTiny
#include <EEPROM.h>
//**********FOR DEBUG***********//
#define DEBUG 0
#define DEBUG_S 0 //debug state mechine
#define RATE 0

//Bottom 3 int is to debug the speed of the sensor
unsigned long time_check = 0;
int hz = 0;
int count_data = 0;
//////////////////////////////////

//********Storing IMU data******//
//0~6 is IMU1 and 7~13 is IMU2 
int IMU[14]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//used to communicate with Robot Raconteur through serial
byte byteBuffer[28];
byte dat[28];//payload size 14
//////////////////////////////////

//***************For RF24***************//
/***** Configure the chosen CE,CS pins *****/
RF24 radio(8,7);
RF24Network network(radio);
RF24Mesh mesh(radio,network);
////////////////////////////////////////

//**Data send checker**//
int val = 0;
////////////////////////

void setup() {
  Serial.begin(9600);
  // Set the nodeID to 0 for the master node
  mesh.setNodeID(0);
  //Serial.println("Gateway");
  //Serial.println(mesh.getNodeID());
  // Connect to the mesh
  mesh.begin(91);//set channel: baxter 1==90/ Baxter2==91 /Baxter3==92
}



void loop() {    
  // Call mesh.update to keep the network updated
  mesh.update();
  // In addition, keep the 'DHCP service' running on the master node so addresses will
  // be assigned to the sensor nodes
  mesh.DHCP();
  // Check for incoming data from the sensors
  if(network.available()){
    RF24NetworkHeader header;
    network.peek(header);
    switch(header.type){
      // Display the incoming IMU data from the sensor nodes
    case 'L': 
      network.read(header,&dat,sizeof(dat));delay(2);
      byteAToIntA1(dat,IMU); 
      if(DEBUG){
        Serial.print("IMU Node1 Saved: ");
        Serial.print(IMU[0]);Serial.print(" ");
        Serial.print(IMU[1]);Serial.print(" ");
        Serial.print(IMU[2]);Serial.print(" ");
        Serial.print(IMU[3]);Serial.print(" ");
        Serial.print(IMU[4]);Serial.print(" ");
        Serial.println(IMU[5]);
        }
        
      if(RATE==1)count_data+=1; //only when RATE == 1
      break;
      
    case 'R': 
      network.read(header,&dat,sizeof(dat));delay(2);
      byteAToIntA2(dat,IMU);
      if(DEBUG){
        Serial.print("IMU Node2 Saved: ");
        Serial.print(IMU[7]);Serial.print(" ");
        Serial.print(IMU[8]);Serial.print(" ");
        Serial.print(IMU[9]);Serial.print(" ");
        Serial.print(IMU[10]);Serial.print(" ");
        Serial.print(IMU[11]);Serial.print(" ");
        Serial.println(IMU[12]);
        } 
      if(RATE==1)count_data+=1; //only when RATE == 1
      break;
    default: 
      network.read(header,0,0); //Serial.println(header.type);break;
    }
    
  }

  //check IMU update rate
  if(RATE){
     if (count_data==1000){
        float temp = 1000000000.0/(micros()-time_check);
        Serial.println("*************************************");
        Serial.println(temp);
        Serial.println("*************************************");
        count_data=0;
        time_check=micros();    
      }
  }
  
  //transmitting data through Serial
  if (Serial.available() >0) {
      val = Serial.read();
      if(DEBUG_S){
        Serial.print("val is : ");
        Serial.println(val);
      }
      if (val == 20){
        intToBytes(IMU,14,byteBuffer);
        Serial.write(byteBuffer,28);
      }
    val = 0; 
  }  

}
//end of loop


// Fill an int array with the contents of a byte array
// assume that the size of the int array is 7
// and that the size of the byte array is 14
void byteAToIntA1(byte bArray[], int iArray[]){
  for(int i =0; i < 7; i++){
    iArray[i] = bArray[2*i] | bArray[(2*i)+1] << 8; 
  }
}
void byteAToIntA2(byte bArray[], int iArray[]){
  for(int i =7; i < 14; i++){
    iArray[i] = bArray[2*i] | bArray[(2*i)+1] << 8; 
  }
}
// convert an array of ints to an array of bytes
// specify size (number of ints) in input as sizeOfIntArray
void intToBytes(int input[], int sizeOfIntArray, byte buf[]){
  for(int i = 0; i < sizeOfIntArray;i++){
    buf[i*2] = (byte) input[i];
    buf[i*2+1] = (byte) (input[i] >> 8);
  }
  return;
}

