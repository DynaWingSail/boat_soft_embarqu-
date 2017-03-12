/*********************************************************************
**   SPI-compatible                                                 **
**   CE - to digital pin 9                                          **
**   CSN - to digital pin 10  (SS pin)                               **
**   SCK - to digital pin 13 (SCK pin)                              **
**   MOSI - to digital pin 11 (MOSI pin)                            **
**   MISO - to digital pin 12 (MISO pin)                            **
**   IRQ - to digital pin 8 (MISO pin)                             **
*********************************************************************/
//bateau (embarqué)
#include <Wire.h>
#include <SPI.h>
#include <math.h>

#include "NRF24L01.h"
#include "Servo_Boat.h"
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>

#define TX_ADR_WIDTH    5   // 5 unsigned chars TX(RX) address width
#define TX_PLOAD_WIDTH  30  // 32 unsigned chars TX payload

#define wind_pin_x 0
#define wind_pin_y 1
int wind_power;
int wind_direction;
int angle_x=0;
int angle_y=0;
int angle_z=0;
int time_ms=0;
long time_prec;
unsigned char RX_ADDRESS[TX_ADR_WIDTH]  =
{
  0x34, 0x43, 0x10, 0x10, 0x01
};
unsigned char TX_ADDRESS[TX_ADR_WIDTH]  =
{
  0x34, 0x43, 0x10, 0x10, 0x01
};
unsigned char rx_buf[TX_PLOAD_WIDTH] = {0};
unsigned char tx_buf[TX_PLOAD_WIDTH] = {0};
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();


void setup()
{
    Serial.begin(9600);                       // Initialize IO     
    Setup_Servo();  
  NRF_Init();                        // Initialize IO
  Serial.println("RX_Mode start...");
  NRF_SetRxMode();
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS0. Check your wiring!");
    while (1);
  }
  Serial.println("init finished");

  pinMode(52,OUTPUT);
  digitalWrite(52,HIGH); // led de tunning.
}
void angle(int* angle)
{
  if (*angle>360)
    *angle-=360;
  if (*angle<0)
    *angle+=360;
}
void update_gyro() {
  long x, y;
  int angle_gyr_x;
  int angle_gyr_y;
  int angle_gyr_z;
  int angle_acc_x;
  int angle_acc_y;
  int angle_com_z;
  
  time_ms = millis()-time_prec;
  time_prec= time_ms;
  lsm.read();

  angle_gyr_x=angle_x+(int)(5*(lsm.gyroData.x+60)/time_ms);
  angle_gyr_y=angle_y+(int)(5*(lsm.gyroData.y-980)/time_ms);
  angle_gyr_z=angle_z+(int)(5*(lsm.gyroData.z+330)/time_ms);


  angle_acc_x=57.3*atan2(lsm.accelData.y,lsm.accelData.z);
  angle_acc_y=57.3*atan2(lsm.accelData.z,lsm.accelData.x);

  angle_com_z=57.3*atan2(lsm.magData.x,lsm.magData.y)+180;

  float ratio=0.2;
  angle_x= ratio*angle_gyr_x+(1-ratio)*angle_acc_x;
  angle_y= ratio*angle_gyr_y+(1-ratio)*angle_acc_y;
  angle_z= ratio*angle_gyr_z+(1-ratio)*(angle_com_z);

  Serial.print("acceleration ");
  
  Serial.print("angle_x ");
  Serial.print(angle_acc_x);
  Serial.print("\tangle_y ");
  Serial.print(angle_y);
  Serial.print("\tangle_z ");
  Serial.println(angle_z);
    
  x = analogRead(wind_pin_x)-512;
  y = analogRead(wind_pin_y)-512;
  wind_power = sqrt(sq(x) + sq(y));
  wind_direction = 180+57.29*atan2(double(y), double(x));
}
void send_data() {
  NRF_SeTxMode();
  do
  {
    int i;

    tx_buf[ 1] = (char) ((unsigned int)lsm.accelData.x >> 8);
    tx_buf[ 2] = (char) ((unsigned int)lsm.accelData.x & 0xff);
    tx_buf[ 3] = (char) ((unsigned int)lsm.accelData.y >> 8);
    tx_buf[ 4] = (char) ((unsigned int)lsm.accelData.y & 0xff);
    tx_buf[ 5] = (char) ((unsigned int)lsm.accelData.z >> 8);
    tx_buf[ 6] = (char) ((unsigned int)lsm.accelData.z & 0xff);
    tx_buf[ 7] = (char) ((unsigned int)lsm.gyroData.x >> 8);
    tx_buf[ 8] = (char) ((unsigned int)lsm.gyroData.x & 0xff);
    tx_buf[ 9] = (char) ((unsigned int)lsm.gyroData.y >> 8);
    tx_buf[10] = (char) ((unsigned int)lsm.gyroData.y & 0xff);
    tx_buf[11] = (char) ((unsigned int)lsm.gyroData.z >> 8);
    tx_buf[12] = (char) ((unsigned int)lsm.gyroData.z & 0xff);
    tx_buf[13] = (char) ((unsigned int)lsm.magData.x >> 8);
    tx_buf[14] = (char) ((unsigned int)lsm.magData.x & 0xff);
    tx_buf[15] = (char) ((unsigned int)lsm.magData.y >> 8);
    tx_buf[16] = (char) ((unsigned int)lsm.magData.y & 0xff);
    tx_buf[17] = (char) ((unsigned int)lsm.magData.z >> 8);
    tx_buf[18] = (char) ((unsigned int)lsm.magData.z & 0xff);
    tx_buf[19] = (char) ((unsigned int)lsm.temperature >> 8);
    tx_buf[20] = (char) ((unsigned int)lsm.temperature & 0xff);
    tx_buf[21] = (char) ((unsigned int)wind_direction >> 8);
    tx_buf[22] = (char) ((unsigned int)wind_direction & 0xff);
    tx_buf[23] = (char) ((unsigned int)wind_power >> 8);
    tx_buf[24] = (char) ((unsigned int)wind_power & 0xff);
    for(i=0;i<30;i++){
      Serial.print(tx_buf[i]);
      Serial.print(",");
    }
    Serial.println();

    NRF_Send(tx_buf);
  }
  while (!NRF_CheckAck());
  NRF_SetRxMode();

}


bool receive_data(){
    if(NRF_Receive(rx_buf))
    {
        Serial.print("RX = ");
        for(int i = 0; i < TX_PLOAD_WIDTH; i++)
        {
            
            Serial.print(rx_buf[i]);
            Serial.print(",");
        }
        Serial.println();
    if(rx_buf[0]==30);
      //Si on reçoit la DATA Correctement : 
      //-------------Mouvement des Servo-----------------//  /*TEST TO DO*/ 
      //--Safety :--// 
      if(rx_buf[1]<200 && rx_buf[1]!=0 && rx_buf[2]!=0 && rx_buf[2]<200) //Fonction qui sert à éviter que le servo fasse de la merde si on reçoit de la merde.
      {
          //----Voile---//
          Servo_voile(rx_buf[1]);
          //----Gouvernaille---//
          Servo_safran(rx_buf[2]);
          //il faudra attendre 500ms entre 2 action de servo
      }
      
      return true;
    }
    return false;

}

void loop()
{
  update_gyro();
  Serial.println("check rx");
  if (receive_data() == true)
    send_data();
  delay(100);
  
  
  //Test_Servo_Potentiometre() 
}

