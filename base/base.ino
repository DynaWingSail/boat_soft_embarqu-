/*********************************************************************
**   SPI-compatible                                                 **
**   CE - to digital pin 9                                          **
**   CSN - to digital pin 10  (SS pin)                               **
**   SCK - to digital pin 13 (SCK pin)                              **
**   MOSI - to digital pin 11 (MOSI pin)                            **
**   MISO - to digital pin 12 (MISO pin)                            **
**   IRQ - to digital pin 8 (MISO pin)                             **
*********************************************************************/
//base (PAS embarqué)
#include <SoftwareSerial.h>
SoftwareSerial DebugSerial(2, 3); // RX, TX


// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).


#include "NRF24L01.h"

#define TX_ADR_WIDTH    5   // 5 unsigned chars TX(RX) address width
#define TX_PLOAD_WIDTH  30  // 32 unsigned chars TX payload

unsigned char RX_ADDRESS[TX_ADR_WIDTH]  = 
{
  0x34,0x43,0x10,0x10,0x01
};
unsigned char TX_ADDRESS[TX_ADR_WIDTH]  = 
{
  0x34,0x43,0x10,0x10,0x01
};
unsigned char rx_buf[TX_PLOAD_WIDTH] = {0};
unsigned char tx_buf[TX_PLOAD_WIDTH] = {0};

void setup()
{
  //  Serial.begin(9600);
    NRF_Init();                        // Initialize IO     
  //  Serial.println("RX_Mode start...");
   Serial.begin(9600);
}

void send_data(unsigned char* data){
    NRF_SeTxMode();
    do
    { 
      int i;
      Serial.println("TX =");
      for(i=0;i<30;i++)
      {
        tx_buf[i]=30-i;
        Serial.print(tx_buf[i]);
        Serial.print(",");
      }
      tx_buf[2]=50;
      Serial.println();        
      NRF_Send(tx_buf);
    }
    while(!NRF_CheckAck());
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
        long temp;
        temp = ((int)rx_buf[21]<<8)+rx_buf[20];
        Serial.print(temp);
        Serial.print(",");
        temp = ((int)rx_buf[23]<<8)+rx_buf[22];
        Serial.println(temp);
    if(rx_buf[0]==30);
      return true;
    }
    return false;
}


void loop()
{
  unsigned char data[30];
  receive_data();  
  send_data(data);
  delay(100);
}

