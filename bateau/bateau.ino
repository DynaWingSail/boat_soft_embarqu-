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
#include "NRF24L01.h"
#include "Servo_Boat.h"

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
    Serial.begin(9600);
    NRF_Init();                        // Initialize IO     
    Serial.println("RX_Mode start...");
    Setup_Servo();  
}

void send_data(unsigned char* data){
    NRF_SeTxMode();
    do
    { 
      int i;
      for(i=0;i<30;i++)
        tx_buf[i]=i;
      Serial.println("TX = ?");
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
    if(rx_buf[0]==30);
      return true;
    }
    return false;
}

void loop()
{

  unsigned char data[30];
  if(receive_data()== true)
    send_data(data);
  delay(30);
  
  Test_Servo_Potentiometre();
  
}

