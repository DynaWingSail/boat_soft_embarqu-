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


// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).

//---------------------------------------Partie Reconaissance Vocale---------------------------------------//

#include "MOVIShield.h"     // Include MOVI library, needs to be *before* the next #include

#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_PIC32)
#include <SoftwareSerial.h> // This is nice and flexible but only supported on AVR and PIC32 architecture, other boards need to use Serial1 
#endif

const int led_port_side = 3;
const int led_star_board = 4;
int Etat=0;

int Pin_Joy_X = A2;    // select the input pin for the potentiometer
int Pin_Joy_Y = A3;

MOVI recognizer(true);            // Get a MOVI object, true enables serial monitor interface, rx and tx can be passed as parameters for alternate communication pins on AVR architecture

//---------------------------------------FIN Partie Reconaissance Vocale------------------------------------//

//---------------------------------------Data utile Bateau--------------------------------------------------//

int consigne_voile=90;
int consigne_safran=100;
int consigne_mode=2;

//---------------------------------------FIN DAta utile Bateau----------------------------------------------//

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

     //--------------------------------------PHRASE APRISES --------------------------------------------------------//
 
  recognizer.init();      // Initialize MOVI (waits for it to boot)
  recognizer.callSign("Arduino"); // Train callsign Arduino (may take 20 seconds)
  recognizer.addSentence("Everything is ok ?"); // Add sentence 1
  recognizer.addSentence("boat go port side"); // Add sentence 2
  recognizer.addSentence("boat go star board"); // Add sentence 3
  recognizer.addSentence("boat go front"); //Add sentence 4
  recognizer.addSentence("Trimmer go week"); //Add sentence 5
  recognizer.addSentence("Trimmer go strong"); //Add sentence 6
  recognizer.addSentence("Trimmer go middle"); //Add sentence 7

  recognizer.addSentence("start up week end"); //Add sentence 8
  recognizer.addSentence("presentation is over"); //Add sentence 9
  recognizer.addSentence("manual control"); //Add sentence 10
  recognizer.addSentence("now Stop Manual Control"); //Add sentence 11
  
  recognizer.train();              // Train (may take 20seconds) 

  recognizer.setThreshold(5);      // uncomment and set to a higher value (valid range 2-95) if you have a problems due to a noisy environment.
  
  //------------------------------------END_PHRASE_APRISE-------------------------------------------------------//
  pinMode(52,INPUT);

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
      }
      tx_buf[1]=consigne_voile;
      tx_buf[2]=consigne_safran; //Gouvernaille

      for(i=0;i<30;i++)
      {
        Serial.print(tx_buf[i]);
        Serial.print(",");
      }
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

if(digitalRead(52)) {Etat=1;} else{Etat=0;}

if(Etat==0)
{  


    signed int res=recognizer.poll(); // Get result from MOVI, 0 denotes nothing happened, negative values denote events (see docs)
  if (res==1) {                     // Sentence 1.
    recognizer.ask("Yes i'm Fine and you ?"); 
  } 

    if (res==2) { // boat go port side
    consigne_safran=46;
    recognizer.say("boat go port side");
  }
  
  if (res==3) { // boat go star board
    consigne_safran=140;
    recognizer.say("boat go star board");
  }

  if (res==4) { // boat go Front
    consigne_safran=100;
    recognizer.say("boat go Front");
  }
    if (res==5) { // Trimmer go week
    consigne_voile=70;
    recognizer.say("Trimmer go week");
  }
    if (res==6) { // Trimmer go strong
    consigne_voile=119;
    recognizer.say("Trimmer go strong");
  }
    if (res==7) { // Trimmer go middle
    consigne_voile=100;
    recognizer.say("Trimmer go middle");
  }
    if (res==8) { // start up week end
    recognizer.say("Hello poeple from start up week end lille, i hope you will enjoy this demonstration lets see what will hapen");
  }
    if (res==9) { // Presentation is over
    recognizer.say("Ok see you By by kiss love keur keur");
  }


}    

if(Etat==1)
{
 //-----------------------------------------Code Joystick------------------------------------//
  
      int Joy_X = analogRead(Pin_Joy_X);
      int Joy_Y = analogRead(Pin_Joy_Y);

      consigne_voile=map(Joy_X,0,1024,70,119);
      consigne_safran=map(Joy_Y,0,1024,46,149);
      delay(100);
  //-----------------------------------------End Code Joystick--------------------------------//
}  
  
}

