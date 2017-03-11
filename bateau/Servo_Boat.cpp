
#include "Servo_Boat.h"

Servo servoSafran;
Servo servoVoile;

void Setup_Servo()
{
  servoSafran.attach(3);
  servoVoile.attach(4); 
}

void Servo_safran(int angledegre)
{ 
  //Le servo est branche sur le port 3 
   
  if(angledegre>149){angledegre=149;}
  if(angledegre<46){angledegre=46;}
  
  servoSafran.write(angledegre);
}

void Servo_voile(int angledegre)
{
  //Traitement de Securite
  if(angledegre>119){angledegre=119;}
  if(angledegre<70){angledegre=70;}
  
  //Le servo est branche sur le port 4 
  servoVoile.write(angledegre);
}

//--------------------Programme de test--------------------///

void Test_Servo_Potentiometre()
{
  int pot=analogRead(A0);
  delay(100);
  pot=map(pot,0,1024,0,180);
  Serial.println(pot);
  Servo_voile(pot);
}

