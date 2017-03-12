#define BLYNK_PRINT Serial


#include <BlynkSimpleSerialBLE.h>
#include <SoftwareSerial.h>

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "5e4b5b7ca2d34196b32a7867061eb006";

SoftwareSerial SerialBLE(16, 17); // RX, TX

void setup()
{
  // Debug console
  Serial.begin(9600);

  SerialBLE.begin(9600);
  Blynk.begin(SerialBLE, auth);

  Serial.println("Waiting for connections...");
}

void loop()
{
  Blynk.run();
  // You can inject your own code or combine it with other sketches.
  // Check other examples on how to communicate with Blynk. Remember
  // to avoid delay() function!
}
