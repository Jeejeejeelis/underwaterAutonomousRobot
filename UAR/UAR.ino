#include <SoftwareSerial.h>
#include <Ethernet.h>

// Altimeter
const int rxPin = 10;
const int txPin = 11;
SoftwareSerial altimeterSerial(rxPin, txPin);
// End of Altimeter


// DVL A50
// MAC address for the Ethernet shield
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
// IP address of the DVL A50
IPAddress dvlIP(192.168.1.100); // Replace with the actual IP address of your DVL A50
// Port number for the DVL A50
const int dvlPort = 16171;
EthernetClient client;
// End of DVL A50


void setup() {
  // put your setup code here, to run once:
   Serial.begin(9600);

   // Altimeter
   altimeterSerial.begin(9600);
   Serial.println("Aquatec Aqualogger 210 Data:");
   // End of Altimeter

   // DVL
   Ethernet.begin(mac);
   delay(1000);
   if (client.connect(dvlIP, dvlPort)) {
    Serial.println("Connected to DVL A50");
    } else {
    Serial.println("Connection failed");
  }
   // End of DVL

}

void loop() {

  //Altimeter: Print raw altimeter data to console
  if (mySerial.available()) {
    String altimeterData = altimeterSerial.readStringUntil('\n');
    Serial.println(altimeterData);
  }
  // End of altimeter data to console

  //DVL: Print raw DVL A50 data to console
  // https://docs.waterlinked.com/dvl/dvl-protocol/
  if (client.connected()) {
    // Check if data is available to read
    while (client.available()) {
      // Read the incoming data
      char c = client.read();
      // Print the data to the Serial Monitor
      Serial.print(c);
    }
  } else {
    // If the client is not connected, try to reconnect
    if (client.connect(dvlIP, dvlPort)) {
      Serial.println("Reconnected to DVL A50");
    }
  }
  // End of DVL


}
