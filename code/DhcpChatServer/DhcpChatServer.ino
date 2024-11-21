#include <SPI.h>
#include <Ethernet.h>
  // Set up SPI and remap pins

// Enter a MAC address and IP address for your controller below.
// Adjust these values to fit your network configuration.



byte mac[] = { 0x96, 0x76, 0xB6, 0x12, 0xfa, 0xfb };
IPAddress ip(192, 168, 1, 177);
IPAddress myDns(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);

// Telnet defaults to port 23
boolean gotAMessage = false; // To track if we received a message from the client

void setup() {
  Serial.begin(9600);  // Initialize serial monitor
  Serial.println("Serial working");
  // Set up SPI and remap pins
  SPI.setMISO(PB4);
  SPI.setMOSI(PB5);
  SPI.setSCLK(PB3);
  SPI.setSSEL(PA15);
  SPI.begin();
  EthernetServer server(23);


  // Initialize Ethernet with CS pin
  Ethernet.init(PA15);

  // Start Ethernet connection
  Serial.println("Trying to get an IP address using DHCP");
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");

    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Ethernet shield was not found. Sorry, can't run without hardware.");
      while (true) {
        delay(1); // Halt if no Ethernet hardware is detected
      }
    }
    if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
    }
    // Initialize Ethernet device with static IP
    Ethernet.begin(mac, ip, myDns, gateway, subnet);
  }

  // Print local IP address
  Serial.print("My IP address: ");
  Serial.println(Ethernet.localIP());

  // Start server listening for clients
  server.begin();
}

void loop() {
  // Wait for a new client
  EthernetServer server(23);
  EthernetClient client = server.available();

  // When the client sends the first byte, respond with a greeting
  if (client) {
    if (!gotAMessage) {
      Serial.println("We have a new client");
      client.println("Hello, client!");
      gotAMessage = true;
    }

    // Read and echo incoming bytes from the client
    if (client.available()) {
      char thisChar = client.read();
      server.write(thisChar);  // Send back to the client
      Serial.print(thisChar);  // Display in Serial Monitor
    }

    // Maintain Ethernet connection
    Ethernet.maintain();
  } else {
    gotAMessage = false; // Reset for new clients
  }
}

