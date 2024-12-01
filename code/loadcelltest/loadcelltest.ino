#include <SPI.h>
#include <Ethernet.h>
#include <HX711.h>

// Network Settings
byte mac[] = { 0xA8, 0x61, 0x0A, 0xAE, 0xE0, 0x2D };
IPAddress ip(10,0, 255, 2);
IPAddress myDns(10, 0, 0, 1);
IPAddress gateway(10, 0, 0, 1);
IPAddress subnet(255, 0, 0, 0);
unsigned int port = 10069;       // Port to connect to
EthernetServer server(port);

// HX711 Settings
#define DOUT_PIN 5   // HX711 data output pin (DT)
#define SCK_PIN 3    // HX711 clock pin (SCK)
HX711 scale;

// Packet Structure for Data Transmission
struct Packet {
    unsigned long timestamp; // Timestamp for the reading
    float force;             // Force measurement

    Packet(unsigned long ts, float f) : timestamp(ts), force(f) {}
};

void setup() {
    // Initialize Serial
    Serial.begin(9600);
    while (!Serial) {
        ; // Wait for serial port to connect (for native USB port)
    }

    // Start Ethernet Connection
    Serial.println("Initializing Ethernet connection...");
    Ethernet.begin(mac, ip, myDns, gateway, subnet);

    // Print Local IP Address
    Serial.print("Device IP Address: ");
    Serial.println(Ethernet.localIP());

    // Start Ethernet Server
    server.begin();

    // Initialize HX711
    scale.begin(DOUT_PIN, SCK_PIN);
    scale.set_scale(-3250.0f); // Adjust based on calibration
    scale.tare();             // Zero the scale
    Serial.println("Setup complete. Ready to serve clients.");
}

void loop() {
    EthernetClient client = server.accept(); // Check for incoming clients

    if (client) {
        Serial.println("Client connected.");
        scale.tare();             // Zero the scale
        while (client.connected()) {
            // Check if HX711 is ready
            if (scale.is_ready()) {
                // Read force value
                float force = scale.get_units(1); // Average over 1 reading
                Packet data(millis(), force);    // Create a data packet

                Serial.println(force);

                // Serialize data
                uint8_t dataBuffer[sizeof(data)];
                memcpy(dataBuffer, &data, sizeof(data));

                // Send data to the client
                client.write(dataBuffer, sizeof(data));
                delay(1); // Adjust delay for desired frequency
            } else {
                //Serial.println("HX711 not ready. Retrying...");
                delay(4); // Small delay before retry
            }
        }

        // Stop the client after disconnect
        client.stop();
        Serial.println("Client disconnected.");
    }
}
