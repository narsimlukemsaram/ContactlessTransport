/*
  WORKING NO BUFFER 1300 PACKETS PER SECOND SPI
  1.5MBs
  ESP32 UDP packet receiver
*/

// using VSPI SPI
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define PACKET_SIZE 1460         // Can increase this with kconfig
#define SERIAL_USB_BAUD 1000000
#define UDP_PORT 54007
// #define UDP_IP "192.168.0.140"
#define VSPI_MISO MISO
#define VSPI_MOSI MOSI
#define VSPI_SCLK SCK
#define VSPI_SS SS

#define LED_BUILTIN 13

static const int spiClk = 20000000;  // 40 MHz - 25mhz is good
SPIClass* vspi = NULL;               // uninitalised pointers to SPI objects

// WiFi network name and password:
const char* networkName = "MSc_IoT"; 
const char* networkPswd = "MSc_IoT@UCL";

// IP address to send UDP data to:
// either use the ip address of the server or
// a network broadcast address
// const char* udpAddress = UDP_IP;
const int udpPort = UDP_PORT;

volatile int totalBytesRx = 0;

String expectedPattern;

// Are we currently connected?
boolean connected = false;

// The udp library class
WiFiUDP udp;
char packet[PACKET_SIZE];

void setup() {
  // make sure these pins are not used as output
  pinMode(12, INPUT);
  pinMode(27, INPUT);
  // delay(2000);
  Serial.begin(SERIAL_USB_BAUD);

  vspi = new SPIClass(VSPI);
  vspi->begin();
  pinMode(vspi->pinSS(), OUTPUT);  // VSPI SS

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  connectToWiFi(networkName, networkPswd);
  digitalWrite(LED_BUILTIN, HIGH);

  // Get the MAC address of the ESP32
  Serial.print("Default Hostname: ");
  Serial.println(WiFi.getHostname());
  Serial.print("MAC address of the ESP32: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  if (connected) {
    int packetSize = udp.parsePacket();

    // Received packets go to buffer
    if (packetSize > 0) {
      int len = udp.read(packet, packetSize);
      // Serial.print("Received: ");
      // Serial.println(packet);
      // for(int i = 0; i < len; i++){
      //   Serial.print(packet[i], DEC); Serial.print(" ");
      // }
      // Serial.println("");

      vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
      vspi->transfer(packet, len);
      vspi->endTransaction();
    }
  }
}

void connectToWiFi(const char* ssid, const char* pwd) {
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);

  // register event handler
  WiFi.onEvent(WiFiEvent);

  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }
}

// wifi event handler
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      // initializes the UDP state
      // This initializes the transfer buffer
      udp.begin(WiFi.localIP(), udpPort);
      connected = true;
      WiFi.setTxPower(WIFI_POWER_19_5dBm);
      // When connected set
      Serial.print("WiFi connected! IP address: ");
      Serial.println(WiFi.localIP());
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      connected = false;
      break;
    default: break;
  }
}