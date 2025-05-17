/***************************************************
 * LoRa Mesh Network for ESP32 and SX1262
 * 
 * Operates in 915MHz band with DVPA routing algorithm
 * Based on successfully tested hardware configuration
 * 
 * Hardware: ESP32 + SX1262 (RA-01SH) + OLED SSD1306
 * Date: 2025-05-17
 ***************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SX126x-Arduino.h>

// OLED display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// LoRa pin configuration for ESP32 with RA-01SH sx1262
// These match the working configuration you provided
hw_config hwConfig;
int PIN_LORA_RESET = 4;    // LORA RESET
int PIN_LORA_NSS = 5;      // LORA SPI CS
int PIN_LORA_SCLK = 18;    // LORA SPI CLK
int PIN_LORA_MISO = 19;    // LORA SPI MISO 
int PIN_LORA_MOSI = 23;    // LORA SPI MOSI
int PIN_LORA_BUSY = 22;    // LORA SPI BUSY
int PIN_LORA_DIO_1 = 21;   // LORA DIO_1
int RADIO_TXEN = 26;       // LORA ANTENNA TX ENABLE
int RADIO_RXEN = 27;       // LORA ANTENNA RX ENABLE
int I2C_SDA = 14;          // OLED SDA
int I2C_SCL = 15;          // OLED SCL

// LoRa parameters for 915MHz operation
#define RF_FREQUENCY  915000000  // Hz (915 MHz)
#define TX_OUTPUT_POWER 22       // dBm (adjust based on regulatory requirements)
#define LORA_BANDWIDTH 0         // 0: 125 kHz
#define LORA_SPREADING_FACTOR 7  // SF7 for higher data rate
#define LORA_CODINGRATE 1        // 4/5 coding rate
#define LORA_PREAMBLE_LENGTH 8   // symbols
#define LORA_SYMBOL_TIMEOUT 0    // symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 3000    // ms
#define TX_TIMEOUT_VALUE 3000    // ms

// Node identification and network parameters
#define NODE_ID 0xABCD           // 16-bit node identifier, change for each node
#define MAX_NEIGHBORS 32         // Maximum number of neighbors to track
#define MAX_ROUTES 64            // Maximum number of routes to store
#define BEACON_INTERVAL_MIN 30   // Minimum beacon interval in seconds
#define BEACON_INTERVAL_MAX 1800 // Maximum beacon interval in seconds (30 minutes)
#define MSG_BUFFER_SIZE 10       // Number of messages to buffer for forwarding

// Protocol constants
#define PKT_TYPE_DATA      0x01
#define PKT_TYPE_BEACON    0x02
#define PKT_TYPE_ROUTE_UPD 0x03
#define PKT_TYPE_JOIN_REQ  0x04
#define PKT_TYPE_JOIN_RESP 0x05
#define PKT_TYPE_ACK       0x06

// QoS flags
#define QOS_NORMAL     0x00
#define QOS_CRITICAL   0x20
#define QOS_RELIABLE   0x40
#define QOS_PRIORITY   0x80

// Packet structure (6-byte header plus variable payload)
typedef struct {
  uint8_t flags;        // Bitfields for packet type, QoS, control flags
  uint16_t sourceID;    // 2-byte source node ID
  uint16_t destID;      // 2-byte destination node ID (0xFFFF for broadcast)
  uint8_t seqNum;       // 1-byte sequence number
  uint8_t ttl;          // Time-to-live counter (optional)
  uint8_t extFlags;     // Extended flags for additional features (optional)
  uint8_t payload[240]; // Variable payload (adjustable)
  uint8_t length;       // Total length of the payload
} MeshPacket;

// Neighbor table entry (9 bytes each)
typedef struct {
  uint16_t nodeID;      // Node identifier
  uint8_t linkQuality;  // Link quality metric (0-255)
  uint32_t lastHeard;   // Last time heard from (millis)
  uint8_t energyStatus; // Energy status (0-255, higher is better)
  uint8_t capabilities; // Capability flags
  bool active;          // Whether this entry is active
} NeighborEntry;

// Routing table entry
typedef struct {
  uint16_t destID;      // Destination node ID
  uint16_t nextHopID;   // Next hop node ID
  uint16_t pathNodes[4]; // Partial path information (up to 4 hops)
  uint8_t hopCount;     // Number of hops to destination
  uint8_t linkQuality;  // Composite link quality
  uint8_t energyMetric; // Energy-based metric
  uint16_t reliability; // Historical reliability (0-1000)
  uint32_t lastUpdated; // Last time route was updated
  float compositMetric; // Calculated composite metric
  bool active;          // Whether this entry is active
} RouteEntry;

// Message buffer for storing messages pending forwarding or acknowledgment
typedef struct {
  MeshPacket packet;
  uint32_t timestamp;
  uint8_t attempts;
  bool active;
} MessageBuffer;

// Protocol timing parameters
uint32_t beaconInterval = BEACON_INTERVAL_MIN * 1000; // in ms
uint32_t lastBeaconTime = 0;
uint32_t lastRouteUpdate = 0;
uint32_t displayUpdateTime = 0;
const uint32_t DISPLAY_UPDATE_INTERVAL = 2000; // Update display every 2 seconds

// Data structures for the mesh protocol
NeighborEntry neighbors[MAX_NEIGHBORS];
RouteEntry routes[MAX_ROUTES];
MessageBuffer messageBuffer[MSG_BUFFER_SIZE];

// Protocol state variables
uint8_t currentSeqNum = 0;
uint16_t messagesProcessed = 0;
uint16_t messagesForwarded = 0;
uint16_t messagesDropped = 0;
uint8_t networkDensity = 0; // Estimated network density

// Event handling system (based on your working example)
#define NO_EVENT 0b0000000000000000
#define TX_FIN 0b0000000000000001
#define N_TX_FIN 0b1111111111111110
#define TX_ERR 0b0000000000000010
#define N_TX_ERR 0b1111111111111101
#define RX_FIN 0b0000000000000100
#define N_RX_FIN 0b1111111111111011
#define RX_ERR 0b0000000000010000
#define N_RX_ERR 0b1111111111101111
#define CAD_FIN 0b0000000000100000
#define N_CAD_FIN 0b1111111111011111

// Semaphore used by events to wake up loop task
SemaphoreHandle_t g_task_sem = NULL;

// Flag for the event type
volatile uint16_t g_task_event_type = NO_EVENT;

// Radio events
RadioEvents_t RadioEvents;

// Received packet data
uint8_t rcvBuffer[256];
uint16_t rcvSize = 0;
int16_t rcvRssi = 0;
int8_t rcvSnr = 0;

// CAD result
bool tx_cadResult;

// Protocol coefficients for the DVPA routing algorithm
float alpha = 0.4; // Weight for link quality
float beta = 0.3;  // Weight for hop count
float gamma = 0.1; // Weight for energy status
float delta = 0.1; // Weight for historical reliability
float epsilon = 0.1; // Weight for estimated transmission time

// Forward declarations
void OnTxDone(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnTxTimeout(void);
void OnRxTimeout(void);
void OnRxError(void);
void OnCadDone(bool cadResult);
void processReceivedPacket(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
float calculateCompositMetric(uint8_t linkQuality, uint8_t hopCount, uint8_t energyStatus, uint16_t reliability, uint8_t etx);
void updateRoutingTable(uint16_t nodeID, uint8_t linkQuality, uint8_t energyStatus);
void sendBeacon(void);
void sendRouteUpdate(bool triggered);
void sendData(uint16_t destID, uint8_t *data, uint8_t length, uint8_t qos);
void maintainNeighborTable(void);
void displayNetworkStatus(void);
void setupLoRaHardware(void);
void updateOLED(void);
void performCadBeforeSend(void);
uint8_t compressData(uint8_t *input, uint8_t length, uint8_t *output);
uint8_t decompressData(uint8_t *input, uint8_t length, uint8_t *output);

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  delay(500);
  
  Serial.println("=====================================");
  Serial.println("LoRa Mesh Network Node Initializing");
  Serial.println("=====================================");
  
  // Initialize OLED Display
  Wire.begin(I2C_SDA, I2C_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("LoRa Mesh Node");
  display.println("Initializing...");
  display.display();
  
  // Initialize protocol data structures
  for(int i = 0; i < MAX_NEIGHBORS; i++) {
    neighbors[i].active = false;
  }
  
  for(int i = 0; i < MAX_ROUTES; i++) {
    routes[i].active = false;
  }
  
  for(int i = 0; i < MSG_BUFFER_SIZE; i++) {
    messageBuffer[i].active = false;
  }
  
  // Initialize the SX1262 LoRa module with the configuration
  // that worked for you in previous tests
  setupLoRaHardware();
  
  // Create the task event semaphore
  g_task_sem = xSemaphoreCreateBinary();
  // Initialize semaphore
  xSemaphoreGive(g_task_sem);
  // Take the semaphore so the loop will go to sleep until an event happens
  xSemaphoreTake(g_task_sem, 10);
  
  // Start the passive network joining phase
  // Listen for beacons and network activity
  Radio.Rx(RX_TIMEOUT_VALUE);
  
  Serial.println("Node initialized with ID: 0x" + String(NODE_ID, HEX));
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Node ID: 0x" + String(NODE_ID, HEX));
  display.println("Listening...");
  display.display();
  
  // Schedule first beacon
  lastBeaconTime = millis() - (beaconInterval - random(5000));
  displayUpdateTime = millis();
}

void loop() {
  // Wait until semaphore is released
  xSemaphoreTake(g_task_sem, portMAX_DELAY);

  // Handle event
  while (g_task_event_type != NO_EVENT) {
    if ((g_task_event_type & TX_FIN) == TX_FIN) {
      g_task_event_type &= N_TX_FIN;
      Serial.println("TX Done");
      
      // After sending, listen for responses
      Radio.Sleep();
      Radio.Rx(RX_TIMEOUT_VALUE);
    }
    
    if ((g_task_event_type & TX_ERR) == TX_ERR) {
      g_task_event_type &= N_TX_ERR;
      Serial.println("TX Timeout");
      
      // If message failed to send, return to listening
      Radio.Sleep();
      Radio.Rx(RX_TIMEOUT_VALUE);
    }
    
    if ((g_task_event_type & RX_FIN) == RX_FIN) {
      g_task_event_type &= N_RX_FIN;
      Serial.println("RX Done");
      
      // Process the received packet
      processReceivedPacket(rcvBuffer, rcvSize, rcvRssi, rcvSnr);
      
      // Return to listening
      Radio.Sleep();
      Radio.Rx(RX_TIMEOUT_VALUE);
    }
    
    if ((g_task_event_type & RX_ERR) == RX_ERR) {
      g_task_event_type &= N_RX_ERR;
      Serial.println("RX Error/Timeout");
      
      // Return to listening
      Radio.Sleep();
      Radio.Rx(RX_TIMEOUT_VALUE);
    }
    
    if ((g_task_event_type & CAD_FIN) == CAD_FIN) {
      g_task_event_type &= N_CAD_FIN;
      
      if (tx_cadResult) {
        Serial.println("Channel busy, delaying transmission");
        // Wait and try again
        delay(random(200, 500));
        performCadBeforeSend();
      } else {
        Serial.println("Channel free, sending message");
        // Channel is free, proceed with transmission
        // The actual transmission will be handled by the calling function
        // which set up the CAD operation
      }
    }
  }
  
  // Current time
  uint32_t currentTime = millis();
  
  // Check if it's time to send a beacon
  if(currentTime - lastBeaconTime >= beaconInterval) {
    sendBeacon();
    lastBeaconTime = currentTime;
    
    // Implement Trickle algorithm for beacon interval
    if(beaconInterval < BEACON_INTERVAL_MAX * 1000) {
      beaconInterval = min(beaconInterval * 2, BEACON_INTERVAL_MAX * 1000);
    }
  }
  
  // Maintenance tasks
  maintainNeighborTable();
  
  // Check if we should trigger route updates (on significant changes)
  if(currentTime - lastRouteUpdate >= 5 * 60 * 1000) { // 5 minutes
    sendRouteUpdate(false);  // Not triggered, just periodic
    lastRouteUpdate = currentTime;
  }
  
  // Process message buffer (retry, expire messages)
  for(int i = 0; i < MSG_BUFFER_SIZE; i++) {
    if(messageBuffer[i].active) {
      // If message has been buffered too long, expire it
      if(currentTime - messageBuffer[i].timestamp > 30000) { // 30 seconds
        messageBuffer[i].active = false;
        messagesDropped++;
        continue;
      }
      
      // If critical message needs retry
      if((messageBuffer[i].packet.flags & QOS_CRITICAL) && 
         messageBuffer[i].attempts < 3 && 
         currentTime - messageBuffer[i].timestamp > 5000) { // Retry every 5 seconds
        
        // Retry sending the message
        MeshPacket *packet = &messageBuffer[i].packet;
        messageBuffer[i].attempts++;
        messageBuffer[i].timestamp = currentTime;
        
        // Check if channel is clear before sending
        performCadBeforeSend();
        
        // The actual send will be handled in the CAD callback
        Radio.Send((uint8_t*)packet, packet->length + 8); // 6 bytes header + 2 bytes extra fields
      }
    }
  }
  
  // Update display periodically
  if(currentTime - displayUpdateTime > DISPLAY_UPDATE_INTERVAL) {
    updateOLED();
    displayUpdateTime = currentTime;
  }
}

void setupLoRaHardware() {
  // Set up hardware configuration based on your working configuration
  hwConfig.CHIP_TYPE = SX1262_CHIP;       // Using an SX1262 module
  hwConfig.PIN_LORA_RESET = PIN_LORA_RESET;
  hwConfig.PIN_LORA_NSS = PIN_LORA_NSS;
  hwConfig.PIN_LORA_SCLK = PIN_LORA_SCLK;
  hwConfig.PIN_LORA_MISO = PIN_LORA_MISO;
  hwConfig.PIN_LORA_DIO_1 = PIN_LORA_DIO_1;
  hwConfig.PIN_LORA_BUSY = PIN_LORA_BUSY;
  hwConfig.PIN_LORA_MOSI = PIN_LORA_MOSI;
  hwConfig.RADIO_TXEN = RADIO_TXEN;
  hwConfig.RADIO_RXEN = RADIO_RXEN;
  hwConfig.USE_DIO2_ANT_SWITCH = false;
  hwConfig.USE_DIO3_TCXO = false;
  hwConfig.USE_DIO3_ANT_SWITCH = false;
  hwConfig.USE_RXEN_ANT_PWR = true; // Important flag from your working code
  
  // Initialize the LoRa hardware
  uint32_t init_result = lora_hardware_init(hwConfig);
  Serial.printf("LoRa init %s\r\n", init_result == 0 ? "success" : "failed");
  
  if (init_result != 0) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("LoRa init failed!");
    display.display();
    while(1); // Stop if initialization failed
  }
  
  // Setup event handlers
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;
  RadioEvents.CadDone = OnCadDone;
  
  // Initialize the radio
  Radio.Init(&RadioEvents);
  
  // Configure radio parameters
  Radio.SetChannel(RF_FREQUENCY);
  
  // Configure transmitter
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                   true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);
  
  // Configure receiver
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
  
  // Set public network mode
  Radio.SetPublicNetwork(false);
  
  Serial.println("SX1262 radio initialized successfully");
}

// Radio event callbacks
void OnTxDone(void) {
  g_task_event_type |= TX_FIN;
  xSemaphoreGive(g_task_sem);
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  // Store the received packet data
  memcpy(rcvBuffer, payload, size);
  rcvSize = size;
  rcvRssi = rssi;
  rcvSnr = snr;
  
  g_task_event_type |= RX_FIN;
  xSemaphoreGive(g_task_sem);
}

void OnTxTimeout(void) {
  g_task_event_type |= TX_ERR;
  xSemaphoreGive(g_task_sem);
}

void OnRxTimeout(void) {
  g_task_event_type |= RX_ERR;
  xSemaphoreGive(g_task_sem);
}

void OnRxError(void) {
  g_task_event_type |= RX_ERR;
  xSemaphoreGive(g_task_sem);
}

void OnCadDone(bool cadResult) {
  tx_cadResult = cadResult;
  g_task_event_type |= CAD_FIN;
  xSemaphoreGive(g_task_sem);
}

void processReceivedPacket(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  // Ensure the packet is at least the size of our header
  if(size < 6) {
    Serial.println("Packet too small to be valid");
    return;
  }
  
  // Cast the payload to our packet structure
  MeshPacket *packet = (MeshPacket*)payload;
  
  // Extract basic information
  uint8_t packetType = packet->flags & 0x0F;  // Lower 4 bits are packet type
  uint8_t qosFlags = packet->flags & 0xF0;    // Upper 4 bits are QoS flags
  uint16_t sourceID = packet->sourceID;
  uint16_t destID = packet->destID;
  
  Serial.printf("Received packet type %d from 0x%04X to 0x%04X\n", 
               packetType, sourceID, destID);
  
  // Update neighbor information based on received packet
  int neighborIndex = -1;
  for(int i = 0; i < MAX_NEIGHBORS; i++) {
    if(neighbors[i].active && neighbors[i].nodeID == sourceID) {
      neighborIndex = i;
      break;
    }
  }
  
  // If this is a new neighbor, find an empty slot
  if(neighborIndex == -1) {
    for(int i = 0; i < MAX_NEIGHBORS; i++) {
      if(!neighbors[i].active) {
        neighborIndex = i;
        neighbors[i].active = true;
        neighbors[i].nodeID = sourceID;
        break;
      }
    }
  }
  
  // If we found a slot to update
  if(neighborIndex != -1) {
    // Calculate link quality based on RSSI and SNR
    // Higher RSSI and SNR result in better link quality
    uint8_t linkQuality = min(255, max(0, (rssi + 130) * 2));
    if(snr < 0) {
      linkQuality = max(0, linkQuality - abs(snr) * 4);
    }
    
    neighbors[neighborIndex].linkQuality = (neighbors[neighborIndex].linkQuality * 3 + linkQuality) / 4; // Weighted average
    neighbors[neighborIndex].lastHeard = millis();
    
    // Extract energy status if available (e.g., in beacons)
    if(packetType == PKT_TYPE_BEACON && size >= 8) {
      neighbors[neighborIndex].energyStatus = packet->payload[0];
    }
  }
  
  // Process the packet based on its type
  switch(packetType) {
    case PKT_TYPE_DATA:
      // Check if the packet is for us
      if(destID == NODE_ID || destID == 0xFFFF) {
        Serial.printf("Received data packet from 0x%04X\n", sourceID);
        
        // If the message requires acknowledgment
        if(qosFlags & QOS_RELIABLE) {
          // Send ACK
          MeshPacket ackPacket;
          ackPacket.flags = PKT_TYPE_ACK;
          ackPacket.sourceID = NODE_ID;
          ackPacket.destID = sourceID;
          ackPacket.seqNum = packet->seqNum;
          ackPacket.length = 0;
          
          // Queue ACK for transmission with a small random delay to avoid collisions
          delay(random(50, 200));
          
          performCadBeforeSend();
          Radio.Send((uint8_t*)&ackPacket, 6);  // Send header only
        }
        
        // Process the actual payload data here
        Serial.print("Data: ");
        for(int i = 0; i < min(packet->length, 16); i++) {
          Serial.printf("%02X ", packet->payload[i]);
        }
        Serial.println();
        
        messagesProcessed++;
      }
      
      // Forward the packet if needed (mesh functionality)
      else if(packet->ttl > 1) { // Only forward if TTL > 1
        // Decrement TTL
        packet->ttl--;
        
        // Check if we have a route to the destination
        bool haveRoute = false;
        for(int i = 0; i < MAX_ROUTES; i++) {
          if(routes[i].active && routes[i].destID == destID) {
            haveRoute = true;
            break;
          }
        }
        
        if(haveRoute) {
          // Find an empty slot in the message buffer
          int bufferIndex = -1;
          for(int i = 0; i < MSG_BUFFER_SIZE; i++) {
            if(!messageBuffer[i].active) {
              bufferIndex = i;
              break;
            }
          }
          
          if(bufferIndex != -1) {
            // Queue for forwarding with a random delay to prevent collisions
            memcpy(&messageBuffer[bufferIndex].packet, packet, size);
            messageBuffer[bufferIndex].timestamp = millis();
            messageBuffer[bufferIndex].attempts = 0;
            messageBuffer[bufferIndex].active = true;
            
            // Add some random delay before forwarding (50-500ms)
            delay(random(50, 500));
            
            performCadBeforeSend();
            Radio.Send((uint8_t*)packet, size);
            messagesForwarded++;
          }
        }
      }
      break;
      
    case PKT_TYPE_BEACON:
      // Process beacon - update neighbor table, reset beacon interval if network changed
      Serial.printf("Received beacon from 0x%04X\n", sourceID);
      
      // If this is a new neighbor, reset the beacon interval to increase network awareness
      if(neighborIndex != -1 && millis() - neighbors[neighborIndex].lastHeard > beaconInterval * 2) {
        beaconInterval = BEACON_INTERVAL_MIN * 1000;
        lastBeaconTime = millis() - (beaconInterval - random(5000));
      }
      break;
      
    case PKT_TYPE_ROUTE_UPD:
      // Process route update - extract routing information
      Serial.printf("Received route update from 0x%04X\n", sourceID);
      
      // Simple parsing of route update packet
      if(size >= 8) { // Minimum size for a route update
        uint8_t numRoutes = packet->payload[0];
        uint8_t offset = 1;
        
        for(int i = 0; i < numRoutes && offset + 5 <= packet->length; i++) {
          uint16_t routeDest = (packet->payload[offset] << 8) | packet->payload[offset + 1];
          uint8_t hopCount = packet->payload[offset + 2];
          uint8_t routeQuality = packet->payload[offset + 3];
          uint8_t routeEnergy = packet->payload[offset + 4];
          
          // Update routing table based on received information
          updateRoutingTable(routeDest, routeQuality, routeEnergy);
          
          offset += 5;
        }
      }
      break;
      
    case PKT_TYPE_JOIN_REQ:
      // Handle join request - respond if we have capacity
      Serial.printf("Received join request from 0x%04X\n", sourceID);
      
      // Count active neighbors to determine if we have capacity
      int activeNeighborCount = 0;
      for(int i = 0; i < MAX_NEIGHBORS; i++) {
        if(neighbors[i].active) {
          activeNeighborCount++;
        }
      }
      
      // Only respond if we have capacity and good link quality
      if(activeNeighborCount < MAX_NEIGHBORS * 0.8 && neighborIndex != -1 && neighbors[neighborIndex].linkQuality > 100) {
        MeshPacket joinResp;
        joinResp.flags = PKT_TYPE_JOIN_RESP;
        joinResp.sourceID = NODE_ID;
        joinResp.destID = sourceID;
        joinResp.seqNum = currentSeqNum++;
        joinResp.length = 2;
        
        // Include our energy status and neighbor count
        joinResp.payload[0] = 200; // Example energy status
        joinResp.payload[1] = activeNeighborCount;
        
        // Add random delay to prevent collision with other responses
        delay(random(100, 1000));
        
        performCadBeforeSend();
        Radio.Send((uint8_t*)&joinResp, 8);
      }
      break;
      
    case PKT_TYPE_JOIN_RESP:
      // Handle join response - evaluate for potential neighbor selection
      Serial.printf("Received join response from 0x%04X\n", sourceID);
      
      // This would be processed during the active joining phase
      // For now, we just update the neighbor table
      break;
      
    case PKT_TYPE_ACK:
      // Handle acknowledgment - clear message from buffer if present
      Serial.printf("Received ACK from 0x%04X\n", sourceID);
      
      // Find the original message in our buffer and mark it as acknowledged
      for(int i = 0; i < MSG_BUFFER_SIZE; i++) {
        if(messageBuffer[i].active && 
           messageBuffer[i].packet.destID == sourceID && 
           messageBuffer[i].packet.seqNum == packet->seqNum) {
          
          messageBuffer[i].active = false;
          Serial.println("Message acknowledged and removed from buffer");
          break;
        }
      }
      break;
      
    default:
      Serial.printf("Unknown packet type: %d\n", packetType);
      break;
  }
}

void sendBeacon() {
  Serial.println("Sending beacon");
  
  MeshPacket beacon;
  beacon.flags = PKT_TYPE_BEACON;
  beacon.sourceID = NODE_ID;
  beacon.destID = 0xFFFF; // Broadcast
  beacon.seqNum = currentSeqNum++;
  beacon.length = 2;
  
  // Include node information in beacon
  beacon.payload[0] = 200; // Example energy status
  beacon.payload[1] = networkDensity; // Network density estimate
  
  performCadBeforeSend();
  Radio.Send((uint8_t*)&beacon, 8);
}

void sendRouteUpdate(bool triggered) {
  Serial.println("Sending route update");
  
  MeshPacket routeUpdate;
  routeUpdate.flags = PKT_TYPE_ROUTE_UPD;
  routeUpdate.sourceID = NODE_ID;
  routeUpdate.destID = 0xFFFF; // Broadcast
  routeUpdate.seqNum = currentSeqNum++;
  
  // Count active routes to include
  int activeRoutes = 0;
  for(int i = 0; i < MAX_ROUTES; i++) {
    if(routes[i].active) {
      activeRoutes++;
    }
  }
  
  // Cap the number of routes to share to fit in packet
  int numRoutesToShare = min(activeRoutes, 20); // At most 20 routes per update
  routeUpdate.payload[0] = numRoutesToShare;
  
  int offset = 1;
  int routesAdded = 0;
  
  // Add routes to the packet
  for(int i = 0; i < MAX_ROUTES && routesAdded < numRoutesToShare; i++) {
    if(routes[i].active) {
      // Add destination ID (2 bytes)
      routeUpdate.payload[offset++] = (routes[i].destID >> 8) & 0xFF;
      routeUpdate.payload[offset++] = routes[i].destID & 0xFF;
      
      // Add hop count
      routeUpdate.payload[offset++] = routes[i].hopCount;
      
      // Add link quality
      routeUpdate.payload[offset++] = routes[i].linkQuality;
      
      // Add energy metric
      routeUpdate.payload[offset++] = routes[i].energyMetric;
      
      routesAdded++;
    }
  }
  
  routeUpdate.length = offset;
  
  performCadBeforeSend();
  Radio.Send((uint8_t*)&routeUpdate, offset + 6);
}

void sendData(uint16_t destID, uint8_t *data, uint8_t length, uint8_t qos) {
  Serial.printf("Sending data to 0x%04X\n", destID);
  
  // Find a route to the destination
  int routeIndex = -1;
  for(int i = 0; i < MAX_ROUTES; i++) {
    if(routes[i].active && routes[i].destID == destID) {
      routeIndex = i;
      break;
    }
  }
  
  if(routeIndex == -1 && destID != 0xFFFF) {
    Serial.println("No route to destination");
    return;
  }
  
  MeshPacket dataPacket;
  dataPacket.flags = PKT_TYPE_DATA | qos;
  dataPacket.sourceID = NODE_ID;
  dataPacket.destID = destID;
  dataPacket.seqNum = currentSeqNum++;
  dataPacket.ttl = 10; // Example TTL value
  
  // Check if compression would be beneficial
  uint8_t compressedData[240];
  uint8_t compressedLength = compressData(data, length, compressedData);
  
  if(compressedLength < length) {
    // Use compressed data
    memcpy(dataPacket.payload, compressedData, compressedLength);
    dataPacket.length = compressedLength;
    // Set compression flag
    dataPacket.extFlags = 0x01;
  } else {
    // Use original data
    memcpy(dataPacket.payload, data, length);
    dataPacket.length = length;
    dataPacket.extFlags = 0x00;
  }
  
  // Store in message buffer if reliable delivery is requested
  if(qos & QOS_RELIABLE) {
    for(int i = 0; i < MSG_BUFFER_SIZE; i++) {
      if(!messageBuffer[i].active) {
        memcpy(&messageBuffer[i].packet, &dataPacket, sizeof(MeshPacket));
        messageBuffer[i].timestamp = millis();
        messageBuffer[i].attempts = 0;
        messageBuffer[i].active = true;
        break;
      }
    }
  }
  
  performCadBeforeSend();
  Radio.Send((uint8_t*)&dataPacket, dataPacket.length + 8); // 6 bytes header + 2 bytes extra fields
}

void maintainNeighborTable() {
  uint32_t currentTime = millis();
  
  // Count active neighbors for network density estimation
  int activeCount = 0;
  
  for(int i = 0; i < MAX_NEIGHBORS; i++) {
    if(neighbors[i].active) {
      // If haven't heard from neighbor in 5x beacon interval, mark inactive
      if(currentTime - neighbors[i].lastHeard > 5 * BEACON_INTERVAL_MAX * 1000) {
        neighbors[i].active = false;
        
        // Reset beacon interval when topology changes
        beaconInterval = BEACON_INTERVAL_MIN * 1000;
        lastBeaconTime = currentTime - (beaconInterval - random(5000));
        
        Serial.printf("Neighbor 0x%04X marked inactive\n", neighbors[i].nodeID);
      } else {
        activeCount++;
      }
    }
  }
  
  // Update network density estimate
  networkDensity = activeCount;
}

void updateRoutingTable(uint16_t nodeID, uint8_t linkQuality, uint8_t energyStatus) {
  // Find if we already have a route to this node
  int routeIndex = -1;
  for(int i = 0; i < MAX_ROUTES; i++) {
    if(routes[i].active && routes[i].destID == nodeID) {
      routeIndex = i;
      break;
    }
  }
  
  // If no existing route, find empty slot
  if(routeIndex == -1) {
    for(int i = 0; i < MAX_ROUTES; i++) {
      if(!routes[i].active) {
        routeIndex = i;
        routes[i].active = true;
        routes[i].destID = nodeID;
        break;
      }
    }
  }
  
  // If we found a slot to update
  if(routeIndex != -1) {
    routes[routeIndex].linkQuality = linkQuality;
    routes[routeIndex].energyMetric = energyStatus;
    routes[routeIndex].lastUpdated = millis();
    
    // For simplicity, we're using a basic metric calculation here
    // In a full implementation, this would use the composite metric formula
    float compositMetric = calculateCompositMetric(
      linkQuality, 
      1, // Direct neighbor is 1 hop
      energyStatus, 
      500, // Assuming 50% reliability for new routes
      10   // Example ETX value
    );
    
    routes[routeIndex].compositMetric = compositMetric;
  }
}

float calculateCompositMetric(uint8_t linkQuality, uint8_t hopCount, uint8_t energyStatus, uint16_t reliability, uint8_t etx) {
  // Normalize input values to 0-1 range
  float normalizedLQ = linkQuality / 255.0;
  float normalizedES = energyStatus / 255.0;
  float normalizedHR = reliability / 1000.0;
  
  // Apply the DVPA formula:
  // CM = α×(1/LQ) + β×HC + γ×(1/ES) + δ×(1/HR) + ε×ETX
  
  float metric = 
    alpha * (1.0 / max(normalizedLQ, 0.01)) + 
    beta * hopCount + 
    gamma * (1.0 / max(normalizedES, 0.01)) + 
    delta * (1.0 / max(normalizedHR, 0.01)) + 
    epsilon * etx;
  
  return metric;
}

void updateOLED() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("LoRa Mesh Node");
  display.printf("ID: 0x%04X\n", NODE_ID);
  
  // Count active neighbors
  int activeNeighbors = 0;
  for(int i = 0; i < MAX_NEIGHBORS; i++) {
    if(neighbors[i].active) {
      activeNeighbors++;
    }
  }
  
  // Count active routes
  int activeRoutes = 0;
  for(int i = 0; i < MAX_ROUTES; i++) {
    if(routes[i].active) {
      activeRoutes++;
    }
  }
  
  display.printf("Neighbors: %d\n", activeNeighbors);
  display.printf("Routes: %d\n", activeRoutes);
  display.printf("Msgs: %d/%d/%d\n", messagesProcessed, messagesForwarded, messagesDropped);
  display.printf("Beacon: %ds\n", (beaconInterval / 1000));
  
  display.display();
}

void performCadBeforeSend() {
  Radio.Sleep();
  Radio.SetCadParams(LORA_CAD_08_SYMBOL, LORA_SPREADING_FACTOR + 13, 10, LORA_CAD_ONLY, 0);
  Radio.StartCad();
  // The CAD callback will handle what happens next
}

// Simplified compression implementation
uint8_t compressData(uint8_t *input, uint8_t length, uint8_t *output) {
  // Simple run-length encoding
  uint8_t outIndex = 0;
  uint8_t inIndex = 0;
  
  while(inIndex < length) {
    uint8_t currentByte = input[inIndex];
    uint8_t runLength = 1;
    
    while(inIndex + runLength < length && input[inIndex + runLength] == currentByte && runLength < 255) {
      runLength++;
    }
    
    if(runLength > 3) { // Only compress runs of 4 or more
      output[outIndex++] = 0;  // Marker for compressed run
      output[outIndex++] = currentByte;
      output[outIndex++] = runLength;
      inIndex += runLength;
    } else {
      output[outIndex++] = 1;  // Marker for literal byte
      output[outIndex++] = currentByte;
      inIndex++;
    }
    
    if(outIndex >= length - 2) { // If compression isn't helping, abort
      return length + 1;  // Return original length + 1 to signal no compression benefit
    }
  }
  
  return outIndex;
}

uint8_t decompressData(uint8_t *input, uint8_t length, uint8_t *output) {
  uint8_t outIndex = 0;
  uint8_t inIndex = 0;
  
  while(inIndex < length) {
    uint8_t marker = input[inIndex++];
    
    if(marker == 0) { // Compressed run
      uint8_t byte = input[inIndex++];
      uint8_t runLength = input[inIndex++];
      
      for(int i = 0; i < runLength; i++) {
        output[outIndex++] = byte;
      }
    } else { // Literal byte
      output[outIndex++] = input[inIndex++];
    }
  }
  
  return outIndex;
}