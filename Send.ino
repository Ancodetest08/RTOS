#define BLYNK_PRINT Serial // Enable debug prints for Blynk

// --- Your Blynk Template Info ---
#define BLYNK_TEMPLATE_ID "TMPL6N61gDFsZ"
#define BLYNK_TEMPLATE_NAME "Intelligent Switchs"

// --- Your Auth Token ---
char auth[] = "74I3j0F-tJ2qP-yiycP_qKpU7otCYawu"; // <<-- REPLACE THIS!

// --- Your Wi-Fi Credentials ---
char ssid[] = "anime";      // <<-- REPLACE THIS!
char pass[] = "33333333"; // <<-- REPLACE THIS!

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h> // For semaphores/mutexes

#include <SPI.h>
#include <mcp2515.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- Hardware Pin Configuration ---
#define SPI_SCK     6
#define SPI_MOSI    7
#define SPI_MISO    2
#define SPI_CS      10
#define DHT_PIN     4
#define LM393_PIN   3 // Digital/Analog pin for light sensor (adjust based on sensor type)

// OLED settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
// This will be updated in setup if OLED is found at 0x3C or 0x3D
uint8_t SCREEN_ADDRESS = 0x3C; 

// I2C pins for ESP32-C3
#define I2C_SDA 5
#define I2C_SCL 8

// --- FreeRTOS Task Priorities ---
#define BLYNK_TASK_PRIORITY        3 // Highest priority for Blynk
#define CAN_TRANSMIT_TASK_PRIORITY 2
#define SENSOR_TASK_PRIORITY       1
#define DISPLAY_TASK_PRIORITY      1
#define CAN_RECEIVE_TASK_PRIORITY  2 // Assuming DTM sends status back

// --- CAN IDs ---
#define SENSOR_DATA_AND_RELAY_STATUS_ID 0x036 // ID for sensor data + relay commands sent from ESP32
#define STATUS_FROM_RECEIVER_ID         0x037 // ID for status feedback from DTM (Receiver)

// --- SPI and CAN Objects ---
SPIClass *spiAcc = nullptr;
// #undef SPI // This line might cause issues if SPI is used by other libraries without re-defining.
//            // It's generally better to use spiAcc-> wherever you use SPI.
// #define SPI (*spiAcc) // This can also cause issues. Better to use spiAcc-> everywhere.

struct can_frame canMsg;
MCP2515 mcp2515(SPI_CS); // Initialize mcp2515 with SPI_CS pin

// --- Sensor and OLED Objects ---
DHT dht(DHT_PIN, DHT22); // DHT22 sensor
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- FreeRTOS Queues and Mutexes ---
QueueHandle_t sensorAndRelayQueue; // Queue for SensorData and RelayState to CANTransmitTask
SemaphoreHandle_t displayMutex;    // Mutex to protect currentSensorData and currentStatus for display
SemaphoreHandle_t relayStateMutex; // Mutex to protect relay1State and relay2State

// --- Data Structures ---
struct SensorData {
    float temperature;
    float humidity;
    int lightLevel;
};

// Structure to send sensor data and relay state together
struct SensorAndRelayData {
    SensorData sensor;
    bool relay1; // True if Relay 1 (Fan) is ON
    bool relay2; // True if Relay 2 (LED) is ON
    bool blynkIsDisconnected; // Added: True if Blynk is disconnected from ESP32
};

// Structure for status received from DTM (if DTM sends feedback)
struct StatusData {
    bool fanOn;            // Actual state of Fan as reported by DTM
    bool ledOn;            // Actual state of LED as reported by DTM
    bool systemActive;     // True if DTM is receiving data
    bool manualOverride;   // True if DTM is in manual control mode (from Blynk, via ESP32)
    float currentTemp;     // Temperature reported by DTM (if it has its own sensor)
    float currentHum;      // Humidity reported by DTM
    int currentLight;      // Light reported by DTM
    unsigned long lastUpdate; // Last time status was received from DTM
};

// --- Global variables shared between Tasks ---
bool blynkConnected = false; // Added: Track Blynk connection status
SensorAndRelayData currentDataToSend = {{0}, false, false, false}; // Sensor data and relay states to be sent
StatusData currentStatus = {false, false, false, false, 0, 0, 0, 0}; // Status received from DTM

// --- Relay State variables (updated by Blynk and Countdown functions) ---
volatile bool relay1State = false; // Current state of Relay 1 (Fan)
volatile bool relay2State = false; // Current state of Relay 2 (LED)

// --- Blynk Timers for countdown functionality ---
BlynkTimer countdownTimer1; // Timer for V2 (Relay 1)
BlynkTimer countdownTimer2; // Timer for V3 (Relay 2)
BlynkTimer displayBlynkRelayStateTimer; // Added: Global declaration for this timer

volatile int remainingCountdownTime1 = 0; 
int countdownTimerId1 = -1; 

volatile int remainingCountdownTime2 = 0; 
int countdownTimerId2 = -1; 

// --- FreeRTOS Task Prototypes ---
void blynkTask(void *pvParameters);
void sensorTask(void *parameter);
void canTransmitTask(void *parameter);
void canReceiveTask(void *parameter); // Task to receive feedback from DTM
void displayTask(void *parameter);

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("========== ESP32C3 Smart Control System Starting... ==========");
    
    // Initialize I2C (for OLED)
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000); // Set I2C clock to 100kHz for stability
    
    delay(100);
    
    // Scan I2C devices to find OLED (for debugging)
    Serial.println("Scanning I2C devices...");
    byte error, address;
    int nDevices = 0;
    for(address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            Serial.printf("I2C device found at address 0x%02X!\n", address);
            nDevices++;
        }
    }
    if (nDevices == 0) {
        Serial.println("No I2C devices found");
    }
    delay(500);
    
    // Initialize OLED with common addresses
    bool oledFound = false;
    uint8_t oledAddresses[] = {0x3C, 0x3D}; 
    for (int i = 0; i < 2; i++) {
        if(display.begin(SSD1306_SWITCHCAPVCC, oledAddresses[i])) {
            Serial.printf("OLED found at address 0x%02X\n", oledAddresses[i]);
            oledFound = true;
            SCREEN_ADDRESS = oledAddresses[i]; // Update global address if found
            break;
        }
    }
    if (!oledFound) {
        Serial.println("* ERROR: SSD1306 not found! Continuing without OLED...");
    } else {
        Serial.println(">>>>> OLED initialized successfully!");
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("System Starting...");
        display.display();
    }
    
    dht.begin();
    pinMode(LM393_PIN, INPUT); // Configure light sensor pin
    
    // Initialize SPI and MCP2515 (CAN controller)
    spiAcc = new SPIClass(FSPI); // Use FSPI for ESP32-C3
    spiAcc->begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS); // Initialize SPI bus with pins
    mcp2515.reset();
    mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ); // Set CAN bitrate
    mcp2515.setNormalMode(); // Put MCP2515 in normal operation mode
    Serial.println(">>>>> CAN Bus initialized successfully!");
    
    // --- Create FreeRTOS Queues and Mutexes ---
    sensorAndRelayQueue = xQueueCreate(5, sizeof(SensorAndRelayData));
    displayMutex = xSemaphoreCreateMutex();
    relayStateMutex = xSemaphoreCreateMutex(); 
    
    if (sensorAndRelayQueue == NULL || displayMutex == NULL || relayStateMutex == NULL) {
        Serial.println("* ERROR: Failed to create queue(s) or mutex(es)!!!");
        while(1); // Halt if creation fails
    } else {
        Serial.println(">>>>> Queues and mutexes created successfully!");
    }
    
    // --- Create FreeRTOS Tasks ---
    xTaskCreate(blynkTask, "BlynkTask", 8192, NULL, BLYNK_TASK_PRIORITY, NULL); 
    xTaskCreate(sensorTask, "SensorTask", 4096, NULL, SENSOR_TASK_PRIORITY, NULL);
    xTaskCreate(canTransmitTask, "CANTransmitTask", 4096, NULL, CAN_TRANSMIT_TASK_PRIORITY, NULL); 
    xTaskCreate(canReceiveTask, "CANReceiveTask", 2048, NULL, CAN_RECEIVE_TASK_PRIORITY, NULL); // To receive status from DTM
    xTaskCreate(displayTask, "DisplayTask", 4096, NULL, DISPLAY_TASK_PRIORITY, NULL);
    
    Serial.println(">>>>> All Tasks created successfully!");
}

void loop() {
    // The loop() function is typically empty in FreeRTOS applications,
    // as all main logic is handled by tasks.
    vTaskDelay(pdMS_TO_TICKS(1000)); 
}

// --- Task for Blynk and Wi-Fi connection ---
void blynkTask(void *pvParameters) {
    Serial.println("Blynk Task: Starting Wi-Fi connection...");

    WiFi.begin(ssid, pass); 

    unsigned long startTime = millis(); 
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(pdMS_TO_TICKS(500)); 
        Serial.print("."); 

        if (millis() - startTime > 30000) { 
            Serial.println("\nBlynk Task: Failed to connect to WiFi after 30 seconds. Restarting...");
            ESP.restart(); 
        }
    }

    Serial.println("\nBlynk Task: Connected to WiFi!");
    Serial.print("Blynk Task: IP Address: ");
    Serial.println(WiFi.localIP());

    Blynk.begin(auth, ssid, pass);
    Serial.println("Blynk Task: Connected to Blynk!");
    
    // Gửi trạng thái ban đầu của relay lên Blynk khi kết nối
    Blynk.virtualWrite(V0, relay1State ? 1 : 0);
    Blynk.virtualWrite(V1, relay2State ? 1 : 0);

    // Optional: Timer để cập nhật trạng thái relay lên Blynk định kỳ
    displayBlynkRelayStateTimer.setInterval(1000L, [](){
        Blynk.virtualWrite(V0, relay1State ? 1 : 0);
        Blynk.virtualWrite(V1, relay2State ? 1 : 0);
        Blynk.virtualWrite(V7, remainingCountdownTime1); // Nếu bạn có widget hiển thị thời gian đếm ngược
        Blynk.virtualWrite(V8, remainingCountdownTime2); // Nếu bạn có widget hiển thị thời gian đếm ngược
    });

    for (;;) { // Removed the extra 'f' here
        Blynk.run();
        countdownTimer1.run();
        countdownTimer2.run();
        displayBlynkRelayStateTimer.run(); // Run the timer for Blynk updates

        // Cập nhật trạng thái kết nối Blynk
        if (Blynk.connected()) {
            blynkConnected = true;
        } else {
            blynkConnected = false;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// --- Task for reading sensors ---
void sensorTask(void *parameter) {
    SensorData data; // Correctly declare data struct
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (1) {
        data.temperature = dht.readTemperature();
        data.humidity = dht.readHumidity();
        data.lightLevel = analogRead(LM393_PIN); // Assuming analog read, adjust if digital

        // Check for DHT sensor read errors
        if (isnan(data.temperature) || isnan(data.humidity)) {
            Serial.println("Sensor Task: Failed to read from DHT sensor!");
            data.temperature = 0; 
            data.humidity = 0;
        }
        
        Serial.println("========== Sensor Readings (ESP32) ==========");
        Serial.printf(">>>>> Temperature: %.2f°C\n", data.temperature);
        Serial.printf(">>>>> Humidity: %.2f%%\n", data.humidity);
        Serial.printf(">>>>> Light Level: %d\n", data.lightLevel);
        
        // Get current relay states safely
        bool r1_state, r2_state;
        if (xSemaphoreTake(relayStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            r1_state = relay1State;
            r2_state = relay2State;
            xSemaphoreGive(relayStateMutex);
        } else {
            Serial.println("* WARNING: Sensor Task: Could not take relayStateMutex!");
            r1_state = false; 
            r2_state = false;
        }

        // Prepare data to send (sensors + relay states + Blynk connection status)
        SensorAndRelayData dataToSend = {data, r1_state, r2_state, !blynkConnected}; // blynkIsDisconnected = !blynkConnected

        // Send combined data to CAN transmit queue
        if (xQueueSend(sensorAndRelayQueue, &dataToSend, pdMS_TO_TICKS(100)) != pdPASS) {
            Serial.println("* ERROR: Sensor Task: Failed to send sensor/relay data to queue!!!");
        }
        
        // Update current sensor data for display task safely
        if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            currentDataToSend.sensor = data; 
            currentDataToSend.relay1 = r1_state; 
            currentDataToSend.relay2 = r2_state;
            // Update the blynkIsDisconnected state for display (optional, based on your display logic)
            currentDataToSend.blynkIsDisconnected = !blynkConnected; 
            xSemaphoreGive(displayMutex);
        } else {
            Serial.println("* WARNING: Sensor Task: Could not take displayMutex!");
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2000)); // Read every 2 seconds
    }
}

// --- Task for transmitting data via CAN Bus ---
void canTransmitTask(void *parameter) {
    SensorAndRelayData dataToTransmit;
    
    while (1) {
        // Wait to receive data from sensorAndRelayQueue (blocking)
        if (xQueueReceive(sensorAndRelayQueue, &dataToTransmit, portMAX_DELAY) == pdPASS) {
            canMsg.can_id = SENSOR_DATA_AND_RELAY_STATUS_ID; // Use the combined ID
            canMsg.can_dlc = 8; // 8 bytes of data

            // Byte 0-1: Temperature (float * 10, then split into 2 bytes)
            int tempInt = (int)(dataToTransmit.sensor.temperature * 10);
            canMsg.data[0] = (tempInt >> 8) & 0xFF; // MSB
            canMsg.data[1] = tempInt & 0xFF;        // LSB
            
            // Byte 2-3: Humidity (float * 10, then split into 2 bytes)
            int humInt = (int)(dataToTransmit.sensor.humidity * 10);
            canMsg.data[2] = (humInt >> 8) & 0xFF; // MSB
            canMsg.data[3] = humInt & 0xFF;        // LSB
            
            // Byte 4-5: Light Level (int, split into 2 bytes)
            canMsg.data[4] = (dataToTransmit.sensor.lightLevel >> 8) & 0xFF; // MSB
            canMsg.data[5] = dataToTransmit.sensor.lightLevel & 0xFF;        // LSB
            
            // Byte 6: Relay 1 (Bit 0), Relay 2 (Bit 1) state from Blynk AND Blynk connection status (Bit 7)
            canMsg.data[6] = 0; // Initialize to 0
            if (dataToTransmit.relay1) canMsg.data[6] |= 0x01; // Set bit 0 if Relay 1 is ON
            if (dataToTransmit.relay2) canMsg.data[6] |= 0x02; // Set bit 1 if Relay 2 is ON
            
            // Bit 7: 1 if Blynk is disconnected, 0 if connected
            if (dataToTransmit.blynkIsDisconnected) {
                canMsg.data[6] |= (1 << 7); // Set Bit 7 to 1
            } else {
                canMsg.data[6] &= ~(1 << 7); // Clear Bit 7 to 0
            }

            canMsg.data[7] = 0x00; // Reserved/unused byte
            
            if (mcp2515.sendMessage(&canMsg) == MCP2515::ERROR_OK) {
                Serial.println("CAN Transmit Task: Sent CAN message (Sensor + Relay Status + Blynk Conn Status).");
            } else {
                Serial.println("* ERROR: CAN Transmit Task: Failed to send CAN message!!!");
            }
            
            vTaskDelay(pdMS_TO_TICKS(100)); // Small delay after sending
        }
    }
}

// --- Task for receiving data from CAN Bus (e.g., status feedback from DTM) ---
void canReceiveTask(void *parameter) {
    while (1) {
        if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
            if (canMsg.can_id == STATUS_FROM_RECEIVER_ID) { // Check if it's a status message from DTM
                Serial.println("CAN Receive Task: Received status from DTM!");
                
                // Debug: In ra raw data nhận được
                Serial.printf("Received CAN Data: [0x%02X] [0x%02X 0x%02X] [0x%02X 0x%02X] [0x%02X 0x%02X] [0x%02X]\n",
                               canMsg.data[0], canMsg.data[1], canMsg.data[2],
                               canMsg.data[3], canMsg.data[4], canMsg.data[5],
                               canMsg.data[6], canMsg.data[7]);
                
                StatusData newStatus;
                
                // Decode Byte 0: Device states and system active status
                newStatus.fanOn = (canMsg.data[0] & 0x01) != 0;      // Bit 0: Fan state
                newStatus.ledOn = (canMsg.data[0] & 0x02) != 0;      // Bit 1: LED state
                newStatus.systemActive = (canMsg.data[0] & 0x04) != 0; // Bit 2: DTM's system active
                newStatus.manualOverride = (canMsg.data[0] & 0x08) != 0; // Bit 3: DTM in manual override mode
                                
                // Decode temperature từ byte 1-2
                uint16_t tempInt = ((uint16_t)canMsg.data[1] << 8) | canMsg.data[2];
                newStatus.currentTemp = tempInt / 10.0;
                
                // Decode humidity từ byte 3-4
                uint16_t humInt = ((uint16_t)canMsg.data[3] << 8) | canMsg.data[4];
                newStatus.currentHum = humInt / 10.0;
                
                // Decode light từ byte 5-6
                uint16_t lightInt = ((uint16_t)canMsg.data[5] << 8) | canMsg.data[6];
                newStatus.currentLight = lightInt;
                
                newStatus.lastUpdate = millis();
                
                // Debug: In ra giá trị sau khi decode
                Serial.printf("Decoded - Temp: %.1f (raw: %d), Hum: %.1f (raw: %d), Light: %d\n", 
                               newStatus.currentTemp, tempInt,
                               newStatus.currentHum, humInt,
                               newStatus.currentLight);
                Serial.printf("Status flags - Fan: %d, LED: %d, Active: %d, Manual: %d\n",
                               newStatus.fanOn, newStatus.ledOn, 
                               newStatus.systemActive, newStatus.manualOverride);
                
                // Update global currentStatus safely
                if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    currentStatus = newStatus;
                    xSemaphoreGive(displayMutex);
                    Serial.println("Status updated successfully!");
                } else {
                    Serial.println("* WARNING: CAN Receive Task: Could not take displayMutex to update status!");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Check CAN every 50ms
    }
}

// --- Task for displaying information on OLED ---
void displayTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    bool oledWorking = true; // Assume OLED is working initially
    
    while (1) {
        // Take mutex to access shared data for display and check OLED status
        if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            
            // Check and try to reconnect OLED
            if (oledWorking) {
                Wire.beginTransmission(SCREEN_ADDRESS); 
                byte error = Wire.endTransmission();
                
                if (error != 0) {
                    Serial.println("Display Task: OLED connection lost, trying to reconnect...");
                    oledWorking = false;
                    delay(100); // Give time for OLED to reset
                    if(display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
                        oledWorking = true;
                        Serial.println("Display Task: OLED reconnected!");
                        display.clearDisplay(); // Clear screen after reconnecting
                    } else {
                        Serial.println("Display Task: OLED reconnection failed");
                    }
                }
            }
            
            if (oledWorking) {
                display.clearDisplay();
                display.setTextSize(1);
                display.setTextColor(SSD1306_WHITE);
                
                // Display title
                display.setCursor(0, 0);
                display.println("Smart Control System");
                display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
                
                // Display local sensor data
                display.setCursor(0, 15);
                display.printf("Temp: %.1fC", currentDataToSend.sensor.temperature); 
                
                display.setCursor(0, 25);
                display.printf("Humi: %.1f%%", currentDataToSend.sensor.humidity); 
                
                display.setCursor(0, 35);
                display.printf("Light: %d", currentDataToSend.sensor.lightLevel); 
                
                // Display actual DTM (STM32) status and mode
                display.setCursor(0, 45); // New line for actual relay status
                display.print(currentStatus.fanOn ? "FAN ON " : "FAN OFF ");
                display.print(currentStatus.ledOn ? "LED ON" : "LED OFF");

                display.setCursor(0, 55); // New line for mode
                display.print("Mode: ");
                display.print(currentStatus.manualOverride ? "MANUAL" : "AUTO");
                display.display();
            }
            
            xSemaphoreGive(displayMutex); // Release mutex
        } else {
            Serial.println("* WARNING: Display Task: Could not take displayMutex!");
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500)); // Update every 0.5 seconds
    }
}

// --- Blynk Handlers ---

// Instant ON/OFF button for Relay 1 (Virtual Pin V0)
BLYNK_WRITE(V0) {
  int pinValue = param.asInt();
  bool state = (pinValue == 1);
  Serial.print("Blynk: Button 1 (V0) -> ");
  Serial.println(state ? "ON" : "OFF");
  
  if (xSemaphoreTake(relayStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    relay1State = state;
    xSemaphoreGive(relayStateMutex);
  } else {
    Serial.println("* WARNING: V0: Could not take relayStateMutex!");
  }
  
  // Cập nhật trạng thái nút trên ứng dụng Blynk để đồng bộ
  Blynk.virtualWrite(V0, state ? 1 : 0); 

  // Optional: Dừng đếm ngược nếu nút tức thời được nhấn
  if (countdownTimerId1 != -1) {
    countdownTimer1.disable(countdownTimerId1);
    countdownTimerId1 = -1;
    remainingCountdownTime1 = 0; 
    Serial.println("Blynk: Countdown 1 stopped by V0 button.");
    Blynk.virtualWrite(V7, 0); // Cập nhật widget đếm ngược về 0
  }
}

// Instant ON/OFF button for Relay 2 (Virtual Pin V1)
BLYNK_WRITE(V1) {
  int pinValue = param.asInt();
  bool state = (pinValue == 1);
  Serial.print("Blynk: Button 2 (V1) -> ");
  Serial.println(state ? "ON" : "OFF");

  if (xSemaphoreTake(relayStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    relay2State = state;
    xSemaphoreGive(relayStateMutex);
  } else {
    Serial.println("* WARNING: V1: Could not take relayStateMutex!");
  }

  // Cập nhật trạng thái nút trên ứng dụng Blynk để đồng bộ
  Blynk.virtualWrite(V1, state ? 1 : 0); 

  // Optional: Dừng đếm ngược nếu nút tức thời được nhấn
  if (countdownTimerId2 != -1) {
    countdownTimer2.disable(countdownTimerId2);
    countdownTimerId2 = -1;
    remainingCountdownTime2 = 0; 
    Serial.println("Blynk: Countdown 2 stopped by V1 button.");
    Blynk.virtualWrite(V8, 0); // Cập nhật widget đếm ngược về 0
  }
}

// Countdown timer for Relay 1 (Virtual Pin V2)
BLYNK_WRITE(V2) {
  int inputSeconds = param.asInt(); 

  if (inputSeconds >= 0) { 
    remainingCountdownTime1 = inputSeconds; 
    Serial.print("Blynk: Received countdown time for Relay 1 (V2): ");
    Serial.print(remainingCountdownTime1);
    Serial.println(" seconds.");

    if (countdownTimerId1 != -1) {
      countdownTimer1.disable(countdownTimerId1);
      countdownTimerId1 = -1; 
      Serial.println("Blynk: Previous countdown 1 stopped.");
    }

    if (inputSeconds > 0) { 
      countdownTimerId1 = countdownTimer1.setInterval(1000L, [](){
        if (remainingCountdownTime1 > 0) {
          remainingCountdownTime1--;
          // Optional: Send current countdown time back to Blynk if you have a widget
          Blynk.virtualWrite(V7, remainingCountdownTime1); 
        } else {
          Serial.println("Blynk: Countdown 1 finished! Toggling Relay 1...");
          countdownTimer1.disable(countdownTimerId1);
          countdownTimerId1 = -1; 
          
          // Toggle the current state of Relay 1 safely
          if (xSemaphoreTake(relayStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            relay1State = !relay1State; 
            Serial.print("Blynk: Relay 1 toggled to ");
            Serial.println(relay1State ? "ON" : "OFF");
            xSemaphoreGive(relayStateMutex);
          } else {
            Serial.println("* WARNING: V2 Timer: Could not take relayStateMutex!");
          }
          Blynk.virtualWrite(V0, relay1State ? 1 : 0); // Update instant button state after toggle
          Blynk.virtualWrite(V7, 0); // Update countdown display to 0
        }
      });
    } else { // If inputSeconds is 0, toggle immediately and stop any countdown
        Serial.println("Blynk: Countdown 1 received 0, toggling Relay 1 immediately.");
        if (xSemaphoreTake(relayStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            relay1State = !relay1State; 
            Serial.print("Blynk: Relay 1 toggled to ");
            Serial.println(relay1State ? "ON" : "OFF");
            xSemaphoreGive(relayStateMutex);
        } else {
             Serial.println("* WARNING: V2 Immediate: Could not take relayStateMutex!");
        }
        Blynk.virtualWrite(V0, relay1State ? 1 : 0); // Update instant button state after toggle
        Blynk.virtualWrite(V7, 0); 
    }

  } else { 
    Serial.println("Blynk: Received invalid countdown time for Relay 1 (negative).");
  }
}

// Countdown timer for Relay 2 (Virtual Pin V3)
BLYNK_WRITE(V3) {
  int inputSeconds = param.asInt(); 

  if (inputSeconds >= 0) { 
    remainingCountdownTime2 = inputSeconds; 
    Serial.print("Blynk: Received countdown time for Relay 2 (V3): ");
    Serial.print(remainingCountdownTime2);
    Serial.println(" seconds.");

    if (countdownTimerId2 != -1) {
      countdownTimer2.disable(countdownTimerId2);
      countdownTimerId2 = -1; 
      Serial.println("Blynk: Previous countdown 2 stopped.");
    }

    if (inputSeconds > 0) { 
      // Start a new countdown timer
      countdownTimerId2 = countdownTimer2.setInterval(1000L, [](){
        if (remainingCountdownTime2 > 0) {
          remainingCountdownTime2--;
          // Optional: Send current countdown time back to Blynk
          Blynk.virtualWrite(V8, remainingCountdownTime2); 
        } else {
          Serial.println("Blynk: Countdown 2 finished! Toggling Relay 2...");
          countdownTimer2.disable(countdownTimerId2);
          countdownTimerId2 = -1; 
          
          // Toggle the current state of Relay 2 safely
          if (xSemaphoreTake(relayStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            relay2State = !relay2State; 
            Serial.print("Blynk: Relay 2 toggled to ");
            Serial.println(relay2State ? "ON" : "OFF");
            xSemaphoreGive(relayStateMutex);
          } else {
            Serial.println("* WARNING: V3 Timer: Could not take relayStateMutex!");
          }
          Blynk.virtualWrite(V1, relay2State ? 1 : 0); // Update instant button state after toggle
          Blynk.virtualWrite(V8, 0); 
        }
      });
    } else { 
        Serial.println("Blynk: Countdown 2 received 0, toggling Relay 2 immediately.");
        if (xSemaphoreTake(relayStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            relay2State = !relay2State; 
            Serial.print("Blynk: Relay 2 toggled to ");
            Serial.println(relay2State ? "ON" : "OFF");
            xSemaphoreGive(relayStateMutex);
        } else {
            Serial.println("* WARNING: V3 Immediate: Could not take relayStateMutex!");
        }
        Blynk.virtualWrite(V1, relay2State ? 1 : 0);
        Blynk.virtualWrite(V8, 0);
    }
  } else { 
    Serial.println("Blynk: Received invalid countdown time for Relay 2 (negative).");
  }
}