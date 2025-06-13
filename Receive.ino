#include <MapleFreeRTOS900.h> // Ensure this library is correctly installed for your STM32 board
#include <SPI.h>
#include <mcp2515.h>

// --- Hardware Pin Configuration ---
#define SPI_CS PA4      // Chip Select (CS) CS pin for MCP2515
#define FAN_PIN PB12    // GPIO pin for Fan control (connected to Relay 1)
#define LED_PIN PB13    // GPIO pin for LED control (connected to Relay 2)

// --- Sensor Thresholds for Automatic Mode ---
#define TEMP_THRESHOLD  29.0    // Temperature threshold (e.g., 30°C)
#define HUM_THRESHOLD   70.0    // Humidity threshold (e.g., 70%)
#define LIGHT_THRESHOLD 2000    // Light threshold (ADC value, e.g., 2000)

// --- FreeRTOS Task Priorities ---
#define CAN_TASK_PRIORITY         3 // Higher priority for reliable CAN reception
#define CONTROL_TASK_PRIORITY     2 // Controls devices based on data/commands
#define STATUS_SEND_TASK_PRIORITY 1 // Sends status feedback to ESP32

// --- CAN IDs ---
#define SENSOR_DATA_AND_RELAY_STATUS_ID 0x036 // ID from ESP32 (sensor data + Blynk relay commands)
#define STATUS_DATA_ID 0x037                  // ID for DTM to send its status back to ESP32

// --- MCP2515 Object and Mutex ---
MCP2515* mcp2515;
struct can_frame canMsg;
SemaphoreHandle_t dataMutex; // Mutex to protect currentData and controlStatus

// --- Sensor Data Structure (received from ESP32) ---
struct SensorData {
    float temperature;
    float humidity;
    int lightLevel;
    bool dataValid;         // Flag if received data is valid
    unsigned long lastUpdate; // Timestamp of last valid data reception
    bool blynkFanCommand;   // Fan control command from Blynk (via ESP32)
    bool blynkLedCommand;   // LED control command from Blynk (via ESP32)
    bool blynkIsDisconnected; // True if ESP32 reports Blynk is disconnected
};

// --- Control Status Structure (DTM's internal state) ---
struct ControlStatus {
    bool fanOn;         // Current ON/OFF state of Fan
    bool ledOn;         // Current ON/OFF state of LED
    bool systemActive;  // True if DTM is actively receiving data from ESP32
    bool manualOverride; // True if DTM is in manual control mode (from Blynk)
    float currentTemp;  // Local copy of temperature (from ESP32's sensor)
    float currentHum;   // Local copy of humidity
    int currentLight;   // Local copy of light level
    bool blynkIsDisconnected; // Track Blynk connection status
};

SensorData currentData = {0}; // Stores received sensor data and Blynk commands
ControlStatus controlStatus = {false, false, false, false, 0, 0, 0, false}; // Initialized with blynkIsDisconnected

// --- FreeRTOS Task Prototypes ---
void canReceiveTask(void *parameter);
void controlTask(void *parameter);
void statusSendTask(void *parameter);

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n--- DTM (Data Terminal Module) Receiver/Controller ---");

    // --- Initialize GPIO pins for Fan and LED ---
    pinMode(FAN_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(FAN_PIN, LOW); // Ensure fan is OFF on startup
    digitalWrite(LED_PIN, LOW); // Ensure LED is OFF on startup
    Serial.println("Fan and LED Pins Initialized to OFF.");
    
    // --- Initialize SPI and MCP2515 ---
    SPI.begin(); // Uses default SPI pins for STM32 (e.g., PA5, PA6, PA7)
    mcp2515 = new MCP2515(SPI_CS);
    mcp2515->reset();
    mcp2515->setBitrate(CAN_125KBPS, MCP_8MHZ); // Set CAN bitrate to 125KBPS
    mcp2515->setNormalMode(); // Set MCP2515 to normal operation mode
    Serial.println("CAN Bus initialized successfully.");
    
    // --- Create Mutex ---
    dataMutex = xSemaphoreCreateMutex();
    if (dataMutex == NULL) {
        Serial.println("* ERROR: Failed to create dataMutex!!! System halted.");
        while(1); // Halt if mutex creation fails
    } else {
        Serial.println("Data Mutex created successfully.");
    }
    
    // --- Create FreeRTOS Tasks ---
    // Increase stack size (e.g., 512, 1024) if you encounter crashes or "Guru Meditation Errors"
    xTaskCreate(canReceiveTask, "CANReceiveTask", 512, NULL, CAN_TASK_PRIORITY, NULL); 
    xTaskCreate(controlTask, "ControlTask", 512, NULL, CONTROL_TASK_PRIORITY, NULL);    
    xTaskCreate(statusSendTask, "StatusSendTask", 512, NULL, STATUS_SEND_TASK_PRIORITY, NULL); 
    
    Serial.println("All Tasks created successfully!");
    
    // Start the FreeRTOS scheduler
    vTaskStartScheduler();
}

void loop() {
    // loop() is typically empty in FreeRTOS applications as tasks handle all logic.
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// --- Task for receiving data via CAN Bus ---
void canReceiveTask(void *parameter) {
    while (1) {
        if (mcp2515->readMessage(&canMsg) == MCP2515::ERROR_OK) {
            if (canMsg.can_id == SENSOR_DATA_AND_RELAY_STATUS_ID) {
                Serial.println("CAN Receive Task: Received Sensor/Relay Status (ID 0x036)");
                SensorData newData;

                // Decode sensor data
                int tempInt = (canMsg.data[0] << 8) | canMsg.data[1];
                newData.temperature = tempInt / 10.0;

                int humInt = (canMsg.data[2] << 8) | canMsg.data[3];
                newData.humidity = humInt / 10.0;

                newData.lightLevel = (canMsg.data[4] << 8) | canMsg.data[5];

                // Decode Relay commands from Byte 6 (Bit 0 for Fan/Relay1, Bit 1 for LED/Relay2, Bit 7 for Blynk disconnected)
                newData.blynkFanCommand = (canMsg.data[6] & 0x01) != 0;
                newData.blynkLedCommand = (canMsg.data[6] & 0x02) != 0;
                newData.blynkIsDisconnected = (canMsg.data[6] & (1 << 7)) != 0; // Decode Blynk connection status

                newData.dataValid = true;
                newData.lastUpdate = millis();

                // Update currentData and controlStatus safely using Mutex
                if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    currentData = newData; // Copy all received data

                    // Cập nhật trạng thái kết nối Blynk của DTM
                    controlStatus.blynkIsDisconnected = newData.blynkIsDisconnected;

                    // --- LOGIC CHO MANUAL OVERRIDE ---
                    if (controlStatus.blynkIsDisconnected) {
                        // Nếu Blynk mất kết nối, luôn về chế độ tự động
                        controlStatus.manualOverride = false;
                        Serial.println("CAN Receive Task: Blynk disconnected. Forced to AUTO mode.");
                    } else {
                        // Nếu Blynk đang kết nối, luôn ở chế độ thủ công (cho phép điều khiển hoàn toàn qua Blynk)
                        controlStatus.manualOverride = true;
                        Serial.println("CAN Receive Task: Blynk connected. Manual Override ACTIVE.");
                    }
                    xSemaphoreGive(dataMutex);
                } else {
                    Serial.println("* WARNING: CAN Receive Task: Failed to take dataMutex for update!");
                }
            } else {
                Serial.print("CAN Receive Task: Received unknown CAN ID: 0x");
                Serial.println(canMsg.can_id, HEX);
            }
        }

        // Check for data timeout from ESP32
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (currentData.dataValid && (millis() - currentData.lastUpdate > 5000)) { // 5-second timeout
                Serial.println("* WARNING: CAN Receive Task: Sensor data from ESP32 timed out!");
                currentData.dataValid = false;
                controlStatus.systemActive = false;
                // Nếu dữ liệu hết hạn, vô hiệu hóa manual override để chuyển về chế độ an toàn/tự động
                controlStatus.manualOverride = false;
                Serial.println("CAN Receive Task: Manual Override DISABLED due to data timeout.");
            } else if (currentData.dataValid) {
                controlStatus.systemActive = true; // Data is valid and recently updated
            }
            xSemaphoreGive(dataMutex);
        } else {
            Serial.println("* WARNING: CAN Receive Task: Failed to take dataMutex for timeout check!");
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // Check CAN bus every 50ms
    }
}

// --- Task for controlling devices (Fan, LED) ---
void controlTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        SensorData localData;
        bool fanShouldBeOn = false;
        bool ledShouldBeOn = false;

        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            localData = currentData;
            
            // Cập nhật các giá trị cảm biến cục bộ của DTM để gửi phản hồi
            controlStatus.currentTemp = localData.temperature;
            controlStatus.currentHum = localData.humidity;
            controlStatus.currentLight = localData.lightLevel;

            if (controlStatus.manualOverride) {
                // Chế độ thủ công: theo lệnh từ Blynk (qua ESP32)
                fanShouldBeOn = localData.blynkFanCommand;
                ledShouldBeOn = localData.blynkLedCommand;
                Serial.println("Control Task: Manual Override Active!");
            } else {
                // Chế độ tự động: điều khiển dựa trên ngưỡng cảm biến
                Serial.println("Control Task: Automatic Mode Active!");
                if (localData.dataValid) {
                    // Logic điều khiển quạt: Bật nếu nhiệt độ HOẶC độ ẩm vượt ngưỡng
                    // (Điều chỉnh thành && nếu bạn muốn cả hai điều kiện cùng đúng)
                    fanShouldBeOn = localData.temperature > TEMP_THRESHOLD || localData.humidity < HUM_THRESHOLD; 
                    ledShouldBeOn = localData.lightLevel > LIGHT_THRESHOLD; 
                } else {
                    // Nếu không có dữ liệu hợp lệ, tắt thiết bị để an toàn
                    fanShouldBeOn = false;
                    ledShouldBeOn = false;
                    Serial.println("Control Task: No valid sensor data, turning OFF devices.");
                }
            }

            // Cập nhật trạng thái điều khiển nội bộ của DTM
            controlStatus.fanOn = fanShouldBeOn;
            controlStatus.ledOn = ledShouldBeOn;

            xSemaphoreGive(dataMutex);
        } else {
            Serial.println("* WARNING: Control Task: Failed to take dataMutex!");
        }

        // Điều khiển các chân GPIO thực tế
        digitalWrite(FAN_PIN, fanShouldBeOn ? HIGH : LOW); // Giả sử relay Active-HIGH
        digitalWrite(LED_PIN, ledShouldBeOn ? HIGH : LOW); // Giả sử relay Active-HIGH

        Serial.print("Control Task: Fan ");
        Serial.print(fanShouldBeOn ? "ON" : "OFF");
        Serial.print(", LED ");
        Serial.print(ledShouldBeOn ? "ON" : "OFF");
        Serial.print(" (Mode: ");
        Serial.print(controlStatus.manualOverride ? "MANUAL" : "AUTO");
        Serial.print(", Blynk Disconnected: ");
        Serial.print(controlStatus.blynkIsDisconnected ? "YES" : "NO");
        Serial.println(")");

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000)); // Cập nhật mỗi 1 giây
    }
}

// --- Task for sending status feedback back to ESP32 ---
void statusSendTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    struct can_frame statusMsg;
    
    while (1) {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            statusMsg.can_id = STATUS_DATA_ID; // Use the DTM's status ID
            statusMsg.can_dlc = 8;
            
            // Byte 0: DTM's device states and system active status
            statusMsg.data[0] = 0;
            if (controlStatus.fanOn) statusMsg.data[0] |= 0x01;      // Bit 0: Actual Fan state
            if (controlStatus.ledOn) statusMsg.data[0] |= 0x02;      // Bit 1: Actual LED state
            if (controlStatus.systemActive) statusMsg.data[0] |= 0x04; // Bit 2: DTM's system active (receiving data)
            if (controlStatus.manualOverride) statusMsg.data[0] |= 0x08; // Bit 3: DTM in manual override mode
            // Bit 4: Thêm trạng thái Blynk Disconnected vào byte 0 để gửi về ESP32
            // Hiện tại ESP32 đã có trạng thái này, nhưng nếu muốn DTM gửi lại thì có thể thêm vào đây
            // if (controlStatus.blynkIsDisconnected) statusMsg.data[0] |= 0x10; // Bit 4

            // Byte 1-2: Temperature (Đảm bảo giá trị không âm và trong giới hạn uint16_t)
            int tempInt = (int)(controlStatus.currentTemp * 10);
            if (tempInt < 0) tempInt = 0; 
            if (tempInt > 65535) tempInt = 65535; // Giới hạn max cho uint16_t
            statusMsg.data[1] = (tempInt >> 8) & 0xFF;   // High byte
            statusMsg.data[2] = tempInt & 0xFF;          // Low byte
            
            // Byte 3-4: Humidity (Đảm bảo giá trị không âm và trong giới hạn uint16_t)
            int humInt = (int)(controlStatus.currentHum * 10);
            if (humInt < 0) humInt = 0;
            if (humInt > 65535) humInt = 65535;
            statusMsg.data[3] = (humInt >> 8) & 0xFF;    // High byte
            statusMsg.data[4] = humInt & 0xFF;           // Low byte
            
            // Byte 5-6: Light Level (Đảm bảo giá trị không âm và trong giới hạn uint16_t)
            uint16_t lightValue = (uint16_t)controlStatus.currentLight;
            // if (lightValue < 0) lightValue = 0; // Thực ra lightValue là uint16_t thì không cần check <0
            statusMsg.data[5] = (lightValue >> 8) & 0xFF; // High byte
            statusMsg.data[6] = lightValue & 0xFF;        // Low byte
            statusMsg.data[7] = 0x00; // Reserved
            
            xSemaphoreGive(dataMutex); // Release mutex after data is copied
            
            // Send message via CAN Bus
            if (mcp2515->sendMessage(&statusMsg) == MCP2515::ERROR_OK) {
                Serial.println("Status Send Task: Sent DTM Status message.");
            } else {
                Serial.println("* ERROR: Status Send Task: Failed to send CAN message!");
            }
        } else {
            Serial.println("* WARNING: Status Send Task: Failed to take dataMutex!");
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2000)); // Send status every 2 seconds
    }
}