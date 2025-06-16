#include <SPI.h>
#include <mcp2515.h>
#include <MapleFreeRTOS900.h>

#define SPI_CS      PA4   
#define FAN_PIN     PB12   
#define LED_PIN     PB13 

#define CAN_TASK_PRIORITY           3 
#define CONTROL_TASK_PRIORITY       2
#define STATUS_SEND_TASK_PRIORITY   1

#define SENSOR_DATA_AND_RELAY_STATUS_ID     0x036 
#define STATUS_DATA_ID                      0x037

#define TEMP_THRESHOLD      29.0   
#define HUM_THRESHOLD       70.0   
#define LIGHT_THRESHOLD     2000  

MCP2515* mcp2515;
struct can_frame canMsg;

SemaphoreHandle_t dataMutex;

struct SensorData {
    float temperature;
    float humidity;
    int lightLevel;
    unsigned long lastUpdate;
    bool blynkFanCommand;  
    bool blynkLedCommand;  
    bool blynkIsConnected;
};

struct ControlStatus {
    bool fanOn;    
    bool ledOn;    
    bool manualOverride; 
    float currentTemp; 
    float currentHum;
    int currentLight;  
};

SensorData sensorData = {0};
ControlStatus controlStatus = {0};

void canReceiveTask(void *parameter);
void controlTask(void *parameter);
void statusSendTask(void *parameter);

void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(FAN_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(FAN_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    
    SPI.begin();
    mcp2515 = new MCP2515(SPI_CS);
    mcp2515->reset();
    mcp2515->setBitrate(CAN_125KBPS, MCP_8MHZ);
    mcp2515->setNormalMode();
    
    dataMutex = xSemaphoreCreateMutex();
    
    xTaskCreate(canReceiveTask, "CANReceiveTask", 256, NULL, CAN_TASK_PRIORITY, NULL); 
    xTaskCreate(controlTask, "ControlTask", 256, NULL, CONTROL_TASK_PRIORITY, NULL);    
    xTaskCreate(statusSendTask, "StatusSendTask", 256, NULL, STATUS_SEND_TASK_PRIORITY, NULL); 
    
    vTaskStartScheduler();
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void canReceiveTask(void *parameter) {
    while (1) {
        if (mcp2515->readMessage(&canMsg) == MCP2515::ERROR_OK) {
            if (canMsg.can_id == SENSOR_DATA_AND_RELAY_STATUS_ID) {
                if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    int tempInt = (canMsg.data[0] << 8) | canMsg.data[1];
                    sensorData.temperature = tempInt / 10.0;
                    int humInt = (canMsg.data[2] << 8) | canMsg.data[3];
                    sensorData.humidity = humInt / 10.0;
                    int lightLevel = (canMsg.data[4] << 8) | canMsg.data[5];
                    sensorData.lightLevel = lightLevel;

                    sensorData.blynkFanCommand = (canMsg.data[6] & 0x01) != 0;
                    sensorData.blynkLedCommand = (canMsg.data[6] & 0x02) != 0;
                    sensorData.blynkIsConnected = (canMsg.data[6] & (1 << 7)) != 0;

                    sensorData.lastUpdate = millis();

                    xSemaphoreGive(dataMutex);
                }
            }
        }

        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (sensorData.blynkIsConnected && (millis() - sensorData.lastUpdate <= 5000)) {
                controlStatus.manualOverride = true; 
            } else {
                controlStatus.manualOverride = false;
            }

            xSemaphoreGive(dataMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void controlTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            controlStatus.currentTemp = sensorData.temperature;
            controlStatus.currentHum = sensorData.humidity;
            controlStatus.currentLight = sensorData.lightLevel;

            if (controlStatus.manualOverride) {
                controlStatus.fanOn = sensorData.blynkFanCommand;
                controlStatus.ledOn = sensorData.blynkLedCommand;
            } else {
                controlStatus.fanOn = controlStatus.currentTemp > TEMP_THRESHOLD || controlStatus.currentHum < HUM_THRESHOLD; 
                controlStatus.ledOn = controlStatus.currentLight > LIGHT_THRESHOLD;
            }

            xSemaphoreGive(dataMutex);
        }

        digitalWrite(FAN_PIN, controlStatus.fanOn ? HIGH : LOW); 
        digitalWrite(LED_PIN, controlStatus.ledOn ? HIGH : LOW); 

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(300)); 
    }
}

void statusSendTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    struct can_frame statusMsg;
    
    while (1) {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            statusMsg.can_id = STATUS_DATA_ID;
            statusMsg.can_dlc = 8;
                
            statusMsg.data[0] = 0;
            if (controlStatus.fanOn) statusMsg.data[0] |= 0x01;
            if (controlStatus.ledOn) statusMsg.data[0] |= 0x02;
            if (controlStatus.manualOverride) statusMsg.data[0] |= 0x04;

            int tempInt = (int)(controlStatus.currentTemp * 10);
            statusMsg.data[1] = (tempInt >> 8) & 0xFF; 
            statusMsg.data[2] = tempInt & 0xFF;        
            int humInt = (int)(controlStatus.currentHum * 10);
            statusMsg.data[3] = (humInt >> 8) & 0xFF;   
            statusMsg.data[4] = humInt & 0xFF;        
            int lightValue = controlStatus.currentLight;
            statusMsg.data[5] = (lightValue >> 8) & 0xFF;
            statusMsg.data[6] = lightValue & 0xFF;    

            statusMsg.data[7] = 0x00;
            
            mcp2515->sendMessage(&statusMsg);

            xSemaphoreGive(dataMutex);
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(300));
    }
}