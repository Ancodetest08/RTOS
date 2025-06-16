#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL6N61gDFsZ"
#define BLYNK_TEMPLATE_NAME "Intelligent Switchs"

char auth[] = "PF2jm_0c-dxjAUga_sc39ZoCjh2CeBrG";
char ssid[] = "ChanGaSaTac";   
char pass[] = "12345432123454321";

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <SPI.h>
#include <mcp2515.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SPI_SCK     6
#define SPI_MOSI    7
#define SPI_MISO    2
#define SPI_CS      10
#define DHT_PIN     4
#define LM393_PIN   3 
#define I2C_SDA     5
#define I2C_SCL     8

#define BLYNK_TASK_PRIORITY         3
#define CAN_TRANSMIT_TASK_PRIORITY  2
#define CAN_RECEIVE_TASK_PRIORITY   2
#define SENSOR_TASK_PRIORITY        1
#define DISPLAY_TASK_PRIORITY       1

#define SENSOR_DATA_AND_RELAY_STATUS_ID   0x036
#define STATUS_FROM_RECEIVER_ID           0x037

SPIClass *spiAcc = nullptr;
#undef SPI
#define SPI (*spiAcc)
struct can_frame canMsg;
MCP2515 mcp2515(SPI_CS);
DHT dht(DHT_PIN, DHT22);
Adafruit_SSD1306 display(128, 64, &Wire, -1);

QueueHandle_t sensorAndRelayQueue; 
SemaphoreHandle_t displayMutex;  
SemaphoreHandle_t relayStateMutex; 

struct SensorData {
  float temperature;
  float humidity;
  int lightLevel;
};

struct SensorAndRelayData {
  SensorData sensor;
  bool relay1;
  bool relay2;
  bool blynkIsConnected;
};

struct StatusData {
  bool fanOn;    
  bool ledOn;    
  bool manualOverride;  
  float currentTemp;  
  float currentHum; 
  int currentLight;  
};

SensorAndRelayData sensorAndRelayData = {0};
StatusData statusData = {0};

BlynkTimer countdownTimer1;
BlynkTimer countdownTimer2;
int remainingCountdownTime1 = 0; 
int countdownTimerId1 = -1;
int remainingCountdownTime2 = 0; 
int countdownTimerId2 = -1;

void blynkTask(void *pvParameters);
void canTransmitTask(void *parameter);
void canReceiveTask(void *parameter); 
void sensorTask(void *parameter);
void displayTask(void *parameter);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("==============================================================");
  Serial.println("========== ESP32C3 Smart Control System Starting... ==========");
  Serial.println("==============================================================");
  
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("System Starting...");
  display.display();

  dht.begin();
  pinMode(LM393_PIN, INPUT);
  
  spiAcc = new SPIClass(FSPI);
  spiAcc->begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  
  sensorAndRelayQueue = xQueueCreate(5, sizeof(SensorAndRelayData));
  displayMutex = xSemaphoreCreateMutex();
  relayStateMutex = xSemaphoreCreateMutex(); 
  
  xTaskCreate(blynkTask, "BlynkTask", 8192, NULL, BLYNK_TASK_PRIORITY, NULL); 
  xTaskCreate(canTransmitTask, "CANTransmitTask", 4096, NULL, CAN_TRANSMIT_TASK_PRIORITY, NULL); 
  xTaskCreate(canReceiveTask, "CANReceiveTask", 4096, NULL, CAN_RECEIVE_TASK_PRIORITY, NULL); 
  xTaskCreate(sensorTask, "SensorTask", 8192, NULL, SENSOR_TASK_PRIORITY, NULL);
  xTaskCreate(displayTask, "DisplayTask", 8192, NULL, DISPLAY_TASK_PRIORITY, NULL);
}

void loop() {}

void blynkTask(void *pvParameters) {
  WiFi.begin(ssid, pass);

  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Blynk Task: Disconnected to WiFi!");
      sensorAndRelayData.blynkIsConnected = false;
      WiFi.begin(ssid, pass);
      vTaskDelay(pdMS_TO_TICKS(2000));
      continue;
    }

    Serial.println("Blynk Task: Connected to WiFi!");

    if (!Blynk.connected()) {
      Serial.println("Blynk Task: Disconnected to Blynk!");
      Blynk.config(auth);
      Blynk.connect(5000);
      sensorAndRelayData.blynkIsConnected = false;
    } else {
      Serial.println("Blynk Task: Connected to Blynk!");
      Blynk.virtualWrite(V0, statusData.fanOn ? 1 : 0);
      Blynk.virtualWrite(V1, statusData.ledOn ? 1 : 0);
      sensorAndRelayData.blynkIsConnected = true;
      Blynk.run();
      countdownTimer1.run();
      countdownTimer2.run();
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void canTransmitTask(void *parameter) {
  SensorAndRelayData tempCanTransmitTask;

  while (1) {    
    if (xQueueReceive(sensorAndRelayQueue, &tempCanTransmitTask, pdMS_TO_TICKS(500)) == pdTRUE) {
      canMsg.can_id = SENSOR_DATA_AND_RELAY_STATUS_ID;
      canMsg.can_dlc = 8;

      int tempInt = (int)(tempCanTransmitTask.sensor.temperature * 10);
      canMsg.data[0] = (tempInt >> 8) & 0xFF;
      canMsg.data[1] = tempInt & 0xFF;
      int humInt = (int)(tempCanTransmitTask.sensor.humidity * 10);
      canMsg.data[2] = (humInt >> 8) & 0xFF; 
      canMsg.data[3] = humInt & 0xFF;        
      int lightInt = tempCanTransmitTask.sensor.lightLevel;
      canMsg.data[4] = (lightInt >> 8) & 0xFF; 
      canMsg.data[5] = lightInt & 0xFF;    

      canMsg.data[6] = 0; 
      if (tempCanTransmitTask.relay1) canMsg.data[6] |= 0x01;
      if (tempCanTransmitTask.relay2) canMsg.data[6] |= 0x02;

      if (tempCanTransmitTask.blynkIsConnected) {
        canMsg.data[6] |= (1 << 7);
      } else {
        canMsg.data[6] &= ~(1 << 7);
      }

      canMsg.data[7] = 0x00;

      if (mcp2515.sendMessage(&canMsg) != MCP2515::ERROR_OK) {
        Serial.println("***** ERROR: CAN Transmit Task: Failed to send CAN message!!!");
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

void canReceiveTask(void *parameter) {
  while (1) {
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
      if (canMsg.can_id == STATUS_FROM_RECEIVER_ID) {              
        statusData.fanOn = (canMsg.data[0] & 0x01) != 0;     
        statusData.ledOn = (canMsg.data[0] & 0x02) != 0;  
        statusData.manualOverride = (canMsg.data[0] & 0x04) != 0;
                        
        uint16_t tempInt = ((uint16_t)canMsg.data[1] << 8) | canMsg.data[2];
        statusData.currentTemp = tempInt / 10.0;
        uint16_t humInt = ((uint16_t)canMsg.data[3] << 8) | canMsg.data[4];
        statusData.currentHum = humInt / 10.0;
        uint16_t lightInt = ((uint16_t)canMsg.data[5] << 8) | canMsg.data[6];
        statusData.currentLight = lightInt;
      }
    } else {
      Serial.println("***** ERROR: CAN Receive Task: Failed to receive CAN message!!!");
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void sensorTask(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  SensorAndRelayData tempSensorTask;
    
  while (1) {
    tempSensorTask = sensorAndRelayData;

    tempSensorTask.sensor.temperature = dht.readTemperature();
    tempSensorTask.sensor.humidity = dht.readHumidity();
    tempSensorTask.sensor.lightLevel = analogRead(LM393_PIN);
        
    Serial.println("========== Sensor Readings (ESP32) ==========");
    Serial.printf(">>>>> Temperature: %.2f°C\n", tempSensorTask.sensor.temperature);
    Serial.printf(">>>>> Humidity: %.2f%%\n", tempSensorTask.sensor.humidity);
    Serial.printf(">>>>> Light Level: %d\n", tempSensorTask.sensor.lightLevel);

    if (xQueueSend(sensorAndRelayQueue, &tempSensorTask, pdMS_TO_TICKS(100)) != pdTRUE) {
      Serial.println("***** ERROR: Failed to send sensor data to queue!!!");
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2000));
  }
}

void displayTask(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
    
  while (1) {
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("Smart Control System");
      display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
        
      display.setCursor(0, 15);
      display.printf("Temp: %.1fC", statusData.currentTemp); 
      display.setCursor(0, 25);
      display.printf("Humi: %.1f%%", statusData.currentHum); 
      display.setCursor(0, 35);
      display.printf("Light: %d", statusData.currentLight); 
        
      display.setCursor(0, 45);
      display.print(statusData.fanOn ? "FAN ON " : "FAN OFF ");
      display.print(statusData.ledOn ? "LED ON" : "LED OFF");

      display.setCursor(0, 55);
      display.print("Mode: ");
      display.print(statusData.manualOverride ? "MANUAL" : "AUTO");
      display.display();

      xSemaphoreGive(displayMutex);
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
  }
}

BLYNK_WRITE(V0) {
  if (xSemaphoreTake(relayStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    int pinValue = param.asInt();
    sensorAndRelayData.relay1 = (pinValue == 1);
    Serial.print("Blynk: Button FAN -> ");
    Serial.println(sensorAndRelayData.relay1 ? "ON" : "OFF");
      
    Blynk.virtualWrite(V0, sensorAndRelayData.relay1 ? 1 : 0); 

    xSemaphoreGive(relayStateMutex);

    if (countdownTimerId1 != -1) {
      countdownTimer1.disable(countdownTimerId1);
      countdownTimerId1 = -1;
      remainingCountdownTime1 = 0; 
      Serial.println("Blynk: Countdown FAN stopped by button FAN.");
    }
  }
}

BLYNK_WRITE(V1) {
  if (xSemaphoreTake(relayStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    int pinValue = param.asInt();
    sensorAndRelayData.relay2 = (pinValue == 1);
    Serial.print("Blynk: Button LED -> ");
    Serial.println(sensorAndRelayData.relay2 ? "ON" : "OFF");

    Blynk.virtualWrite(V1, sensorAndRelayData.relay2 ? 1 : 0); 

    xSemaphoreGive(relayStateMutex);

    if (countdownTimerId2 != -1) {
      countdownTimer2.disable(countdownTimerId2);
      countdownTimerId2 = -1;
      remainingCountdownTime2 = 0; 
      Serial.println("Blynk: Countdown LED stopped by button LED.");
    }
  }
}

BLYNK_WRITE(V2) {
  if (xSemaphoreTake(relayStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    int inputSeconds = param.asInt(); 

    if (inputSeconds >= 0) { 
      remainingCountdownTime1 = inputSeconds; 
      Serial.print("Blynk: Received countdown time for Relay FAN: ");
      Serial.print(remainingCountdownTime1);
      Serial.println(" seconds.");

      if (countdownTimerId1 != -1) {
        countdownTimer1.disable(countdownTimerId1);
        countdownTimerId1 = -1;
        Serial.println("Blynk: Previous countdown FAN stopped.");
      }

      if (inputSeconds > 0) { 
        countdownTimerId1 = countdownTimer1.setInterval(1000L, []() {
          if (remainingCountdownTime1 > 0) {
            remainingCountdownTime1--;
          } else {
            Serial.println("Blynk: Countdown FAN finished! Toggling Relay FAN...");
            countdownTimer1.disable(countdownTimerId1);
            countdownTimerId1 = -1;
            
            sensorAndRelayData.relay1 = !sensorAndRelayData.relay1; 
            Serial.print("Blynk: Relay FAN toggled to ");
            Serial.println(sensorAndRelayData.relay1 ? "ON" : "OFF");

            Blynk.virtualWrite(V0, sensorAndRelayData.relay1 ? 1 : 0); 
          }
        });
      } else { 
        Serial.println("Blynk: Countdown FAN received 0, toggling Relay FAN immediately.");
        sensorAndRelayData.relay1 = !sensorAndRelayData.relay1; 
        Serial.print("Blynk: Relay FAN toggled to ");
        Serial.println(sensorAndRelayData.relay1 ? "ON" : "OFF");

        Blynk.virtualWrite(V0, sensorAndRelayData.relay1 ? 1 : 0); 
      }
    }

    xSemaphoreGive(relayStateMutex);
  }
}

BLYNK_WRITE(V3) {
  if (xSemaphoreTake(relayStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    int inputSeconds = param.asInt(); 

    if (inputSeconds >= 0) { 
      remainingCountdownTime2 = inputSeconds; 
      Serial.print("Blynk: Received countdown time for Relay LED: ");
      Serial.print(remainingCountdownTime2);
      Serial.println(" seconds.");

      if (countdownTimerId2 != -1) {
        countdownTimer2.disable(countdownTimerId2);
        countdownTimerId2 = -1; 
        Serial.println("Blynk: Previous countdown LED stopped.");
      }

      if (inputSeconds > 0) { 
        countdownTimerId2 = countdownTimer2.setInterval(1000L, []() {
          if (remainingCountdownTime2 > 0) {
            remainingCountdownTime2--;
          } else {
            Serial.println("Blynk: Countdown LED finished! Toggling Relay LED...");
            countdownTimer2.disable(countdownTimerId2);
            countdownTimerId2 = -1; 
            
            sensorAndRelayData.relay2 = !sensorAndRelayData.relay2; 
            Serial.print("Blynk: Relay LED toggled to ");
            Serial.println(sensorAndRelayData.relay2 ? "ON" : "OFF");

            Blynk.virtualWrite(V1, sensorAndRelayData.relay2 ? 1 : 0); 
          }
        });
      } else { 
        Serial.println("Blynk: Countdown LED received 0, toggling Relay LED immediately.");
        sensorAndRelayData.relay2 = !sensorAndRelayData.relay2; 
        Serial.print("Blynk: Relay LED toggled to ");
        Serial.println(sensorAndRelayData.relay2 ? "ON" : "OFF");
        
        Blynk.virtualWrite(V1, sensorAndRelayData.relay2 ? 1 : 0);
      }
    }

    xSemaphoreGive(relayStateMutex);
  }
}