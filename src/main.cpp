#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

struct SensorData {
    float waterTempIn;
    float waterTempOut;
    float waterFlowRate;
    float airIntakeTemp;
    float massFlowIntake;
    float exhaustTemp;
    float massFlowExhaust;
    int rpm;
    float gasFlowRate;
    float voltage;
    float ampere;
    float loadCell;
    char dateTime[25];
};

uint8_t broadcastAddress[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}; // Dummy broadcast MAC

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
    Serial.begin(115200);
    randomSeed(analogRead(0)); // for dummy random values

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);

    // Register peer
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0; // pick a channel
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
}

void loop() {
    SensorData data;

    // Generate dummy values
    data.waterTempIn = random(200, 300) / 10.0; // 20.0 to 30.0
    data.waterTempOut = random(180, 280) / 10.0; // 18.0 to 28.0
    data.waterFlowRate = random(50, 150) / 10.0; // 5.0 to 15.0
    data.airIntakeTemp = random(150, 250) / 10.0; // 15.0 to 25.0
    data.massFlowIntake = random(100, 500) / 10.0; // 10.0 to 50.0
    data.exhaustTemp = random(300, 600) / 10.0; // 30.0 to 60.0
    data.massFlowExhaust = random(200, 600) / 10.0; // 20.0 to 60.0
    data.rpm = random(1000, 5000); // 1000 to 5000
    data.gasFlowRate = random(10, 50) / 10.0; // 1.0 to 5.0
    data.voltage = random(110, 130) / 10.0; // 11.0 to 13.0
    data.ampere = random(50, 150) / 10.0; // 5.0 to 15.0
    data.loadCell = random(1000, 5000) / 10.0; // 100.0 to 500.0

    // Use actual dateTime (simulated with current time string)
    strcpy(data.dateTime, "2025-09-24T23:49:30.366Z"); // Based on current time provided

    // Send dummy data over ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &data, sizeof(data));
    if (result == ESP_OK) {
        Serial.println("Sent with success");
    } else {
        Serial.println("Error sending the data");
    }

    // Also print to serial for debugging
    Serial.println("Dummy Sensor Data:");
    Serial.print("waterTempIn: "); Serial.println(data.waterTempIn);
    Serial.print("waterTempOut: "); Serial.println(data.waterTempOut);
    Serial.print("waterFlowRate: "); Serial.println(data.waterFlowRate);
    Serial.print("airIntakeTemp: "); Serial.println(data.airIntakeTemp);
    Serial.print("massFlowIntake: "); Serial.println(data.massFlowIntake);
    Serial.print("exhaustTemp: "); Serial.println(data.exhaustTemp);
    Serial.print("massFlowExhaust: "); Serial.println(data.massFlowExhaust);
    Serial.print("rpm: "); Serial.println(data.rpm);
    Serial.print("gasFlowRate: "); Serial.println(data.gasFlowRate);
    Serial.print("voltage: "); Serial.println(data.voltage);
    Serial.print("ampere: "); Serial.println(data.ampere);
    Serial.print("loadCell: "); Serial.println(data.loadCell);
    Serial.print("dateTime: "); Serial.println(data.dateTime);
    Serial.println("---");

    delay(50); // Send every 5 seconds
}