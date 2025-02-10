#include "BLEDevice.h"

#define LED_PIN 4 

BLEScan* pBLEScan;
float distances[3];
int scanCount = 0; 

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        String targetDevice = "20:91:48:d2:6a:ec"; 
        String deviceAddress = advertisedDevice.getAddress().toString();

        if (deviceAddress != targetDevice) return;  // 🔹 대상 장치만 처리

        Serial.println("=== Target Device Found ===");

        int rssi = advertisedDevice.getRSSI();
        float A = -68;
        float n = 3.2;
        float distance = pow(10, (A - rssi) / (10 * n));

        if (scanCount < 3) {
            distances[scanCount] = distance;
            scanCount++;
        }
    }
};

float calculateAverageDistance() {
    float sum = 0.0;
    for (int i = 0; i < 3; i++) {
        sum += distances[i];
    }
    return sum / 3.0;
}

void performScan() {
    if (pBLEScan != nullptr && scanCount < 3) {
        pBLEScan->start(1, false);  // 🔹 스캔 시간을 1초로 줄임
        pBLEScan->clearResults();   // 🔹 메모리 해제
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    Serial.println("ESP32-CAM BLE Scan Starting...");

    BLEDevice::init("ESP32-CAM");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);

    performScan();
}

void loop() {
    if (scanCount < 3) {
        performScan();
    } else {
        float averageDistance = calculateAverageDistance();
        Serial.print("Average Estimated Distance: ");
        Serial.print(averageDistance);
        Serial.println(" meters");
        if(averageDistance < 2.5) {
          digitalWrite(LED_PIN, HIGH);
        } else {
          digitalWrite(LED_PIN, LOW);
        }
        scanCount = 0;
    }
}
