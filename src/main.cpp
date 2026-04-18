#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#define BLE_SERVICE_UUID  "12345678-1234-1234-1234-123456789abc"
#define BLE_CHAR_UUID     "abcdefab-1234-1234-1234-abcdefabcdef"

static char ble_cmd[64] = {0};
static bool ble_cmd_ready = false;

class CmdCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String val = c->getValue();
    if (val.length() > 0 && val.length() < sizeof(ble_cmd)) {
      memcpy(ble_cmd, val.c_str(), val.length());
      ble_cmd[val.length()] = '\0';
      ble_cmd_ready = true;
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("BLE test starting...");

  BLEDevice::init("Rocket-FC");
  BLEServer *server = BLEDevice::createServer();
  BLEService *service = server->createService(BLE_SERVICE_UUID);
  BLECharacteristic *ch = service->createCharacteristic(
    BLE_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  ch->setCallbacks(new CmdCallback());
  service->start();
  BLEDevice::getAdvertising()->start();

  Serial.println("BLE advertising as 'Rocket-FC' — waiting for commands...");
}

void loop() {
  if (ble_cmd_ready) {
    ble_cmd_ready = false;
    Serial.print("BLE received: '");
    Serial.print(ble_cmd);
    Serial.println("'");

    if (strcmp(ble_cmd, "CALIBRATE") == 0) {
      Serial.println(">> Would enter CALIBRATION state");
    } else if (strcmp(ble_cmd, "RESET") == 0) {
      Serial.println(">> Would enter IDLE state");
    } else {
      Serial.println(">> Unknown command");
    }
  }
}
