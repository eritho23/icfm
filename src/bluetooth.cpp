#include "bluetooth.h"

static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
static bool g_connected = false;
static bool g_cmd_ready = false;
static char g_cmd_buf[BLE_CMD_BUF_LEN];

static BLEServer *g_server = NULL;
static BLECharacteristic *g_cmd_char = NULL;
static BLECharacteristic *g_log_char = NULL;

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *) override {
    portENTER_CRITICAL(&g_mux);
    g_connected = true;
    portEXIT_CRITICAL(&g_mux);
  }

  void onDisconnect(BLEServer *) override {
    portENTER_CRITICAL(&g_mux);
    g_connected = false;
    portEXIT_CRITICAL(&g_mux);
    // Restart advertising so the client can reconnect
    BLEDevice::startAdvertising();
  }
};

class CmdCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String val = c->getValue();
    u8 len = (u8)val.length();

    if (len == 0 || len >= BLE_CMD_BUF_LEN) {
      return;
    }

    portENTER_CRITICAL(&g_mux);
    // Drop incoming command if previous one hasn't been consumed yet.
    // Main loop should be polling fast enough that this never happens.
    if (!g_cmd_ready) {
      memcpy(g_cmd_buf, val.c_str(), len);
      g_cmd_buf[len] = '\0';
      g_cmd_ready = true;
    }
    portEXIT_CRITICAL(&g_mux);
  }
};

// Static instances — no heap allocation
static ServerCB g_server_cb;
static CmdCB g_cmd_cb;

void ble_init(void) {
  BLEDevice::init("Rocket-FC");

  g_server = BLEDevice::createServer();
  g_server->setCallbacks(&g_server_cb);

  BLEService *svc = g_server->createService(BLE_SERVICE_UUID);

  g_cmd_char = svc->createCharacteristic(
    BLE_CMD_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  g_cmd_char->setCallbacks(&g_cmd_cb);

  g_log_char = svc->createCharacteristic(
    BLE_LOG_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );

  svc->start();
  BLEDevice::getAdvertising()->start();
}

b32 ble_poll_cmd(char *out, u8 max_len) {
  if (!out || max_len < 2) {
    return false;
  }

  portENTER_CRITICAL(&g_mux);
  const bool ready = g_cmd_ready;
  if (ready) {
    strncpy(out, g_cmd_buf, max_len - 1);
    out[max_len - 1] = '\0';
    g_cmd_ready = false;
  }
  portEXIT_CRITICAL(&g_mux);

  return (b32)ready;
}

b32 ble_connected(void) {
  portENTER_CRITICAL(&g_mux);
  const bool c = g_connected;
  portEXIT_CRITICAL(&g_mux);
  return (b32)c;
}

void ble_send(const char *line) {
  if (!line || !g_log_char) {
    return;
  }

  // Read connected flag once — avoids holding lock across notify calls
  portENTER_CRITICAL(&g_mux);
  const bool connected = g_connected;
  portEXIT_CRITICAL(&g_mux);

  if (!connected) {
    return;
  }

  // Send in chunks that fit within BLE MTU
  const size_t len = strlen(line);
  size_t off = 0;
  while (off < len) {
    size_t n = len - off;
    if (n > BLE_TX_CHUNK) {
      n = BLE_TX_CHUNK;
    }
    g_log_char->setValue((uint8_t *)(line + off), n);
    g_log_char->notify();
    off += n;
  }

  // Newline terminator so the receiver knows the line is complete
  static const uint8_t nl = '\n';
  g_log_char->setValue((uint8_t *)&nl, 1);
  g_log_char->notify();
}

void ble_sendf(const char *fmt, ...) {
  if (!fmt) {
    return;
  }

  // Early-out before touching the stack buffer
  portENTER_CRITICAL(&g_mux);
  const bool connected = g_connected;
  portEXIT_CRITICAL(&g_mux);
  if (!connected) {
    return;
  }

  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  ble_send(buf);
}
