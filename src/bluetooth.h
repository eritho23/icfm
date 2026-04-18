#ifndef ICFM_BLUETOOTH_H
#define ICFM_BLUETOOTH_H

#include <Arduino.h>

#include "../include/base.h"

#define BLE_SERVICE_UUID "12345678-1234-1234-1234-123456789abc"
#define BLE_CMD_CHAR_UUID "abcdefab-1234-1234-1234-abcdefabcdef"
#define BLE_LOG_CHAR_UUID "fedcbafe-4321-4321-4321-abcdefabcdef"

#define BLE_CMD_BUF_LEN 64
#define BLE_TX_CHUNK 180

void ble_init(void);
b32 ble_poll_cmd(char *out, u8 max_len);
b32 ble_connected(void);
void ble_send(const char *line);
void ble_sendf(const char *fmt, ...);

#endif
