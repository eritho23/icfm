#ifndef ICFM_FLIGHT_LOG_H
#define ICFM_FLIGHT_LOG_H

#include <Arduino.h>
#include <LittleFS.h>
#include <stdarg.h>

#include "bluetooth.h"
#include "../include/base.h"

void flog_init(void);
void flog_write(const char *line);
void flog_writef(const char *fmt, ...);
void flog_flush(void);
void flog_close(void);
b32 flog_dump_ble(void);

#endif
