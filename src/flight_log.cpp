#include "flight_log.h"
#include "bluetooth.h"

#include <LittleFS.h>
#include <stdarg.h>

#define LOG_PATH "/flight.csv"
#define FLUSH_EVERY_ROWS 10
#define LINE_BUF_LEN 256

static File g_file;
static bool g_open = false;
static u32 g_rows_unflushed = 0;

static bool fs_mount(void) {
  // LittleFS.begin() is idempotent when already mounted
  if (!LittleFS.begin(true)) {
    ble_send("ERROR: LittleFS mount failed");
    return false;
  }
  return true;
}

static void do_flush(void) {
  g_file.flush();
  g_rows_unflushed = 0;
}

void flog_init(void) {
  if (!fs_mount()) {
    return;
  }

  // Always start with a clean file so we don't mix flights
  if (LittleFS.exists(LOG_PATH)) {
    LittleFS.remove(LOG_PATH);
  }

  g_file = LittleFS.open(LOG_PATH, FILE_WRITE);
  if (!g_file) {
    ble_send("ERROR: flog_init: open failed");
    return;
  }

  g_open           = true;
  g_rows_unflushed = 0;

  // CSV header — extend this to match your actual columns
  g_file.println("t_ms,wx,wy,wz,roll_deg,pitch_deg,yaw_deg,"
                 "fin0,fin1,fin2,fin3");
  ble_send("LOG: flash logging started");
}

void flog_write(const char *line) {
  if (!line || !g_open) {
    return;
  }

  g_file.println(line);
  g_rows_unflushed++;

  if (g_rows_unflushed >= FLUSH_EVERY_ROWS) {
    do_flush();
  }
}

void flog_writef(const char *fmt, ...) {
  if (!fmt || !g_open) {
    return;
  }

  char buf[LINE_BUF_LEN];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  flog_write(buf);
}

void flog_flush(void) {
  if (!g_open) {
    return;
  }
  do_flush();
}

void flog_close(void) {
  if (!g_open) {
    return;
  }
  do_flush();
  g_file.close();
  g_open = false;
  ble_send("LOG: file closed");
}

b32 flog_dump_ble(void) {
  if (!fs_mount()) {
    return false;
  }

  File f = LittleFS.open(LOG_PATH, FILE_READ);
  if (!f) {
    ble_send("ERROR: flog_dump_ble: no log file");
    return false;
  }

  ble_send("DUMP_BEGIN");

  // Read byte-by-byte and reassemble lines to avoid missing the last line
  // if it has no trailing newline
  char line[LINE_BUF_LEN];
  u32  idx = 0;

  while (f.available()) {
    const int v = f.read();
    if (v < 0) {
      break;
    }
    const char c = (char)v;

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      line[idx] = '\0';
      if (idx > 0) {
        ble_send(line);
      }
      idx = 0;
      continue;
    }

    if (idx < (u32)(LINE_BUF_LEN - 1)) {
      line[idx++] = c;
    }
  }

  // Flush any remaining chars without a trailing newline
  if (idx > 0) {
    line[idx] = '\0';
    ble_send(line);
  }

  f.close();
  ble_send("DUMP_END");
  return true;
}
