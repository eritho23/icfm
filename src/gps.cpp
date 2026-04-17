#include "gps.h"

SFE_UBLOX_GNSS gps;
long last_time =
    0; // Simple local timer. Limits amount of I2C traffic to u-blox module.

static b32 g_has_origin = false;
static double g_lat0_rad = 0.0;
static double g_lon0_rad = 0.0;
static double g_alt0_m = 0.0;
static u32 g_last_i_tow_ms = 0;

static const double k_deg_to_rad = 0.01745329251994329576923690768489;
static const double k_earth_radius_m = 6378137.0;

void gps_setup() {
  // Assume that the U-Blox GNSS is running at 9600 baud (the default) or at
  // 38400 baud. Loop until we're in sync and then ensure it's at 38400 baud.
  do {
    Serial.println("GNSS: trying 38400 baud");
    Serial1.begin(38400, SERIAL_8N1, 21, 20);
    if (gps.begin(Serial1) == true)
      break;

    delay(100);
    Serial.println("GNSS: trying 9600 baud");
    Serial1.begin(9600, SERIAL_8N1, 21, 20);
    if (gps.begin(Serial1) == true) {
      Serial.println("GNSS: connected at 9600 baud, switching to 38400");
      gps.setSerialRate(38400);
      delay(100);
    } else {
      // gps.factoryReset();
      delay(2000); // Wait a bit before trying again to limit the Serial output
    }
  } while (1);
  Serial.println("GNSS serial connected");
  gps.setUART1Output(COM_TYPE_UBX); // Set the UART port to output UBX only
  gps.setNavigationFrequency(10); // 10 Hz update rate
  gps.setAutoPVT(true);
  gps.saveConfiguration(); // Save the current settings to flash and BBR
}

b32 gps_read_local_enu(gps_measurement_t *out) {
  if (!out) return false;

  out->valid = false;
  out->fresh = false;

  if (!gps.getPVT()) return true;
  
  const u32 i_tow_ms = (u32)gps.getTimeOfWeek();
  if (i_tow_ms == g_last_i_tow_ms) return true;
  g_last_i_tow_ms  = i_tow_ms;
  out->i_tow_ms    = i_tow_ms;
  out->fresh       = true;

  if (gps.getFixType() < 3) return true;

  const double lat_rad = ((double)gps.getLatitude() * 1e-7) * k_deg_to_rad;
  const double lon_rad = ((double)gps.getLongitude() * 1e-7) * k_deg_to_rad;
  const double alt_m = (double)gps.getAltitude() * 1e-3;

  if (!g_has_origin) {
    g_lat0_rad = lat_rad;
    g_lon0_rad = lon_rad;
    g_alt0_m = alt_m;
    g_has_origin = true;
  }

  const double d_lat = lat_rad - g_lat0_rad;
  const double d_lon = lon_rad - g_lon0_rad;

  out->x_east_m = (f32)(d_lon * k_earth_radius_m * cos(g_lat0_rad));
  out->y_north_m = (f32)(d_lat * k_earth_radius_m);
  out->z_up_m = (f32)(alt_m - g_alt0_m);
  out->valid = true;

  return true;
}

void gps_test() {
  static unsigned long last_time = 0;
  if (millis() - last_time < 1000) return;
  last_time = millis();

  gps_measurement_t m;
  gps_read_local_enu(&m);

  Serial.print(F("Lat: "));    Serial.print(gps.getLatitude());
  Serial.print(F(" Lon: "));   Serial.print(gps.getLongitude());
  Serial.print(F(" (deg*1e-7)  Alt: ")); Serial.print(gps.getAltitude());
  Serial.print(F(" mm  SIV: ")); Serial.print(gps.getSIV());
  Serial.print(F("  Fix: "));  Serial.print(gps.getFixType());

  if (m.valid) {
    Serial.print(F("  ENU: "));
    Serial.print(m.x_east_m,  2); Serial.print(F("E "));
    Serial.print(m.y_north_m, 2); Serial.print(F("N "));
    Serial.print(m.z_up_m,    2); Serial.print(F("U (m)"));
  } else if (m.fresh) {
    Serial.print(F("  ENU: waiting for 3D fix"));
  }

  Serial.println();
}
