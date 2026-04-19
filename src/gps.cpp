#include "gps.h"

#include "bluetooth.h"

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
    ble_send("GNSS: trying 38400 baud");
    Serial1.begin(38400, SERIAL_8N1, 21, 20);
    if (gps.begin(Serial1) == true)
      break;

    delay(100);
    ble_send("GNSS: trying 9600 baud");
    Serial1.begin(9600, SERIAL_8N1, 21, 20);
    if (gps.begin(Serial1) == true) {
      ble_send("GNSS: connected at 9600 baud, switching to 38400");
      gps.setSerialRate(38400);
      delay(100);
    } else {
      // gps.factoryReset();
      delay(2000); // Wait a bit before trying again to limit the Serial output
    }
  } while (1);
  ble_send("GNSS serial connected");
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
  g_last_i_tow_ms = i_tow_ms;
  out->i_tow_ms = i_tow_ms;
  out->fresh = true;

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

void gps_reset_origin(void) {
  g_has_origin = false;
  g_lat0_rad = 0.0;
  g_lon0_rad = 0.0;
  g_alt0_m = 0.0;
  g_last_i_tow_ms = 0;
}
