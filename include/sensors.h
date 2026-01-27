/*
    Code for reading the various sensors used on the rocket.
*/

/* GPS */

/*
    Struct to store a GPS measurement.
*/
struct gps_coords {
    float time;
    float lat;
    float lon;
};

/*
    Read GPS coordinates into a GPS coord struct.
*/
void gps_read_coords(gps_coords * target);

/*
    Performs startup health checks on the GPS.

    Return value:
        - 0 for OK,
        - 1 for generic error.
*/
int gps_ok();

/*
    Perform necessary initializations for the GPS.
    A health check should be done after with `gps_ok`.
*/
void gps_init();


/* ACCELEROMETER */
struct accel_data {
    float time;
    float x;
    float y;
    float z;
};

void accel_read_data(accel_data * target);

/*
    Performs startup health checks on the accelerometer.

    Return value:
        - 0 for OK,
        - 1 for generic error.
*/
int accel_ok();

void accel_init();

/* GYROSCOPE */
struct gyro_data {
    float time;
    float x;
    float y;
    float z;
};

void gyro_read_data(gyro_data * target);

/*
    Performs startup health checks on the gyro.

    Return value:
        - 0 for OK,
        - 1 for generic error.
*/
int gyro_ok();

void gyro_init();