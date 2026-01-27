/*
    This file contains the implementation of PID used in the rocket.
*/

struct pid_coefficients {
    /*
        Proportional component: p * E(t).
    */
    float p;
    /*
        Integral component: i * ∫ E(t) dt.
    */
    float i;
    /*
        Detivative component: d * D (E(t)).
    */
    float d;
};

/*
    Estimates the integral of the error by adding a Riemann rectangle
    to a target integral error value.

    Usage sketch:
    ```c
        float sensor_read_time = get_time();
        float desired_value = ...;
        float integral_error = 0.0;
        // ...
        // get_time should be in the current (relative) time as a time unit
        float sensor_value = read_sensors(...);
        float sensor_post_read_time = get_time();
        pid_update_integral_error(sensor_post_read_time - sensor_read time, sensor_value - desired_value, &integral_error)
    ```
*/
void pid_update_integral_error(float time_delta, float error, float * target);

/*
    Updates the actual error by setting the target variable.

    Usage sketch:
    ```c
        float desired_value = ...; 
        float sensor_value = read_sensor();
        float error = 0.0;
        pid_update_error(sensor_value - desired_value, &error);

    ```
*/
void pid_update_error(float error, float * target);

/*
    Estimates the current derivative of the error by comparing the current
    error value to the last read error value. TODO: Consider making this
    consider more values of the error.
*/
void pid_update_derivative_error(float error_delta, float time_delta, float * target);

/*
    Performs the deflection calculation to be used in actuation. Uses a 
    coefficient definition in a struct and returns the value.
*/
float pid_calculate_deflection(pid_coefficients* coefficents, float current_error, float integral_error, float derivative_error);