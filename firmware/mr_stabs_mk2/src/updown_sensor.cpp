#include <updown_sensor.h>

using namespace updown_sensor;

UpdownSensor::UpdownSensor()
{
    wire = &Wire1;
    sensor = new Adafruit_BNO055(55, BNO055_ADDRESS, wire);
    grav_vec = make_unit_vector(0.0, 0.0, -9.81);
    max_grav_vec = init_vector3(0.0, 0.0, 0.0);
    min_grav_vec = init_vector3(0.0, 0.0, 0.0);
    orientation = init_vector3(0.0, 0.0, 0.0);
    gyro_vec = init_vector3(0.0, 0.0, 0.0);
}

bool UpdownSensor::begin()
{
    if (initialized)
        return true;
    // IMUPLUS fuses accelerometer and gyro only. NDOF (the library default) also fuses the
    // magnetometer, which the drive motor current can bend, moving heading under throttle.
    // Heading is now relative to the orientation at boot.
    if (sensor->begin(OPERATION_MODE_IMUPLUS))
    {
        delay(1000);
        initialized = true;
        sensor->setExtCrystalUse(true);
        return true;
    }
    else
    {
        initialized = false;
        return false;
    }
}

vector3_t *UpdownSensor::make_unit_vector(float x, float y, float z)
{
    float magnitude = sqrt(x * x + y * y + z * z);
    vector3_t *unit_vector = (vector3_t *)malloc(sizeof(vector3_t));
    unit_vector->x = x / magnitude;
    unit_vector->y = y / magnitude;
    unit_vector->z = z / magnitude;
    return unit_vector;
}

vector3_t *UpdownSensor::init_vector3(float x, float y, float z)
{
    vector3_t *vec = (vector3_t *)malloc(sizeof(vector3_t));
    vec->x = x;
    vec->y = y;
    vec->z = z;
    return vec;
}

bool UpdownSensor::get_is_upside_down(bool radio_connected)
{
    if (!update_sensor(radio_connected))
        return is_upside_down;

    float z = -1 * grav_vec->z;

    if (z < RIGHT_SIDE_UP_THRESHOLD)
        is_upside_down = false;
    else if (z > UPSIDE_DOWN_THRESHOLD)
        is_upside_down = true;
    return is_upside_down;
}

bool UpdownSensor::update_sensor(bool radio_connected)
{
    // An absent or unplugged sensor is never read: each failed read blocks the loop for
    // milliseconds, and at the 10 ms sample interval that starved the ESCs of DShot frames.
    // begin() blocks for over a second, so it is only retried with the radio disconnected.
    // main.cpp skips this call with the link down, so in practice a sensor that drops out stays
    // off, and heading hold with it, until the next reboot.
    if (!initialized)
    {
        if (!radio_connected && millis() - reconnect_timer > RECONNECT_INTERVAL)
        {
            begin();
            reconnect_timer = millis();
        }
        return false;
    }
    uint32_t now = millis();
    if (now - sample_timer < SAMPLE_INTERVAL)
    {
        return false;
    }
    sample_timer = now;

    // The Adafruit reads ignore I2C errors and return zeros, so check the sensor still ACKs.
    // One failed probe costs a single transaction; then reads stop until begin() succeeds.
    wire->beginTransmission(BNO055_ADDRESS);
    if (wire->endTransmission() != 0)
    {
        initialized = false;
        return false;
    }
    uint32_t start_time = now;
    sensors_event_t gravity_data, orientation_data, gyro_data;
    sensor->getEvent(&gravity_data, Adafruit_BNO055::VECTOR_GRAVITY);
    sensor->getEvent(&orientation_data, Adafruit_BNO055::VECTOR_EULER);
    sensor->getEvent(&gyro_data, Adafruit_BNO055::VECTOR_GYROSCOPE);
    uint32_t end_time = millis();

    if (end_time - start_time > 250)
    {
        initialized = false;
        return false;
    }

    grav_vec->x = gravity_data.acceleration.x;
    grav_vec->y = gravity_data.acceleration.y;
    grav_vec->z = gravity_data.acceleration.z;

    max_grav_vec->x = max(max_grav_vec->x, grav_vec->x);
    max_grav_vec->y = max(max_grav_vec->y, grav_vec->y);
    max_grav_vec->z = max(max_grav_vec->z, grav_vec->z);

    min_grav_vec->x = min(min_grav_vec->x, grav_vec->x);
    min_grav_vec->y = min(min_grav_vec->y, grav_vec->y);
    min_grav_vec->z = min(min_grav_vec->z, grav_vec->z);

    orientation->x = orientation_data.orientation.x;
    orientation->y = orientation_data.orientation.y;
    orientation->z = orientation_data.orientation.z;

    gyro_vec->x = gyro_data.gyro.x;
    gyro_vec->y = gyro_data.gyro.y;
    gyro_vec->z = gyro_data.gyro.z;
    sample_us = micros();

    return true;
}