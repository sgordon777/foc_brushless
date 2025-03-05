
// *****************************************************************************************************************************
#include "Arduino.h"
#include "common/base_classes/FOCMotor.h"
#include "common/base_classes/Sensor.h"
//#include "SmoothingSensor.h"
#include "common/foc_utils.h"
#include "common/time_utils.h"


/**
  SmoothingSensor is a wrapper class which is inserted inbetween a sensor and motor to provide better
  quality angle readings from sensors that are low resolution or are slow to communicate. It provides
  no improvement for sensors that are high resolution and quick to update.
  It uses the timestamp of the last angle reading from the sensor and the low-pass filtered velocity
  from the motor to predict what the angle should be at the current time.
*/

class SmoothingSensor : public Sensor
{
  public:
    /**
    SmoothingSensor class constructor
    @param s  Wrapped sensor
    @param m  Motor that the SmoothingSensor will be linked to
    */
    SmoothingSensor(Sensor& s, const FOCMotor& m);

    void update() override;
    float getVelocity() override;
    int needsSearch() override;

    // For sensors with slow communication, use these to poll less often
    unsigned int sensor_downsample = 0; // parameter defining the ratio of downsampling for sensor update
    unsigned int sensor_cnt = 0; // counting variable for downsampling

    // For hall sensors, the predicted angle is always 0 to 60 electrical degrees ahead of where it would be without
    // smoothing, so offset it backward by 30 degrees (set this to -PI_6) to get the average in phase with the rotor
    float phase_correction = 0;

  protected:
    float getSensorAngle() override;
    void init() override;

    Sensor& _wrapped;
    const FOCMotor& _motor;
};

SmoothingSensor::SmoothingSensor(Sensor& s, const FOCMotor& m) : _wrapped(s), _motor(m)
{
}


void SmoothingSensor::update() {
  // Update sensor, with optional downsampling of update rate
  if(sensor_cnt++ >= sensor_downsample) {
    sensor_cnt = 0;
    _wrapped.update();
  }

  // Copy state variables from the sensor
  angle_prev = _wrapped.angle_prev;
  angle_prev_ts = _wrapped.angle_prev_ts;
  full_rotations = _wrapped.full_rotations;

  // Perform angle prediction, using low-pass filtered velocity. But don't advance more than
  // pi/3 (equivalent to one step of block commutation) from the last true angle reading.
  float dt = (_micros() - angle_prev_ts) * 1e-6f;
  angle_prev += _motor.sensor_direction * _constrain(_motor.shaft_velocity * dt, -_PI_3 / _motor.pole_pairs, _PI_3 / _motor.pole_pairs);

  // Apply phase correction if needed
  if (phase_correction != 0) {
    if (_motor.shaft_velocity < -0) angle_prev -= _motor.sensor_direction * phase_correction / _motor.pole_pairs;
    else if (_motor.shaft_velocity > 0) angle_prev += _motor.sensor_direction * phase_correction / _motor.pole_pairs;
  }

  // Handle wraparound of the projected angle
  if (angle_prev < 0) full_rotations -= 1, angle_prev += _2PI;
  else if (angle_prev >= _2PI) full_rotations += 1, angle_prev -= _2PI;
}


float SmoothingSensor::getVelocity() {
  return _wrapped.getVelocity();
}


int SmoothingSensor::needsSearch() {
  return _wrapped.needsSearch();
}


float SmoothingSensor::getSensorAngle() {
  return _wrapped.getSensorAngle();
}


void SmoothingSensor::init() {
  _wrapped.init();
}



// IN declarstions
// instantiate the smoothing sensor, providing the real sensor as a constructor argument
//SmoothingSensor smooth = SmoothingSensor(sensor, motor);


  // IN SETUP
  //smooth.phase_correction = -_PI_6; // FOR HALL SENSOR
  //motor.linkSensor(&smooth);


// *****************************************************************************************************************************

