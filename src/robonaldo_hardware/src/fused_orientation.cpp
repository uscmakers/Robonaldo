#pragma once
#include "ros/ros.h"
#include "robonaldo_hardware/imu_values.h"
#include "robonaldo/orientation.h"


//Literally the entirety of the MahonyAHRS.cgpp because I am stupid and confused 
#include <stdint.h>
#include <math.h>

#define DEFAULT_PROPORTIONAL_GAIN 0.5f
#define DEFAULT_INTEGRAL_GAIN 0.0f

class Mahony {
public:
  /*!
  @brief The constructor of the filter.
  @param[in] samplePeriod: the period between `update` calls (in
  seconds).
  @note The `samplePeriod` parameter is the time in seconds between
  calls to the `update` function/s.
  */
  explicit Mahony(const float samplePeriod);

  /*!
  @brief Proportional gain setter.
  @param[in] p: the proportional gain of the filter.
  */
  void setP(const float p);

  /*!
  @brief Integral gain setter.
  @param[in] i: the integral gain of the filter.
  */
  void setI(const float i);

  /*!
  @brief Calculates Euler angles...
  from 3-axis accelerations and 3-axis angular velocities. The
  calculated Euler angles will be written to the references passed to
  the function (in radians). The passed acceleration values can be
  relative to each other, but the passed angular velocities must be
  in radians per second.
  @param[out] &yaw: this reference will be set to the calculated yaw
  angle (in radians).
  @param[out] &pitch: this reference will be set to the calculated
  pitch angle (in radians).
  @param[out] &roll: this reference will be set to the calculated
  roll angle (in radians).
  @param[in] ax: x-axis acceleration.
  @param[in] ay: y-axis acceleration.
  @param[in] az: z-axis acceleration.
  @param[in] gx: x-axis angular velocity (in radians per second).
  @param[in] gy: y-axis angular velocity (in radians per second).
  @param[in] gz: z-axis angular velocity (in radians per second).
  */
  void update(float &yaw, float &pitch, float &roll,
              float ax, float ay, float az,
              float gx, float gy, float gz);

  /*!
  @brief Calculates Euler angles...
  from 3-axis accelerations, 3-axis angular velocities and 3-axis
  magnetic fields. The calculated Euler angles will be written to the
  references passed to the function (in radians). The passed
  acceleration and magnetic field values can be relative to each
  other, but the passed angular velocities must be in radians per
  second.
  @param[out] &yaw: this reference will be set to the calculated yaw
  angle (in radians).
  @param[out] &pitch: this reference will be set to the calculated
  pitch angle (in radians).
  @param[out] &roll: this reference will be set to the calculated
  roll angle (in radians).
  @param[in] ax: x-axis acceleration.
  @param[in] ay: y-axis acceleration.
  @param[in] az: z-axis acceleration.
  @param[in] gx: x-axis angular velocity (in radians per second).
  @param[in] gy: y-axis angular velocity (in radians per second).
  @param[in] gz: z-axis angular velocity (in radians per second).
  @param[in] ax: x-axis magnetic field.
  @param[in] ay: y-axis magnetic field.
  @param[in] az: z-axis magnetic field.
  */
  void update(float &yaw, float &pitch, float &roll,
              float ax, float ay, float az,
              float gx, float gy, float gz,
              float mx, float my, float mz);

private:
  float _twoKp = 2.0f * DEFAULT_PROPORTIONAL_GAIN; //! 2 * proportional gain
  float _twoKi = 2.0f * DEFAULT_INTEGRAL_GAIN; //! 2 * integral gain
  float _quaternion[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
  float _integralFBx = 0.0f;
  float _integralFBy = 0.0f;
  float _integralFBz = 0.0f;
  float _samplePeriod;

  /*!
  @brief Inverse square root.
  @param[in] x.
  */
  static float _invSqrt(const float x);

  /*!
  @brief Converts quaternion to Euler angles.
  @param[out] &yaw: this reference will be set to the calculated yaw
  angle (in radians).
  @param[out] &pitch: this reference will be set to the calculated
  pitch angle (in radians).
  @param[out] &roll: this reference will be set to the calculated
  roll angle (in radians).
  */
  void _toYawPitchRoll(float &yaw, float &pitch, float &roll);
};

//Now the entire Mahony.cpp because I! Am! Illiterate! 

Mahony::Mahony(const float samplePeriod)
  : _samplePeriod(samplePeriod) {}

void Mahony::setP(const float p) {
  _twoKp = 2.0f * p;
}

void Mahony::setI(const float i) {
  _twoKi = 2.0f * i;
}

void Mahony::update(float &yaw, float &pitch, float &roll,
            float ax, float ay, float az,
            float gx, float gy, float gz) {
  float recipNorm;

// Compute feedback only if accelerometer measurement valid
// (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = Mahony::_invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity
  float halfvx, halfvy, halfvz;
    halfvx = _quaternion[1] * _quaternion[3] - _quaternion[0] * _quaternion[2];
    halfvy = _quaternion[0] * _quaternion[1] + _quaternion[2] * _quaternion[3];
    halfvz = _quaternion[0] * _quaternion[0] - 0.5f + _quaternion[3] * _quaternion[3];

    // Error is sum of cross product between estimated
    // and measured direction of gravity
  float halfex, halfey, halfez;
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if (_twoKi > 0.0f) {
      // integral error scaled by Ki
      _integralFBx += _twoKi * halfex * _samplePeriod;
      _integralFBy += _twoKi * halfey * _samplePeriod;
      _integralFBz += _twoKi * halfez * _samplePeriod;
      gx += _integralFBx;  // apply integral feedback
      gy += _integralFBy;
      gz += _integralFBz;
    } else {
      _integralFBx = 0.0f; // prevent integral windup
      _integralFBy = 0.0f;
      _integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += _twoKp * halfex;
    gy += _twoKp * halfey;
    gz += _twoKp * halfez;
  }

// Integrate rate of change of quaternion
  gx *= (0.5f * _samplePeriod);   // pre-multiply common factors
  gy *= (0.5f * _samplePeriod);
  gz *= (0.5f * _samplePeriod);
  float qa, qb, qc;
  qa = _quaternion[0];
  qb = _quaternion[1];
  qc = _quaternion[2];
  _quaternion[0] += (-qb * gx - qc * gy - _quaternion[3] * gz);
  _quaternion[1] += (qa * gx + qc * gz - _quaternion[3] * gy);
  _quaternion[2] += (qa * gy - qb * gz + _quaternion[3] * gx);
  _quaternion[3] += (qa * gz + qb * gy - qc * gx);

// Normalise quaternion
  recipNorm = Mahony::_invSqrt(_quaternion[0] * _quaternion[0] + _quaternion[1] * _quaternion[1] + _quaternion[2] * _quaternion[2] + _quaternion[3] * _quaternion[3]);
  _quaternion[0] *= recipNorm;
  _quaternion[1] *= recipNorm;
  _quaternion[2] *= recipNorm;
  _quaternion[3] *= recipNorm;

  _toYawPitchRoll(yaw, pitch, roll);
}

void Mahony::update(float &yaw, float &pitch, float &roll,
            float ax, float ay, float az,
            float gx, float gy, float gz,
            float mx, float my, float mz) {
  float recipNorm;

  // Use IMU algorithm if magnetometer measurement invalid
  // (avoids NaN in magnetometer normalisation)
  if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    update(yaw, pitch, roll, ax, ay, az, gx, gy, gz);
    return;
  }

  // Compute feedback only if accelerometer measurement valid
  // (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = Mahony::_invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = Mahony::_invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    q0q0 = _quaternion[0] * _quaternion[0];
    q0q1 = _quaternion[0] * _quaternion[1];
    q0q2 = _quaternion[0] * _quaternion[2];
    q0q3 = _quaternion[0] * _quaternion[3];
    q1q1 = _quaternion[1] * _quaternion[1];
    q1q2 = _quaternion[1] * _quaternion[2];
    q1q3 = _quaternion[1] * _quaternion[3];
    q2q2 = _quaternion[2] * _quaternion[2];
    q2q3 = _quaternion[2] * _quaternion[3];
    q3q3 = _quaternion[3] * _quaternion[3];

    // Reference direction of Earth's magnetic field
  float hx, hy, bx, bz;
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrtf(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    // Estimated direction of gravity and magnetic field
  float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    // Error is sum of cross product between estimated direction
    // and measured direction of field vectors
  float halfex, halfey, halfez;
    halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
    halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
    halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

    // Compute and apply integral feedback if enabled
    if (_twoKi > 0.0f) {
      // integral error scaled by Ki
      _integralFBx += _twoKi * halfex * _samplePeriod;
      _integralFBy += _twoKi * halfey * _samplePeriod;
      _integralFBz += _twoKi * halfez * _samplePeriod;
      gx += _integralFBx;  // apply integral feedback
      gy += _integralFBy;
      gz += _integralFBz;
    } else {
      _integralFBx = 0.0f; // prevent integral windup
      _integralFBy = 0.0f;
      _integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += _twoKp * halfex;
    gy += _twoKp * halfey;
    gz += _twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * _samplePeriod);   // pre-multiply common factors
  gy *= (0.5f * _samplePeriod);
  gz *= (0.5f * _samplePeriod);
  float qa, qb, qc;
  qa = _quaternion[0];
  qb = _quaternion[1];
  qc = _quaternion[2];
  _quaternion[0] += (-qb * gx - qc * gy - _quaternion[3] * gz);
  _quaternion[1] += (qa * gx + qc * gz - _quaternion[3] * gy);
  _quaternion[2] += (qa * gy - qb * gz + _quaternion[3] * gx);
  _quaternion[3] += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = Mahony::_invSqrt(_quaternion[0] * _quaternion[0] + _quaternion[1] * _quaternion[1] + _quaternion[2] * _quaternion[2] + _quaternion[3] * _quaternion[3]);
  _quaternion[0] *= recipNorm;
  _quaternion[1] *= recipNorm;
  _quaternion[2] *= recipNorm;
  _quaternion[3] *= recipNorm;


  _toYawPitchRoll(yaw, pitch, roll);
}

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float Mahony::_invSqrt(const float x) {
  float halfx = 0.5f * x;
  float y = x;
  int32_t i = *(int32_t*)&y; // not portable
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i; // not portable
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
}

void Mahony::_toYawPitchRoll(float &yaw, float &pitch, float &roll) {
  // Quaternion to Euler
  // pitch (y-axis rotation)
  float sinp = +2.0 * (_quaternion[0] * _quaternion[2] - _quaternion[3] * _quaternion[1]);
  if (fabs(sinp) >= 1) {
    pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  } else {
    pitch = asin(sinp);
  }

  // roll (x-axis rotation)
  float sinr = +2.0 * (_quaternion[0] * _quaternion[1] + _quaternion[2] * _quaternion[3]);
  float cosr = +1.0 - 2.0 * (_quaternion[1] * _quaternion[1] + _quaternion[2] * _quaternion[2]);
  roll = atan2(sinr, cosr);

  // yaw (z-axis rotation)
  float siny = +2.0 * (_quaternion[0] * _quaternion[3] + _quaternion[1] * _quaternion[2]);
  float cosy = +1.0 - 2.0 * (_quaternion[2] * _quaternion[2] + _quaternion[3] * _quaternion[3]);
  yaw = atan2(siny, cosy);
  yaw += M_PI;
}

//do all the math here plz 
void userInputCallback(const robonaldo_hardware::imu_values::ConstPtr& msg){
	ROS_INFO("ax: %d ay: %d az: %d mx: %d my: %d mz: %d gx: %d gy: %d gz: %d", msg->ax, msg->ay, msg->az, msg->mx, msg->my, msg->mz, msg->gx, msg->gy, msg->gz);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "fused_orientation");
	ros::NodeHandle n;
	ros::Publisher orientation_pub = n.advertise<robonaldo::orientation>("orientation", 1000);
	ros::Subscriber sub = n.subscribe("imu_values", 1000, userInputCallback);
	ros::Rate loop_rate(10);
	
	const float mahonyProportionalGain = 0.5f;
	const float mahonyIntegralGain = 0.0f;
	const float dps_to_rad=.0174533;

	mahony.setP(mahonyProportionalGain);
	mahony.setI(mahonyIntegralGain);

	while (ros::ok()) {

		//Since I am stupid for now we will just use values
		// that I set because C++ and ROS confuse me because I am a failure
		// but also I want to convert the values
		int axRaw = imu_values.ax, ayRaw = imu_values.ay, azRaw = imu_values.az; // 3-axis raw acceleration
        int gxRaw = imu_values.gx, gyRaw = imu_values.gy, gzRaw = imu_values.gz; // 3-axis raw angular velocity
        int mxRaw = imu_values.mx, myRaw = imu_values.my, mzRaw = imu_values.mz;

        //convert velocity from degrees per second to radians per second 
        float gxRads=gxRaw*dps_to_rad;
        float gyRads=gyRaw*dps_to_rad;
        float gzRads=gzRaw*dps_to_rad;

        float yawRad=0;
        float pitchRad=0;
        float rollRad=0;

        //update function to implement the sensor fusion algorithm 

        mahony.update(yawRad, pitchRad, rollRad, 
                  axRaw, ayRaw, azRaw, 
                  gxRadS, gyRadS, gzRadS, 
                  mxRaw, myRaw, mzRaw);

		robonaldo::orientation msg;
		msg.yaw = yawRad * 57.295779513f;
		msg.pitch = pitchRad * 57.295779513f;
		msg.roll = rollRad * 57.295779513f;

		orientation_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}
