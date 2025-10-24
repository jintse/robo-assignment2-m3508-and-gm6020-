#pragma once
#include "main.h"
#include "math.h"

//==============================//
// Your code ends here          //
// For evaluation purpose only  //
// DO NOT MODIFY                //
//==============================//
namespace Modules {
namespace Evaluation {
class Test {
public:
  /**
   * @brief These functions will output different waveforms
   * @return The evaluated angle in radians
   * @note The output angle is normalized to the range [-π, π]
   * @note You can call these functions directly to get the output
   */
  float eval_Sin();
  float eval_Square();
  float eval_Triangle();
  float eval_FakeAim();

  uint8_t setFrequency(float _frequency);
  uint8_t setCenter(float _center);
  uint8_t setAmplitude(float _amplitude);

  /**
   * @brief Constructor to initialize the Test class with frequency, center, and
   * amplitude.
   * @param _frequency The frequency of the waveform (in Hz).
   * @param _center The center value of the waveform (in radians).
   * @param _amplitude The amplitude of the waveform (in radians).
   */
  Test(float _frequency, float _center, float _amplitude)
      : frequency(_frequency), center(_center), amplitude(_amplitude),
        flag_fake_aim(false), last_update_fake_aim(false), cd_duration(10) {
    configAssert(frequency >= 0.0f);
    configAssert(amplitude >= 0.0f);
  }

private:
  float frequency;
  float center;
  float amplitude;

  bool flag_fake_aim;
  uint32_t last_update_fake_aim;
  uint16_t cd_duration;
  uint32_t cd_start_time;
  float current_angle;
  void configAssert(uint8_t x);
  inline float normalizeAngle(float value);
};
} // namespace Evaluation
} // namespace Modules