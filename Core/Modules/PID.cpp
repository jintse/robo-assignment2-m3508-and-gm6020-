// #include "PID.hpp"
// #include <math.h> // For fmaxf and fminf

// /*
// Todo: Implement your functions or classes here
// */
// namespace Modules {
//     namespace PID {

//       float calculate(const float target, const float current, const float kp,
//                       const float ki, const float kd, float &integral,
//                       float &previousError, const float dt) {
//         float error = target - current;
//         integral += ki * error * dt;
//         float derivative = kd * (error - previousError) / dt;
//         previousError = error;
//         return ((kp * error) + integral + derivative);
//       }
      
//       void resetPIDState(float &integral, float &previousError) {
//         integral = 0.0f;
//         previousError = 0.0f;
//       }
      
//       float pidClampMinMax(const float input, const float value) {
//         if (input > value) return value;
//         if (input < -value) return -value;
//         return input;
//       }
    
//     } // namespace PID
// } // namespace Modules

#include "PID.hpp"
#include <math.h> // For fmaxf and fminf

/*
Todo: Implement your functions or classes here
*/
namespace Modules {
    namespace PID {

      float calculate(const float target, const float current, const float kp,
                      const float ki, const float kd, float &integral,
                      float &previousError, const float dt) {
        float error = target - current;
        float proportional = kp * error;
        float derivative = (dt > 0.0001f) ? kd * (error - previousError) / dt : 0.0f;
        float tentative_output = proportional + integral + derivative;

        // Anti-windup: Only integrate if not saturated
        if (ki != 0.0f) {
          float max_out = 10000.0f;  // Match max_current
          if (fabs(tentative_output) < max_out || (error * tentative_output <= 0.0f)) {
            integral += ki * error * dt;
          }
        }

        float output = proportional + integral + derivative;
        previousError = error;
        return output;
      }
      
      void resetPIDState(float &integral, float &previousError) {
        integral = 0.0f;
        previousError = 0.0f;
      }
      
      float pidClampMinMax(const float input, const float value) {
        if (input > value) return value;
        if (input < -value) return -value;
        return input;
      }
    
    } // namespace PID
} // namespace Modules