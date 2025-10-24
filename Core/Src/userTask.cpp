#include "DJIMotor.hpp"
#include "Evaluation.hpp"
#include "PID.hpp"
#include "gpio.h"
#include "main.h"
#include <string.h>
#include <math.h>  // For M_PI

extern FDCAN_HandleTypeDef hfdcan1;  // Ensure this is declared (from main.h or fdcan.h)
extern UART_HandleTypeDef huart1;  // Ensure this is declared (from main.h or usart.h)

extern "C" {

  // Forward declarations
  void RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
  void ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs);
  
  uint8_t rxData[8] = {0};

  uint8_t rx_uart_Buf[5] = {0};  // For UART1 DMA receive

  // Debug globals (watch in debugger)
  HAL_StatusTypeDef rx_status = HAL_ERROR;  // For GetRxMessage

  bool valid_rxdata = false; 
  bool running = false;

  uint8_t button_state;  // Debounced state
  uint8_t last_reading;  // Raw reading
  uint32_t last_debounce = 0;
  const uint32_t debounce_ms = 10;

  float target_pos; 
  float current_pos; 
  float output;
  int16_t send_current_m3508 = 0;
  int16_t send_current_gm6020 = 0;

  float target_speed = 0;

  float dt = 0.0f;

  uint32_t last_pid_tick = 0;
  uint32_t now = 0;
  
  float kp = 100000.0f;
  float ki = 1.0f;
  float kd = 5.0f;
  
  float integral = 0.0f;
  float integral_m3508 = 0.0f;
  float error = 0;

  float prev_error = 0.0f;

  float max_current = 10000.0f;

  float speed_scale = 1.0f;  // Increased for visible rotation

  uint8_t prev_uart_in = 0;

  float accumulated_pos = 0.0f; float last_raw_pos = 0.0f;

  bool first_called = true;
  float prev_position = 0.0f;

  float filtered_output = 0.0f;  // Initial filtered value
  float alpha = 0.3f;  // Filter constant (tune 0.2-0.5)

  float rad_dif = 0.0f; 
  float time_angle_before = 0.0f;
  float time_angle_after = 0.0f;
  float time_angle = 0.0f;
  bool record_start = false; 
  bool recode_end = false;

  float ratio_m3508 = 3591.0f / 187.0f; 

  // RX Callback
  void RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {

    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {


      FDCAN_RxHeaderTypeDef rxHeader;
      rx_status = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData);

      if (rx_status == HAL_OK) {

        if (rxHeader.Identifier == 0x201) {
          Motor::M3508::feedback.id = rxHeader.Identifier;
          Motor::M3508::feedback.angle = (rxData[0] << 8) | rxData[1];
          Motor::M3508::feedback.speed = ((rxData[2] << 8) | rxData[3]);
          Motor::M3508::feedback.current = (int16_t)((rxData[4] << 8) | rxData[5]);
          Motor::M3508::feedback.temperature = rxData[6];

          uint32_t now = HAL_GetTick();
          if (Motor::M3508::feedback.last_time_ms > 0) {
            float dt_ms = (float)(now - Motor::M3508::feedback.last_time_ms);
            Motor::M3508::feedback.frequency_Hz = 1000.0f / dt_ms;
          }
          Motor::M3508::feedback.last_time_ms = now;

          valid_rxdata = (Motor::M3508::feedback.angle <= 8191) &&
                       (abs(Motor::M3508::feedback.speed) < 10000) &&
                       (abs(Motor::M3508::feedback.current) < 20000) &&
                       (Motor::M3508::feedback.temperature < 150);
          
          if (!valid_rxdata) {
            running = false;  // Handle error
          }
        } 

        else if (rxHeader.Identifier == 0x205) {
          Motor::GM6020::feedback.id = rxHeader.Identifier;
          Motor::GM6020::feedback.angle = (rxData[0] << 8) | rxData[1];
          Motor::GM6020::feedback.speed = (int16_t)((rxData[2] << 8) | rxData[3]);
          Motor::GM6020::feedback.current = (int16_t)((rxData[4] << 8) | rxData[5]);
          Motor::GM6020::feedback.temperature = rxData[6];

          now = HAL_GetTick();
          if (Motor::GM6020::feedback.last_time_ms > 0) {
            float dt_ms = (float)(now - Motor::GM6020::feedback.last_time_ms);
            Motor::GM6020::feedback.frequency_Hz = 1000.0f / dt_ms;
          }
          Motor::GM6020::feedback.last_time_ms = now;

          valid_rxdata = (Motor::GM6020::feedback.angle <= 8191) &&
                       (abs(Motor::GM6020::feedback.speed) < 20000) &&
                       (abs(Motor::GM6020::feedback.current) < 40000) &&
                       (Motor::GM6020::feedback.temperature < 150);
          }

          if (!valid_rxdata) {
            running = false;  // Handle error
          }

      }
      // Re-activate notification (good practice)
      HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING, 0xFFFFFFFF);
    }
  }
  
  void ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
    if (ErrorStatusITs & FDCAN_IT_BUS_OFF) {
      __HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_FLAG_BUS_OFF);  // Fix: use hfdcan param
      hfdcan->Instance->CCCR &= ~FDCAN_CCCR_INIT;
  
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); 
      running = 0;
      Modules::PID::resetPIDState(integral, prev_error);
  
      while (hfdcan->Instance->CCCR & FDCAN_CCCR_INIT);
    } else if (ErrorStatusITs & FDCAN_IT_ERROR_PASSIVE) {
      __HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_FLAG_ERROR_PASSIVE);
      // Optional: Signal warning (e.g., blink GPIO)
    } else if (ErrorStatusITs & FDCAN_IT_ERROR_WARNING) {
      __HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_FLAG_ERROR_WARNING);
      // Optional: Early stop
      running = false;
    }
  }

  void UART1custom_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
      HAL_UART_Transmit(&huart1, rx_uart_Buf, sizeof(rx_uart_Buf), 100);
    }
    // Re-arm for next frame
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_uart_Buf, 1);
}

  void mainTask(void) {
  
    Modules::Evaluation::Test eval(1.0f, 0.0f, (float)M_PI / 4);
  
    // CAN Init
    FDCAN_FilterTypeDef filter_m3508 = Motor::M3508::getFilter(0x7F0, 0x200);
    Motor::M3508::init(RxFifo0Callback, ErrorStatusCallback, &filter_m3508);

    FDCAN_FilterTypeDef filter_gm6020 = Motor::GM6020::getFilter(0x7F0, 0x205);
    Motor::GM6020::init(RxFifo0Callback, ErrorStatusCallback, &filter_gm6020);

    // Button initial
    button_state = HAL_GPIO_ReadPin(BTN_0_GPIO_Port, BTN_0_Pin);
    last_reading = button_state;
  
    last_pid_tick = HAL_GetTick();

      // For UART1 DMA
    HAL_UART_RegisterRxEventCallback(&huart1, UART1custom_RxEventCallback);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_uart_Buf, 1);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); 
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); 
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); 


    while (1) {
      now = HAL_GetTick();
  
      // Button check
      uint8_t reading = HAL_GPIO_ReadPin(BTN_0_GPIO_Port, BTN_0_Pin);
      if (reading != last_reading) {
        last_debounce = now;
        last_reading = reading;
      }
      if ((now - last_debounce) > debounce_ms) {
        if (reading != button_state) {
          button_state = reading;
          if (button_state == GPIO_PIN_RESET) { 
            running = !running;
            if (!running) {
              Modules::PID::resetPIDState(integral, prev_error);
              first_called = true;
              record_start = false;
              recode_end = false;
              float rad_dif = 0.0f; 
              float time_angle_before = 0.0f;
              float time_angle_after = 0.0f;
              float time_angle = 0.0f;
            }
          }
        }
      }
  
      dt = (float)(now - last_pid_tick) / 1000.0f; //dt = 0.001f
      if (dt < 0.001f) dt = 0.001f;
      last_pid_tick = now;
      // button check end
  
      send_current_m3508 = 0;
      send_current_gm6020 = 0;

      if (prev_uart_in != rx_uart_Buf[0]) {
        prev_uart_in = rx_uart_Buf[0];
        prev_error = 0.0f;
      }

      if ((HAL_GetTick() - Motor::M3508::feedback.last_time_ms) > 100) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); 
      }
      else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); 
      }
      if ((HAL_GetTick() - Motor::GM6020::feedback.last_time_ms) > 100) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); 
      }
      else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET); 
      }


        if (running) {

          if (rx_uart_Buf[0] == 's') {  // Sin        
            target_pos = eval.eval_Sin() ;  // Position target in rad (scale for larger range)
            current_pos = (float)((Motor::GM6020::feedback.angle * (2.0f * (float)M_PI / 8192.0f) - (float)M_PI) );  // Convert 0-8191 to -pi to pi rad
            // target_pos = (150.0f*sinf(2.0f * (float)M_PI*(HAL_GetTick()%1000)/1000.0f));
            // current_pos = (float)((Motor::M3508::feedback.angle * (2.0f * (float)M_PI / 8192.0f) - (float)M_PI) * (150 / M_PI) );  // Convert 0-8191 to -pi to pi rad
            error = target_pos - current_pos;
            output = Modules::PID::calculate(target_pos, current_pos, kp, ki, kd, integral, prev_error, dt);
            output = Modules::PID::pidClampMinMax(output, max_current);
            prev_error = error;
            send_current_m3508 = 0;
            send_current_gm6020 = (int16_t)output;
            
          }
          else if (rx_uart_Buf[0] == 'q') {  // Square
  
            target_pos = eval.eval_Square() ;  // Position target in rad (scale for larger range)
            current_pos = (float)((Motor::GM6020::feedback.angle * (2.0f * (float)M_PI / 8192.0f) - (float)M_PI) );  // Convert 0-8191 to -pi to pi rad
            error = target_pos - current_pos;
            output = Modules::PID::calculate(target_pos, current_pos, kp, ki, kd, integral, prev_error, dt);
            output = Modules::PID::pidClampMinMax(output, max_current);
            prev_error = error;
            send_current_m3508 = 0;
            send_current_gm6020 = (int16_t)output;
          }
          else if (rx_uart_Buf[0] == 't') {  // Triangle

  
            target_pos = eval.eval_Triangle() ;  // Position target in rad (scale for larger range)
            current_pos = (float)((Motor::GM6020::feedback.angle * (2.0f * (float)M_PI / 8192.0f) - (float)M_PI) );  // Convert 0-8191 to -pi to pi rad
            error = target_pos - current_pos;
            output = Modules::PID::calculate(target_pos, current_pos, kp, ki, kd, integral, prev_error, dt);
            output = Modules::PID::pidClampMinMax(output, max_current);
            prev_error = error;
            send_current_m3508 = 0;
            send_current_gm6020 = (int16_t)output;
          }
          else if (rx_uart_Buf[0] == 'f') {  // FakeAim

            target_pos = eval.eval_FakeAim() ;  // Position target in rad (scale for larger range)
            current_pos = (float)((Motor::GM6020::feedback.angle * (2.0f * (float)M_PI / 8192.0f) - (float)M_PI));  // Convert 0-8191 to -pi to pi rad
            error = target_pos - current_pos;
            output = Modules::PID::calculate(target_pos, current_pos, 200000, ki, 400, integral, prev_error, dt);
            output = Modules::PID::pidClampMinMax(output, max_current);
            prev_error = error;
            send_current_m3508 = 0;
            send_current_gm6020 = (int16_t)output;

          }
          else if (rx_uart_Buf[0] == 'g') {  // sin

            target_speed = (150.0f*sinf(2.0f * (float)M_PI*(HAL_GetTick()%1000)/1000.0f)) * ratio_m3508;  // Position target in rad (scale for larger range)
            error = target_speed - Motor::M3508::feedback.speed;
            output = Modules::PID::calculate(target_speed, Motor::M3508::feedback.speed, 15, 0.5f, 0.0f, integral, prev_error, dt);
            output = Modules::PID::pidClampMinMax(output, max_current);
            prev_error = error;
            send_current_m3508 = (int16_t)output;
            send_current_gm6020 = 0;

          }
          else if (rx_uart_Buf[0] == '1') {  // speed 50rpm 
            target_speed = 300.0f * ratio_m3508;  // Target in RPM
            error = target_speed - Motor::M3508::feedback.speed;
            output = Modules::PID::calculate(target_speed, Motor::M3508::feedback.speed, 12.0f, 30.0f, 0.0f, integral, prev_error, dt);
            output = Modules::PID::pidClampMinMax(output, max_current);
            prev_error = error;
            send_current_m3508 = (int16_t)output;
            // send_current_m3508 = 500;
            send_current_gm6020 = 0;

          
          }
          else if (rx_uart_Buf[0] == '2') {  // absolute angle 
            float target_angle = 90.0f;  //  90 degrees in rad

            rad_dif = (Motor::GM6020::feedback.angle * (2.0f * (float)M_PI / 8192.0f) - (float)M_PI) - (target_angle * M_PI / 180.0f); 
            if (record_start == false){
              time_angle_before = HAL_GetTick();
              record_start = true;
            }
            else if (record_start == true && recode_end == false){ 
              if (fabs(rad_dif) < 0.1f){
                time_angle_after = HAL_GetTick();
                time_angle = (time_angle_after - time_angle_before); // in ms
                recode_end = true;
              }
            }


            target_pos = (target_angle * M_PI / 180.0f) * speed_scale;  // Position target in rad (scale for larger range)
            current_pos = (float)((Motor::GM6020::feedback.angle * (2.0f * (float)M_PI / 8192.0f) - (float)M_PI) * speed_scale);  // Convert 0-8191 to -pi to pi rad
            error = target_pos - current_pos;
            output = Modules::PID::calculate(target_pos, current_pos, kp, ki, kd, integral, prev_error, dt);
            output = Modules::PID::pidClampMinMax(output, max_current);
            prev_error = error;
            send_current_gm6020 = (int16_t)output;
            // send_current_gm6020 = 1000;
            send_current_m3508 = 0;
            

          }
          else if (rx_uart_Buf[0] == '3') {  // relative angle 
            if (first_called){
              prev_position = (float)Motor::GM6020::feedback.angle * (2.0f * (float)M_PI / 8192.0f) - (float)M_PI;  // Convert to -pi to pi, no scale
              first_called = false;
            }
            
            float added_angle_rad = M_PI / 2.0f;  // 90 degrees in rad
            current_pos = (float)((Motor::GM6020::feedback.angle * (2.0f * (float)M_PI / 8192.0f) - (float)M_PI) * speed_scale);  // Convert to -pi to pi
            target_pos = prev_position + added_angle_rad;  // Relative add

            // Normalize target_pos to -pi to pi
            target_pos = (fmod(target_pos + (float)M_PI, 2.0f * (float)M_PI) - (float)M_PI) * speed_scale;  // Position target in rad (scale for larger range)

            error = target_pos - current_pos;
            output = Modules::PID::calculate(target_pos, current_pos, kp, ki, kd, integral, prev_error, dt);
            output = Modules::PID::pidClampMinMax(output, max_current);
            prev_error = error;
            send_current_gm6020 = (int16_t)output;
            send_current_m3508 = 0;

            rad_dif = (Motor::GM6020::feedback.angle * (2.0f * (float)M_PI / 8192.0f) - (float)M_PI) - ((target_pos / speed_scale) * M_PI / 180.0f); 
            if (record_start == false){
              time_angle_before = HAL_GetTick();
              record_start = true;
            }
            else if (record_start == true && recode_end == false){ 
              if (fabs(rad_dif) < 1.0f){
                time_angle_after = HAL_GetTick();
                time_angle = (time_angle_after - time_angle_before); // in ms
                recode_end = true;
              }
            }
          }

          
        }

        
        FDCAN_TxHeaderTypeDef header_m3508 = Motor::M3508::getTxHeader(1, Motor::MotorType::M3508);
        Motor::M3508::constructTxData(Motor::M3508::txdata, 1, send_current_m3508);
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header_m3508, Motor::M3508::txdata);

        FDCAN_TxHeaderTypeDef header_gm6020 = Motor::GM6020::getTxHeader(1, Motor::MotorType::GM6020);
        Motor::GM6020::constructTxData(Motor::GM6020::txdata, 1, send_current_gm6020);
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header_gm6020, Motor::GM6020::txdata);

    
        HAL_Delay(1);
      }
    }
  
  
}// extern "C"