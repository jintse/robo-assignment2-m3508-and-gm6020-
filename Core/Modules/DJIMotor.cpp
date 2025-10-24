#include "DJIMotor.hpp"
#include <string.h>  // For memset
#include <math.h>    // For PI

/*
Todo: Implement your functions or classes here
*/
namespace Motor {

  namespace M3508 {

    MotorFeedback feedback = {1, 0, 0, 0, 0, 0, 0.0f}; // Definition

    uint8_t txdata[8] = {0}; // Definition

    FDCAN_FilterTypeDef getFilter(uint16_t filterID2, uint16_t filterID1) {
      FDCAN_FilterTypeDef filter = {
        .IdType = FDCAN_STANDARD_ID,
        .FilterIndex = 0,
        .FilterType = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1 = filterID1, // e.g., 0x200 for M3508 motors, 0x1FF for GM6020 motors
        .FilterID2 = filterID2 // e.g., 0x7F0 both motor
      };
      return filter;
    }

    FDCAN_TxHeaderTypeDef getTxHeader(uint8_t id, MotorType type) {
      FDCAN_TxHeaderTypeDef header = {
        .Identifier = 0,
        .IdType = FDCAN_STANDARD_ID,
        .TxFrameType = FDCAN_DATA_FRAME,
        .DataLength = FDCAN_DLC_BYTES_8,
        .ErrorStateIndicator = FDCAN_ESI_PASSIVE,
        .BitRateSwitch = FDCAN_BRS_OFF,
        .FDFormat = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
        .MessageMarker = 0
      };
    
      if (type == MotorType::M3508) {
        header.Identifier = (id <= 4) ? 0x200 : 0x1FF;
      } else {
        header.Identifier = (id <= 4) ? 0x1FE : 0x2FE;
      }
      return header;
    }
    
    void init(pFDCAN_RxFifo0CallbackTypeDef callback, pFDCAN_ErrorStatusCallbackTypeDef errorCallback, FDCAN_FilterTypeDef* filter) {
      HAL_FDCAN_RegisterRxFifo0Callback(&hfdcan1, callback);
      HAL_FDCAN_RegisterErrorStatusCallback(&hfdcan1, errorCallback);
      HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
      HAL_FDCAN_ConfigFilter(&hfdcan1, filter);
      HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING, 0xFFFFFFFF);
      HAL_FDCAN_Start(&hfdcan1);
    }
    
    void constructTxData(uint8_t data[8], const uint8_t id, int16_t current) {
      memset(data, 0, 8);
      uint8_t offset = 2 * ((id - 1) % 4);
      data[offset] = (current >> 8) & 0xFF;
      data[offset + 1] = current & 0xFF;

    
    }
    
    void send(const FDCAN_TxHeaderTypeDef *header, const uint8_t data[8]) {
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, header, const_cast<uint8_t*>(data));
    }
  
  } // namespace M3508

  /////////////////////

  namespace GM6020 {
      
    MotorFeedback feedback = {1, 0, 0, 0, 0, 0, 0.0f}; // Definition

    uint8_t txdata[8] = {0}; // Definition

    FDCAN_FilterTypeDef getFilter(uint16_t filterID2, uint16_t filterID1) {
      FDCAN_FilterTypeDef filter = {
        .IdType = FDCAN_STANDARD_ID,
        .FilterIndex = 0,
        .FilterType = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1 = filterID1, // e.g., 0x200 for M3508 motors, 0x1FF for GM6020 motors
        .FilterID2 = filterID2 // e.g., 0x7F0 both motor
      };
      return filter;
    }
  
    FDCAN_TxHeaderTypeDef getTxHeader(uint8_t id, MotorType type) {
      FDCAN_TxHeaderTypeDef header = {
        .Identifier = 0,
        .IdType = FDCAN_STANDARD_ID,
        .TxFrameType = FDCAN_DATA_FRAME,
        .DataLength = FDCAN_DLC_BYTES_8,
        .ErrorStateIndicator = FDCAN_ESI_PASSIVE,
        .BitRateSwitch = FDCAN_BRS_OFF,
        .FDFormat = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
        .MessageMarker = 0
      };
    
      if (type == MotorType::GM6020) {  
        header.Identifier = (id <= 4) ? 0x1FF : 0x2FE;
      } 

      return header;
    }
    
    void init(pFDCAN_RxFifo0CallbackTypeDef callback, pFDCAN_ErrorStatusCallbackTypeDef errorCallback, FDCAN_FilterTypeDef* filter) {
      HAL_FDCAN_RegisterRxFifo0Callback(&hfdcan1, callback);
      HAL_FDCAN_RegisterErrorStatusCallback(&hfdcan1, errorCallback);
      HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
      HAL_FDCAN_ConfigFilter(&hfdcan1, filter);
      HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING, 0xFFFFFFFF);
      HAL_FDCAN_Start(&hfdcan1);
    }
    
    void constructTxData(uint8_t data[8], const uint8_t id, const int16_t current) {
      memset(data, 0, 8);
      uint8_t offset = 2 * ((id - 1) % 4);
      data[offset] = (current >> 8) & 0xFF;
      data[offset + 1] = current & 0xFF;
    }
    
    void send(const FDCAN_TxHeaderTypeDef *header, const uint8_t data[8]) {
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, header, const_cast<uint8_t*>(data));
    }
  
  } // namespace GM6020

} // namespace Motor