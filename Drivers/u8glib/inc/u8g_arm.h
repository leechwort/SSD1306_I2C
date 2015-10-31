 #ifndef _U8G_ARM_H
 #define _U8G_ARM_H


 #include "u8g.h"
 #include "stm32f4xx_hal.h"


 #define DATA_BUFFER_SIZE 1000
 #define I2C_TIMEOUT 10000
 #define DEVICE_ADDRESS 0x78 //device address is written on back side of your display
 #define I2C_HANDLER hi2c3 // use your i2c handler

 extern I2C_HandleTypeDef I2C_HANDLER; 


 uint8_t u8g_com_hw_i2c_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);

 #endif 
