/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       IST8310middleware.c/h
  * @brief      IST8310�������м�㣬���IST8310��IICͨ��,��Ϊ����MPU6500��SPIͨ��
  *             �������õ���ͨ��mpu6500��IIC_SLV0��ɶ�ȡ��IIC_SLV4���д�롣
  * @note       IST8310ֻ֧��IIC��ȡ
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "ist8310driver_middleware.h"
#include "main.h"
//#include "cmsis_os.h"
//#include "bsp_delay.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
extern I2C_HandleTypeDef hi2c3;


void ist8310_GPIO_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	__HAL_RCC_GPIOG_CLK_ENABLE();
	
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_SET);//G6

	/*Configure GPIO pin : PtPin */
	GPIO_InitStruct.Pin = RSTN_IST8310_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(RSTN_IST8310_GPIO_Port, &GPIO_InitStruct);
}

void ist8310_com_init(void)
{
}


uint8_t ist8310_IIC_read_single_reg(uint8_t reg)
{
    uint8_t res;
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 100);
    return res;
}
void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

}
void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}
void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}
void ist8310_delay_ms(uint16_t ms)
{
    vTaskDelay(ms);//osDelay(ms);
}
void ist8310_delay_us(uint32_t us)//(uint16_t us)
{
    delay_us(us);
}

void ist8310_RST_H(void)
{
    HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_SET);
}
extern void ist8310_RST_L(void)
{
    HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_RESET);
}
