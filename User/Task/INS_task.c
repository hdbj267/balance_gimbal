/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia  
 * @Teammate：
 * @Version: V3.0
 * @Date:2020.4.13
 * @Description:    机器人的姿态解算（YAW轴会漂移，新板子就不会），需要解决
 *                  9轴只开了6轴，磁力计未矫正所以没开。
 *                  C板每换一个位置就要修正一次安装矩阵（第208行）
 * @Note:       
 * @Others: 
**/
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpio.h"


#include "INS_task.h"

#include "main.h"

//#include "cmsis_os.h"

//#include "bsp_imu_pwm.h"
//#include "bsp_spi.h"
#include "bmi088driver.h"
#include "ist8310driver.h"
#include "pid.h"

#include "MahonyAHRS.h"
#include "math.h"
#include "arm_math.h"

typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */


INS_t INS;


static void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
static fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}
/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             主要利用陀螺仪bmi088，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过bmi088的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm给定
//安装旋转矩阵   *hyj
#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \

#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}  
    
/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_temp_control(fp32 temp);

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_cmd_spi_dma(void);


void AHRS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3]);
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3]);
void get_angle(fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll);

extern SPI_HandleTypeDef hspi1;

static TaskHandle_t INS_Task_Handler;//INS_task_local_handler;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2,0xFF,0xFF,0xFF};


volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;


bmi088_real_data_t bmi088_real_data;
fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];
fp32 gyro_cali_offset[3];

fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];
fp32 accel_cali_offset[3];

ist8310_real_data_t ist8310_real_data;
fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};    //比例因子
fp32 mag_offset[3];                                                   //静态偏移量（硬铁干扰）
fp32 mag_cali_offset[3];

static uint8_t first_temperate;
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static pid_type_def imu_temp_pid;

static const float timing_time = 0.001f;   //tast run time , unit s.任务运行的时间 单位 s


//加速度计低通滤波
static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};




fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};                   //euler angle, unit rad.欧拉角 单位 rad

/**
  * @brief          旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式，
  *                 三个轴量度不同，磁力计有硬磁和软磁干扰，加椭球拟合应用     *hyj
  * @param[out]     gyro: 加上零漂和旋转
  * @param[out]     accel: 加上零漂和旋转
  * @param[out]     mag: 加上零漂和旋转
  * @param[in]      bmi088: 陀螺仪和加速度计数据
  * @param[in]      ist8310: 磁力计数据
  * @retval         none
  */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
        mag[i] = (ist8310->mag[0]) * mag_scale_factor[i][0] + (ist8310->mag[1]) * mag_scale_factor[i][1] + ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];            
        //mag[i] = (ist8310->mag[0] + 4.2658f) * mag_scale_factor[i][0] + (ist8310->mag[1] - 2.95210048f) * mag_scale_factor[i][1] + ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];            
    }
}   
/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL 
  * @retval         none
  */



void AHRS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f; 
    quat[2] = 0.0f;
    quat[3] = 0.0f;

}

void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3])
{
    MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
}
void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);     //atan2f:返回原点到（后，前）点的角度
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));                                    //返回该正弦值对应角度
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);      

}

/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        PID_calc(&imu_temp_pid, temp, 45.0f);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        //in beginning, max power
        if (temp > 45.0f)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                //
                first_temperate = 1;
                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT1_ACCEL_Pin)
    {
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == INT1_GYRO_Pin)
    {
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == DRDY_IST8310_Pin)
    {
        mag_update_flag |= 1 << IMU_DR_SHFITS;

//        if(mag_update_flag &= 1 << IMU_DR_SHFITS)
//        {
//            mag_update_flag &= ~(1<< IMU_DR_SHFITS);
//            mag_update_flag |= (1 << IMU_SPI_SHFITS);

//            //ist8310_read_mag(ist8310_real_data.mag);
//        }
    }
    else if(GPIO_Pin == GPIO_PIN_0)
    {
        //wake up the task
        //唤醒任务
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INS_Task_Handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }


}

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_cmd_spi_dma(void)
{

        //开启陀螺仪的DMA传输
        if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
            gyro_update_flag |= (1 << IMU_SPI_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
            return;
        }
        //开启加速度计的DMA传输
        if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            accel_update_flag &= ~(1 << IMU_DR_SHFITS);
            accel_update_flag |= (1 << IMU_SPI_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
            return;
        }
        
        if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
            accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
            return;
        }
}


void DMA2_Stream2_IRQHandler(void)
{

    if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        //gyro read over
        //陀螺仪读取完毕
        if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
            
        }

        //accel read over
        //加速度计读取完毕
        if(accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //temperature read over
        //温度读取完毕
        if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        imu_cmd_spi_dma();

        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}

const fp32 *get_INS_angle_point(void)
{
    return INS_angle;
}

/**
  * @brief          get the rotation speed, 0:x-axis, 1:y-axis, 2:roll-axis,unit rad/s
  * @param[in]      none
  * @retval         the point of INS_gyro
  */
/**
  * @brief          获取角速度,0:x轴, 1:y轴, 2:roll轴 单位 rad/s
  * @param[in]      none
  * @retval         INS_gyro的指针
  */
extern const fp32 *get_gyro_data_point(void)
{
    return INS_gyro;
}

/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */


float yaw_temp,pitch_temp,roll_temp;
float last_yaw_temp,last_pitch_temp,last_roll_temp;
float yaw_cnt,pitch_cnt,roll_cnt;

INS_t INS;

void get_angle_and_gyro_value(INS_t *INS)
{

	INS->gyro_x = INS_gyro[0]*180/PI;//rad/s to angle/s
	INS->gyro_y = INS_gyro[1]*180/PI;
	INS->gyro_z = INS_gyro[2]*180/PI;

	last_yaw_temp = yaw_temp;           
	yaw_temp = INS_angle[0]*180/PI;
	if(yaw_temp-last_yaw_temp>=330)  //yaw轴角度经过处理后变成连续的，实现记录持续旋转！
	{
		yaw_cnt--;
	}
	else if (yaw_temp-last_yaw_temp<=-330)
	{
		yaw_cnt++;
	}
	INS->yaw_angle = yaw_temp + 360*yaw_cnt;
	INS->pitch_angle = -INS_angle[1]*180/PI;
	INS->roll_angle = INS_angle[2]*180/PI;


}

INS_t *get_INS_point(void)
{
	return &INS;
}

float sum0;
float avg0;
float aa0;
float sum1;
float avg1;
float aa1;
float sum2;
float avg2;
float aa2;



/**
  * @brief          imu任务, 初始化 bmi088, ist8310, 计算欧拉角
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void INS_task(void *argument)
{
    //wait a time
    vTaskDelay(INS_TASK_INIT_TIME);
    while(BMI088_init())
    {
        vTaskDelay(100);
    }
    while(ist8310_init())
    {
        vTaskDelay(100);
    }
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    //rotate and zero drift 
    imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
//    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
//    AHRS_init(INS_quat, bmi088_real_data.accel, ist8310_real_data.mag);
	PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
    AHRS_init(INS_quat, INS_accel, INS_mag);
	accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
    //get the handle of task
    //获取当前任务的任务句柄，
    INS_Task_Handler = xTaskGetHandle(pcTaskGetName(NULL));
    //set spi frequency
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        //Error_Handler();
    }

    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
    
    imu_start_dma_flag = 1;
    while (1)
    {

        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }
		
        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
        }

        if(accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
        }

        if(accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
        }
        imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
        accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];

        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];

        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];

        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];
		
		sum0+= INS_gyro[0];
		aa0++;
		avg0=sum0/aa0;
		sum1+= INS_gyro[1];
		aa1++;
		avg1=sum1/aa1;
		sum2+= INS_gyro[2];
		aa2++;
		avg2=sum2/aa2;
		
		INS_gyro[0] -= 0.00787779316;
		INS_gyro[1] -= -0.000948130386;
		INS_gyro[2] -= 0.00331324618;
        
		AHRS_update(INS_quat, timing_time, INS_gyro, accel_fliter_3, INS_mag);
        get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);

		get_angle_and_gyro_value(&INS);
        if(mag_update_flag &= 1 << IMU_DR_SHFITS)
        {
            mag_update_flag &= ~(1 << IMU_DR_SHFITS);
            mag_update_flag |= (1 << IMU_SPI_SHFITS);
            // ist8310_read_mag(ist8310_real_data.mag);     // 关磁力计  *hyj
        }
        

    }
}
