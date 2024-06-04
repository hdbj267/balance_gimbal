/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia
 * @Teammate
 * @Version: V1.0
 * @Date: 2021.4.13
 * @Description:     根据当届规则进行应用修改           
 * @Note:           
 * @Others: 
**/
#include "rule.h"
#include "can2_app.h"

extern ext_Judge_data_t Judge_data;
shoot_real_mag shoot_mag;
void shoot_limit(void)                      
{
	if (Judge_data.hurt_type == 0x3)                                      //读取超热量标志，辅助
	{
		shoot_mag.Excess_Heat_flag =  1;
	}
	else
	{
		shoot_mag.Excess_Heat_flag =  0;
	}
	if (Judge_data.hurt_type == 0x2)                                      //读取超射速标志，辅助
	{
		shoot_mag.Excess_Speed_shoot_flag =  1;
	}
	else
	{
		shoot_mag.Excess_Speed_shoot_flag =  0;
	}
    shoot_heat_limit();
	shoot_speed_limit();
}
/**
  * @brief         发射机构热量限制
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void shoot_heat_limit(void)                   
{	
    if (Judge_data.shooter_id1_17mm_cooling_limit > Judge_data.shooter_id1_17mm_cooling_heat )
	{

		shoot_mag.residual_heat = Judge_data.shooter_id1_17mm_cooling_limit - Judge_data.shooter_id1_17mm_cooling_heat;
	}
	else
	{
		shoot_mag.residual_heat = 0;

	}
	shoot_mag.allow_shoot = (uint16_t)(shoot_mag.residual_heat/10);            //+10 
	if (shoot_mag.allow_shoot > 1  )
	{
		shoot_mag.allow_shoot_flag = 1;
	}
	else
	{
		shoot_mag.allow_shoot_flag = 0;
	}

}
/**
  * @brief      射速限制
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
//int shoot_B;
float bullet_speed=18;

void shoot_speed_limit(void)     //18的要小心     *hyj
{
//	int shoot_B;
//	switch(Judge_data.shooter_id1_17mm_speed_limit)
//	{
//		case 15:shoot_B=245; break;
//		case 18:shoot_B=200; break;
//		case 22:shoot_B=180; break;
//		case 30:shoot_B=110; break;
//		default:  break;
//	}
	shoot_mag.allow_shoot_speed = (bullet_speed-30)/15*340+1000;//23届弹速无限制，30m/s都可以，后面把这个自变量换成裁判系统限制的最大弹速
	//y=(x-x2)/(x1-x2)*(y1-y2)+y2  (15,660) (30,1000) 两点式，自变量是弹速，结果是摩擦轮速度
	//加最值限制
	if (shoot_mag.allow_shoot_speed > 1000)
	{
		shoot_mag.allow_shoot_speed = 1000;
	}
	shoot_mag.allow_fri_speed = shoot_mag.allow_shoot_speed;     
	
}

