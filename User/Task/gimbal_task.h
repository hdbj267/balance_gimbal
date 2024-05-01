#ifndef GIMBAL_APP_H
#define GIMBAL_APP_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "pid.h"
#include "remote_app.h"
#include "INS_task.h"


#define GIMBAL_TASK_TIME_1MS                  (1u)           

#define ROBOT_COMMON_MODE_KEY                 KEY_PRESSED_OFFSET_E            
#define ROBOT_ROTATE_STOP_MODE_KEY            KEY_PRESSED_OFFSET_Q        
#define ROBOT_WEAK_MODE_KEY          		  KEY_PRESSED_OFFSET_F 


#define GIMBAL_TASK_INIT_TIME                 (3005)

#define GAMBAL_YAW_MAX_ANGLE_SET_FACT         (0.0002f)   //闁儲甯堕柆銉︽綄閻忓灚鏅辨惔锟�0.0005
#define GAMBAL_PITCH_MAX_ANGLE_SET_FACT       (0.0002f)

#define GAMBAL_MOUSE_PITCH_MAX_ANGLE_SET_FACT (-0.005f)    //婢堆傜秴缁夊绱堕弽鍥╀紥閺佸繐瀹�
#define GAMBAL_MOUSE_YAW_MAX_ANGLE_SET_FACT   (-0.005f)
#define GAMBAL_MOUSE_PITCH_MIN_ANGLE_SET_FACT (-0.008f)    //鐏忓繋缍呯粔濠氱炊閺嶅洨浼掗弫蹇撳
#define GAMBAL_MOUSE_YAW_MIN_ANGLE_SET_FACT   (-0.008f)

#define GAMBAL_YAW_INIT_ENCODE_VALUE          (3427.0f)
#define GAMBAL_PITCH_INIT_ENCODE_VALUE        (6034.0f) //4200//(4096.0f)

#define GAMBAL_YAW_angle_VALUE     (22.755555f)                    //yaw
#define GAMBAL_PITCH_angle_VALUE   (22.755555f)                    //pitch
#define CAMBAL_PITHC_LEVEL_ANGLE_TO_ENCODE    (3700.0f)//娴滄垵褰村鏉戦挬閸婏拷
#define CAMBAL_PITCH_MIN_ANGLE_TO_ENCODE      (3200.0f)
#define CAMBAL_PITCH_MAX_ANGLE_TO_ENCODE      (4285.0f)//娴ｅ骸銇旀稉瀣

#define GAMBAL_ENCODE_TO_ANGLE                ((float)0.0439506775729459)  //   360.0/8191.0

typedef enum
{
	ROBOT_CALI_MODE = 0,     //鐠嬪啳鐦Ο鈥崇础
	ROBOT_INIT_MODE,	     //閸掓繂顫愰崠锟�
	ROBOT_INIT_END_MODE,     //閸掓繂顫愰崠鏍波閺夌喎鍨忛幑銏㈠仯
	ROBOT_COMMON_MODE,       //閺咁噣鈧艾绨抽惄妯跨闂呭繑膩瀵拷
	ROBOT_ROTATE_STOP_MODE,  //闂堟瑦顒涚亸蹇涙閾伙拷
	ROBOT_WEAK_MODE,		 //鍊掍笅璧板姩	
	ROBOT_VISION_MODE,       //鐟欏棜顫庨懛顏嗙€Ο鈥崇础
	ROBOT_ERROR_MODE,        //闁挎瑨顕�
}robot_work_mode_e;

typedef enum
{
	GIMBAL_CALI_MODE = 0,
	GIMBAL_INIT_MODE,  
	GIMBAL_ABSOLUTE_ANGLE_MODE,
	GIMBAL_RELATIVE_ANGLE_MODE, 
//	GIMBAL_MOTIONLESS, 	
}gimbal_work_mode_e;
  
typedef enum
{
    GIMBAL_MOTOR_RAW = 0,    //閻㈠灚婧€閸樼喎顫愰崐鍏煎付閸掞拷
    GIMBAL_MOTOR_GYRO,       //閻㈠灚婧€闂勨偓閾昏桨鍗庣憴鎺戝閹貉冨煑
    GIMBAL_MOTOR_ENCONDE,    //閻㈠灚婧€缂傛牜鐖滈崐鑹邦潡鎼达附甯堕崚锟�
}gimbal_motor_feedback_mode_e;

typedef enum
{
	KEY_MOUSE_MODE = 0,
	REMOTE_MODE,
	GUI_CALI_MODE,
}robot_control_mode_e;

typedef struct
{
	int16_t yaw_motor;
	int16_t pitch_motor;
	
	int16_t trigger_motor;
	int16_t fric1_motor;
	int16_t fric2_motor;
}given_current_t;

typedef struct
{
	cascade_pid_t yaw_pid;	
	cascade_pid_t pitch_pid;

}gimbal_pid_t;

typedef struct
{
	RC_ctrl_t *rc_ctrl;
	const INS_t *gimbal_INS;
	motor_msg_t *gimbal_yaw_motor_msg;
	motor_msg_t *gimbal_pitch_motor_msg;
	
	given_current_t given_current;
	
	gimbal_motor_feedback_mode_e yaw_motor_fdb_mode;
	gimbal_motor_feedback_mode_e pitch_motor_fdb_mode;
	
	float gimbal_yaw_set;
	float gimbal_yaw_fdb;
	float gimbal_pitch_set;
	float gimbal_pitch_fdb;

	int16_t vision_yaw;
	int16_t vision_pitch;
}gimbal_control_data_t;


extern given_current_t given_current;
uint8_t get_robot_control_mode(void);
uint8_t get_robot_work_mode(void);
uint8_t get_gimbal_work_mode(void);
void set_robot_control_mode(robot_control_mode_e mode);

void gimbal_set_and_fdb_update(gimbal_control_data_t *gimbal_control_data,   \
							   robot_control_mode_e robot_control_mode, 
							   vision_control_data_t control_data);
#endif
