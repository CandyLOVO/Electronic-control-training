#include "Chassis_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"

pid_struct_t motor_pid_chassis[4];
pid_struct_t supercap_pid;
motor_info_t  motor_info_chassis[8];       //电机信息结构体
 fp32 chassis_motor_pid [3]={30,0.5,10};   //用的原来的pid
 fp32 superpid[3] = {120,0.1,0};

volatile int16_t Vx=0,Vy=0,Wz=0;
volatile float wheel_rpm[4]; //各轮子速度
float speed_Max = 1000; //限速 3508最大转速480rpm
float pid_out[4]; //输出电流
                           
int16_t Temp_Vx;
int16_t Temp_Vy;
int fllowflag = 0;
volatile int16_t motor_speed_target[4];
 extern RC_ctrl_t rc_ctrl; //声明遥控器数据结构体
 uint8_t sbus_rx_buffer[18]; //声明遥控器缓存数组

 extern ins_data_t ins_data;
 extern float powerdata[4];
 extern uint16_t shift_flag;
// Save imu data

int8_t chassis_mode = 1;//判断底盘状态，用于UI编写

//获取imu——Yaw角度差值参数
static void Get_Err(); 

//参数重置
static void Chassis_loop_Init(); 

//super_cap
void power_limit(int *speed);

int chassis_mode_flag =0;

void qe();

#define angle_valve 5
#define angle_weight 55
#define RADIAN_COEF 57.3 //180/pi
float rc_convert(float get, float max, float dead_limit);
	
   void Chassis_task(void const *pvParameters)
	{
 			for (uint8_t i = 0; i < 4; i++)
			{
        pid_init(&motor_pid_chassis[i], chassis_motor_pid, 6000, 6000); //init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
			} 
			//超级电容
				pid_init(&supercap_pid, superpid, 3000, 3000); //init pid parameter, kp=40, ki=3, kd=0, output limit = 16384	
			
    for(;;)				//底盘运动任务
    {	                                                                                                                         
			Vx = rc_ctrl.rc.ch[0]; //右拨杆初始0 向左-548 向右556 vy[-1024,-1016] 左右
			Vy = rc_ctrl.rc.ch[1]; //右拨杆初始-1023 向上393 向下-391 前后
			Wz = rc_ctrl.rc.ch[4]; //滚动初始0 向上732 向下556

			if(Vy==-1023){
				Vy = 0;
			}
			
			Vx = rc_convert(Vx,556,500);
			Vy = rc_convert(Vy,393,300);
//			Wz = speed_Max * rc_convert(Wz,732,500);                   
			
			//计算各轮子速度 
			wheel_rpm[0] = -Vx + Vy + Wz; //右前 //转换为rad/s
			wheel_rpm[1] = -(Vx + Vy - Wz); //左前
			wheel_rpm[2] = -(- Vx + Vy - Wz); //左后
			wheel_rpm[3] = Vx + Vy + Wz; //右后
			//将轮子速度转换为电机内转子速度
			//LH说忽略 :(
			
			//功率限制
			float max = wheel_rpm[0];
			for(int i=1;i<=3;i++){
				if(fabs(wheel_rpm[i])>fabs(max)){
					max = wheel_rpm[i];
				}
			}
			if(fabs(max) > speed_Max){
				float rate = speed_Max / fabs(max);
				for(int i=0;i<=3;i++){
					wheel_rpm[i] *= rate;
				}
			}
			
			//PID速度控制 将速度转化为电流值
			for(int i=0;i<=3;i++){
				pid_out[i] = pid_calc(&motor_pid_chassis[i],motor_info_chassis[i].rotor_speed ,wheel_rpm[i]);
			}
			
			//控制电机
				set_motor_current_can2(0,pid_out[0],pid_out[1],pid_out[2],pid_out[3]);

			osDelay(1); 
    }
	}
//处理遥控器数据 //控制信号归一化
float rc_convert(float get, float max, float dead_limit)
{
	float proportion;
	float limit;
	if(get > dead_limit)
	{
		proportion = (get - dead_limit) / (max - dead_limit);
		limit = speed_Max;
	}
	else if(get < -dead_limit)
	{
		proportion = (-dead_limit - get) / (-dead_limit + max);
		limit = -speed_Max;
	}
	else
		proportion = 0;
	return proportion * limit;
}




static void Chassis_loop_Init()
{
	Vx = 0;
	Vy = 0;
	Wz = 0;
}

//运动解算
void chassis_motol_speed_calculate()
{
	
	  motor_speed_target[CHAS_LF] =  0;
    motor_speed_target[CHAS_RF] =  0;
    motor_speed_target[CHAS_RB] =  0; 
    motor_speed_target[CHAS_LB] =  0;
}
//运动解算
//速度限制函数
  void Motor_Speed_limiting(volatile int16_t *motor_speed,int16_t limit_speed)  
{
    uint8_t i=0;
    int16_t max = 0;
    int16_t temp =0;
    int16_t max_speed = limit_speed;
    fp32 rate=0;
    for(i = 0; i<4; i++)
    {
      temp = (motor_speed[i]>0 )? (motor_speed[i]) : (-motor_speed[i]);//求绝对值
		
      if(temp>max)
        {
          max = temp;
        }
     }	
	
    if(max>max_speed)
    {
          rate = max_speed*1.0/max;   //*1.0转浮点类型，不然可能会直接得0   *1.0 to floating point type, otherwise it may directly get 0
          for (i = 0; i < 4; i++)
        {
            motor_speed[i] *= rate;
        }

    }

}
//电机电流控制
void chassis_current_give() 
{
	
    uint8_t i=0;
        
    for(i=0 ; i<4; i++)
    {
        motor_info_chassis[i].set_current = pid_calc(&motor_pid_chassis[i], motor_info_chassis[i].rotor_speed,motor_speed_target[i]);
    }
    	set_motor_current_can2(0, motor_info_chassis[0].set_current, motor_info_chassis[1].set_current, motor_info_chassis[2].set_current, motor_info_chassis[3].set_current);
 

}






