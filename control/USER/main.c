#include "stm32f10x.h"
#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "serial.h"
#include "timer.h"
#include "pid.h"

//全局变量定义
PID_TypeDef PID_x, PID_y;//两个PID结构体PID_x和PID_y

int coords[2];//接收到的坐标数据

u16 targetX = 320;//目标x坐标
u16 targetY = 240;//目标y坐标

int main(void)
{
	u16 pwmval_x, pwmval_y;//占空比参数和pwmval_x和pwmval_y

	delay_init();		 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC优先级分组2:2位抢占优先级2位响应优先级
	uart_init(115200);	 //串口初始化115200
 	LED_Init();			 //LED初始化	
	KEY_Init();	          //按键初始化，不要删除	
	TIM3_PWM_Init(9999,143);	 //定时器3初始化72*10^6/(9999+1)/(143+1)=50Hz

	//pid参数初始化
	pid_init(0.04, 0, 0.30, &PID_x);
	pid_init(0.05, 0, 0.30, &PID_y);
	
	//初始化接收数据
	coords[0] = 320;
	coords[1] = 240;
	
	//初始化pwm参数
	pwmval_x = 650;
	pwmval_y = 650;
	
  while(1)
	{
		//1接收串口数据并将数据存在coords数组里
		recieveData();
				// Debug: Print received buffer and parsed coordinates
				// Debug: Print received buffer and parsed coordinates
		if(USART_RX_STA & 0x8000) // Only print if a complete message was received
		{
			printf("Received: %s\\r\\n", USART_RX_BUF);
			printf("Parsed Coords: X=%d, Y=%d\\r\\n", coords[0], coords[1]);
		}
		delay_ms(50); // Add a small delay to slow down debug output


		//2调用pid函数计算得到PWM的增加值，修改pwm参数
		pwmval_x = pwmval_x + pid(coords[0],targetX, &PID_x);
		pwmval_y = pwmval_y - pid(coords[1],targetY, &PID_y);

		//3对pwm参数进行限幅保护确保参数在有效范围内，3个pwm参数给1/2/3通道赋值
		if(pwmval_x>300 && pwmval_x<1200)
			TIM_SetCompare1(TIM3,pwmval_x);
		if(pwmval_y>300 && pwmval_y<1000)
			TIM_SetCompare2(TIM3,pwmval_y);
				// Debug: Print calculated PWM values
				// Debug: Print calculated PWM values
		printf("PWM Values: X=%d, Y=%d\\r\\n", pwmval_x, pwmval_y);
		delay_ms(50); // Add a small delay to slow down debug output


	}	 
}