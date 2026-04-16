#include "sys.h"
#include "usart.h"	
//////////////////////////////////////////////////////////////////////////////////	 
//这是ALIENTEK为了同学们学习ucos而做的例程，例程秉承简单即可用的理念
//对外配置Systick中断和pendSV中断，为了减小代码篇幅，这里不再对外配置NVIC
#if SYSTEM_SUPPORT_OS
#include "includes.h"				//ucos 使用	  
#endif

//定义一个FILE文件，标准库需要这个	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式	  
void _sys_exit(int x) 
{ 
	x = x; 
}
//重定义fputc函数  
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 


/*使用microLib的方法*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
    
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
 */
 
#if EN_USART1_RX   //串口1接收和发送中断
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误    

u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符
//接收状态
//bit15，清零标记
//bit14，接收到0x0d
//bit13~0，接收到的有效字节数

u16 USART_RX_STA=0;       //接收状态	  
  
void uart_init(u32 bound){
  //GPIO初始化
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//先占优先级3级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//接收模式

  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART1, ENABLE);                    //使能串口1 

}

void USART1_IRQHandler(void)                	//串口1中断服务程序
	{
	u8 Res;
#if SYSTEM_SUPPORT_OS		//使用ucos的话
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //检查USART1发生接受中断与否，接收0x0d 0x0a是两个回车换行符
		{
		Res =USART_ReceiveData(USART1);	//读取接收到的数据
		
		if((USART_RX_STA&0x8000)==0)//检查是否接受完毕
			{
			if(USART_RX_STA&0x4000)//检查是否接受到回车0x0d
				{
				if(Res!=0x0a)USART_RX_STA=0;//没接受到0x0a，错误，重新开始	
				else USART_RX_STA|=0x8000;	//接受成功
			}
			else //木有接受到回车0X0d
				{		
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
					{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
			}   		 
     } 
#if SYSTEM_SUPPORT_OS		//使用ucos的话
	OSIntExit();   						
#endif
} 
#endif	