#include "miscellaneousTools.h"
#include "GlobalVariable.h" 

// 线程函数
static void thread_savepos_entry(void *parameter)
{
	uint8_t state1 = 0;					 // 状态机（模拟机器启动与停止）
	uint8_t state2 = 0;					 // 状态机（模拟工作步骤）
	rt_uint32_t recved_event; 	 // 事件的接收
	char status_code_str[10]; // 电机的字符形式的状态码
	uint32_t satus_code = 0;		 // 十六进制形式的状态码
	int ret_val;								 // 电机通讯应答
	int mov_flag = -1;					 // 电机是否运动标志位
	int stall_flag = -1;				 // 堵转标志位
	
	while(1)
	{
		switch(state1)
		{
			case 0:		// 一直等待 THREAD_GET_WORK 事件
				rt_event_recv(Schedule, THREAD_GET_WORK, RT_EVENT_FLAG_OR, RT_WAITING_FOREVER, &recved_event);
				state1 = 10;
				rt_kprintf("SavePos Thread Ready to work. \n");
				break;
			
			
			
			case 10:	// 正常工作状态
				/*--------------------读取一些会打断正常工作的突发事件------------------*/
				recved_event = 0;
				rt_event_recv(Schedule, THREAD_GET_WORK, RT_EVENT_FLAG_OR, 0, &recved_event);
			
				if(!(recved_event & THREAD_GET_WORK))   // 关机事件
				{
					state1 = 99;
					break;
				}
				/*----------------------------------------------------------------------*/
			
				
				
				/*------------------------------正常工作流程----------------------------*/
				// 读取电机状态
				ret_val = ToMotorWithReturn("SC", status_code_str);
				if(!ret_val)  // 读取失败
				{
					rt_kprintf("SavePos Thread Read Status Code FAIL!! \n");
				}
				else					// 读取成功
				{
					// 将读取到的字符解析成一个十六进制数
					StrToHex(status_code_str, &satus_code);
					mov_flag = satus_code & 0x10;   // 获取电机运动标志位
					stall_flag = satus_code & 0x04; // 获取电机堵转标志位
					
					// 更新堵转寄存器
					if(stall_flag) // 如果堵转，更新相应寄存器
					{
						WriteShortToReg(STALL_DETECTION, 1);
					}
					else           // 如果不堵转，更新相应寄存器
					{
						WriteShortToReg(STALL_DETECTION, 0);
					}
					
					// 运动检测和电机位置实时保存
					switch(state2)
					{
						case 0 : // 检查电机是否运动
							if(mov_flag) 	// 如果电机在运动
							{
								rt_event_send(Schedule, MOV_STA); // 设置 MOV_STA， 告知其他线程电机正在运动中
								state2 = 10; // 更新状态机，接下来检查电机什么时候停止
								rt_kprintf("Motor is moving !! \n");
							}
							break;
						
						case 10: // 检查电机是否停止
							if(!mov_flag)
							{
								rt_kprintf("Motor stopped, it's time to save encoder !! \n");
								SaveEncoderData(); // 将此时最新的电机编码值存入EEPROME
								rt_event_recv(Schedule, MOV_STA, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, 0, &recved_event); // 电机停止，清空 MOV_STA 事件标志
								state2 = 0;
							}
							break;
					
						default:
							break;
					}
				}
				/*----------------------------------------------------------------------*/
				
				rt_thread_mdelay(REFRESH_BASE);
				break;
			
			
			/*-----------------------处理突发事件的相应入口，类似于中断向量表------------------------------*/
				
			case 99:  // 关机，做一些善后工作
				state1 = 0;
				state2 = 0;
				rt_kprintf("SavePos Thread Shut Down. \n");
				break;		
			
			/*---------------------------------------------------------------------------------------------*/
			
			default:
				break;
		}
	}
}

// 保存编码器线程初始化
int savepos_init()
{
	// 开启线程
	rt_thread_t thread_savepos = rt_thread_create("thread_savepos", thread_savepos_entry, RT_NULL, 4096, SAVEPOS_PRIORITY, 10);
	if (thread_savepos != RT_NULL)
  {
     rt_thread_startup(thread_savepos);
  }
  else
  {
		 rt_kprintf("SavePos thread Create Fail!\n");
     return RT_ERROR;
  }
	
	// the end
	return RT_EOK;
}

