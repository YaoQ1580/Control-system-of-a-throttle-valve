// 电机反复往返，测试往返定位精度

#include "miscellaneousTools.h"
#include "GlobalVariable.h" 

static void thread_test_entry(void *parameter)
{
	uint8_t state = 0;
	int tik = 0;
	rt_uint32_t recved_event;
	int flag = 1; 
	char cmd[32];
	int str_len, ret_val;
	uint8_t high_char, low_char;
	uint32_t i, cnt;
	
	while(1)
	{
		switch(state)
		{
			case 0:   // 初始状态，应该等待事件：开始测试
				/*-------------等待测试1------------*/
				tik = 0;
				recved_event = 0;
				rt_event_recv(Schedule2, START_TEST, RT_EVENT_FLAG_OR, 0, &recved_event); // 等待开始测试
				if(recved_event & START_TEST)
				{
					state = 10;
					break;
				}
				/*-----------------------------------*/
				
				/*--------------等待电机透传模式下发来指令，这个指令是来一次处理一次----------------*/
				recved_event = 0;
				rt_event_recv(Schedule2, MOTOR_DIRECT_CMD, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, 0, &recved_event);
				if(recved_event & MOTOR_DIRECT_CMD)
				{
					state = 50;
					break;
				}
				/*----------------------------------------------------------------------------------*/
				
				rt_thread_mdelay(REFRESH_BASE);
				break;
			
			/*---------------------------测试1阶段------------------------------*/
			case 10:  // 测试阶段
				recved_event = 0;
				rt_event_recv(Schedule2, START_TEST, RT_EVENT_FLAG_OR, 0, &recved_event);
				if(!(recved_event & START_TEST))  // 如果RTU线程命令结束调试，那么就结束调试
				{
					state = 20;
					break;
				}
				
				// 下面进入正常的调试阶段
				recved_event = 0;
				rt_event_recv(Schedule, MOV_STA, RT_EVENT_FLAG_OR, 0, &recved_event); // 检查 MOV_STA，看电机是否在运动
				if(!(recved_event & MOV_STA)) // 如果不在运动则可以发送运动指令
				{
					tik++;
					if(tik >= 300)  // 每次运动之前先等 3000ms 让工作人员记录一下当前位置
					{
						tik = 0;
						if(flag == 1)
						{
							FeedPosValve(10);
						}
						else
						{
							FeedPosValve(0);
						}
					
						flag = -flag;
					}
				}
				
				// 线程休眠，将CPU资源让出来给别人
				rt_thread_mdelay(REFRESH_BASE);
				
				break;
			
			case 20:  // 结束测试
				ToMotorNoReturnNoParameter("SKD");
			
				state = 0;  // 更新状态
				break;
			/*------------------------测试1终止----------------------------*/

			/*--------------------透传模式下的指令处理-----------------------*/
			case 50:
				rt_kprintf("Test Thread Receive MOTOR_DIRECT_CMD event! \n");
			
				// 由于显示屏幕先发送的是低 8 位，再发送高 8 位的，所以显示的字符会错乱，这里要调整一下 Reg 里面的内容
				cnt = 0;
				i = MOTOR_CMD_STR;
				do{
					high_char = Reg[i] >> 8;
					low_char = (uint8_t)Reg[i];
					cmd[cnt++] = high_char;
					cmd[cnt++] = low_char;
					i++;
				}while(low_char != '\0');
				
				// 检查收到的命令长度
				str_len = rt_strlen(cmd);
				if(str_len < 2)
				{
					rt_kprintf("Weired Cmd, Will Not Sent to Motor. \n");
				}
				else
				{
					rt_kprintf("Received Cmd: %s \n", cmd);
					ret_val = ToMotorNoReturnNoParameter(cmd);
				}
				
				state = 0;
				rt_kprintf("MOTOR_DIRECT_CMD event Done ! \n");
				break;
			/*--------------------------------------------------------------*/
			
			
			default:
				break;
		}
	}
}

// 线程初始化函数
int TestThread_init()
{
	// 初始化线程
	rt_thread_t thread_test = rt_thread_create("thread_test", thread_test_entry, RT_NULL, 4096, DATA_PRIORITY, 10);
	if (thread_test != RT_NULL)
  {
     rt_thread_startup(thread_test);
  }
  else
  {
     return RT_ERROR;
  }
	
	// the end
	return RT_EOK;
}

