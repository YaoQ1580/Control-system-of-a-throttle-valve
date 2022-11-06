#include "miscellaneousTools.h"
#include "GlobalVariable.h" 

/*------静态全局变量-----*/
static int type = 0; 							// 控制模式：0：不控制。1：出口压力控制。2：出口流量控制
static float sum_press_diff = 0;  // 压力传感器目标值和当前值的偏差累积
static float sum_flux_diff = 0;   // 流量传感器目标值和当前值的偏差累积
static float Trg;

// PID控制函数
void PIDctr(uint32_t P_reg_addr, uint32_t I_reg_addr, uint32_t D_reg_addr, uint32_t Trg_reg_addr, uint32_t Cur_reg_addr, float* ctr_threshold)
{
	uint32_t recved_event;
	
	// PID相关参数
	float* sum_type_addr; 
	float diff = 0; 
	float tmp = 0; 
	float compare = 0;
	
	if(type == 1)
		sum_type_addr = &sum_press_diff;
	if(type == 2)
		sum_type_addr = &sum_flux_diff;
	
	recved_event = 0;
	rt_event_recv(Schedule, MOV_STA, RT_EVENT_FLAG_OR, 0, &recved_event); // 检查 MOV_STA，看电机是否在运动
	if(!(recved_event & MOV_STA)) 	// 如果不在运动则可以发送“1FL”的运动指令
	{
		//diff = GetFloatFromReg(Trg_reg_addr) - GetFloatFromReg(Cur_reg_addr); // 计算目标值和当前值的差值
		diff = Trg - GetFloatFromReg(Cur_reg_addr);
		
		compare = diff >= 0 ? diff : -diff;
		if(compare >= *ctr_threshold) // 如果大于阈值则进行PID控制
		{
			// 将 diff 值进行累加，为后续的 PD 或 PI 控制提供方便
			*sum_type_addr = *sum_type_addr + diff;
							
			// 进行P控制
			tmp = diff * GetFloatFromReg(P_reg_addr);  // + (*sum_type_addr) * GetFloatFromReg(OUTLET_PRESS_I);
			tmp = tmp * 20000; // 乘以电机的“EG”值以防需要设置的P、I、D数值过大
							
			// 将结果转化成电机命令
			FeedLenEnc((int32_t)tmp);
		}
	}
}



// 线程函数
static void thread_PressAdj_entry(void *parameter)
{
	rt_uint32_t recved_event;
	uint16_t state1 = 0;
	uint16_t stall_detec;
	
	// 电机通讯相关
	int ret_val;
	
	// PID刷新时间片
	uint16_t pid_tick = 0;
	
	while(1)
	{
		switch(state1) // 状态机
		{
			case 0:    // 一直等待 THREAD_GET_WORK 事件
				rt_event_recv(Schedule, THREAD_GET_WORK, RT_EVENT_FLAG_OR, RT_WAITING_FOREVER, &recved_event);
				state1 = 10;
				rt_kprintf("PID Adj Thread Ready to work. \n");
				break;
			
			
			case 10:	 // 正常工作状态 
				/*--------------------读取一些会打断正常工作的突发事件------------------*/
				recved_event = 0;
				rt_event_recv(Schedule, THREAD_GET_WORK, RT_EVENT_FLAG_OR, 0, &recved_event);
				if(!(recved_event & THREAD_GET_WORK))   // 关机事件
				{
					state1 = 99;
					break;
				}
				
				recved_event = 0;
				rt_event_recv(Schedule, QUIT_PID, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, &recved_event);
				if(recved_event & QUIT_PID)							// QUIT_PID 事件
				{
					state1 = 40;
					break;
				}
				/*----------------------------------------------------------------------*/
			
			
				/*-----------------------------------正常工作流程---------------------------------*/
				/*------更新控制模式：依据是：堵转、各种事件--------*/
				// 检测 SET_PRESS_OUT 和 SET_FLUX 事件
				recved_event = 0;
				rt_event_recv(Schedule, SET_PRESS_OUT | SET_FLUX, 
				RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, &recved_event);
				if(recved_event & SET_PRESS_OUT)	// 如果收到压力控制事件，那么就设置为1
				{
					rt_kprintf("PID Adj Thread Receive SET_PRESS_OUT event. \n");
					Trg = GetFloatFromReg(TRG_OUTLET_PRESS);
					rt_kprintf("Trg Press(int cast): %d \n", (int)Trg);
					type = 1;
				}
				if(recved_event & SET_FLUX)				// 如果收到流量控制事件，那么就设置为2
				{
					rt_kprintf("PID Adj Thread Receive SET_FLUX_OUT event. \n");
					Trg = GetFloatFromReg(TRG_OUTLET_FLUX);
					rt_kprintf("Trg Flux(int cast): %d \n", (int)Trg);
					type = 2;
				}
				
				// 但是如果检测到堵转的话就清空 type
				// 这里没有将 stall 列为突发事件，因为本线程没有更改堵转寄存器的资格，如果列为突发事件，那么会陷入：检测到->处理->检测到->处理...，导致死机
				stall_detec = GetShortFromReg(STALL_DETECTION);
				if(stall_detec) // 如果检测到堵转
				{
					sum_press_diff = 0;
					sum_flux_diff = 0;
					type = 0;
				}
				/*--------------------------------------------------*/
				
				switch(type)
				{
					case 0:		// 默认状态，什么都不做
						pid_tick = 0;
						break;
					
					case 1:   // 压力控制
						pid_tick++;
						if(pid_tick >= PID_UPDATE_CNT)
						{
							pid_tick = 0;
							PIDctr(OUTLET_PRESS_P, OUTLET_PRESS_I, OUTLET_PRESS_D, TRG_OUTLET_PRESS, PRESENT_OUTLET_PRESS, &press_threshold);
						}
						break;
					
					case 2:   // 流量控制
						pid_tick++;
						if(pid_tick >= PID_UPDATE_CNT)
						{
							pid_tick = 0;
							PIDctr(OUTLET_FLUX_P, OUTLET_FLUX_I, OUTLET_FLUX_D, TRG_OUTLET_FLUX, PRESENT_OUTLET_FLUX, &flux_threshold);
						}
						break;
				}
				/*--------------------------------------------------------------------------------*/
				
				rt_thread_mdelay(REFRESH_BASE); // 刷新延迟
				break;
			
			/*-----------------------处理突发事件的相应入口，类似于中断向量表------------------------------*/
			case 40:  // QUIT_PID 事件
				sum_press_diff = 0;
				sum_flux_diff = 0;
				type = 0;			// 清空 PID 控制模式
				// 给电机发送 SK 命令，否则可能会出现退出 PID 控制，然而上一次的 FL 命令仍然在执行
				ret_val = ToMotorNoReturnNoParameter("SK");
				if(ret_val)
				{
					rt_kprintf("Quit PID control, Shut Down the Motor Successfully!! \n");
				}
				else
				{
					rt_kprintf("Quit PID control, But Fail to Shut Down the Motor !! \n");
				}
				state1 = 10;  // 回到正常工作状态，等待进一步的指令
				rt_kprintf("PID Adj Thread Quit PID Control !! \n");
				break;
			
				
				
			case 99:  // 关机事件
				sum_press_diff = 0;
				sum_flux_diff = 0;
				type = 0;			// 清空 PID 控制模式
				state1 = 0;		// 回到等待开机状态
				rt_kprintf("PID Adj Thread Shut Down !! \n");
				break;
			/*---------------------------------------------------------------------------------------------*/	
					
			
			default:
				break;
		}
	}
}

// 线程初始化函数
int PressAdjThread_init()
{
	// 初始化线程
	rt_thread_t thread_PressAdj = rt_thread_create("thread_PressAdj", thread_PressAdj_entry, RT_NULL, 4096, PRESS_ADJ_PRIORITY, 10);
	if (thread_PressAdj != RT_NULL)
  {
     rt_thread_startup(thread_PressAdj);
  }
  else
  {
		 rt_kprintf("PressAdj Thread Create Fail !! \n");
     return RT_ERROR;
  }
	
	// the end
	return RT_EOK;
}

