#include "miscellaneousTools.h"
#include "GlobalVariable.h" 

/*------静态全局变量------*/
static uint8_t index = 0;   						 // ADC array index
static rt_uint32_t value1[MOV_AVG_NUM];
static rt_uint32_t value2[MOV_AVG_NUM];
static rt_uint32_t value3[MOV_AVG_NUM];
static uint32_t sum1 = 0;
static uint32_t sum2 = 0;
static uint32_t sum3 = 0;
static uint8_t state_ADC = 0;




// 清空与ADC读取有关的变量
void ADC_clear()
{
	index = 0;
	sum1 = 0; sum2 = 0; sum3 = 0;
	state_ADC = 0;	// 重置 ADC 状态机
}




// 刷新ADC度数，用到了滑动平均滤波，读8个数，然后进行滤波
void ADC_Refresh()
{
	uint8_t i;
	uint32_t avg1, avg2, avg3;
	float avg1_f, avg2_f, avg3_f;
	
	switch(state_ADC) // 状态机
	{
		case 0 :  // 首先读满 MOV_AVG_NUM 个ADC数值，为之后的滑动平均做准备
			value1[index] = rt_adc_read(adc_dev, PRESS_IN_CHANNEL);  // 读入口压力
			sum1 += value1[index];
			value2[index] = rt_adc_read(adc_dev, PRESS_OUT_CHANNEL); // 读出口压力
			sum2 += value2[index];
			value3[index] = rt_adc_read(adc_dev, FLUX_CHANNEL);      // 读出口流量
			sum3 += value3[index];
		
			index ++;
			if(index >= MOV_AVG_NUM)
			{
				index = 0;
				state_ADC = 10; 	// 如果读满8个的话，更新状态机
			}
			break;
		
			
			
		case 10:  // 进行滑动平均滤波
			// 将第一个数据丢弃掉
			sum1 -= value1[0];
			sum2 -= value2[0];
			sum3 -= value3[0];
		
			// 移动数据，为最新读取的数据挪出位置
			for(i = 0; i < MOV_AVG_NUM - 1; i ++)
			{
				value1[i] = value1[i + 1];
				value2[i] = value2[i + 1];
				value3[i] = value3[i + 1];
			}
			value1[MOV_AVG_NUM - 1] = rt_adc_read(adc_dev, PRESS_IN_CHANNEL);  // 读取最新的入口压力数据
			sum1 += value1[MOV_AVG_NUM - 1];
			value2[MOV_AVG_NUM - 1] = rt_adc_read(adc_dev, PRESS_OUT_CHANNEL); // 读取最新的出口压力数据
			sum2 += value2[MOV_AVG_NUM - 1];
			value3[MOV_AVG_NUM - 1] = rt_adc_read(adc_dev, FLUX_CHANNEL);      // 读取最新的出口流量数据
			sum3 += value3[MOV_AVG_NUM - 1];
			
			// 进行滑动平均滤波
			avg1 = sum1 / MOV_AVG_NUM; 
			avg2 = sum2 / MOV_AVG_NUM;  
			avg3 = sum3 / MOV_AVG_NUM; 

			// 将 avg 转成 float 类型为之后的计算提供方便
			avg1_f = (float) avg1;
			avg2_f = (float) avg2;
			avg3_f = (float) avg3;
			
			// 将数据写入寄存器
			WriteFloatToReg(INLET_PRESS_RAW, avg1_f);   // 将未转化的入口压力存入寄存器，以防需要重新标定当前入口压力
			WriteFloatToReg(OUTLET_PRESS_RAW, avg2_f);  // 将未转化的出口压力存入寄存器，以防需要重新标定当前出口压力
			WriteFloatToReg(OUTLET_FLUX_RAW, avg3_f);   // 将未转化的出口流量存入寄存器，以防需要重新标定当前出口流量 			
			WriteFloatToReg(PRESENT_INLET_PRESS, press_inlet_phy_low + (avg1_f - press_inlet_enc_low) / adc_to_press_inlet);
			WriteFloatToReg(PRESENT_OUTLET_PRESS, press_outlet_phy_low + (avg2_f - press_outlet_enc_low) / adc_to_press_outlet); // 将ADC数值乘以转换系数后写入寄存器以供上位机读取																		
			WriteFloatToReg(PRESENT_OUTLET_FLUX, flux_phy_low + (avg3_f - flux_enc_low) / adc_to_flux_outlet);   // 将ADC数值乘以转换系数后写入寄存器以供上位机读取
			break;
	}
}



// 读取当前位置
void IE_Refresh()
{
	int32_t encoder_value = 0;  // 编码器的Int型数据
	float encoder_f, pose_data;
	char encoder_value_str[32]; // 编码器的字符形式
	
	int ret_val;								// 电机通讯是否成功
	
	// 向电机发送“1IE”指令来读取电机编码值
	ret_val = ToMotorWithReturn("IE", encoder_value_str);
	if(ret_val)		// 电机应答成功
	{
		// 将字符格式的编码器数值转化成int形式
		sscanf(encoder_value_str, "%d", &encoder_value);
		
		// 因为后面要做除法，所以这里先转为 float 型
		encoder_f = (float)encoder_value;
		
		// 除以转换系数从而得到真正的位移
		pose_data = encoder_f / coeff_env_to_valve;
		
		// 将阀芯开度写入寄存器
		WriteFloatToReg(PRESENT_VALVE_POS, pose_data);
	}
	else					// 电机应答失败
	{
		rt_kprintf("Refresh IE Fail !! \n");
	}
}



// 线程函数
static void thread_data_entry(void *parameter)
{
	rt_uint32_t recved_event;
	uint16_t state1 = 0;
	
	uint8_t ie_tick = 0;  // 刷新当前位置时间片
	uint8_t adc_tick = 0; // 刷新ADC读数时间片
	
	while(1)
	{
		switch(state1) // 状态机
		{
			case 0 :   // 一直等待 THREAD_GET_WORK 事件
				rt_event_recv(Schedule, THREAD_GET_WORK, RT_EVENT_FLAG_OR, RT_WAITING_FOREVER, &recved_event);
				state1 = 10;
				rt_kprintf("Data Thread Ready to work. \n");
				break;
			
			
			
			case 10:   // 正常工作状态 
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
				ie_tick++; 
				adc_tick++; 
				
				if(ie_tick >= IE_TICK)
				{	
					ie_tick = 0;
					IE_Refresh();  // 当显示当前位置时间片到了后刷新显示一次当前位置
				}
				
				if(adc_tick >= ADC_TICK)
				{	
					adc_tick = 0;
					ADC_Refresh(); // 当显示ADC数值时间片到了后刷新显示一次ADC
				}
				
				/*----------------------------------------------------------------------*/
				
				rt_thread_mdelay(REFRESH_BASE); // 刷新延迟
				break;
			
				
				
			/*-----------------------处理突发事件的相应入口，类似于中断向量表------------------------------*/
				
			case 99: 		// 关机，做一些善后工作
				// 清空所有寄存器（由于寄存器数量比较多，所以这一步就先省略了，防止这个线程在这边耗时太久）
//				rt_enter_critical();
//				for(i = 0; i < REG_NUM; i++)
//				{
//					Reg[i] = 0;
//				}
//				rt_exit_critical();
				
			  // 重置时间片
				ie_tick = 0;
				adc_tick = 0;
			
				// 因为 ADC 读取中用到了状态机，所以需要重置 ADC 相关变量
				ADC_clear();
			
				// 重置状态机
				state1 = 0;
				rt_kprintf("Data Thread Shut Down. \n");
				break;
			
			/*---------------------------------------------------------------------------------------------*/
				
			default:
				break;
		}
	}
}



// ADC硬件初始化函数
int init_ADC()
{
	rt_err_t ret = RT_EOK;
	
	// 获取ADC硬件句柄
	adc_dev = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
	if (adc_dev == RT_NULL)
  {
     rt_kprintf("can't find %s device!\n", ADC_DEV_NAME);
  }
	
	ret = rt_adc_enable(adc_dev, PRESS_IN_CHANNEL);
	if(ret != RT_EOK)
	{
		rt_kprintf("can't enable PRESS_IN_CHANNEL \n");
	}
	
	ret = rt_adc_enable(adc_dev, PRESS_OUT_CHANNEL);
	if(ret != RT_EOK)
	{
		rt_kprintf("can't enable PRESS_OUT_CHANNEL \n");
	}
	
	ret = rt_adc_enable(adc_dev, FLUX_CHANNEL);
	if(ret != RT_EOK)
	{
		rt_kprintf("can't enable FLUX_CHANNEL \n");
	}
	
	return 1;
}

// 线程初始化函数
int dataThread_init()
{
	// 初始化ADC
	init_ADC();
	
	// 初始化线程
	rt_thread_t thread_data = rt_thread_create("thread_data", thread_data_entry, RT_NULL, 4096, DATA_PRIORITY, 10);
	if (thread_data != RT_NULL)
  {
     rt_thread_startup(thread_data);
  }
  else
  {
			rt_kprintf("Data thread Create Fail!\n");
     return RT_ERROR;
  }
	
	// the end
	return RT_EOK;
}

