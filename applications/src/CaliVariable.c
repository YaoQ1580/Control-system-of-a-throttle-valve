// 调试的时候发现，在 Postprocess 中擦写 EEPROME 会导致程序错乱（因为写 EEPROME 之间需要间隔 5ms），所以写这个 thread 来专门处理标定的问题
#include "miscellaneousTools.h"
#include "GlobalVariable.h" 

// 线程回调函数
static void thread_calivar_entry(void *parameter)
{
	rt_uint32_t recved_event;
	uint8_t state1 = 0;
	char str[32];
	int ret_val = 0;
	int ee_ret;
	float current_pose;
	float received_data;
	int tmp, encoder_val, pulse_pos;
	
	while(1)
	{
		switch(state1)
		{
			case 0: // 不断检查各种各样的事件，如果发生了相应事件那么就去相应的状态里面去处理
				recved_event = 0;
				rt_event_recv(Schedule, 
CALI_PRESENT_FLUX | CALI_PRESENT_OUT_PRESS | CALI_PRESENT_IN_PRESS | SET_PRESS_PID | SET_FLUX_PID | SET_PRESS_SENSOR_MAX | SET_PRESS_SENSOR_MIN | SET_FLUX_SENSOR_MAX | SET_FLUX_SENSOR_MIN | POWER_ON | POWER_OFF | RESET_MOTOR | CALI_POS | MAN_MOVE_UP | MAN_MOVE_DOWN | MAN_MOVE_STOP | CHANGE_MAN_MOVE_SPEED | CHANGE_FEED_MODE_SPEED | CHANGE_CURRENT | CHANGE_MAN_ACC | CHANGE_FEED_ACC | CHANGE_PRESS_THRESHOLD | CHANGE_FLUX_THRESHOLD | MOVE_TO_TRG_POS | SHUT_DOWN_EVENT, 
			RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_event);
			
				if(recved_event & CALI_PRESENT_FLUX)  				// 如果收到标定当前流量事件
				{
					state1 = 10;
					break;
				}
				
				if(recved_event & CALI_PRESENT_OUT_PRESS)  		// 如果收到标定当前出口压力事件
				{
					state1 = 20;
					break;
				}
				
				if(recved_event & CALI_PRESENT_IN_PRESS)  		// 如果收到标定当前入口压力事件
				{
					state1 = 30;
					break;
				}
				
				if(recved_event & SET_PRESS_PID)  						// 如果收到标定出口压力控制PID事件
				{
					state1 = 40;
					break;
				}
				
				if(recved_event & SET_FLUX_PID)  							// 如果收到标定出口流量控制PID事件
				{
					state1 = 50;
					break;
				}
				
				if(recved_event & SET_PRESS_SENSOR_MAX)  			// 如果收到标定压力传感器上限事件
				{
					state1 = 60;
					break;
				}
				
				if(recved_event & SET_PRESS_SENSOR_MIN)  			// 如果收到标定压力传感器下限事件
				{
					state1 = 70;
					break;
				}
				
				if(recved_event & SET_FLUX_SENSOR_MAX)  			// 如果收到标定流量传感器上限事件
				{
					state1 = 80;
					break;
				}
				
				if(recved_event & SET_FLUX_SENSOR_MIN)  			// 如果收到标定流量传感器下限事件
				{
					state1 = 90;
					break;
				}
				
				if(recved_event & POWER_ON)  			// 如果收到标定流量传感器下限事件
				{
					state1 = 95;
					break;
				}
				
				if(recved_event & POWER_OFF)  			// 如果收到标定流量传感器下限事件
				{
					state1 = 100;
					break;
				}
			
				if(recved_event & RESET_MOTOR)  			// 如果收到标定流量传感器下限事件
				{
					state1 = 105;
					break;
				}
				
				if(recved_event & CALI_POS)
				{
					state1 = 110;
					break;
				}
				
				if(recved_event & MAN_MOVE_UP)
				{
					state1 = 115;
					break;
				}
				
				if(recved_event & MAN_MOVE_DOWN)
				{
					state1 = 120;
					break;
				}
				
				if(recved_event & MAN_MOVE_STOP)
				{
					state1 = 125;
					break;
				}
				
				if(recved_event & CHANGE_MAN_MOVE_SPEED)
				{
					state1 = 130;
					break;
				}
				
				if(recved_event & CHANGE_FEED_MODE_SPEED)
				{
					state1 = 135;
					break;
				}
				
				if(recved_event & CHANGE_CURRENT)
				{
					state1 = 140;
					break;
				}
				
				if(recved_event & CHANGE_MAN_ACC)
				{
					state1 = 145;
					break;
				}
				
				if(recved_event & CHANGE_FEED_ACC)
				{
					state1 = 150;
					break;
				}
				
				if(recved_event & CHANGE_PRESS_THRESHOLD)
				{
					state1 = 155;
					break;
				}
				
				if(recved_event & CHANGE_FLUX_THRESHOLD)
				{
					state1 = 160;
					break;
				}
				
				if(recved_event & MOVE_TO_TRG_POS)
				{
					state1 = 165;
					break;
				}
				
				if(recved_event & SHUT_DOWN_EVENT)
				{
					state1 = 170;
					break;
				}
				
				break;
				
				
			case 10: // 标定当前流量事件
				rt_kprintf("Receive CALI_PRESENT_FLUX event. \n");
			
				CaliPresentADC(OUTLET_FLUX_RAW, CALI_PRESENT_OUTLET_FLUX, DEF_OUTLET_FLUX_MAX, DEF_OUTLET_FLUX_MIN, FLUX_ENC_HIGH, FLUX_PHY_HIGH, FLUX_ENC_LOW, FLUX_PHY_LOW, ADC_TO_FLUX_OUTLET, &flux_enc_high, &flux_phy_high, &flux_enc_low, &flux_phy_low, &adc_to_flux_outlet);
			
				// 做完事情之后记得重置状态
				state1 = 0;
				rt_kprintf("CALI_PRESENT_FLUX event Done. \n");
				break;
			
			
			
			
			case 20: // 标定当前出口压力事件
				rt_kprintf("Receive CALI_PRESENT_OUT_PRESS event. \n");
			
				CaliPresentADC(OUTLET_PRESS_RAW, CALI_PRESENT_OUTLET_PRESS, DEF_OUTLET_PRESS_MAX, DEF_OUTLET_PRESS_MIN, PRESS_OUT_ENC_HIGH, PRESS_OUT_PHY_HIGH, PRESS_OUT_ENC_LOW, PRESS_OUT_PHY_LOW, ADC_TO_PRESS_OUTLET, &press_outlet_enc_high, &press_outlet_phy_high, &press_outlet_enc_low, &press_outlet_phy_low, &adc_to_press_outlet);
			
				// 做完事情之后记得重置状态
				state1 = 0;
				rt_kprintf("CALI_PRESENT_OUT_PRESS event Done. \n");
				break;
			
			
			
			
			case 30: // 标定当前入口压力事件
				rt_kprintf("Receive CALI_PRESENT_IN_PRESS event. \n");
			
				CaliPresentADC(INLET_PRESS_RAW, CALI_PRESENT_INLET_PRESS, DEF_OUTLET_PRESS_MAX, DEF_OUTLET_PRESS_MIN, PRESS_IN_ENC_HIGH, PRESS_IN_PHY_HIGH, PRESS_IN_ENC_LOW, PRESS_IN_PHY_LOW, ADC_TO_PRESS_INLET, &press_inlet_enc_high, &press_inlet_phy_high, &press_inlet_enc_low, &press_inlet_phy_low, &adc_to_press_inlet);
			
				// 做完事情之后记得重置状态
				state1 = 0;
				rt_kprintf("CALI_PRESENT_IN_PRESS event Done. \n");
				break;
			
			
			
			
			case 40: // 标定出口压力控制PID事件
				rt_kprintf("Receive SET_PRESS_PID event. \n");
			
				ee_ret = WriteFloatRegToEeprome(OUTLET_PRESS_P, OUTPRESS_P);
				if(ee_ret)
					rt_kprintf("Save Press Control P Successfully !! \n");
				else
					rt_kprintf("Save Press Control P Fail !! \n");
				ee_ret = WriteFloatRegToEeprome(OUTLET_PRESS_I, OUTPRESS_I);
				if(ee_ret)
					rt_kprintf("Save Press Control I Successfully !! \n");
				else
					rt_kprintf("Save Press Control I Fail !! \n");
				ee_ret = WriteFloatRegToEeprome(OUTLET_PRESS_D, OUTPRESS_D);
				if(ee_ret)
					rt_kprintf("Save Press Control D Successfully !! \n");
				else
					rt_kprintf("Save Press Control D Fail !! \n");
			
				// 做完事情之后记得重置状态
				state1 = 0;
				rt_kprintf("SET_PRESS_PID event Done. \n");
				break;
			
			
			
			
			case 50: // 标定出口流量控制PID事件
				rt_kprintf("Receive SET_FLUX_PID event. \n");
			
				ee_ret = WriteFloatRegToEeprome(OUTLET_FLUX_P, OUTFLUX_P);
				if(ee_ret)
					rt_kprintf("Save Flux Control P Successfully !! \n");
				else
					rt_kprintf("Save Flux Control P Fail !! \n");
				ee_ret = WriteFloatRegToEeprome(OUTLET_FLUX_I, OUTFLUX_I);
				if(ee_ret)
					rt_kprintf("Save Flux Control I Successfully !! \n");
				else
					rt_kprintf("Save Flux Control I Fail !! \n");
				ee_ret = WriteFloatRegToEeprome(OUTLET_FLUX_D, OUTFLUX_D);
				if(ee_ret)
					rt_kprintf("Save Flux Control D Successfully !! \n");
				else
					rt_kprintf("Save Flux Control D Fail !! \n");
			
				// 做完事情之后记得重置状态
				state1 = 0;
				rt_kprintf("SET_FLUX_PID event Done. \n");
				break;
			
			
			
			
			case 60: // 标定压力传感器上限事件
				rt_kprintf("Receive SET_PRESS_SENSOR_MAX event. \n");
			
				ret_val = CaliSensor(DEF_OUTLET_PRESS_MAX, OUTLET_PRESS_MAX_ADDR, ADC_TO_PRESS_OUTLET, &last_press_min, &last_press_max, &adc_to_press_outlet, 1);
//				if(ret_val)
//				{
//					adc_to_press_inlet = adc_to_press_outlet;
//					ee_ret = WriteFloatToEEPROME(ADC_TO_PRESS_INLET, adc_to_press_inlet);
//					if(ee_ret)
//						rt_kprintf("Save adc_to_press_inlet(int cast): %d To EEPROME Successfully\n", (int)adc_to_press_inlet);
//					else
//						rt_kprintf("Save adc_to_press_inlet To EEPROME Fail\n");
//				}
				
				// 做完事情之后记得重置状态
				state1 = 0;
				rt_kprintf("SET_PRESS_SENSOR_MAX event Done. \n");
				break;
			
			
			
			
			case 70: // 标定压力传感器下限事件
				rt_kprintf("Receive SET_PRESS_SENSOR_MIN event. \n");
			
				ret_val = CaliSensor(DEF_OUTLET_PRESS_MIN, OUTLET_PRESS_MIN_ADDR, ADC_TO_PRESS_OUTLET, &last_press_min, &last_press_max, &adc_to_press_outlet, 0);
//				if(ret_val)
//				{
//					adc_to_press_inlet = adc_to_press_outlet;
//					ee_ret = WriteFloatToEEPROME(ADC_TO_PRESS_INLET, adc_to_press_inlet);
//					if(ee_ret)
//						rt_kprintf("Save adc_to_press_inlet(int cast): %d To EEPROME Successfully\n", (int)adc_to_press_inlet);
//					else
//						rt_kprintf("Save adc_to_press_inlet To EEPROME Fail\n");
//				}
				
				// 做完事情之后记得重置状态
				state1 = 0;
				rt_kprintf("SET_PRESS_SENSOR_MIN event Done. \n");
				break;
			
			
			
			
			case 80: // 标定流量传感器上限事件
				rt_kprintf("Receive SET_FLUX_SENSOR_MAX event. \n");
			
				CaliSensor(DEF_OUTLET_FLUX_MAX, OUTLET_FLUX_MAX_ADDR, ADC_TO_FLUX_OUTLET, &last_flux_min, &last_flux_max, &adc_to_flux_outlet, 1);
				
				// 做完事情之后记得重置状态
				state1 = 0;
				rt_kprintf("SET_FLUX_SENSOR_MAX event Done. \n");
				break;
			
			
			
			
			case 90: // 标定流量传感器下限事件
				rt_kprintf("Receive SET_FLUX_SENSOR_MIN event. \n");
			
				CaliSensor(DEF_OUTLET_FLUX_MIN, OUTLET_FLUX_MIN_ADDR, ADC_TO_FLUX_OUTLET, &last_flux_min, &last_flux_max, &adc_to_flux_outlet, 0);
				
				// 做完事情之后记得重置状态
				state1 = 0;
				rt_kprintf("SET_FLUX_SENSOR_MIN event Done. \n");
				break;
			
			
			
			
			case 95: // 上电
				rt_kprintf("Receive POWER_ON event. \n");
				// 电机之前可能会发送一些奇怪的数据给 Motor Thread，Motor Thread会误认为这是开机信息，所以这里先清空一下 GET_MOTOR_VERSION 事件为后面接收这个事件做准备
				rt_event_recv(Schedule, GET_MOTOR_VERSION, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, 0, &recved_event);
				rt_pin_write(MOTOR_CTR, PIN_HIGH);     // 操作继电器从而给电机供电		
				WriteShortToReg(POWER_UP_PROGRESS, 10); // 更新上电进度
			
				// 下面等待 motor 线程读取电机版本号，这里会一直等待
				recved_event = 0;
				rt_event_recv(Schedule, GET_MOTOR_VERSION, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_event);
				rt_kprintf("Receive GET_MOTOR_VERSION event. \n");
				WriteShortToReg(POWER_UP_PROGRESS, 30);  // 更新上电进度
			
				// 获取电机版本号之后需要等待电机固件启动
				rt_kprintf("Motor drive initialization ... \n");
				rt_thread_mdelay(MOTOR_INITIAL);
				WriteShortToReg(POWER_UP_PROGRESS, 50);
			
				// 尝试和电机通讯来检查电机是否启动成功，并且更新通讯状态寄存器
				rt_kprintf("Check if Motor Working... \n");
				ret_val = ToMotorWithReturn("EP", str);
				if(ret_val)
				{
					rt_kprintf("Motor Communication OK, upgrate POWER_UP_PROGRESS ! \n");
				}
				else
				{
					rt_kprintf("Motor Communication Error, Please reboot the System ! \n");
				}
				WriteShortToReg(POWER_UP_PROGRESS, 60);
				
				// 电机固件启动之后就可以进行初始化工作。
				// 包括：载入上一次关机时的 EP 指令，载入 EEPROME 数据到寄存器，载入 EEPROME 数据到全局变量
				DataInitialize();
				rt_kprintf("Data Initialized!!! \n");
				WriteShortToReg(POWER_UP_PROGRESS, 80);
			
				// 数据载入完毕之后，发送 THREAD_GET_WORK 标志线程全部开始工作
				rt_event_send(Schedule, THREAD_GET_WORK);
				rt_kprintf("Send THREAD_GET_WORK event \n");
			
				// 此时初始化工作全部完成，系统开始运作，上电进度更新为 100%
				WriteShortToReg(POWER_UP_PROGRESS, 100);
				
				// 做完事情之后记得重置状态
				state1 = 0;
				rt_kprintf("POWER_ON event Done. \n");
				break;
			
			
			
			
			case 100: // 断电
				rt_kprintf("Receive POWER_OFF event. \n");
			
				// 清空 THREAD_GET_WORK 事件，所有线程退出工作
				rt_event_recv(Schedule, THREAD_GET_WORK, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, 0, &recved_event);
				rt_kprintf("Clear THREAD_GET_WORK event. \n");
			
				// 等待其余线程退出
				rt_thread_mdelay(20);
			
				// 发送“SK”给电机，立即停止电机的一切操作
				ret_val = ToMotorNoReturnNoParameter("SK");
				if(!ret_val)  // 发送命令失败
				{
					rt_kprintf("Send SK FAIL!!! \n");
				}
				else
				{
					rt_kprintf("Send SK Succeed!!! \n");
				}
				
				// 等待“SK”命令执行完成
				rt_thread_mdelay(200); 
				
				// 将当前电机编码值保存入EEPROME
				SaveEncoderData();
			
				// 驱动继电器来让电机断电
				rt_pin_write(MOTOR_CTR, PIN_LOW);
			
				// 更新上电进度寄存器
				WriteShortToReg(POWER_UP_PROGRESS, 0);
			
				// 做完事情之后记得重置状态
				state1 = 0;
				rt_kprintf("POWER_OFF event Done. \n");
				break;
			
				
				
				
			case 105:  // 重新使能电机
				rt_kprintf("Receive RESET_MOTOR event. \n");
			
				// 向电机发送“AR”外加“ME”指令
				ret_val = ToMotorNoReturnNoParameter("AR");
				if(!ret_val)
				{
					rt_kprintf("Send AR FAIL!!! \n");
				}
				ret_val = ToMotorNoReturnNoParameter("ME");
				if(!ret_val)
				{
					rt_kprintf("Send ME FAIL!!! \n");
				}
				
				// 另外还要重置通讯寄存器为正常
				WriteShortToReg(COMMUNICATION_STA, 1);
				
				// 由于堵转可能会引起丢步，丢步导致 IP 和 EP 不再是 IP_TO_EP 的倍数关系，所以会产生一些精度误差
				// 所以这里尝试读取 EP ，然后将 EP 值再次写入电机来重置 IP
				rt_thread_mdelay(700);	// 重新使能可能会发生瞬间抖动导致数据误差，所以这里先等待一些时间让电机稳定
				ret_val = ToMotorWithReturn("EP", str);
				if(ret_val)  // 读取 EP 成功
				{
					// 通过 SP 指令来设置当前的电机脉冲数
					ToMotorNoReturnWithParameter("EP", str);
					sscanf((const char*)str, "%d", &encoder_val);
					pulse_pos = encoder_val * IP_TO_EP;
					sprintf(str, "%d", pulse_pos);
					ret_val = ToMotorNoReturnWithParameter("SP", str);
					if(ret_val)
					{
						rt_kprintf("Resetting Motor: Reset SP Successfully !! \n");
					}
					else
					{
						rt_kprintf("Resetting Motor: Read EP Successfully But Reset SP Fail !! \n");
					}
				}
				else
				{
					rt_kprintf("Resetting Motor: Reset SP Fail !! \n");
				}
				
				// 做完事情之后记得重置状态
				state1 = 0;
				rt_kprintf("RESET_MOTOR event Done. \n");
				break;
				
				
				
				
			case 110:  //标定当前位置
				rt_kprintf("Receive CALI_POS event. \n");
				current_pose = GetFloatFromReg(DEF_PRESENT_POS);  // 从寄存器中获取上位机发来的当前位置数值
					 
				if(current_pose == 0) // 如果是要标定0点，则采用默认的计算过的转换系数
				{
					// 将电机编码值清0
					ret_val = ToMotorNoReturnWithParameter("EP", "0");
					if(ret_val)	   // 与电机通讯成功
					{
						if(ret_val != 2)  // 电机不忙
						{
							rt_kprintf("Motor Is Not Busy, Calibrating Zero Point. \n");
							// 将零点写入EEPROME
							WriteInt32ToEEPROME(ENCODER_ADDR, 0);
						}
						else							// 电机忙，电机忙完之后由 SavPose 线程帮忙存入编码器数值到 EEprome
						{
							rt_kprintf("Cmd Is Sent But Motor Is Busy, SavPose Will Help us to save Encoder Value. \n");
						}
						
						// 根据传动比计算转换系数（注意符号，如果之前传动比为负数，那么这里也要保证为负数，仅仅是改变数值大小）
						float result = EVC_RESOLUTION * CHUAN_DONG_BI / DAO_CHENG;
						coeff_env_to_valve = (coeff_env_to_valve > 0)? result : -result;
						rt_kprintf("Computing default coeff_env_to_valve(int cast): %d \n", (int32_t)coeff_env_to_valve );
					
						// 将转换系数存入 EEPROME
						ee_ret = WriteFloatToEEPROME(ENC_TO_POS, coeff_env_to_valve);
						if(ee_ret)
							rt_kprintf("Save coeff_env_to_valve To EEPROME Successfully!! \n");
						else
							rt_kprintf("Save coeff_env_to_valve Fail!! \n");
					}
					else		// 与电机通讯不成功
					{
						rt_kprintf("Calibrating Zero Point FAIL!!! \n");
					}
				}
				else 							 // 如果不是标定0点
				{
					// 首先获取电机当前编码值
					int32_t encoder_value;
					ret_val = GetEnvFromMotor(&encoder_value);
					if(ret_val)		// 通讯成功
					{
						rt_kprintf("Current EP value: %d  \n", encoder_value);
						float encoder_f = (float)encoder_value;  // 先将编码数值转为 float 为下面做除法做准备
						
						// 求出新的转换系数
						float tmp = encoder_f / current_pose;
						// 注意：可能出现用户一开始标了0，然后输了另外一个非零的数，然后不小心按了确定，那么此时的系数就变成了0，这里要避免这个情况
						if(-100 < tmp && tmp < 100) // 标定点和 0 点过于接近的话就不标定了
						{
							rt_kprintf("Weird coeff_env_to_valve: %d   System will not accept that. \n", (int32_t)tmp);
						}
						else
						{
							// 更新转换系数
							coeff_env_to_valve = tmp;
							rt_kprintf("coeff_env_to_valve(int cast): %d   will be saved. \n", (int32_t)tmp);
							
							// 将更新后的转换系数写入EEPROME
							ee_ret = WriteFloatToEEPROME(ENC_TO_POS, coeff_env_to_valve);
							if(ee_ret)
								rt_kprintf("Save coeff_env_to_valve To EEPROME Successfully!! \n");
							else
								rt_kprintf("Save coeff_env_to_valve Fail!! \n");
						}
					}
					else					// 通讯失败
					{
						rt_kprintf("Calibrating Position FAIL!!! \n");
					}
				}
				
				// 做完事情之后记得重置状态
				state1 = 0;
				rt_kprintf("CALI_POS event Done. \n");
				break;
				
				
				
				
			case 115:  //手动上移
				rt_kprintf("Receive MAN_MOVE_UP event. \n");
						
				// 先发送 1SK 让之前的运动指令取消
				ret_val = ToMotorNoReturnNoParameter("SK");
				if(!ret_val)
					rt_kprintf("Send SK FAIL!! \n");
				// 再电机发送“1DI”表明运动方向
				ret_val = ToMotorNoReturnWithParameter("DI", "100");
				if(!ret_val)
					rt_kprintf("Send DI 100 FAIL!! \n");
				// 再向电机发送“1CJ”让电机不断运动
				ret_val = ToMotorNoReturnNoParameter("CJ");
				if(!ret_val)
					rt_kprintf("Send CJ FAIL!! \n");
			
				// 做完事情之后记得重置状态
				state1 = 0;
				rt_kprintf("MAN_MOVE_UP event Done. \n");
				break;
			
			
			
			case 120:  //手动下移
				rt_kprintf("Receive MAN_MOVE_DOWN event. \n");
				
				// 先发送 1SK 让之前的运动指令取消
				ret_val = ToMotorNoReturnNoParameter("SK");
				if(!ret_val)
					rt_kprintf("Send SK FAIL!! \n");
				// 再电机发送“1DI”表明运动方向
				ret_val = ToMotorNoReturnWithParameter("DI", "-100");
				if(!ret_val)
					rt_kprintf("Send DI -100 FAIL!! \n");
				// 再向电机发送“1CJ”让电机不断运动
				ret_val = ToMotorNoReturnNoParameter("CJ");
				if(!ret_val)
					rt_kprintf("Send CJ FAIL!! \n");
			
				// 做完事情之后记得重置状态
				state1 = 0;
				rt_kprintf("MAN_MOVE_DOWN event Done. \n");
				break;
				
			
			
			case 125:  //手动停止
				rt_kprintf("Receive MAN_MOVE_STOP event. \n");
				
				// 向电机发送“1SJ”来让电机停止运动
				ret_val = ToMotorNoReturnNoParameter("SJ");
				if(!ret_val)
					rt_kprintf("Send SJ FAIL!! \n");
			
				// 做完事情之后记得重置状态
				state1 = 0;
				rt_kprintf("MAN_MOVE_STOP event Done. \n");
				break;
			
				
				
			
			case 130:   // 设定手动调节阀时的运动速度
				rt_kprintf("Receive CHANGE_MAN_MOVE_SPEED event. \n");
			
				// 先将寄存器的数据读过来，然后转化为字符串形式
				received_data = GetFloatFromReg(MANUAL_SPEED);
				sprintf(str, "%f", received_data);
				tmp = rt_strlen(str);  // 截取数据长度，防止因为 float 的精度误差导致出现 .99999999999 这样的情况
				if(tmp > 6)
				{
					tmp = 6;
					str[tmp] = '\0';
				}
				
				rt_kprintf("The Manual Move Speed is: %s rps. \n", str);
	
				ret_val = ToMotorNoReturnWithParameter("JS", str);
				if(ret_val)
					rt_kprintf("Change Manual Move Speed Success !! \n");
			  else
					rt_kprintf("Change Manual Move Speed Fail !! \n");
				
				ret_val = ToMotorNoReturnNoParameter("SA");
				if(ret_val)
					rt_kprintf("Save Change Success !! \n");
			  else
					rt_kprintf("Save Change Fail !! \n");
				
				// 做完事情之后记得重置状态
				state1 = 0;
				rt_kprintf("CHANGE_MAN_MOVE_SPEED event Done. \n");
				break;
			
				
			case 135:		// 设定FEED模式下调节阀时的运动速度
				rt_kprintf("Receive CHANGE_FEED_MODE_SPEED event. \n");
			
				// 先将寄存器的数据读过来，然后转化为字符串形式
				received_data = GetFloatFromReg(PID_SPEED);
				sprintf(str, "%f", received_data);
				tmp = rt_strlen(str);  // 截取数据长度，防止因为 float 的精度误差导致出现 .99999999999 这样的情况
				if(tmp > 6)
				{
					tmp = 6;
					str[tmp] = '\0';
				}
				
				rt_kprintf("The Feed Mode Speed is: %s rps. \n", str);
	
				ret_val = ToMotorNoReturnWithParameter("VE", str);
				if(ret_val)
					rt_kprintf("Change Feed Mode Speed Success !! \n");
			  else
					rt_kprintf("Change Feed Mode Speed Fail !! \n");
				
				ret_val = ToMotorNoReturnNoParameter("SA");
				if(ret_val)
					rt_kprintf("Save Change Success !! \n");
			  else
					rt_kprintf("Save Change Fail !! \n");
				
				// 做完事情之后记得重置状态
				state1 = 0;
				rt_kprintf("CHANGE_FEED_MODE_SPEED event Done. \n");
				break;
				
				
			
			case 140:		// 设定电流
				rt_kprintf("Receive CHANGE_CURRENT event. \n");
			
				// 先将寄存器的数据读过来，然后转化为字符串形式
				received_data = GetFloatFromReg(CURRENT);
				sprintf(str, "%f", received_data);
				tmp = rt_strlen(str);  // 截取数据长度，防止因为 float 的精度误差导致出现 .99999999999 这样的情况
				if(tmp > 6)
				{
					tmp = 6;
					str[tmp] = '\0';
				}
				
				rt_kprintf("The CC is: %s A. \n", str);
	
				ret_val = ToMotorNoReturnWithParameter("CC", str);
				if(ret_val)
					rt_kprintf("Change CC Success !! \n");
			  else
					rt_kprintf("Change CC Fail !! \n");
				
				ret_val = ToMotorNoReturnWithParameter("CI", "1.5");
				if(ret_val)
					rt_kprintf("Change CI Success !! \n");
			  else
					rt_kprintf("Change CI Fail !! \n");
				
				ret_val = ToMotorNoReturnNoParameter("SA");
				if(ret_val)
					rt_kprintf("Save Change Success !! \n");
			  else
					rt_kprintf("Save Change Fail !! \n");
				
				// 做完事情之后记得重置状态
				state1 = 0;
				rt_kprintf("CHANGE_CURRENT event Done. \n");
				break;
			

				
				
			case 145:  // 设置手动模式下的加速度
				rt_kprintf("Receive CHANGE_MAN_ACC event. \n");
			
				// 先将寄存器的数据读过来，然后转化为字符串形式
				received_data = GetFloatFromReg(MANUAL_ACC);
				sprintf(str, "%f", received_data);
				tmp = rt_strlen(str);  // 截取数据长度，防止因为 float 的精度误差导致出现 .99999999999 这样的情况
				if(tmp > 6)
				{
					tmp = 6;
					str[tmp] = '\0';
				}
				
				rt_kprintf("The JA is: %s rpss. \n", str);
	
				ret_val = ToMotorNoReturnWithParameter("JA", str);
				if(ret_val)
					rt_kprintf("Change JA Success !! \n");
			  else
					rt_kprintf("Change JA Fail !! \n");
				
				ret_val = ToMotorNoReturnNoParameter("SA");
				if(ret_val)
					rt_kprintf("Save Change Success !! \n");
			  else
					rt_kprintf("Save Change Fail !! \n");
				
				// 做完事情之后记得重置状态
				state1 = 0;
				rt_kprintf("CHANGE_MAN_ACC event Done. \n");
				break;
			
			
				
			
			case 150:  // 设置 FEED 模式下的加速度
				rt_kprintf("Receive CHANGE_FEED_ACC event. \n");
			
				// 先将寄存器的数据读过来，然后转化为字符串形式
				received_data = GetFloatFromReg(PID_ACC);
				sprintf(str, "%f", received_data);
				tmp = rt_strlen(str);  // 截取数据长度，防止因为 float 的精度误差导致出现 .99999999999 这样的情况
				if(tmp > 6)
				{
					tmp = 6;
					str[tmp] = '\0';
				}
				
				rt_kprintf("The AC is: %s rpss. \n", str);
	
				ret_val = ToMotorNoReturnWithParameter("AC", str);
				if(ret_val)
					rt_kprintf("Change AC Success !! \n");
			  else
					rt_kprintf("Change AC Fail !! \n");
				
				ret_val = ToMotorNoReturnWithParameter("DE", str);
				if(ret_val)
					rt_kprintf("Change DE Success !! \n");
			  else
					rt_kprintf("Change DE Fail !! \n");
				
				ret_val = ToMotorNoReturnNoParameter("SA");
				if(ret_val)
					rt_kprintf("Save Change Success !! \n");
			  else
					rt_kprintf("Save Change Fail !! \n");
				
				// 做完事情之后记得重置状态
				state1 = 0;
				rt_kprintf("CHANGE_FEED_ACC event Done. \n");
				break;

				
			
			case 155:   // 设定压力控制阈值
				rt_kprintf("Receive CHANGE_PRESS_THRESHOLD event. \n");
			
				SetThreshold(OUT_PRESS_CTR_THRESHOLD, EE_PRESS_CTR_THRESHOLD, &press_threshold);
		
				// 做完事情之后记得重置状态
				state1 = 0;
				rt_kprintf("CHANGE_PRESS_THRESHOLD event Done. \n");
				break;
				
			
				
				
			case 160:   // 设定流量控制阈值
				rt_kprintf("Receive CHANGE_FLUX_THRESHOLD event. \n");
			
				SetThreshold(OUT_FLUX_CTR_THRESHOLD, EE_FLUX_CTR_THRESHOLD, &flux_threshold);
		
				// 做完事情之后记得重置状态
				state1 = 0;
				rt_kprintf("CHANGE_FLUX_THRESHOLD event Done. \n");
				break;

			
			
			
			case 165:   // 运动到特定位置
				rt_kprintf("Receive MOVE_TO_TRG_POS event. \n");
			
				received_data = GetFloatFromReg(TRG_VALVE_POS);
				if(received_data <= 10 && received_data >= 0)
				{
					FeedPosValve(received_data);
				}
				else
				{
					rt_kprintf("Invalid Input, The Input Pos Should be in range [0, 10]mm !!  \n");
				}
		
				// 做完事情之后记得重置状态
				state1 = 0;
				rt_kprintf("MOVE_TO_TRG_POS event Done. \n");
				break;
			
				
				
				
			case 170:   // 急停指令
				rt_kprintf("Receive SHUT_DOWN_EVENT event. \n");
			
				ret_val = ToMotorNoReturnNoParameter("SK");
				if(ret_val)
				{
					rt_kprintf("Shut Down Motor Successfully !! \n");
				}
				else
				{
					rt_kprintf("Shut Down Motor Fail !! \n");
				}
			
				// 做完事情之后记得重置状态
				state1 = 0;
				rt_kprintf("SHUT_DOWN_EVENT event Done. \n");
				break;
				
			default:
				state1 = 0;
				break;
		}
	}
}

// 标定参数线程初始化
int calivar_init()
{
	// 开启线程
	rt_thread_t thread_calivar = rt_thread_create("thread_calivar", thread_calivar_entry, RT_NULL, 4096, CALIVAR_PRIORITY, 10);
	if (thread_calivar != RT_NULL)
  {
		 rt_kprintf("Calivar thread ready to work!\n");
     rt_thread_startup(thread_calivar);
  }
  else
  {
		 rt_kprintf("Calivar thread fail!\n");
     return RT_ERROR;
  }
	
	// the end
	return RT_EOK;
}


