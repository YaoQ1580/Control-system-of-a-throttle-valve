// ���Ե�ʱ���֣��� Postprocess �в�д EEPROME �ᵼ�³�����ң���Ϊд EEPROME ֮����Ҫ��� 5ms��������д��� thread ��ר�Ŵ���궨������
#include "miscellaneousTools.h"
#include "GlobalVariable.h" 

// �̻߳ص�����
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
			case 0: // ���ϼ����ָ������¼��������������Ӧ�¼���ô��ȥ��Ӧ��״̬����ȥ����
				recved_event = 0;
				rt_event_recv(Schedule, 
CALI_PRESENT_FLUX | CALI_PRESENT_OUT_PRESS | CALI_PRESENT_IN_PRESS | SET_PRESS_PID | SET_FLUX_PID | SET_PRESS_SENSOR_MAX | SET_PRESS_SENSOR_MIN | SET_FLUX_SENSOR_MAX | SET_FLUX_SENSOR_MIN | POWER_ON | POWER_OFF | RESET_MOTOR | CALI_POS | MAN_MOVE_UP | MAN_MOVE_DOWN | MAN_MOVE_STOP | CHANGE_MAN_MOVE_SPEED | CHANGE_FEED_MODE_SPEED | CHANGE_CURRENT | CHANGE_MAN_ACC | CHANGE_FEED_ACC | CHANGE_PRESS_THRESHOLD | CHANGE_FLUX_THRESHOLD | MOVE_TO_TRG_POS | SHUT_DOWN_EVENT, 
			RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_event);
			
				if(recved_event & CALI_PRESENT_FLUX)  				// ����յ��궨��ǰ�����¼�
				{
					state1 = 10;
					break;
				}
				
				if(recved_event & CALI_PRESENT_OUT_PRESS)  		// ����յ��궨��ǰ����ѹ���¼�
				{
					state1 = 20;
					break;
				}
				
				if(recved_event & CALI_PRESENT_IN_PRESS)  		// ����յ��궨��ǰ���ѹ���¼�
				{
					state1 = 30;
					break;
				}
				
				if(recved_event & SET_PRESS_PID)  						// ����յ��궨����ѹ������PID�¼�
				{
					state1 = 40;
					break;
				}
				
				if(recved_event & SET_FLUX_PID)  							// ����յ��궨������������PID�¼�
				{
					state1 = 50;
					break;
				}
				
				if(recved_event & SET_PRESS_SENSOR_MAX)  			// ����յ��궨ѹ�������������¼�
				{
					state1 = 60;
					break;
				}
				
				if(recved_event & SET_PRESS_SENSOR_MIN)  			// ����յ��궨ѹ�������������¼�
				{
					state1 = 70;
					break;
				}
				
				if(recved_event & SET_FLUX_SENSOR_MAX)  			// ����յ��궨���������������¼�
				{
					state1 = 80;
					break;
				}
				
				if(recved_event & SET_FLUX_SENSOR_MIN)  			// ����յ��궨���������������¼�
				{
					state1 = 90;
					break;
				}
				
				if(recved_event & POWER_ON)  			// ����յ��궨���������������¼�
				{
					state1 = 95;
					break;
				}
				
				if(recved_event & POWER_OFF)  			// ����յ��궨���������������¼�
				{
					state1 = 100;
					break;
				}
			
				if(recved_event & RESET_MOTOR)  			// ����յ��궨���������������¼�
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
				
				
			case 10: // �궨��ǰ�����¼�
				rt_kprintf("Receive CALI_PRESENT_FLUX event. \n");
			
				CaliPresentADC(OUTLET_FLUX_RAW, CALI_PRESENT_OUTLET_FLUX, DEF_OUTLET_FLUX_MAX, DEF_OUTLET_FLUX_MIN, FLUX_ENC_HIGH, FLUX_PHY_HIGH, FLUX_ENC_LOW, FLUX_PHY_LOW, ADC_TO_FLUX_OUTLET, &flux_enc_high, &flux_phy_high, &flux_enc_low, &flux_phy_low, &adc_to_flux_outlet);
			
				// ��������֮��ǵ�����״̬
				state1 = 0;
				rt_kprintf("CALI_PRESENT_FLUX event Done. \n");
				break;
			
			
			
			
			case 20: // �궨��ǰ����ѹ���¼�
				rt_kprintf("Receive CALI_PRESENT_OUT_PRESS event. \n");
			
				CaliPresentADC(OUTLET_PRESS_RAW, CALI_PRESENT_OUTLET_PRESS, DEF_OUTLET_PRESS_MAX, DEF_OUTLET_PRESS_MIN, PRESS_OUT_ENC_HIGH, PRESS_OUT_PHY_HIGH, PRESS_OUT_ENC_LOW, PRESS_OUT_PHY_LOW, ADC_TO_PRESS_OUTLET, &press_outlet_enc_high, &press_outlet_phy_high, &press_outlet_enc_low, &press_outlet_phy_low, &adc_to_press_outlet);
			
				// ��������֮��ǵ�����״̬
				state1 = 0;
				rt_kprintf("CALI_PRESENT_OUT_PRESS event Done. \n");
				break;
			
			
			
			
			case 30: // �궨��ǰ���ѹ���¼�
				rt_kprintf("Receive CALI_PRESENT_IN_PRESS event. \n");
			
				CaliPresentADC(INLET_PRESS_RAW, CALI_PRESENT_INLET_PRESS, DEF_OUTLET_PRESS_MAX, DEF_OUTLET_PRESS_MIN, PRESS_IN_ENC_HIGH, PRESS_IN_PHY_HIGH, PRESS_IN_ENC_LOW, PRESS_IN_PHY_LOW, ADC_TO_PRESS_INLET, &press_inlet_enc_high, &press_inlet_phy_high, &press_inlet_enc_low, &press_inlet_phy_low, &adc_to_press_inlet);
			
				// ��������֮��ǵ�����״̬
				state1 = 0;
				rt_kprintf("CALI_PRESENT_IN_PRESS event Done. \n");
				break;
			
			
			
			
			case 40: // �궨����ѹ������PID�¼�
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
			
				// ��������֮��ǵ�����״̬
				state1 = 0;
				rt_kprintf("SET_PRESS_PID event Done. \n");
				break;
			
			
			
			
			case 50: // �궨������������PID�¼�
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
			
				// ��������֮��ǵ�����״̬
				state1 = 0;
				rt_kprintf("SET_FLUX_PID event Done. \n");
				break;
			
			
			
			
			case 60: // �궨ѹ�������������¼�
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
				
				// ��������֮��ǵ�����״̬
				state1 = 0;
				rt_kprintf("SET_PRESS_SENSOR_MAX event Done. \n");
				break;
			
			
			
			
			case 70: // �궨ѹ�������������¼�
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
				
				// ��������֮��ǵ�����״̬
				state1 = 0;
				rt_kprintf("SET_PRESS_SENSOR_MIN event Done. \n");
				break;
			
			
			
			
			case 80: // �궨���������������¼�
				rt_kprintf("Receive SET_FLUX_SENSOR_MAX event. \n");
			
				CaliSensor(DEF_OUTLET_FLUX_MAX, OUTLET_FLUX_MAX_ADDR, ADC_TO_FLUX_OUTLET, &last_flux_min, &last_flux_max, &adc_to_flux_outlet, 1);
				
				// ��������֮��ǵ�����״̬
				state1 = 0;
				rt_kprintf("SET_FLUX_SENSOR_MAX event Done. \n");
				break;
			
			
			
			
			case 90: // �궨���������������¼�
				rt_kprintf("Receive SET_FLUX_SENSOR_MIN event. \n");
			
				CaliSensor(DEF_OUTLET_FLUX_MIN, OUTLET_FLUX_MIN_ADDR, ADC_TO_FLUX_OUTLET, &last_flux_min, &last_flux_max, &adc_to_flux_outlet, 0);
				
				// ��������֮��ǵ�����״̬
				state1 = 0;
				rt_kprintf("SET_FLUX_SENSOR_MIN event Done. \n");
				break;
			
			
			
			
			case 95: // �ϵ�
				rt_kprintf("Receive POWER_ON event. \n");
				// ���֮ǰ���ܻᷢ��һЩ��ֵ����ݸ� Motor Thread��Motor Thread������Ϊ���ǿ�����Ϣ���������������һ�� GET_MOTOR_VERSION �¼�Ϊ�����������¼���׼��
				rt_event_recv(Schedule, GET_MOTOR_VERSION, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, 0, &recved_event);
				rt_pin_write(MOTOR_CTR, PIN_HIGH);     // �����̵����Ӷ����������		
				WriteShortToReg(POWER_UP_PROGRESS, 10); // �����ϵ����
			
				// ����ȴ� motor �̶߳�ȡ����汾�ţ������һֱ�ȴ�
				recved_event = 0;
				rt_event_recv(Schedule, GET_MOTOR_VERSION, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_event);
				rt_kprintf("Receive GET_MOTOR_VERSION event. \n");
				WriteShortToReg(POWER_UP_PROGRESS, 30);  // �����ϵ����
			
				// ��ȡ����汾��֮����Ҫ�ȴ�����̼�����
				rt_kprintf("Motor drive initialization ... \n");
				rt_thread_mdelay(MOTOR_INITIAL);
				WriteShortToReg(POWER_UP_PROGRESS, 50);
			
				// ���Ժ͵��ͨѶ��������Ƿ������ɹ������Ҹ���ͨѶ״̬�Ĵ���
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
				
				// ����̼�����֮��Ϳ��Խ��г�ʼ��������
				// ������������һ�ιػ�ʱ�� EP ָ����� EEPROME ���ݵ��Ĵ��������� EEPROME ���ݵ�ȫ�ֱ���
				DataInitialize();
				rt_kprintf("Data Initialized!!! \n");
				WriteShortToReg(POWER_UP_PROGRESS, 80);
			
				// �����������֮�󣬷��� THREAD_GET_WORK ��־�߳�ȫ����ʼ����
				rt_event_send(Schedule, THREAD_GET_WORK);
				rt_kprintf("Send THREAD_GET_WORK event \n");
			
				// ��ʱ��ʼ������ȫ����ɣ�ϵͳ��ʼ�������ϵ���ȸ���Ϊ 100%
				WriteShortToReg(POWER_UP_PROGRESS, 100);
				
				// ��������֮��ǵ�����״̬
				state1 = 0;
				rt_kprintf("POWER_ON event Done. \n");
				break;
			
			
			
			
			case 100: // �ϵ�
				rt_kprintf("Receive POWER_OFF event. \n");
			
				// ��� THREAD_GET_WORK �¼��������߳��˳�����
				rt_event_recv(Schedule, THREAD_GET_WORK, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, 0, &recved_event);
				rt_kprintf("Clear THREAD_GET_WORK event. \n");
			
				// �ȴ������߳��˳�
				rt_thread_mdelay(20);
			
				// ���͡�SK�������������ֹͣ�����һ�в���
				ret_val = ToMotorNoReturnNoParameter("SK");
				if(!ret_val)  // ��������ʧ��
				{
					rt_kprintf("Send SK FAIL!!! \n");
				}
				else
				{
					rt_kprintf("Send SK Succeed!!! \n");
				}
				
				// �ȴ���SK������ִ�����
				rt_thread_mdelay(200); 
				
				// ����ǰ�������ֵ������EEPROME
				SaveEncoderData();
			
				// �����̵������õ���ϵ�
				rt_pin_write(MOTOR_CTR, PIN_LOW);
			
				// �����ϵ���ȼĴ���
				WriteShortToReg(POWER_UP_PROGRESS, 0);
			
				// ��������֮��ǵ�����״̬
				state1 = 0;
				rt_kprintf("POWER_OFF event Done. \n");
				break;
			
				
				
				
			case 105:  // ����ʹ�ܵ��
				rt_kprintf("Receive RESET_MOTOR event. \n");
			
				// �������͡�AR����ӡ�ME��ָ��
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
				
				// ���⻹Ҫ����ͨѶ�Ĵ���Ϊ����
				WriteShortToReg(COMMUNICATION_STA, 1);
				
				// ���ڶ�ת���ܻ����𶪲����������� IP �� EP ������ IP_TO_EP �ı�����ϵ�����Ի����һЩ�������
				// �������ﳢ�Զ�ȡ EP ��Ȼ�� EP ֵ�ٴ�д���������� IP
				rt_thread_mdelay(700);	// ����ʹ�ܿ��ܻᷢ��˲�䶶���������������������ȵȴ�һЩʱ���õ���ȶ�
				ret_val = ToMotorWithReturn("EP", str);
				if(ret_val)  // ��ȡ EP �ɹ�
				{
					// ͨ�� SP ָ�������õ�ǰ�ĵ��������
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
				
				// ��������֮��ǵ�����״̬
				state1 = 0;
				rt_kprintf("RESET_MOTOR event Done. \n");
				break;
				
				
				
				
			case 110:  //�궨��ǰλ��
				rt_kprintf("Receive CALI_POS event. \n");
				current_pose = GetFloatFromReg(DEF_PRESENT_POS);  // �ӼĴ����л�ȡ��λ�������ĵ�ǰλ����ֵ
					 
				if(current_pose == 0) // �����Ҫ�궨0�㣬�����Ĭ�ϵļ������ת��ϵ��
				{
					// ���������ֵ��0
					ret_val = ToMotorNoReturnWithParameter("EP", "0");
					if(ret_val)	   // ����ͨѶ�ɹ�
					{
						if(ret_val != 2)  // �����æ
						{
							rt_kprintf("Motor Is Not Busy, Calibrating Zero Point. \n");
							// �����д��EEPROME
							WriteInt32ToEEPROME(ENCODER_ADDR, 0);
						}
						else							// ���æ�����æ��֮���� SavPose �̰߳�æ�����������ֵ�� EEprome
						{
							rt_kprintf("Cmd Is Sent But Motor Is Busy, SavPose Will Help us to save Encoder Value. \n");
						}
						
						// ���ݴ����ȼ���ת��ϵ����ע����ţ����֮ǰ������Ϊ��������ô����ҲҪ��֤Ϊ�����������Ǹı���ֵ��С��
						float result = EVC_RESOLUTION * CHUAN_DONG_BI / DAO_CHENG;
						coeff_env_to_valve = (coeff_env_to_valve > 0)? result : -result;
						rt_kprintf("Computing default coeff_env_to_valve(int cast): %d \n", (int32_t)coeff_env_to_valve );
					
						// ��ת��ϵ������ EEPROME
						ee_ret = WriteFloatToEEPROME(ENC_TO_POS, coeff_env_to_valve);
						if(ee_ret)
							rt_kprintf("Save coeff_env_to_valve To EEPROME Successfully!! \n");
						else
							rt_kprintf("Save coeff_env_to_valve Fail!! \n");
					}
					else		// ����ͨѶ���ɹ�
					{
						rt_kprintf("Calibrating Zero Point FAIL!!! \n");
					}
				}
				else 							 // ������Ǳ궨0��
				{
					// ���Ȼ�ȡ�����ǰ����ֵ
					int32_t encoder_value;
					ret_val = GetEnvFromMotor(&encoder_value);
					if(ret_val)		// ͨѶ�ɹ�
					{
						rt_kprintf("Current EP value: %d  \n", encoder_value);
						float encoder_f = (float)encoder_value;  // �Ƚ�������ֵתΪ float Ϊ������������׼��
						
						// ����µ�ת��ϵ��
						float tmp = encoder_f / current_pose;
						// ע�⣺���ܳ����û�һ��ʼ����0��Ȼ����������һ�����������Ȼ��С�İ���ȷ������ô��ʱ��ϵ���ͱ����0������Ҫ����������
						if(-100 < tmp && tmp < 100) // �궨��� 0 ����ڽӽ��Ļ��Ͳ��궨��
						{
							rt_kprintf("Weird coeff_env_to_valve: %d   System will not accept that. \n", (int32_t)tmp);
						}
						else
						{
							// ����ת��ϵ��
							coeff_env_to_valve = tmp;
							rt_kprintf("coeff_env_to_valve(int cast): %d   will be saved. \n", (int32_t)tmp);
							
							// �����º��ת��ϵ��д��EEPROME
							ee_ret = WriteFloatToEEPROME(ENC_TO_POS, coeff_env_to_valve);
							if(ee_ret)
								rt_kprintf("Save coeff_env_to_valve To EEPROME Successfully!! \n");
							else
								rt_kprintf("Save coeff_env_to_valve Fail!! \n");
						}
					}
					else					// ͨѶʧ��
					{
						rt_kprintf("Calibrating Position FAIL!!! \n");
					}
				}
				
				// ��������֮��ǵ�����״̬
				state1 = 0;
				rt_kprintf("CALI_POS event Done. \n");
				break;
				
				
				
				
			case 115:  //�ֶ�����
				rt_kprintf("Receive MAN_MOVE_UP event. \n");
						
				// �ȷ��� 1SK ��֮ǰ���˶�ָ��ȡ��
				ret_val = ToMotorNoReturnNoParameter("SK");
				if(!ret_val)
					rt_kprintf("Send SK FAIL!! \n");
				// �ٵ�����͡�1DI�������˶�����
				ret_val = ToMotorNoReturnWithParameter("DI", "100");
				if(!ret_val)
					rt_kprintf("Send DI 100 FAIL!! \n");
				// ���������͡�1CJ���õ�������˶�
				ret_val = ToMotorNoReturnNoParameter("CJ");
				if(!ret_val)
					rt_kprintf("Send CJ FAIL!! \n");
			
				// ��������֮��ǵ�����״̬
				state1 = 0;
				rt_kprintf("MAN_MOVE_UP event Done. \n");
				break;
			
			
			
			case 120:  //�ֶ�����
				rt_kprintf("Receive MAN_MOVE_DOWN event. \n");
				
				// �ȷ��� 1SK ��֮ǰ���˶�ָ��ȡ��
				ret_val = ToMotorNoReturnNoParameter("SK");
				if(!ret_val)
					rt_kprintf("Send SK FAIL!! \n");
				// �ٵ�����͡�1DI�������˶�����
				ret_val = ToMotorNoReturnWithParameter("DI", "-100");
				if(!ret_val)
					rt_kprintf("Send DI -100 FAIL!! \n");
				// ���������͡�1CJ���õ�������˶�
				ret_val = ToMotorNoReturnNoParameter("CJ");
				if(!ret_val)
					rt_kprintf("Send CJ FAIL!! \n");
			
				// ��������֮��ǵ�����״̬
				state1 = 0;
				rt_kprintf("MAN_MOVE_DOWN event Done. \n");
				break;
				
			
			
			case 125:  //�ֶ�ֹͣ
				rt_kprintf("Receive MAN_MOVE_STOP event. \n");
				
				// �������͡�1SJ�����õ��ֹͣ�˶�
				ret_val = ToMotorNoReturnNoParameter("SJ");
				if(!ret_val)
					rt_kprintf("Send SJ FAIL!! \n");
			
				// ��������֮��ǵ�����״̬
				state1 = 0;
				rt_kprintf("MAN_MOVE_STOP event Done. \n");
				break;
			
				
				
			
			case 130:   // �趨�ֶ����ڷ�ʱ���˶��ٶ�
				rt_kprintf("Receive CHANGE_MAN_MOVE_SPEED event. \n");
			
				// �Ƚ��Ĵ��������ݶ�������Ȼ��ת��Ϊ�ַ�����ʽ
				received_data = GetFloatFromReg(MANUAL_SPEED);
				sprintf(str, "%f", received_data);
				tmp = rt_strlen(str);  // ��ȡ���ݳ��ȣ���ֹ��Ϊ float �ľ������³��� .99999999999 ���������
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
				
				// ��������֮��ǵ�����״̬
				state1 = 0;
				rt_kprintf("CHANGE_MAN_MOVE_SPEED event Done. \n");
				break;
			
				
			case 135:		// �趨FEEDģʽ�µ��ڷ�ʱ���˶��ٶ�
				rt_kprintf("Receive CHANGE_FEED_MODE_SPEED event. \n");
			
				// �Ƚ��Ĵ��������ݶ�������Ȼ��ת��Ϊ�ַ�����ʽ
				received_data = GetFloatFromReg(PID_SPEED);
				sprintf(str, "%f", received_data);
				tmp = rt_strlen(str);  // ��ȡ���ݳ��ȣ���ֹ��Ϊ float �ľ������³��� .99999999999 ���������
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
				
				// ��������֮��ǵ�����״̬
				state1 = 0;
				rt_kprintf("CHANGE_FEED_MODE_SPEED event Done. \n");
				break;
				
				
			
			case 140:		// �趨����
				rt_kprintf("Receive CHANGE_CURRENT event. \n");
			
				// �Ƚ��Ĵ��������ݶ�������Ȼ��ת��Ϊ�ַ�����ʽ
				received_data = GetFloatFromReg(CURRENT);
				sprintf(str, "%f", received_data);
				tmp = rt_strlen(str);  // ��ȡ���ݳ��ȣ���ֹ��Ϊ float �ľ������³��� .99999999999 ���������
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
				
				// ��������֮��ǵ�����״̬
				state1 = 0;
				rt_kprintf("CHANGE_CURRENT event Done. \n");
				break;
			

				
				
			case 145:  // �����ֶ�ģʽ�µļ��ٶ�
				rt_kprintf("Receive CHANGE_MAN_ACC event. \n");
			
				// �Ƚ��Ĵ��������ݶ�������Ȼ��ת��Ϊ�ַ�����ʽ
				received_data = GetFloatFromReg(MANUAL_ACC);
				sprintf(str, "%f", received_data);
				tmp = rt_strlen(str);  // ��ȡ���ݳ��ȣ���ֹ��Ϊ float �ľ������³��� .99999999999 ���������
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
				
				// ��������֮��ǵ�����״̬
				state1 = 0;
				rt_kprintf("CHANGE_MAN_ACC event Done. \n");
				break;
			
			
				
			
			case 150:  // ���� FEED ģʽ�µļ��ٶ�
				rt_kprintf("Receive CHANGE_FEED_ACC event. \n");
			
				// �Ƚ��Ĵ��������ݶ�������Ȼ��ת��Ϊ�ַ�����ʽ
				received_data = GetFloatFromReg(PID_ACC);
				sprintf(str, "%f", received_data);
				tmp = rt_strlen(str);  // ��ȡ���ݳ��ȣ���ֹ��Ϊ float �ľ������³��� .99999999999 ���������
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
				
				// ��������֮��ǵ�����״̬
				state1 = 0;
				rt_kprintf("CHANGE_FEED_ACC event Done. \n");
				break;

				
			
			case 155:   // �趨ѹ��������ֵ
				rt_kprintf("Receive CHANGE_PRESS_THRESHOLD event. \n");
			
				SetThreshold(OUT_PRESS_CTR_THRESHOLD, EE_PRESS_CTR_THRESHOLD, &press_threshold);
		
				// ��������֮��ǵ�����״̬
				state1 = 0;
				rt_kprintf("CHANGE_PRESS_THRESHOLD event Done. \n");
				break;
				
			
				
				
			case 160:   // �趨����������ֵ
				rt_kprintf("Receive CHANGE_FLUX_THRESHOLD event. \n");
			
				SetThreshold(OUT_FLUX_CTR_THRESHOLD, EE_FLUX_CTR_THRESHOLD, &flux_threshold);
		
				// ��������֮��ǵ�����״̬
				state1 = 0;
				rt_kprintf("CHANGE_FLUX_THRESHOLD event Done. \n");
				break;

			
			
			
			case 165:   // �˶����ض�λ��
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
		
				// ��������֮��ǵ�����״̬
				state1 = 0;
				rt_kprintf("MOVE_TO_TRG_POS event Done. \n");
				break;
			
				
				
				
			case 170:   // ��ָͣ��
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
			
				// ��������֮��ǵ�����״̬
				state1 = 0;
				rt_kprintf("SHUT_DOWN_EVENT event Done. \n");
				break;
				
			default:
				state1 = 0;
				break;
		}
	}
}

// �궨�����̳߳�ʼ��
int calivar_init()
{
	// �����߳�
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


