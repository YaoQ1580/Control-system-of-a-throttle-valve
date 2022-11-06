#include "miscellaneousTools.h"
#include "GlobalVariable.h" 

/*------��̬ȫ�ֱ���------*/
static uint8_t index = 0;   						 // ADC array index
static rt_uint32_t value1[MOV_AVG_NUM];
static rt_uint32_t value2[MOV_AVG_NUM];
static rt_uint32_t value3[MOV_AVG_NUM];
static uint32_t sum1 = 0;
static uint32_t sum2 = 0;
static uint32_t sum3 = 0;
static uint8_t state_ADC = 0;




// �����ADC��ȡ�йصı���
void ADC_clear()
{
	index = 0;
	sum1 = 0; sum2 = 0; sum3 = 0;
	state_ADC = 0;	// ���� ADC ״̬��
}




// ˢ��ADC�������õ��˻���ƽ���˲�����8������Ȼ������˲�
void ADC_Refresh()
{
	uint8_t i;
	uint32_t avg1, avg2, avg3;
	float avg1_f, avg2_f, avg3_f;
	
	switch(state_ADC) // ״̬��
	{
		case 0 :  // ���ȶ��� MOV_AVG_NUM ��ADC��ֵ��Ϊ֮��Ļ���ƽ����׼��
			value1[index] = rt_adc_read(adc_dev, PRESS_IN_CHANNEL);  // �����ѹ��
			sum1 += value1[index];
			value2[index] = rt_adc_read(adc_dev, PRESS_OUT_CHANNEL); // ������ѹ��
			sum2 += value2[index];
			value3[index] = rt_adc_read(adc_dev, FLUX_CHANNEL);      // ����������
			sum3 += value3[index];
		
			index ++;
			if(index >= MOV_AVG_NUM)
			{
				index = 0;
				state_ADC = 10; 	// �������8���Ļ�������״̬��
			}
			break;
		
			
			
		case 10:  // ���л���ƽ���˲�
			// ����һ�����ݶ�����
			sum1 -= value1[0];
			sum2 -= value2[0];
			sum3 -= value3[0];
		
			// �ƶ����ݣ�Ϊ���¶�ȡ������Ų��λ��
			for(i = 0; i < MOV_AVG_NUM - 1; i ++)
			{
				value1[i] = value1[i + 1];
				value2[i] = value2[i + 1];
				value3[i] = value3[i + 1];
			}
			value1[MOV_AVG_NUM - 1] = rt_adc_read(adc_dev, PRESS_IN_CHANNEL);  // ��ȡ���µ����ѹ������
			sum1 += value1[MOV_AVG_NUM - 1];
			value2[MOV_AVG_NUM - 1] = rt_adc_read(adc_dev, PRESS_OUT_CHANNEL); // ��ȡ���µĳ���ѹ������
			sum2 += value2[MOV_AVG_NUM - 1];
			value3[MOV_AVG_NUM - 1] = rt_adc_read(adc_dev, FLUX_CHANNEL);      // ��ȡ���µĳ�����������
			sum3 += value3[MOV_AVG_NUM - 1];
			
			// ���л���ƽ���˲�
			avg1 = sum1 / MOV_AVG_NUM; 
			avg2 = sum2 / MOV_AVG_NUM;  
			avg3 = sum3 / MOV_AVG_NUM; 

			// �� avg ת�� float ����Ϊ֮��ļ����ṩ����
			avg1_f = (float) avg1;
			avg2_f = (float) avg2;
			avg3_f = (float) avg3;
			
			// ������д��Ĵ���
			WriteFloatToReg(INLET_PRESS_RAW, avg1_f);   // ��δת�������ѹ������Ĵ������Է���Ҫ���±궨��ǰ���ѹ��
			WriteFloatToReg(OUTLET_PRESS_RAW, avg2_f);  // ��δת���ĳ���ѹ������Ĵ������Է���Ҫ���±궨��ǰ����ѹ��
			WriteFloatToReg(OUTLET_FLUX_RAW, avg3_f);   // ��δת���ĳ�����������Ĵ������Է���Ҫ���±궨��ǰ�������� 			
			WriteFloatToReg(PRESENT_INLET_PRESS, press_inlet_phy_low + (avg1_f - press_inlet_enc_low) / adc_to_press_inlet);
			WriteFloatToReg(PRESENT_OUTLET_PRESS, press_outlet_phy_low + (avg2_f - press_outlet_enc_low) / adc_to_press_outlet); // ��ADC��ֵ����ת��ϵ����д��Ĵ����Թ���λ����ȡ																		
			WriteFloatToReg(PRESENT_OUTLET_FLUX, flux_phy_low + (avg3_f - flux_enc_low) / adc_to_flux_outlet);   // ��ADC��ֵ����ת��ϵ����д��Ĵ����Թ���λ����ȡ
			break;
	}
}



// ��ȡ��ǰλ��
void IE_Refresh()
{
	int32_t encoder_value = 0;  // ��������Int������
	float encoder_f, pose_data;
	char encoder_value_str[32]; // ���������ַ���ʽ
	
	int ret_val;								// ���ͨѶ�Ƿ�ɹ�
	
	// �������͡�1IE��ָ������ȡ�������ֵ
	ret_val = ToMotorWithReturn("IE", encoder_value_str);
	if(ret_val)		// ���Ӧ��ɹ�
	{
		// ���ַ���ʽ�ı�������ֵת����int��ʽ
		sscanf(encoder_value_str, "%d", &encoder_value);
		
		// ��Ϊ����Ҫ������������������תΪ float ��
		encoder_f = (float)encoder_value;
		
		// ����ת��ϵ���Ӷ��õ�������λ��
		pose_data = encoder_f / coeff_env_to_valve;
		
		// ����о����д��Ĵ���
		WriteFloatToReg(PRESENT_VALVE_POS, pose_data);
	}
	else					// ���Ӧ��ʧ��
	{
		rt_kprintf("Refresh IE Fail !! \n");
	}
}



// �̺߳���
static void thread_data_entry(void *parameter)
{
	rt_uint32_t recved_event;
	uint16_t state1 = 0;
	
	uint8_t ie_tick = 0;  // ˢ�µ�ǰλ��ʱ��Ƭ
	uint8_t adc_tick = 0; // ˢ��ADC����ʱ��Ƭ
	
	while(1)
	{
		switch(state1) // ״̬��
		{
			case 0 :   // һֱ�ȴ� THREAD_GET_WORK �¼�
				rt_event_recv(Schedule, THREAD_GET_WORK, RT_EVENT_FLAG_OR, RT_WAITING_FOREVER, &recved_event);
				state1 = 10;
				rt_kprintf("Data Thread Ready to work. \n");
				break;
			
			
			
			case 10:   // ��������״̬ 
				/*--------------------��ȡһЩ��������������ͻ���¼�------------------*/
				recved_event = 0;
				rt_event_recv(Schedule, THREAD_GET_WORK, RT_EVENT_FLAG_OR, 0, &recved_event);
			
				if(!(recved_event & THREAD_GET_WORK))   // �ػ��¼�
				{
					state1 = 99;
					break;
				}
				/*----------------------------------------------------------------------*/
				
				
				
				/*------------------------------������������----------------------------*/
				ie_tick++; 
				adc_tick++; 
				
				if(ie_tick >= IE_TICK)
				{	
					ie_tick = 0;
					IE_Refresh();  // ����ʾ��ǰλ��ʱ��Ƭ���˺�ˢ����ʾһ�ε�ǰλ��
				}
				
				if(adc_tick >= ADC_TICK)
				{	
					adc_tick = 0;
					ADC_Refresh(); // ����ʾADC��ֵʱ��Ƭ���˺�ˢ����ʾһ��ADC
				}
				
				/*----------------------------------------------------------------------*/
				
				rt_thread_mdelay(REFRESH_BASE); // ˢ���ӳ�
				break;
			
				
				
			/*-----------------------����ͻ���¼�����Ӧ��ڣ��������ж�������------------------------------*/
				
			case 99: 		// �ػ�����һЩ�ƺ���
				// ������мĴ��������ڼĴ��������Ƚ϶࣬������һ������ʡ���ˣ���ֹ����߳�����ߺ�ʱ̫�ã�
//				rt_enter_critical();
//				for(i = 0; i < REG_NUM; i++)
//				{
//					Reg[i] = 0;
//				}
//				rt_exit_critical();
				
			  // ����ʱ��Ƭ
				ie_tick = 0;
				adc_tick = 0;
			
				// ��Ϊ ADC ��ȡ���õ���״̬����������Ҫ���� ADC ��ر���
				ADC_clear();
			
				// ����״̬��
				state1 = 0;
				rt_kprintf("Data Thread Shut Down. \n");
				break;
			
			/*---------------------------------------------------------------------------------------------*/
				
			default:
				break;
		}
	}
}



// ADCӲ����ʼ������
int init_ADC()
{
	rt_err_t ret = RT_EOK;
	
	// ��ȡADCӲ�����
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

// �̳߳�ʼ������
int dataThread_init()
{
	// ��ʼ��ADC
	init_ADC();
	
	// ��ʼ���߳�
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

