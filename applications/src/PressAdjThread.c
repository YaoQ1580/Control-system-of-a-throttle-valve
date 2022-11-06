#include "miscellaneousTools.h"
#include "GlobalVariable.h" 

/*------��̬ȫ�ֱ���-----*/
static int type = 0; 							// ����ģʽ��0�������ơ�1������ѹ�����ơ�2��������������
static float sum_press_diff = 0;  // ѹ��������Ŀ��ֵ�͵�ǰֵ��ƫ���ۻ�
static float sum_flux_diff = 0;   // ����������Ŀ��ֵ�͵�ǰֵ��ƫ���ۻ�
static float Trg;

// PID���ƺ���
void PIDctr(uint32_t P_reg_addr, uint32_t I_reg_addr, uint32_t D_reg_addr, uint32_t Trg_reg_addr, uint32_t Cur_reg_addr, float* ctr_threshold)
{
	uint32_t recved_event;
	
	// PID��ز���
	float* sum_type_addr; 
	float diff = 0; 
	float tmp = 0; 
	float compare = 0;
	
	if(type == 1)
		sum_type_addr = &sum_press_diff;
	if(type == 2)
		sum_type_addr = &sum_flux_diff;
	
	recved_event = 0;
	rt_event_recv(Schedule, MOV_STA, RT_EVENT_FLAG_OR, 0, &recved_event); // ��� MOV_STA��������Ƿ����˶�
	if(!(recved_event & MOV_STA)) 	// ��������˶�����Է��͡�1FL�����˶�ָ��
	{
		//diff = GetFloatFromReg(Trg_reg_addr) - GetFloatFromReg(Cur_reg_addr); // ����Ŀ��ֵ�͵�ǰֵ�Ĳ�ֵ
		diff = Trg - GetFloatFromReg(Cur_reg_addr);
		
		compare = diff >= 0 ? diff : -diff;
		if(compare >= *ctr_threshold) // ���������ֵ�����PID����
		{
			// �� diff ֵ�����ۼӣ�Ϊ������ PD �� PI �����ṩ����
			*sum_type_addr = *sum_type_addr + diff;
							
			// ����P����
			tmp = diff * GetFloatFromReg(P_reg_addr);  // + (*sum_type_addr) * GetFloatFromReg(OUTLET_PRESS_I);
			tmp = tmp * 20000; // ���Ե���ġ�EG��ֵ�Է���Ҫ���õ�P��I��D��ֵ����
							
			// �����ת���ɵ������
			FeedLenEnc((int32_t)tmp);
		}
	}
}



// �̺߳���
static void thread_PressAdj_entry(void *parameter)
{
	rt_uint32_t recved_event;
	uint16_t state1 = 0;
	uint16_t stall_detec;
	
	// ���ͨѶ���
	int ret_val;
	
	// PIDˢ��ʱ��Ƭ
	uint16_t pid_tick = 0;
	
	while(1)
	{
		switch(state1) // ״̬��
		{
			case 0:    // һֱ�ȴ� THREAD_GET_WORK �¼�
				rt_event_recv(Schedule, THREAD_GET_WORK, RT_EVENT_FLAG_OR, RT_WAITING_FOREVER, &recved_event);
				state1 = 10;
				rt_kprintf("PID Adj Thread Ready to work. \n");
				break;
			
			
			case 10:	 // ��������״̬ 
				/*--------------------��ȡһЩ��������������ͻ���¼�------------------*/
				recved_event = 0;
				rt_event_recv(Schedule, THREAD_GET_WORK, RT_EVENT_FLAG_OR, 0, &recved_event);
				if(!(recved_event & THREAD_GET_WORK))   // �ػ��¼�
				{
					state1 = 99;
					break;
				}
				
				recved_event = 0;
				rt_event_recv(Schedule, QUIT_PID, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, &recved_event);
				if(recved_event & QUIT_PID)							// QUIT_PID �¼�
				{
					state1 = 40;
					break;
				}
				/*----------------------------------------------------------------------*/
			
			
				/*-----------------------------------������������---------------------------------*/
				/*------���¿���ģʽ�������ǣ���ת�������¼�--------*/
				// ��� SET_PRESS_OUT �� SET_FLUX �¼�
				recved_event = 0;
				rt_event_recv(Schedule, SET_PRESS_OUT | SET_FLUX, 
				RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, &recved_event);
				if(recved_event & SET_PRESS_OUT)	// ����յ�ѹ�������¼�����ô������Ϊ1
				{
					rt_kprintf("PID Adj Thread Receive SET_PRESS_OUT event. \n");
					Trg = GetFloatFromReg(TRG_OUTLET_PRESS);
					rt_kprintf("Trg Press(int cast): %d \n", (int)Trg);
					type = 1;
				}
				if(recved_event & SET_FLUX)				// ����յ����������¼�����ô������Ϊ2
				{
					rt_kprintf("PID Adj Thread Receive SET_FLUX_OUT event. \n");
					Trg = GetFloatFromReg(TRG_OUTLET_FLUX);
					rt_kprintf("Trg Flux(int cast): %d \n", (int)Trg);
					type = 2;
				}
				
				// ���������⵽��ת�Ļ������ type
				// ����û�н� stall ��Ϊͻ���¼�����Ϊ���߳�û�и��Ķ�ת�Ĵ������ʸ������Ϊͻ���¼�����ô�����룺��⵽->����->��⵽->����...����������
				stall_detec = GetShortFromReg(STALL_DETECTION);
				if(stall_detec) // �����⵽��ת
				{
					sum_press_diff = 0;
					sum_flux_diff = 0;
					type = 0;
				}
				/*--------------------------------------------------*/
				
				switch(type)
				{
					case 0:		// Ĭ��״̬��ʲô������
						pid_tick = 0;
						break;
					
					case 1:   // ѹ������
						pid_tick++;
						if(pid_tick >= PID_UPDATE_CNT)
						{
							pid_tick = 0;
							PIDctr(OUTLET_PRESS_P, OUTLET_PRESS_I, OUTLET_PRESS_D, TRG_OUTLET_PRESS, PRESENT_OUTLET_PRESS, &press_threshold);
						}
						break;
					
					case 2:   // ��������
						pid_tick++;
						if(pid_tick >= PID_UPDATE_CNT)
						{
							pid_tick = 0;
							PIDctr(OUTLET_FLUX_P, OUTLET_FLUX_I, OUTLET_FLUX_D, TRG_OUTLET_FLUX, PRESENT_OUTLET_FLUX, &flux_threshold);
						}
						break;
				}
				/*--------------------------------------------------------------------------------*/
				
				rt_thread_mdelay(REFRESH_BASE); // ˢ���ӳ�
				break;
			
			/*-----------------------����ͻ���¼�����Ӧ��ڣ��������ж�������------------------------------*/
			case 40:  // QUIT_PID �¼�
				sum_press_diff = 0;
				sum_flux_diff = 0;
				type = 0;			// ��� PID ����ģʽ
				// ��������� SK ���������ܻ�����˳� PID ���ƣ�Ȼ����һ�ε� FL ������Ȼ��ִ��
				ret_val = ToMotorNoReturnNoParameter("SK");
				if(ret_val)
				{
					rt_kprintf("Quit PID control, Shut Down the Motor Successfully!! \n");
				}
				else
				{
					rt_kprintf("Quit PID control, But Fail to Shut Down the Motor !! \n");
				}
				state1 = 10;  // �ص���������״̬���ȴ���һ����ָ��
				rt_kprintf("PID Adj Thread Quit PID Control !! \n");
				break;
			
				
				
			case 99:  // �ػ��¼�
				sum_press_diff = 0;
				sum_flux_diff = 0;
				type = 0;			// ��� PID ����ģʽ
				state1 = 0;		// �ص��ȴ�����״̬
				rt_kprintf("PID Adj Thread Shut Down !! \n");
				break;
			/*---------------------------------------------------------------------------------------------*/	
					
			
			default:
				break;
		}
	}
}

// �̳߳�ʼ������
int PressAdjThread_init()
{
	// ��ʼ���߳�
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

