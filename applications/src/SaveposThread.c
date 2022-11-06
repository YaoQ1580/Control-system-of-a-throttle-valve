#include "miscellaneousTools.h"
#include "GlobalVariable.h" 

// �̺߳���
static void thread_savepos_entry(void *parameter)
{
	uint8_t state1 = 0;					 // ״̬����ģ�����������ֹͣ��
	uint8_t state2 = 0;					 // ״̬����ģ�⹤�����裩
	rt_uint32_t recved_event; 	 // �¼��Ľ���
	char status_code_str[10]; // ������ַ���ʽ��״̬��
	uint32_t satus_code = 0;		 // ʮ��������ʽ��״̬��
	int ret_val;								 // ���ͨѶӦ��
	int mov_flag = -1;					 // ����Ƿ��˶���־λ
	int stall_flag = -1;				 // ��ת��־λ
	
	while(1)
	{
		switch(state1)
		{
			case 0:		// һֱ�ȴ� THREAD_GET_WORK �¼�
				rt_event_recv(Schedule, THREAD_GET_WORK, RT_EVENT_FLAG_OR, RT_WAITING_FOREVER, &recved_event);
				state1 = 10;
				rt_kprintf("SavePos Thread Ready to work. \n");
				break;
			
			
			
			case 10:	// ��������״̬
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
				// ��ȡ���״̬
				ret_val = ToMotorWithReturn("SC", status_code_str);
				if(!ret_val)  // ��ȡʧ��
				{
					rt_kprintf("SavePos Thread Read Status Code FAIL!! \n");
				}
				else					// ��ȡ�ɹ�
				{
					// ����ȡ�����ַ�������һ��ʮ��������
					StrToHex(status_code_str, &satus_code);
					mov_flag = satus_code & 0x10;   // ��ȡ����˶���־λ
					stall_flag = satus_code & 0x04; // ��ȡ�����ת��־λ
					
					// ���¶�ת�Ĵ���
					if(stall_flag) // �����ת��������Ӧ�Ĵ���
					{
						WriteShortToReg(STALL_DETECTION, 1);
					}
					else           // �������ת��������Ӧ�Ĵ���
					{
						WriteShortToReg(STALL_DETECTION, 0);
					}
					
					// �˶����͵��λ��ʵʱ����
					switch(state2)
					{
						case 0 : // ������Ƿ��˶�
							if(mov_flag) 	// ���������˶�
							{
								rt_event_send(Schedule, MOV_STA); // ���� MOV_STA�� ��֪�����̵߳�������˶���
								state2 = 10; // ����״̬���������������ʲôʱ��ֹͣ
								rt_kprintf("Motor is moving !! \n");
							}
							break;
						
						case 10: // ������Ƿ�ֹͣ
							if(!mov_flag)
							{
								rt_kprintf("Motor stopped, it's time to save encoder !! \n");
								SaveEncoderData(); // ����ʱ���µĵ������ֵ����EEPROME
								rt_event_recv(Schedule, MOV_STA, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, 0, &recved_event); // ���ֹͣ����� MOV_STA �¼���־
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
			
			
			/*-----------------------����ͻ���¼�����Ӧ��ڣ��������ж�������------------------------------*/
				
			case 99:  // �ػ�����һЩ�ƺ���
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

// ����������̳߳�ʼ��
int savepos_init()
{
	// �����߳�
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

