// �����������������������λ����

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
			case 0:   // ��ʼ״̬��Ӧ�õȴ��¼�����ʼ����
				/*-------------�ȴ�����1------------*/
				tik = 0;
				recved_event = 0;
				rt_event_recv(Schedule2, START_TEST, RT_EVENT_FLAG_OR, 0, &recved_event); // �ȴ���ʼ����
				if(recved_event & START_TEST)
				{
					state = 10;
					break;
				}
				/*-----------------------------------*/
				
				/*--------------�ȴ����͸��ģʽ�·���ָ����ָ������һ�δ���һ��----------------*/
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
			
			/*---------------------------����1�׶�------------------------------*/
			case 10:  // ���Խ׶�
				recved_event = 0;
				rt_event_recv(Schedule2, START_TEST, RT_EVENT_FLAG_OR, 0, &recved_event);
				if(!(recved_event & START_TEST))  // ���RTU�߳�����������ԣ���ô�ͽ�������
				{
					state = 20;
					break;
				}
				
				// ������������ĵ��Խ׶�
				recved_event = 0;
				rt_event_recv(Schedule, MOV_STA, RT_EVENT_FLAG_OR, 0, &recved_event); // ��� MOV_STA��������Ƿ����˶�
				if(!(recved_event & MOV_STA)) // ��������˶�����Է����˶�ָ��
				{
					tik++;
					if(tik >= 300)  // ÿ���˶�֮ǰ�ȵ� 3000ms �ù�����Ա��¼һ�µ�ǰλ��
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
				
				// �߳����ߣ���CPU��Դ�ó���������
				rt_thread_mdelay(REFRESH_BASE);
				
				break;
			
			case 20:  // ��������
				ToMotorNoReturnNoParameter("SKD");
			
				state = 0;  // ����״̬
				break;
			/*------------------------����1��ֹ----------------------------*/

			/*--------------------͸��ģʽ�µ�ָ���-----------------------*/
			case 50:
				rt_kprintf("Test Thread Receive MOTOR_DIRECT_CMD event! \n");
			
				// ������ʾ��Ļ�ȷ��͵��ǵ� 8 λ���ٷ��͸� 8 λ�ģ�������ʾ���ַ�����ң�����Ҫ����һ�� Reg ���������
				cnt = 0;
				i = MOTOR_CMD_STR;
				do{
					high_char = Reg[i] >> 8;
					low_char = (uint8_t)Reg[i];
					cmd[cnt++] = high_char;
					cmd[cnt++] = low_char;
					i++;
				}while(low_char != '\0');
				
				// ����յ��������
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

// �̳߳�ʼ������
int TestThread_init()
{
	// ��ʼ���߳�
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

