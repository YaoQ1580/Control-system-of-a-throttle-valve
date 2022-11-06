#include "miscellaneousTools.h"
#include "GlobalVariable.h" 

/*---------��̬����----------*/
static struct rt_semaphore motro_rx_sem; // ���ڽ����ź���

// ���ڽ��ջص�����
static rt_err_t motor_rx(rt_device_t dev, rt_size_t size)
{
	// ÿ�յ�һ��byte���ͷ�һ���ź����������ź��������������յ����ֽ�����
	rt_sem_release(&motro_rx_sem);
  return RT_EOK;
}

// �̺߳���
static void thread_motor_entry(void *parameter)
{
	rt_err_t ret = RT_ERROR;    
	uint8_t state = 0;          // ״̬��
	uint8_t cnt = 0;            // ���ݳ���
	uint8_t rx_buffer[32];      // ���ݽ��ջ���
	uint8_t* ptr = rx_buffer;   // ���ݻ���ָ��
	uint8_t rx_buffer_str[128]; 
	
	while(1)
	{
		switch(state)
		{
			case 0:
				// �����ڴ˵ȴ�ֱ���յ�һ���ź������������ڽ��ջ�����������
				rt_sem_take(&motro_rx_sem, RT_WAITING_FOREVER);
				// �յ�һ���źź����Ҫ��ȡһ���ֽڣ������ź����������ֽ��������Ա���һ��
				rt_device_read(motor, -1, ptr, 1);
				//rt_kprintf("Incoming data: %x", *ptr);
				ptr ++;
				cnt ++;
				// ����״̬������һ��״̬
				state = 10;
				break;
			
			case 10:
				// ѭ�������ź�����������ź�����˵�������������ַ���������� WAIT_BYTE ʱ��û���յ��ź����������һ֡���ݽ���
				ret = rt_sem_take(&motro_rx_sem, WAIT_BYTE); 
				if(ret == RT_EOK) // ����յ��ź���
				{
					rt_device_read(motor, -1, ptr, 1); // ��ȡ�������ַ�
					ptr ++;
					cnt ++;
				}
				else // ���û���յ��ź�������ô˵��һ֡���ݽ���
				{
					switch(rx_buffer[0]) // �����յ��ĵ�һ���ֽ�
					{
						case 0xE0:
						case 0x3F:
						case 0xFF:
							rt_kprintf("Maybe Power On Ack: ");
							DecToHex(rx_buffer, rx_buffer_str, cnt); // ������汾�Ŵ�ӡ����
							rt_kprintf("%s\n", rx_buffer_str);
							rt_event_send(Schedule, GET_MOTOR_VERSION); // �����¼�����������ϵ�ɹ������������������һ�ε���ʱ�ı���ֵ
							break;
						
						case MOTOR_ADDR: // ����ڲ���SCLָ���ַ
							//rt_kprintf("Motor: Now acking... \n");
							*ptr = '\0';				
							SendTxtMail(ack, rx_buffer); // �������Ӧ���͵������Ա������߳�ʹ��
							//rt_kprintf("Get ack: %s \n", rx_buffer);
							break;
						
						default:
							break;
					}
					ptr = rx_buffer; // �������ݻ�����ָ��
					cnt = 0;
					state = 0; 			 // ����״̬��
				}
				break;
			
			default:
				break;
		}
	}
}

// ��ʼ������ͨѶ���߳�
int motor_init()
{
	struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
	
	// ��ʼ�����ڽ����ź���
	rt_sem_init(&motro_rx_sem, "motro_rx_sem", 0, RT_IPC_FLAG_FIFO);
	
	// ��ʼ��ͨѶ����
	ack = rt_mb_create("ack", 1, RT_IPC_FLAG_FIFO);          
  if (ack == RT_NULL)
  {
     rt_kprintf("create ack mailbox failed.\n");
     return RT_ERROR;
  }
	
	// ��ʼ��ͨѶӲ�����ڣ�����������ΪMOTOR_BAUD_RATE
	motor = rt_device_find(MOTOR_SERIAL);
	if (!motor)
	{
     rt_kprintf("find motor failed!\n");
     return RT_ERROR;		
  }
	rt_device_open(motor, RT_DEVICE_FLAG_INT_RX);
	config.baud_rate = MOTOR_BAUD_RATE;
	rt_device_control(motor, RT_DEVICE_CTRL_CONFIG, &config);
	rt_device_set_rx_indicate(motor, motor_rx);
	
	// ��ʼ��ͨѶ�߳�
	rt_thread_t thread_motor = rt_thread_create("thread_motor", thread_motor_entry, RT_NULL, 4096, MOTOR_PRIORITY, 10);
	if (thread_motor != RT_NULL)
  {
     rt_thread_startup(thread_motor);
  }
  else
  {
		 rt_kprintf("Motor Thread Create fail!\n");
     return RT_ERROR;
  }
	
	// the end
	return RT_EOK;
}

