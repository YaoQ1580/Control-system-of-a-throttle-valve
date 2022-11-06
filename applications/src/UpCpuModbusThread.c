#include "miscellaneousTools.h"
#include "GlobalVariable.h" 

/*---------��̬����----------*/
static struct rt_semaphore rtu_rx_sem; // ���ڽ����ź���

// ���ڽ��ջص�����
static rt_err_t rtu_rx(rt_device_t dev, rt_size_t size)
{
	// ÿ�յ�һ��byte���ͷ�һ���ź����������ź��������������յ����ֽ�����
	rt_sem_release(&rtu_rx_sem);
  return RT_EOK;
}

// �̺߳���
static void thread_upcpu_rtu_entry(void *parameter)
{
	rt_err_t ret = RT_ERROR;    
	uint8_t state = 0;          // ״̬������
	uint8_t cnt = 0;            // �յ��������ֽ���
	uint8_t rx_buffer[64];      // �������ݻ�����
	uint8_t* ptr = rx_buffer;   // �������ݻ�����ָ��
	uint16_t CRC_code;          // CRCУ��λ
	uint8_t CRC_code_byte[2];   // CRCУ��λ(�ַ���ʽ)
	
	while(1)
	{
		switch(state)
		{
			case 0:
				// �����ڴ˵ȴ�ֱ���յ�һ���ź������������ڽ��ջ�����������
				rt_sem_take(&rtu_rx_sem, RT_WAITING_FOREVER);
				// �յ�һ���źź����Ҫ��ȡһ���ֽڣ������ź����������ֽ��������Ա���һ��
				rt_device_read(upcpu_rtu, -1, ptr, 1);
				ptr ++;
				cnt ++;
				// ����״̬������һ��״̬
				state = 10;
				break;
			
			case 10:
				// ѭ�������ź�����������ź�����˵�������������ַ����������WAIT_2Tʱ��û���յ��ź����������һ֡���ݽ���
				// ����RTUЭ�飬WAIT_2TӦ��Ҫ����1.5���ַ�����ʱ�䣬һ���ַ���8λ�������ʼ����ֹλ����һ���ַ�Ϊ10λ�����ݲ�����λ115200���������
			  // 1.5���ַ�ʱ��Ϊ��1/115200 * 10 * 1.5 = 1.3020e-4 s = 0.13ms, ������1msʱ��Ƚ��ȶ�
				ret = rt_sem_take(&rtu_rx_sem, UPCPU_WAIT_2T); 
				if(ret == RT_EOK) // ����յ��ź���
				{
					rt_device_read(upcpu_rtu, -1, ptr, 1); // ��ȡ�������ַ�
					ptr ++;
					cnt ++;
				}
				else // ���û���յ��ź�������ô˵��һ֡���ݽ���
				{
					// һ֡���ݽ��������CRCУ��λ�Ƿ���ȷ
					CRC_code = CheckCRC(rx_buffer, cnt - 2); // ����CRCУ��λ
					CRC_code_byte[0] = CRC_code >> 8;        
					CRC_code_byte[1] = CRC_code & 0x00FF;
					if(!rt_memcmp(CRC_code_byte, rx_buffer + cnt - 2, 2)) // ������ɵ�У��λ���յ���У��λһ��
					{
						//rt_kprintf("Get modbus rtu msg, CRC: %x  \n", CRC_code);
						Process(rx_buffer, cnt, upcpu_rtu);     // ����mosbus RTUЭ��������Ӧ����Ȧ�ͼĴ������ж�д����
						PostProcess(rx_buffer, cnt); // �����յ���modbus�ַ�����ִ����Ӧ������
					}
					else
					{
						rt_kprintf("Modbus rtu communication error  \n");
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

// ��ʼ���봮����RTUͨѶ���߳�
int upcpu_rtu_init()
{
	struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
	
	// ��ʼ�����ڽ����ź���
	rt_sem_init(&rtu_rx_sem, "upcpu_rtu_rx_sem", 0, RT_IPC_FLAG_FIFO);
	
	// ��ʼ��ͨѶӲ�����ڣ�����������Ϊ115200
	upcpu_rtu = rt_device_find(UPCPU_RTU_SERIAL);
	if (!upcpu_rtu)
	{
     rt_kprintf("find upcpu_rtu serial failed!\n");
     return RT_ERROR;		
  }
	rt_device_open(upcpu_rtu, RT_DEVICE_FLAG_INT_RX);
	config.baud_rate = UPCPU_RTU_BAUD_RATE;
	rt_device_control(upcpu_rtu, RT_DEVICE_CTRL_CONFIG, &config);
	rt_device_set_rx_indicate(upcpu_rtu, rtu_rx);
	
	// ��ʼ��ͨѶ�߳�
	rt_thread_t thread_upcpu_rtu = rt_thread_create("thread_upcpu_rtu", thread_upcpu_rtu_entry, RT_NULL, 4096, UP_CPU_PRIORITY, 10);
	if (thread_upcpu_rtu != RT_NULL)
  {
     rt_thread_startup(thread_upcpu_rtu);
  }
  else
  {
		 rt_kprintf("UpCpu Thread Create Fail !! \n");
     return RT_ERROR;
  }
	
	// the end
	return RT_EOK;
}

