#include "miscellaneousTools.h"
#include "GlobalVariable.h" 

/*---------静态变量----------*/
static struct rt_semaphore rtu_rx_sem; // 串口接收信号量

// 串口接收回调函数
static rt_err_t rtu_rx(rt_device_t dev, rt_size_t size)
{
	// 每收到一个byte便释放一个信号量，这样信号量的数量便是收到的字节数量
	rt_sem_release(&rtu_rx_sem);
  return RT_EOK;
}

// 线程函数
static void thread_rtu_entry(void *parameter)
{
	rt_err_t ret = RT_ERROR;    
	uint8_t state = 0;          // 状态机变量
	uint8_t cnt = 0;            // 收到的数据字节数
	uint8_t rx_buffer[64];      // 接收数据缓冲区
	uint8_t* ptr = rx_buffer;   // 接收数据缓冲区指针
	uint16_t CRC_code;          // CRC校验位
	uint8_t CRC_code_byte[2];   // CRC校验位(字符格式)
	char receive_cmd[128];
	
	while(1)
	{
		switch(state)
		{
			case 0:
				// 程序在此等待直到收到一个信号量，表明串口接收缓冲区有数据
				rt_sem_take(&rtu_rx_sem, RT_WAITING_FOREVER);
				// 收到一个信号后便需要读取一个字节，这样信号量数量和字节数量可以保持一致
				rt_device_read(rtu, -1, ptr, 1);
				ptr ++;
				cnt ++;
				// 更新状态机到下一个状态
				state = 10;
				break;
			
			case 10:
				// 循环接收信号量，如果有信号量则说明缓冲区还有字符，如果超过WAIT_2T时间没有收到信号量，则表明一帧数据结束
				// 根据RTU协议，WAIT_2T应该要超过1.5个字符发送时间，一个字符是8位，外加起始和终止位，则一个字符为10位，根据波特率位9600，可以算出
			  // 1.5个字符时间为：1/9600 * 10 * 1.5 = 0.0015625s = 1.6ms, 经测试3ms时间比较稳定
				ret = rt_sem_take(&rtu_rx_sem, WAIT_2T); 
				if(ret == RT_EOK) // 如果收到信号量
				{
					rt_device_read(rtu, -1, ptr, 1); // 读取缓冲区字符
					ptr ++;
					cnt ++;
				}
				else // 如果没有收到信号量，那么说明一帧数据结束
				{
					// 一帧数据结束，检查CRC校验位是否正确
					CRC_code = CheckCRC(rx_buffer, cnt - 2); // 生成CRC校验位
					CRC_code_byte[0] = CRC_code >> 8;        
					CRC_code_byte[1] = CRC_code & 0x00FF;
					if(!rt_memcmp(CRC_code_byte, rx_buffer + cnt - 2, 2)) // 如果生成的校验位与收到的校验位一致
					{
						//rt_kprintf("Get modbus rtu msg, CRC: %x  \n", CRC_code);
						Process(rx_buffer, cnt, rtu);     // 根据mosbus RTU协议来对相应的线圈和寄存器进行读写操作
						PostProcess(rx_buffer, cnt); // 根据收到的modbus字符串，执行相应的命令
					}
					else
					{
						//rt_kprintf("Modbus rtu communication error.  \n");
						//Uint8ArrToHexCharStr(rx_buffer, cnt, receive_cmd);
						//rt_kprintf("Received Str: %s  \n", receive_cmd);
					}

					ptr = rx_buffer; // 重置数据缓冲区指针
					cnt = 0;
					state = 0; 			 // 重置状态机
				}
				break;
			
			default:
				break;
		}
	}
}

// 初始化与串口屏RTU通讯的线程
int rtu_init()
{
	struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
	
	// 初始化串口接收信号量
	rt_sem_init(&rtu_rx_sem, "rtu_rx_sem", 0, RT_IPC_FLAG_FIFO);
	
	// 初始化通讯硬件串口
	rtu = rt_device_find(RTU_SERIAL);
	if (!rtu)
	{
     rt_kprintf("find rtu serial failed!\n");
     return RT_ERROR;		
  }
	rt_device_open(rtu, RT_DEVICE_FLAG_INT_RX);
	config.baud_rate = RTU_BAUD_RATE;
	rt_device_control(rtu, RT_DEVICE_CTRL_CONFIG, &config);
	rt_device_set_rx_indicate(rtu, rtu_rx);
	
	// 初始化通讯线程
	rt_thread_t thread_rtu = rt_thread_create("thread_rtu", thread_rtu_entry, RT_NULL, 4096, RTU_PRIORITY, 10);
	if (thread_rtu != RT_NULL)
  {
     rt_thread_startup(thread_rtu);
  }
  else
  {
		 rt_kprintf("RTU Thread Create Fail!\n");
     return RT_ERROR;
  }
	
	// the end
	return RT_EOK;
}

