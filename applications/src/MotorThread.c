#include "miscellaneousTools.h"
#include "GlobalVariable.h" 

/*---------静态变量----------*/
static struct rt_semaphore motro_rx_sem; // 串口接收信号量

// 串口接收回调函数
static rt_err_t motor_rx(rt_device_t dev, rt_size_t size)
{
	// 每收到一个byte便释放一个信号量，这样信号量的数量便是收到的字节数量
	rt_sem_release(&motro_rx_sem);
  return RT_EOK;
}

// 线程函数
static void thread_motor_entry(void *parameter)
{
	rt_err_t ret = RT_ERROR;    
	uint8_t state = 0;          // 状态机
	uint8_t cnt = 0;            // 数据长度
	uint8_t rx_buffer[32];      // 数据接收缓存
	uint8_t* ptr = rx_buffer;   // 数据缓存指针
	uint8_t rx_buffer_str[128]; 
	
	while(1)
	{
		switch(state)
		{
			case 0:
				// 程序在此等待直到收到一个信号量，表明串口接收缓冲区有数据
				rt_sem_take(&motro_rx_sem, RT_WAITING_FOREVER);
				// 收到一个信号后便需要读取一个字节，这样信号量数量和字节数量可以保持一致
				rt_device_read(motor, -1, ptr, 1);
				//rt_kprintf("Incoming data: %x", *ptr);
				ptr ++;
				cnt ++;
				// 更新状态机到下一个状态
				state = 10;
				break;
			
			case 10:
				// 循环接收信号量，如果有信号量则说明缓冲区还有字符，如果超过 WAIT_BYTE 时间没有收到信号量，则表明一帧数据结束
				ret = rt_sem_take(&motro_rx_sem, WAIT_BYTE); 
				if(ret == RT_EOK) // 如果收到信号量
				{
					rt_device_read(motor, -1, ptr, 1); // 读取缓冲区字符
					ptr ++;
					cnt ++;
				}
				else // 如果没有收到信号量，那么说明一帧数据结束
				{
					switch(rx_buffer[0]) // 解析收到的第一个字节
					{
						case 0xE0:
						case 0x3F:
						case 0xFF:
							rt_kprintf("Maybe Power On Ack: ");
							DecToHex(rx_buffer, rx_buffer_str, cnt); // 将电机版本号打印出来
							rt_kprintf("%s\n", rx_buffer_str);
							rt_event_send(Schedule, GET_MOTOR_VERSION); // 设置事件，表明电机上电成功，可以往电机载入上一次掉电时的编码值
							break;
						
						case MOTOR_ADDR: // 电机内部的SCL指令地址
							//rt_kprintf("Motor: Now acking... \n");
							*ptr = '\0';				
							SendTxtMail(ack, rx_buffer); // 将电机的应答发送到邮箱以便其他线程使用
							//rt_kprintf("Get ack: %s \n", rx_buffer);
							break;
						
						default:
							break;
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

// 初始化与电机通讯的线程
int motor_init()
{
	struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
	
	// 初始化串口接收信号量
	rt_sem_init(&motro_rx_sem, "motro_rx_sem", 0, RT_IPC_FLAG_FIFO);
	
	// 初始化通讯邮箱
	ack = rt_mb_create("ack", 1, RT_IPC_FLAG_FIFO);          
  if (ack == RT_NULL)
  {
     rt_kprintf("create ack mailbox failed.\n");
     return RT_ERROR;
  }
	
	// 初始化通讯硬件串口，波特率设置为MOTOR_BAUD_RATE
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
	
	// 初始化通讯线程
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

