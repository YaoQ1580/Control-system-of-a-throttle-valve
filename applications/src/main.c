/*
 * 高压气动节流阀
 *
 */

#include "miscellaneousTools.h"
#include "GlobalVariable.h"

/*-----------引用外部的函数----------*/
extern int motor_init(void);           // 初始化电机通讯线程
extern int savepos_init(void);         // 初始化保存位置线程
extern int rtu_init(void);             // 初始化与串口屏通讯的modbusRTU接收线程
extern int upcpu_rtu_init(void);       // 初始化与上位机通讯的modbusRTU接收线程
extern int dataThread_init(void);      // 初始化读取数据线程
extern int PressAdjThread_init(void);  // 初始化出口压力和出口流量控制线程
extern int calivar_init(void);				 // 初始化标定参数线程
extern int TestThread_init(void);

int main(void)
{
		/* 初始化 */
		rt_pin_mode(MOTOR_CTR, PIN_MODE_OUTPUT);   // 初始化电机继电器控制引脚
	
 		i2c_init(); // 初始化I2C
	
		// 初始化事件控制器
		Schedule = rt_event_create("Schedule", RT_IPC_FLAG_PRIO);
		if (Schedule == RT_NULL)
		{
			rt_kprintf("fail to create Schedule !!!\n");
			return RT_ERROR;
		}
		Schedule2 = rt_event_create("Schedule2", RT_IPC_FLAG_PRIO);
		if (Schedule2 == RT_NULL)
		{
			rt_kprintf("fail to create Schedule2 !!!\n");
			return RT_ERROR;
		}
	
		MotorBus = rt_mutex_create("MotorBus",RT_IPC_FLAG_FIFO);  // 初始化电机互锁量
		if (MotorBus == RT_NULL)
		{
			rt_kprintf("fail to create MotorBus mutex !!!\n");
			return RT_ERROR;
		}
		
		I2CBus = rt_mutex_create("I2CBus",RT_IPC_FLAG_FIFO);  // 初始化I2C通信互锁量
		if (I2CBus == RT_NULL)
		{
			rt_kprintf("fail to create I2CBus mutex !!!\n");
			return RT_ERROR;
		}
		
		motor_init();     			// 初始化电机通讯线程
		savepos_init();   			// 初始化保存位置线程
		rtu_init();       			// 初始化与串口屏RTU通讯线程
		calivar_init();
		upcpu_rtu_init();       // 初始化与上位机RTU通讯线程
		dataThread_init();			// 初始化数据更新线程
		PressAdjThread_init();	// 初始化出口压力和出口流量控制线程
		
		//往返测试线程（正式交接时应该注释掉或者删去）
		TestThread_init();
		
    return RT_EOK;
}
