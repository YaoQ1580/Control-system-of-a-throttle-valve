/*
 * ��ѹ����������
 *
 */

#include "miscellaneousTools.h"
#include "GlobalVariable.h"

/*-----------�����ⲿ�ĺ���----------*/
extern int motor_init(void);           // ��ʼ�����ͨѶ�߳�
extern int savepos_init(void);         // ��ʼ������λ���߳�
extern int rtu_init(void);             // ��ʼ���봮����ͨѶ��modbusRTU�����߳�
extern int upcpu_rtu_init(void);       // ��ʼ������λ��ͨѶ��modbusRTU�����߳�
extern int dataThread_init(void);      // ��ʼ����ȡ�����߳�
extern int PressAdjThread_init(void);  // ��ʼ������ѹ���ͳ������������߳�
extern int calivar_init(void);				 // ��ʼ���궨�����߳�
extern int TestThread_init(void);

int main(void)
{
		/* ��ʼ�� */
		rt_pin_mode(MOTOR_CTR, PIN_MODE_OUTPUT);   // ��ʼ������̵�����������
	
 		i2c_init(); // ��ʼ��I2C
	
		// ��ʼ���¼�������
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
	
		MotorBus = rt_mutex_create("MotorBus",RT_IPC_FLAG_FIFO);  // ��ʼ�����������
		if (MotorBus == RT_NULL)
		{
			rt_kprintf("fail to create MotorBus mutex !!!\n");
			return RT_ERROR;
		}
		
		I2CBus = rt_mutex_create("I2CBus",RT_IPC_FLAG_FIFO);  // ��ʼ��I2Cͨ�Ż�����
		if (I2CBus == RT_NULL)
		{
			rt_kprintf("fail to create I2CBus mutex !!!\n");
			return RT_ERROR;
		}
		
		motor_init();     			// ��ʼ�����ͨѶ�߳�
		savepos_init();   			// ��ʼ������λ���߳�
		rtu_init();       			// ��ʼ���봮����RTUͨѶ�߳�
		calivar_init();
		upcpu_rtu_init();       // ��ʼ������λ��RTUͨѶ�߳�
		dataThread_init();			// ��ʼ�����ݸ����߳�
		PressAdjThread_init();	// ��ʼ������ѹ���ͳ������������߳�
		
		//���������̣߳���ʽ����ʱӦ��ע�͵�����ɾȥ��
		TestThread_init();
		
    return RT_EOK;
}
