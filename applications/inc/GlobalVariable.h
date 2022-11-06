#ifndef GLOBALVARIABLE
#define GLOBALVARIABLE

#include <rtthread.h>
#include <rtdevice.h>

#ifndef DEF
#define EXT extern
#else
#define EXT 
#endif

/*-------------modbusRTUЭ�����-------------*/
#define REG_NUM 150
EXT uint32_t coil;              			// modbusRTU����Ȧ
EXT uint16_t Reg[REG_NUM];           			// modbusRTU�Ĵ���

/*-----------RS485ͨѶ�豸------------*/
EXT rt_device_t rtu;            			// ���������ڲ������
EXT rt_device_t motor; 								// ���ͨѶ���ڲ������
EXT rt_device_t upcpu_rtu;        		// ��λ��ͨѶ���ڲ������

/*------------ADCģ���źŲɼ��豸----------*/
EXT rt_adc_device_t adc_dev;					// ģ���źŶ�ȡ�豸

/*--------------rt thread�߳�ͨѶ���--------------*/
EXT rt_mailbox_t ack;  								// ���Ӧ������
EXT rt_mutex_t MotorBus;  	 					// ���ͨѶ������
EXT rt_mutex_t I2CBus;  	 						// EEPROME ��д������
EXT rt_event_t Schedule;     					// �����¼��ļ���
EXT rt_event_t Schedule2;							// Schedule �������ˣ���չ���µ��¼�����

/*--------------����ȫ�ֱ���--------------*/
EXT float coeff_env_to_valve;   			// �ӱ�����ֵת������оλ�õ�ת��ϵ��
EXT float adc_to_press_outlet;  			// ��ADC�Ķ���ת��������ѹ����ת��ϵ��
EXT float adc_to_flux_outlet;   			// ��ADC�Ķ���ת��������������ת��ϵ��
EXT float adc_to_press_inlet;   			// ��ADC�Ķ���ת�������ѹ����ת��ϵ��
EXT float press_threshold;      			// ����ѹ��PID������ֵ�����������Χ�Ž���PID����
EXT float flux_threshold;       			// ��������PID������ֵ�����������Χ�Ž���PID����
EXT float press_inlet_enc_low;			// �±궨�㴦�����ѹ������ֵ
EXT float press_inlet_phy_low;				// �±궨�㴦�����ѹ����������ֵ
EXT float press_inlet_enc_high;		// �ϱ궨�㴦�����ѹ������ֵ
EXT float press_inlet_phy_high;				// �ϱ궨�㴦�����ѹ����������ֵ
EXT float press_outlet_enc_low;		// �±궨�㴦�ĳ���ѹ������ֵ
EXT float press_outlet_phy_low;				// �±궨�㴦�ĳ���ѹ����������ֵ
EXT float press_outlet_enc_high;		// �ϱ궨�㴦�ĳ���ѹ������ֵ
EXT float press_outlet_phy_high;			// �ϱ궨�㴦�ĳ���ѹ����������ֵ
EXT float flux_enc_low;						// �±궨�㴦����������������ֵ
EXT float flux_phy_low;								// �±궨�㴦��������������������ֵ
EXT float flux_enc_high;						// �ϱ궨�㴦����������������ֵ
EXT float flux_phy_high;							// �ϱ궨�㴦��������������������ֵ
EXT float last_press_min;							// ��һ�μ�¼��ѹ������������
EXT float last_press_max;							// ��һ�μ�¼��ѹ������������
EXT float last_flux_min;							// ��һ�μ�¼����������������
EXT float last_flux_max;							// ��һ�μ�¼����������������

#endif

