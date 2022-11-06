#ifndef GLOBALVARIABLE
#define GLOBALVARIABLE

#include <rtthread.h>
#include <rtdevice.h>

#ifndef DEF
#define EXT extern
#else
#define EXT 
#endif

/*-------------modbusRTU协议变量-------------*/
#define REG_NUM 150
EXT uint32_t coil;              			// modbusRTU的线圈
EXT uint16_t Reg[REG_NUM];           			// modbusRTU寄存器

/*-----------RS485通讯设备------------*/
EXT rt_device_t rtu;            			// 串口屏串口操作句柄
EXT rt_device_t motor; 								// 电机通讯串口操作句柄
EXT rt_device_t upcpu_rtu;        		// 上位机通讯串口操作句柄

/*------------ADC模拟信号采集设备----------*/
EXT rt_adc_device_t adc_dev;					// 模拟信号读取设备

/*--------------rt thread线程通讯相关--------------*/
EXT rt_mailbox_t ack;  								// 电机应答邮箱
EXT rt_mutex_t MotorBus;  	 					// 电机通讯互锁量
EXT rt_mutex_t I2CBus;  	 						// EEPROME 读写互锁量
EXT rt_event_t Schedule;     					// 各种事件的集合
EXT rt_event_t Schedule2;							// Schedule 不够用了，拓展了新的事件集合

/*--------------其余全局变量--------------*/
EXT float coeff_env_to_valve;   			// 从编码器值转换到阀芯位置的转换系数
EXT float adc_to_press_outlet;  			// 从ADC的读数转换到出口压力的转换系数
EXT float adc_to_flux_outlet;   			// 从ADC的读数转换到出口流量的转换系数
EXT float adc_to_press_inlet;   			// 从ADC的读数转换到入口压力的转换系数
EXT float press_threshold;      			// 出口压力PID调节阈值，超出这个范围才进行PID调节
EXT float flux_threshold;       			// 出口流量PID调节阈值，超出这个范围才进行PID调节
EXT float press_inlet_enc_low;			// 下标定点处的入口压力编码值
EXT float press_inlet_phy_low;				// 下标定点处的入口压力物理量的值
EXT float press_inlet_enc_high;		// 上标定点处的入口压力编码值
EXT float press_inlet_phy_high;				// 上标定点处的入口压力物理量的值
EXT float press_outlet_enc_low;		// 下标定点处的出口压力编码值
EXT float press_outlet_phy_low;				// 下标定点处的出口压力物理量的值
EXT float press_outlet_enc_high;		// 上标定点处的出口压力编码值
EXT float press_outlet_phy_high;			// 上标定点处的出口压力物理量的值
EXT float flux_enc_low;						// 下标定点处的流量传感器编码值
EXT float flux_phy_low;								// 下标定点处的流量传感器物理量的值
EXT float flux_enc_high;						// 上标定点处的流量传感器编码值
EXT float flux_phy_high;							// 上标定点处的流量传感器物理量的值
EXT float last_press_min;							// 上一次记录的压力传感器下限
EXT float last_press_max;							// 上一次记录的压力传感器上限
EXT float last_flux_min;							// 上一次记录的流量传感器下限
EXT float last_flux_max;							// 上一次记录的流量传感器上限

#endif

