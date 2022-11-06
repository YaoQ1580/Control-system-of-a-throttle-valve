#ifndef MISCELLAN
#define MISCELLAN

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

/*----------辅助函数----------*/
void StrToHex(char* src, uint32_t* dst);              		// src表示一串字符，该函数将src视作16进制数，并将其从字符形式转为无符号数值，举例：src = "010", 则dst = 0x10 = 16;
void Uint8ArrToHexCharStr(uint8_t* src, int cnt, char* dst); // src表示一个uint8_t的数组，cnt表示这个数组的长度，该函数将src里面的内容转化成hex形式的字符串，例：src[] = "0x01, 0x02, 0x03"，那么dst[] = "01 02 03";
void DecToHex(uint8_t* src, uint8_t* trg, uint16_t len); // src表示一组十进制数据，该函数将src转化成字符形式，举例：src[3] = {0 ,1, 2}，则trg[] = "0 1 2"，len = 3；
void itob(int32_t src, uint8_t* dst);                    // 将32位数据分割成4个8位数据，举例：src = 0x12345678，则dst[] = {0x78, 0x56, 0x34, 0x12};
int32_t btoi(uint8_t* src);                              // 将4个8位数据转化成一个32位数据，举例src[] = {0x78, 0x56, 0x34, 0x12}，则函数返回0x12345678;

/*----------事件---------*/
// Schedule 的事件集合
#define POWER_ON								 0x01
#define POWER_OFF								(0x01) << 1
#define GET_MOTOR_VERSION 	  	(0x01) << 2   // 电机返回版本号的标志
#define THREAD_GET_WORK    			(0x01) << 3  	// 从EEPROME读取常量成功标志
#define CALI_POS         				(0x01) << 4  	// 标定当前位置命令标志
#define SET_PRESS_IN     				(0x01) << 5   // 设定入口压力命令标志
#define SET_PRESS_OUT    				(0x01) << 6   // 设定出口压力命令标志
#define SET_FLUX         				(0x01) << 7   // 设定出口流量命令标志
#define QUIT_PID         				(0x01) << 8   // 退出PID控制模式命令标志
#define MOV_STA          				(0x01) << 9   // 电机是否在运动标志
#define CALI_PRESENT_FLUX				(0x01) << 10  // 标定当前流量
#define CALI_PRESENT_OUT_PRESS  (0x01) << 11  // 标定当前出口压力
#define CALI_PRESENT_IN_PRESS		(0x01) << 12  // 标定当前入口压力
#define SET_PRESS_PID						(0x01) << 13  // 标定出口压力调节PID
#define SET_FLUX_PID						(0x01) << 14  // 标定出口流量调节PID
#define SET_PRESS_SENSOR_MAX		(0x01) << 15  // 标定出口压力传感器上限
#define SET_PRESS_SENSOR_MIN		(0x01) << 16  // 标定出口压力传感器下限
#define SET_FLUX_SENSOR_MAX			(0x01) << 17  // 标定出口流量传感器上限
#define SET_FLUX_SENSOR_MIN			(0x01) << 18  // 标定出口流量传感器下限
#define SHUT_DOWN_EVENT					(0x01) << 19  // 开始进行电机往返位移测试
#define RESET_MOTOR							(0x01) << 20
#define MAN_MOVE_UP							(0x01) << 21
#define MAN_MOVE_DOWN						(0x01) << 22
#define MAN_MOVE_STOP						(0x01) << 23
#define CHANGE_MAN_MOVE_SPEED		(0x01) << 24
#define CHANGE_FEED_MODE_SPEED  (0x01) << 25
#define CHANGE_CURRENT					(0x01) << 26
#define CHANGE_MAN_ACC					(0x01) << 27
#define CHANGE_FEED_ACC					(0x01) << 28
#define CHANGE_PRESS_THRESHOLD	(0x01) << 29
#define CHANGE_FLUX_THRESHOLD		(0x01) << 30
#define MOVE_TO_TRG_POS					(uint32_t)(0x01) << 31

// Schedule2 的事件集合
#define START_TEST							 0x01
#define MOTOR_DIRECT_CMD				(0x01) << 1

/*-------------线程优先级---------------*/
#define UP_CPU_PRIORITY     12  // 与上位机进行modbus通讯的线程优先级
#define RTU_PRIORITY      	13  // 与串口屏进行modbus通讯的线程优先级
#define SAVEPOS_PRIORITY 		17  // 保存位置线程优先级
#define MOTOR_PRIORITY 			16 	// 电机通讯线程优先级
#define DATA_PRIORITY     	18  // 数据更新线程优先级
#define PRESS_ADJ_PRIORITY  19  // 出口流量和出口压力控制线程优先级
#define CALIVAR_PRIORITY    15  // 标定参数线程优先级
#define TEST_PRIORITY    		20  // 电机往返运动测试线程的优先级（因为仅仅是测试所以不需要很高优先级）

/*------------I2C变量和函数------------*/
#define AT24C02_ADDR 						0x50    // 板载EEPROME的I2C地址
#define I2C_BUS_NAME    				"i2c1" 
#define I2C_WRITE_DELAY         5				// I2C写缓冲区延时

// EEPROME内部存储的变量地址
#define ENCODER_ADDR      			0x00  // int型，电机编码器数值
#define OUTLET_PRESS_MIN_ADDR 	0x04  // float型，出口压力下限
#define OUTLET_PRESS_MAX_ADDR   0x08  // float型，出口压力上限
#define OUTLET_FLUX_MIN_ADDR  	0x0C  // float型，出口流量下限
#define OUTLET_FLUX_MAX_ADDR  	0x10  // float型，出口流量上限
#define ENC_TO_POS  						0x14  // float型，从电机编码值转换成阀芯位置的转换系数
#define ADC_TO_PRESS_OUTLET     0x18  // float型，从ADC编码器转换成出口压力的转换系数
#define ADC_TO_FLUX_OUTLET      0x1C  // float型，从ADC编码值转换成出口流量的转换系数
#define ADC_TO_PRESS_INLET      0x20  // float型，从ADC编码值转换成入口压力的转换系数
#define OUTPRESS_P              0x24  // float型，出口压力PID调节的P值
#define OUTPRESS_I              0x28  // float型，出口压力PID调节的I值
#define OUTPRESS_D              0x2C  // float型，出口压力PID调节的D值
#define OUTFLUX_P              	0x30  // float型，出口流量PID调节的P值
#define OUTFLUX_I              	0x34  // float型，出口流量PID调节的I值
#define OUTFLUX_D              	0x38  // float型，出口流量PID调节的D值
#define EE_PRESS_CTR_THRESHOLD  0x3C  // float型，出口压力控制阈值
#define EE_FLUX_CTR_THRESHOLD   0x40  // float型，出口流量控制阈值
#define PRESS_IN_ENC_LOW        0x44  // int型，入口压力编码值的下标定点
#define PRESS_IN_PHY_LOW        0x48  // float型，入口压力物理值的下标定点
#define PRESS_IN_ENC_HIGH       0x4C  // int型，入口压力编码值的上标定点
#define PRESS_IN_PHY_HIGH       0x50  // float型，入口压力物理值的上标定点
#define PRESS_OUT_ENC_LOW       0x54  // int型，出口压力编码值的下标定点
#define PRESS_OUT_PHY_LOW       0x58  // float型，出口压力物理值的下标定点
#define PRESS_OUT_ENC_HIGH      0x5C  // int型，出口压力编码值的上标定点
#define PRESS_OUT_PHY_HIGH      0x60  // float型，出口压力物理值的上标定点
#define FLUX_ENC_LOW        		0x64  // int型，流量传感器编码值的下标定点
#define FLUX_PHY_LOW         		0x68  // float型，流量传感器物理值的下标定点
#define FLUX_ENC_HIGH        		0x6C  // int型，流量传感器编码值的上标定点
#define FLUX_PHY_HIGH        		0x70  // float型，流量传感器物理值的上标定点

rt_err_t i2c_init(void);                                               // 初始化函数
int WriteInt32ToEEPROME(uint32_t addr, int32_t num);
int WriteFloatToEEPROME(uint32_t addr, float num);
int ReadInt32FromEEPROME(uint32_t addr, int32_t* num);
int ReadFloatFromEEPROME(uint32_t addr, float* num);
rt_err_t write_AT24C02(uint32_t addr, int32_t num);                // 将int型的数值num写入起始地址为addr的EEPROME存储区，least significant字节在先
rt_err_t write_AT24C02_float(uint32_t addr, float num);            // 将float型的数值num写入起始地址为addr的EEPROME存储区，least significant字节在先
rt_err_t read_AT24C02(uint32_t addr, int32_t* dst);                // 从起始地址为addr的EEPROME存储区读取四个字节并将其按照int型对待
rt_err_t read_AT24C02_float(uint32_t addr, float* dst);            // 从起始地址为addr的EEPROME存储区读取四个字节并将其按照float型对待
int WriteFloatRegToEeprome(uint32_t reg_addr, uint32_t ee_addr);  // 将modbus寄存器中的float数值写入起始地址为addr的EEPROME存储区

/*-----------------------继电器-----------------------*/
#define MOTOR_CTR   GET_PIN(C, 7)     // 电机通电继电器控制引脚 PC7

/*-------------------Modbus RTU----------------------*/
#define RTU_SERIAL      		 "uart5"   // 与串口屏通讯串口的硬件名称
#define WAIT_2T							 3         // modbusRTU通讯规定的1.5个字符时间
#define RTU_BAUD_RATE        BAUD_RATE_57600    

#define UPCPU_RTU_SERIAL     "uart3"   // 与上位机通讯串口的硬件名称
#define UPCPU_WAIT_2T				 3         // modbusRTU通讯规定的1.5个字符时间
#define UPCPU_RTU_BAUD_RATE  BAUD_RATE_57600   

// modbusRTU寄存器索引
#define POWER_UP_PROGRESS   			0   // 上电进度
#define STALL_DETECTION     			1   // 堵转检测：0表示不堵转，1表示堵转
#define PRESENT_VALVE_POS   			2   // 当前阀芯位置 
#define PRESENT_OUTLET_PRESS  		4   // 当前出口压力
#define PRESENT_OUTLET_FLUX   		6   // 当前出口流量
#define PRESENT_INLET_PRESS  		  8   // 当前入口压力
#define DEF_PRESENT_POS       		10  // 设定当前阀芯位置数值
#define DEF_OUTLET_PRESS_MIN  		12  // 设定压力传感器下限
#define DEF_OUTLET_PRESS_MAX  		14  // 设定压力传感器上限
#define DEF_OUTLET_FLUX_MIN   		16  // 设定流量传感器下限
#define DEF_OUTLET_FLUX_MAX   		18  // 设定流量传感器上限
#define TRG_OUTLET_PRESS      		20  // 设定目标出口压力
#define TRG_OUTLET_FLUX       		22  // 设定目标出口流量
#define OUTLET_PRESS_P        		24  // 设定出口压力PID调节的P
#define OUTLET_PRESS_I        		26  // 设定出口压力PID调节的I
#define OUTLET_PRESS_D        		28  // 设定出口压力PID调节的D
#define OUTLET_FLUX_P         		30  // 设定出口流量PID调节的P
#define OUTLET_FLUX_I         		32  // 设定出口流量PID调节的I
#define OUTLET_FLUX_D         		34  // 设定出口流量PID调节的D
#define CALI_PRESENT_OUTLET_PRESS	36  // 设定当前出口压力
#define CALI_PRESENT_OUTLET_FLUX 	38  // 设定当前出口流量
#define CALI_PRESENT_INLET_PRESS 	40  // 设定当前入口压力
#define MANUAL_SPEED              42  // 手动调节阀芯位移时的速度
#define PID_SPEED                 44  // PID调节时的速度
#define CURRENT                   46  // 电机运行电流
#define MANUAL_ACC              	48  // 手动调节阀芯位移时的加速度
#define PID_ACC                 	50  // PID调节时的加速度
#define OUT_PRESS_CTR_THRESHOLD   52  // 出口压力控制阈值
#define OUT_FLUX_CTR_THRESHOLD    54  // 出口流量控制阈值
#define OUTLET_PRESS_RAW          56  // 当前出口压力的ADC编码数值（0-4095）
#define OUTLET_FLUX_RAW           58  // 当前出口流量的ADC编码数值（0-4095）
#define INLET_PRESS_RAW           60  // 当前入口压力的ADC编码数值（0-4095）
#define TRG_VALVE_POS	            62  // 目标位置（0-10mm）
#define COMMUNICATION_STA	        64  // 通讯报警（1表示正常，0表示出现问题）
#define MOTOR_CMD_STR							71	// 从显示屏幕直接下发的电机指令
#define MOTOR_RET_STR							102	// 电机返回的指令存储起始寄存器

// modbusRTU线圈位定义
#define OPS_POWER_ACTION        	0   // 开关机指令
#define OPS_MOTOR_ENABLE        	1   // 电机重新使能指令
#define OPS_PRESENT_POS     			2   // 设定当前位置指令
#define OPS_MANUALLY_VALVE_UP   	3   // 手动调节阀芯位置上移
#define OPS_MANUALLY_VALVE_DOWN 	4   // 手动调节阀芯位置下移
#define OPS_OUTLET_PRESS_MIN 			5   // 设定出口压力下限指令
#define OPS_OUTLET_PRESS_MAX 			6   // 设定出口压力上限指令
#define OPS_OUTLET_FLUX_MIN 			7   // 设定出口流量下限指令
#define OPS_OUTLET_FLUX_MAX 			8   // 设定出口流量上限指令
#define OPS_TRG_OUTLET_PRESS      9   // 设定目标出口压力指令
#define OPS_TRG_OUTLET_FLUX       10  // 设定目标出口流量指令
#define OPS_OUTLET_PRESS_PID      11  // 设定出口压力调节PID指令
#define OPS_OUTLET_FLUX_PID       12  // 设定出口流量调节PID指令
#define OPS_QUIT_PID       				13  // 退出PID调节模式指令
#define OPS_PRESENT_OUTLET_PRESS  14  // 设定当前出口压力指令
#define OPS_PRESENT_OUTLET_FLUX   15  // 设定当前出口流量指令
#define OPS_PRESENT_INLET_PRESS   16  // 设定当前入口压力指令
#define OPS_MANUAL_SPEED			    17  // 设定电机手动调节时的速度
#define OPS_FEED_SPEED   					18  // 设定电机PID模式下运行时的速度
#define OPS_CURRENT   						19  // 设定电机运行电流
#define OPS_MANUAL_ACC   					20  // 设定电机手动调节时的加速度
#define OPS_FEED_ACC   						21  // 设定电机PID模式下运行时的加速度
#define OPS_SET_PRESS_THRESHOLD   22  // 设定压力控制阈值 
#define OPS_SET_FLUX_THRESHOLD    23  // 设定流量控制阈值
#define OPS_TRG_POS        				24  // 开始运动到目标位置
#define OPS_TEST        					25  // 开始进行电机往返运动测试
#define OPS_SHUT_DOWN             26  // 电机急停
#define OPS_MOTOR_DEBUG_MODE			27	// 进入电机指令透传模式
#define OPS_SEND_DIR_MOTOR_CMD		28	// 透传模式下直接给电机发指令

uint16_t CheckCRC(uint8_t *pData, uint8_t siLen); 					 // CRC校验位生成函数
uint16_t GetShortFromReg(uint8_t index);          					 // 从起始索引为index的modbus寄存器中读取short类型的数
uint32_t GetIntFromReg(uint8_t index);            					 // 从起始索引为index的modbus寄存器中读取int类型的数（要读两个寄存器）
float GetFloatFromReg(uint8_t index);             					 // 从起始索引为index的modbus寄存器中读取float类型的数（要读两个寄存器）
void WriteShortToReg(uint8_t index, int16_t num); 					 // 从起始索引为index的modbus寄存器中写入short类型的数
void WriteIntToReg(uint8_t index, int32_t num);   					 // 从起始索引为index的modbus寄存器中写入int类型的数（要写两个寄存器）
void WriteFloatToReg(uint8_t index, float num);   					 // 从起始索引为index的modbus寄存器中写入float类型的数（要写两个寄存器）
void Process(uint8_t* src, uint8_t len, rt_device_t handle); // 根据modbus协议进行读写线圈和寄存器的操作
void PostProcess(uint8_t* src, uint8_t len);      					 // 将寄存器与线圈值解析成相应的命令

/*--------------电机通讯相关------------------*/
#define MOTOR_SERIAL      "uart4"  				// 与电机通讯串口名称 
#define MOTOR_ADDR        '1'      				// 电机内部的SCL指令地址
#define MOTOR_BAUD_RATE   BAUD_RATE_57600 // 电机波特率
#define WAIT_BYTE 				3 			 				// 等待电机返回一个字节的超时时间
#define WAIT_MAIL     		50 			 				// 从给电机发送指令到完整接收电机返回一帧数据的超时时间
#define WAIT_485          200			 				// 等待与电机通讯互锁量释放的超时时间
#define CHECK_MAIL    		3  			 				// 与电机通讯失败后的再次通讯尝试次数

void SendTxtMail(rt_mailbox_t mail, uint8_t* src);   // 将数据串指针写入线程间通讯邮箱
int ToMotorNoReturnNoParameter(char* cmd);                     // 将指令发送给电机（无应答，无参数）
int ToMotorNoReturnWithParameter(char* cmd, char* parameter);  // 将指令发送给电机（无应答，有参数）
int ToMotorWithReturn(char* cmd, char* dst);      // 将指令发送给电机（有应答）
int GetFloatFromMotor(char* cmd, float* f_value);    // 从电机获取应答并将应答解释为float型

/*-----------------保存电机编码器位置相关----------------*/
#define MOTOR_INITIAL     2500 // 等待电机固件初始化时间
#define SC_REFRESH_RATE   50   // 读取电机 status code 的间隔时间
#define CHECK_MOV         2    // 根据 status code 检查电机是否正在移动的检查次数
#define CHECK_STOP        2    // 根据 status code 检查电机是否已经停止的检查次数

void ReadEncoderDataToMotor(void);                // 从EEPROME读取编码器数值并写入电机
int GetEnvFromMotor(int32_t* env_data);           // 从电机读取int型的编码器数值
void SaveEncoderData(void);                       // 将电机编码器数值保存进EEPROME

/*---------------更新数据相关----------------*/
#define REFRESH_BASE      10   // 时间基准(单位：ms)
#define IE_TICK           5    // 每 IE_TICK 个时间基准刷新一次当前位置
#define ADC_TICK        	2    // 每 ADC_TICK 个时间基准刷新一次ADC数据

/*----------------------ADC相关-----------------------*/
#define ADC_DEV_NAME      		"adc1"
#define PRESS_OUT_CHANNEL   	14   		// 出口压力频道(PC4) 	
#define FLUX_CHANNEL   				15   		// 出口流量频道(PC5)
#define PRESS_IN_CHANNEL   		8    		// 入口压力频道(PB0)
#define PRESS_IN_RANGE_HIGH   3669.0  // ADC读数范围(对应3V)
#define PRESS_IN_RANGE_LOW		0.0			// 对应0V
#define PRESS_OUT_RANGE_HIGH  3668.0  // ADC读数范围(对应3V)
#define PRESS_OUT_RANGE_LOW		0.0			// 对应0V
#define FLUX_RANGE_HIGH   		3677.0  // ADC读数范围(对应3V)
#define FLUX_RANGE_LOW				0.0			// 对应0V
#define MOV_AVG_NUM       		64    		// 滑动平均滤波所用的ADC数据个数 

/*----------------出口流量和压力调节相关---------------*/
#define PID_UPDATE_CNT    			10    // PID_UPDATE_CNT 个时间基准进行一次PID控制

/*----------------其他辅助函数---------------*/
void DataInitialize(void);					// savepos线程从EEPROME读取数据 以及 从电机读取参数
void FeedLenEnc(int32_t enc_val);    // 运动到特定的电机编码位置 
void FeedPosValve(float pos);				// 运动到特定的阀芯位置
void CaliPresentADC(uint32_t now_enc_val_addr, uint32_t userSet_phy_val_addr, uint32_t phy_upperBound_addr, uint32_t phy_lowerBound_addr, uint32_t EncHigh_eeaddr, uint32_t PhyHigh_eeaddr, uint32_t EncLow_eeaddr, uint32_t PhyLow_eeaddr, uint32_t coeff_addr, float* EncHigh, float* PhyHigh, float* EncLow, float* PhyLow, float* convert_coeff);
int CaliSensor(uint32_t bound_reg_addr, uint32_t bound_ee_addr, uint32_t convert_coeff_ee_addr, float* old_lower_bound, float* old_upper_bound, float* convert_coeff, int type); // 标定传感器的上下限
void SetThreshold(uint32_t reg_addr, uint32_t ee_addr, float* threshold);

/*----------------其他辅助宏-----------------*/
#define SPLITE_POINT 		0.4						// 传感器标定上下限的分割点
#define CHUAN_DONG_BI		62						// 蜗轮蜗杆传动比
#define DAO_CHENG				4							// 丝杠导程
#define EVC_RESOLUTION  4000					// 电机转一圈编码器走的步数
#define IP_TO_EP				5							// 电机编码器电子齿轮的传动比，对 FL 和 FP 有效

#endif

