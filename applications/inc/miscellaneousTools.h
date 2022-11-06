#ifndef MISCELLAN
#define MISCELLAN

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

/*----------��������----------*/
void StrToHex(char* src, uint32_t* dst);              		// src��ʾһ���ַ����ú�����src����16����������������ַ���ʽתΪ�޷�����ֵ��������src = "010", ��dst = 0x10 = 16;
void Uint8ArrToHexCharStr(uint8_t* src, int cnt, char* dst); // src��ʾһ��uint8_t�����飬cnt��ʾ�������ĳ��ȣ��ú�����src���������ת����hex��ʽ���ַ���������src[] = "0x01, 0x02, 0x03"����ôdst[] = "01 02 03";
void DecToHex(uint8_t* src, uint8_t* trg, uint16_t len); // src��ʾһ��ʮ�������ݣ��ú�����srcת�����ַ���ʽ��������src[3] = {0 ,1, 2}����trg[] = "0 1 2"��len = 3��
void itob(int32_t src, uint8_t* dst);                    // ��32λ���ݷָ��4��8λ���ݣ�������src = 0x12345678����dst[] = {0x78, 0x56, 0x34, 0x12};
int32_t btoi(uint8_t* src);                              // ��4��8λ����ת����һ��32λ���ݣ�����src[] = {0x78, 0x56, 0x34, 0x12}����������0x12345678;

/*----------�¼�---------*/
// Schedule ���¼�����
#define POWER_ON								 0x01
#define POWER_OFF								(0x01) << 1
#define GET_MOTOR_VERSION 	  	(0x01) << 2   // ������ذ汾�ŵı�־
#define THREAD_GET_WORK    			(0x01) << 3  	// ��EEPROME��ȡ�����ɹ���־
#define CALI_POS         				(0x01) << 4  	// �궨��ǰλ�������־
#define SET_PRESS_IN     				(0x01) << 5   // �趨���ѹ�������־
#define SET_PRESS_OUT    				(0x01) << 6   // �趨����ѹ�������־
#define SET_FLUX         				(0x01) << 7   // �趨�������������־
#define QUIT_PID         				(0x01) << 8   // �˳�PID����ģʽ�����־
#define MOV_STA          				(0x01) << 9   // ����Ƿ����˶���־
#define CALI_PRESENT_FLUX				(0x01) << 10  // �궨��ǰ����
#define CALI_PRESENT_OUT_PRESS  (0x01) << 11  // �궨��ǰ����ѹ��
#define CALI_PRESENT_IN_PRESS		(0x01) << 12  // �궨��ǰ���ѹ��
#define SET_PRESS_PID						(0x01) << 13  // �궨����ѹ������PID
#define SET_FLUX_PID						(0x01) << 14  // �궨������������PID
#define SET_PRESS_SENSOR_MAX		(0x01) << 15  // �궨����ѹ������������
#define SET_PRESS_SENSOR_MIN		(0x01) << 16  // �궨����ѹ������������
#define SET_FLUX_SENSOR_MAX			(0x01) << 17  // �궨������������������
#define SET_FLUX_SENSOR_MIN			(0x01) << 18  // �궨������������������
#define SHUT_DOWN_EVENT					(0x01) << 19  // ��ʼ���е������λ�Ʋ���
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

// Schedule2 ���¼�����
#define START_TEST							 0x01
#define MOTOR_DIRECT_CMD				(0x01) << 1

/*-------------�߳����ȼ�---------------*/
#define UP_CPU_PRIORITY     12  // ����λ������modbusͨѶ���߳����ȼ�
#define RTU_PRIORITY      	13  // �봮��������modbusͨѶ���߳����ȼ�
#define SAVEPOS_PRIORITY 		17  // ����λ���߳����ȼ�
#define MOTOR_PRIORITY 			16 	// ���ͨѶ�߳����ȼ�
#define DATA_PRIORITY     	18  // ���ݸ����߳����ȼ�
#define PRESS_ADJ_PRIORITY  19  // ���������ͳ���ѹ�������߳����ȼ�
#define CALIVAR_PRIORITY    15  // �궨�����߳����ȼ�
#define TEST_PRIORITY    		20  // ��������˶������̵߳����ȼ�����Ϊ�����ǲ������Բ���Ҫ�ܸ����ȼ���

/*------------I2C�����ͺ���------------*/
#define AT24C02_ADDR 						0x50    // ����EEPROME��I2C��ַ
#define I2C_BUS_NAME    				"i2c1" 
#define I2C_WRITE_DELAY         5				// I2Cд��������ʱ

// EEPROME�ڲ��洢�ı�����ַ
#define ENCODER_ADDR      			0x00  // int�ͣ������������ֵ
#define OUTLET_PRESS_MIN_ADDR 	0x04  // float�ͣ�����ѹ������
#define OUTLET_PRESS_MAX_ADDR   0x08  // float�ͣ�����ѹ������
#define OUTLET_FLUX_MIN_ADDR  	0x0C  // float�ͣ�������������
#define OUTLET_FLUX_MAX_ADDR  	0x10  // float�ͣ�������������
#define ENC_TO_POS  						0x14  // float�ͣ��ӵ������ֵת���ɷ�оλ�õ�ת��ϵ��
#define ADC_TO_PRESS_OUTLET     0x18  // float�ͣ���ADC������ת���ɳ���ѹ����ת��ϵ��
#define ADC_TO_FLUX_OUTLET      0x1C  // float�ͣ���ADC����ֵת���ɳ���������ת��ϵ��
#define ADC_TO_PRESS_INLET      0x20  // float�ͣ���ADC����ֵת�������ѹ����ת��ϵ��
#define OUTPRESS_P              0x24  // float�ͣ�����ѹ��PID���ڵ�Pֵ
#define OUTPRESS_I              0x28  // float�ͣ�����ѹ��PID���ڵ�Iֵ
#define OUTPRESS_D              0x2C  // float�ͣ�����ѹ��PID���ڵ�Dֵ
#define OUTFLUX_P              	0x30  // float�ͣ���������PID���ڵ�Pֵ
#define OUTFLUX_I              	0x34  // float�ͣ���������PID���ڵ�Iֵ
#define OUTFLUX_D              	0x38  // float�ͣ���������PID���ڵ�Dֵ
#define EE_PRESS_CTR_THRESHOLD  0x3C  // float�ͣ�����ѹ��������ֵ
#define EE_FLUX_CTR_THRESHOLD   0x40  // float�ͣ���������������ֵ
#define PRESS_IN_ENC_LOW        0x44  // int�ͣ����ѹ������ֵ���±궨��
#define PRESS_IN_PHY_LOW        0x48  // float�ͣ����ѹ������ֵ���±궨��
#define PRESS_IN_ENC_HIGH       0x4C  // int�ͣ����ѹ������ֵ���ϱ궨��
#define PRESS_IN_PHY_HIGH       0x50  // float�ͣ����ѹ������ֵ���ϱ궨��
#define PRESS_OUT_ENC_LOW       0x54  // int�ͣ�����ѹ������ֵ���±궨��
#define PRESS_OUT_PHY_LOW       0x58  // float�ͣ�����ѹ������ֵ���±궨��
#define PRESS_OUT_ENC_HIGH      0x5C  // int�ͣ�����ѹ������ֵ���ϱ궨��
#define PRESS_OUT_PHY_HIGH      0x60  // float�ͣ�����ѹ������ֵ���ϱ궨��
#define FLUX_ENC_LOW        		0x64  // int�ͣ���������������ֵ���±궨��
#define FLUX_PHY_LOW         		0x68  // float�ͣ���������������ֵ���±궨��
#define FLUX_ENC_HIGH        		0x6C  // int�ͣ���������������ֵ���ϱ궨��
#define FLUX_PHY_HIGH        		0x70  // float�ͣ���������������ֵ���ϱ궨��

rt_err_t i2c_init(void);                                               // ��ʼ������
int WriteInt32ToEEPROME(uint32_t addr, int32_t num);
int WriteFloatToEEPROME(uint32_t addr, float num);
int ReadInt32FromEEPROME(uint32_t addr, int32_t* num);
int ReadFloatFromEEPROME(uint32_t addr, float* num);
rt_err_t write_AT24C02(uint32_t addr, int32_t num);                // ��int�͵���ֵnumд����ʼ��ַΪaddr��EEPROME�洢����least significant�ֽ�����
rt_err_t write_AT24C02_float(uint32_t addr, float num);            // ��float�͵���ֵnumд����ʼ��ַΪaddr��EEPROME�洢����least significant�ֽ�����
rt_err_t read_AT24C02(uint32_t addr, int32_t* dst);                // ����ʼ��ַΪaddr��EEPROME�洢����ȡ�ĸ��ֽڲ����䰴��int�ͶԴ�
rt_err_t read_AT24C02_float(uint32_t addr, float* dst);            // ����ʼ��ַΪaddr��EEPROME�洢����ȡ�ĸ��ֽڲ����䰴��float�ͶԴ�
int WriteFloatRegToEeprome(uint32_t reg_addr, uint32_t ee_addr);  // ��modbus�Ĵ����е�float��ֵд����ʼ��ַΪaddr��EEPROME�洢��

/*-----------------------�̵���-----------------------*/
#define MOTOR_CTR   GET_PIN(C, 7)     // ���ͨ��̵����������� PC7

/*-------------------Modbus RTU----------------------*/
#define RTU_SERIAL      		 "uart5"   // �봮����ͨѶ���ڵ�Ӳ������
#define WAIT_2T							 3         // modbusRTUͨѶ�涨��1.5���ַ�ʱ��
#define RTU_BAUD_RATE        BAUD_RATE_57600    

#define UPCPU_RTU_SERIAL     "uart3"   // ����λ��ͨѶ���ڵ�Ӳ������
#define UPCPU_WAIT_2T				 3         // modbusRTUͨѶ�涨��1.5���ַ�ʱ��
#define UPCPU_RTU_BAUD_RATE  BAUD_RATE_57600   

// modbusRTU�Ĵ�������
#define POWER_UP_PROGRESS   			0   // �ϵ����
#define STALL_DETECTION     			1   // ��ת��⣺0��ʾ����ת��1��ʾ��ת
#define PRESENT_VALVE_POS   			2   // ��ǰ��оλ�� 
#define PRESENT_OUTLET_PRESS  		4   // ��ǰ����ѹ��
#define PRESENT_OUTLET_FLUX   		6   // ��ǰ��������
#define PRESENT_INLET_PRESS  		  8   // ��ǰ���ѹ��
#define DEF_PRESENT_POS       		10  // �趨��ǰ��оλ����ֵ
#define DEF_OUTLET_PRESS_MIN  		12  // �趨ѹ������������
#define DEF_OUTLET_PRESS_MAX  		14  // �趨ѹ������������
#define DEF_OUTLET_FLUX_MIN   		16  // �趨��������������
#define DEF_OUTLET_FLUX_MAX   		18  // �趨��������������
#define TRG_OUTLET_PRESS      		20  // �趨Ŀ�����ѹ��
#define TRG_OUTLET_FLUX       		22  // �趨Ŀ���������
#define OUTLET_PRESS_P        		24  // �趨����ѹ��PID���ڵ�P
#define OUTLET_PRESS_I        		26  // �趨����ѹ��PID���ڵ�I
#define OUTLET_PRESS_D        		28  // �趨����ѹ��PID���ڵ�D
#define OUTLET_FLUX_P         		30  // �趨��������PID���ڵ�P
#define OUTLET_FLUX_I         		32  // �趨��������PID���ڵ�I
#define OUTLET_FLUX_D         		34  // �趨��������PID���ڵ�D
#define CALI_PRESENT_OUTLET_PRESS	36  // �趨��ǰ����ѹ��
#define CALI_PRESENT_OUTLET_FLUX 	38  // �趨��ǰ��������
#define CALI_PRESENT_INLET_PRESS 	40  // �趨��ǰ���ѹ��
#define MANUAL_SPEED              42  // �ֶ����ڷ�оλ��ʱ���ٶ�
#define PID_SPEED                 44  // PID����ʱ���ٶ�
#define CURRENT                   46  // ������е���
#define MANUAL_ACC              	48  // �ֶ����ڷ�оλ��ʱ�ļ��ٶ�
#define PID_ACC                 	50  // PID����ʱ�ļ��ٶ�
#define OUT_PRESS_CTR_THRESHOLD   52  // ����ѹ��������ֵ
#define OUT_FLUX_CTR_THRESHOLD    54  // ��������������ֵ
#define OUTLET_PRESS_RAW          56  // ��ǰ����ѹ����ADC������ֵ��0-4095��
#define OUTLET_FLUX_RAW           58  // ��ǰ����������ADC������ֵ��0-4095��
#define INLET_PRESS_RAW           60  // ��ǰ���ѹ����ADC������ֵ��0-4095��
#define TRG_VALVE_POS	            62  // Ŀ��λ�ã�0-10mm��
#define COMMUNICATION_STA	        64  // ͨѶ������1��ʾ������0��ʾ�������⣩
#define MOTOR_CMD_STR							71	// ����ʾ��Ļֱ���·��ĵ��ָ��
#define MOTOR_RET_STR							102	// ������ص�ָ��洢��ʼ�Ĵ���

// modbusRTU��Ȧλ����
#define OPS_POWER_ACTION        	0   // ���ػ�ָ��
#define OPS_MOTOR_ENABLE        	1   // �������ʹ��ָ��
#define OPS_PRESENT_POS     			2   // �趨��ǰλ��ָ��
#define OPS_MANUALLY_VALVE_UP   	3   // �ֶ����ڷ�оλ������
#define OPS_MANUALLY_VALVE_DOWN 	4   // �ֶ����ڷ�оλ������
#define OPS_OUTLET_PRESS_MIN 			5   // �趨����ѹ������ָ��
#define OPS_OUTLET_PRESS_MAX 			6   // �趨����ѹ������ָ��
#define OPS_OUTLET_FLUX_MIN 			7   // �趨������������ָ��
#define OPS_OUTLET_FLUX_MAX 			8   // �趨������������ָ��
#define OPS_TRG_OUTLET_PRESS      9   // �趨Ŀ�����ѹ��ָ��
#define OPS_TRG_OUTLET_FLUX       10  // �趨Ŀ���������ָ��
#define OPS_OUTLET_PRESS_PID      11  // �趨����ѹ������PIDָ��
#define OPS_OUTLET_FLUX_PID       12  // �趨������������PIDָ��
#define OPS_QUIT_PID       				13  // �˳�PID����ģʽָ��
#define OPS_PRESENT_OUTLET_PRESS  14  // �趨��ǰ����ѹ��ָ��
#define OPS_PRESENT_OUTLET_FLUX   15  // �趨��ǰ��������ָ��
#define OPS_PRESENT_INLET_PRESS   16  // �趨��ǰ���ѹ��ָ��
#define OPS_MANUAL_SPEED			    17  // �趨����ֶ�����ʱ���ٶ�
#define OPS_FEED_SPEED   					18  // �趨���PIDģʽ������ʱ���ٶ�
#define OPS_CURRENT   						19  // �趨������е���
#define OPS_MANUAL_ACC   					20  // �趨����ֶ�����ʱ�ļ��ٶ�
#define OPS_FEED_ACC   						21  // �趨���PIDģʽ������ʱ�ļ��ٶ�
#define OPS_SET_PRESS_THRESHOLD   22  // �趨ѹ��������ֵ 
#define OPS_SET_FLUX_THRESHOLD    23  // �趨����������ֵ
#define OPS_TRG_POS        				24  // ��ʼ�˶���Ŀ��λ��
#define OPS_TEST        					25  // ��ʼ���е�������˶�����
#define OPS_SHUT_DOWN             26  // �����ͣ
#define OPS_MOTOR_DEBUG_MODE			27	// ������ָ��͸��ģʽ
#define OPS_SEND_DIR_MOTOR_CMD		28	// ͸��ģʽ��ֱ�Ӹ������ָ��

uint16_t CheckCRC(uint8_t *pData, uint8_t siLen); 					 // CRCУ��λ���ɺ���
uint16_t GetShortFromReg(uint8_t index);          					 // ����ʼ����Ϊindex��modbus�Ĵ����ж�ȡshort���͵���
uint32_t GetIntFromReg(uint8_t index);            					 // ����ʼ����Ϊindex��modbus�Ĵ����ж�ȡint���͵�����Ҫ�������Ĵ�����
float GetFloatFromReg(uint8_t index);             					 // ����ʼ����Ϊindex��modbus�Ĵ����ж�ȡfloat���͵�����Ҫ�������Ĵ�����
void WriteShortToReg(uint8_t index, int16_t num); 					 // ����ʼ����Ϊindex��modbus�Ĵ�����д��short���͵���
void WriteIntToReg(uint8_t index, int32_t num);   					 // ����ʼ����Ϊindex��modbus�Ĵ�����д��int���͵�����Ҫд�����Ĵ�����
void WriteFloatToReg(uint8_t index, float num);   					 // ����ʼ����Ϊindex��modbus�Ĵ�����д��float���͵�����Ҫд�����Ĵ�����
void Process(uint8_t* src, uint8_t len, rt_device_t handle); // ����modbusЭ����ж�д��Ȧ�ͼĴ����Ĳ���
void PostProcess(uint8_t* src, uint8_t len);      					 // ���Ĵ�������Ȧֵ��������Ӧ������

/*--------------���ͨѶ���------------------*/
#define MOTOR_SERIAL      "uart4"  				// ����ͨѶ�������� 
#define MOTOR_ADDR        '1'      				// ����ڲ���SCLָ���ַ
#define MOTOR_BAUD_RATE   BAUD_RATE_57600 // ���������
#define WAIT_BYTE 				3 			 				// �ȴ��������һ���ֽڵĳ�ʱʱ��
#define WAIT_MAIL     		50 			 				// �Ӹ��������ָ��������յ������һ֡���ݵĳ�ʱʱ��
#define WAIT_485          200			 				// �ȴ�����ͨѶ�������ͷŵĳ�ʱʱ��
#define CHECK_MAIL    		3  			 				// ����ͨѶʧ�ܺ���ٴ�ͨѶ���Դ���

void SendTxtMail(rt_mailbox_t mail, uint8_t* src);   // �����ݴ�ָ��д���̼߳�ͨѶ����
int ToMotorNoReturnNoParameter(char* cmd);                     // ��ָ��͸��������Ӧ���޲�����
int ToMotorNoReturnWithParameter(char* cmd, char* parameter);  // ��ָ��͸��������Ӧ���в�����
int ToMotorWithReturn(char* cmd, char* dst);      // ��ָ��͸��������Ӧ��
int GetFloatFromMotor(char* cmd, float* f_value);    // �ӵ����ȡӦ�𲢽�Ӧ�����Ϊfloat��

/*-----------------������������λ�����----------------*/
#define MOTOR_INITIAL     2500 // �ȴ�����̼���ʼ��ʱ��
#define SC_REFRESH_RATE   50   // ��ȡ��� status code �ļ��ʱ��
#define CHECK_MOV         2    // ���� status code ������Ƿ������ƶ��ļ�����
#define CHECK_STOP        2    // ���� status code ������Ƿ��Ѿ�ֹͣ�ļ�����

void ReadEncoderDataToMotor(void);                // ��EEPROME��ȡ��������ֵ��д����
int GetEnvFromMotor(int32_t* env_data);           // �ӵ����ȡint�͵ı�������ֵ
void SaveEncoderData(void);                       // �������������ֵ�����EEPROME

/*---------------�����������----------------*/
#define REFRESH_BASE      10   // ʱ���׼(��λ��ms)
#define IE_TICK           5    // ÿ IE_TICK ��ʱ���׼ˢ��һ�ε�ǰλ��
#define ADC_TICK        	2    // ÿ ADC_TICK ��ʱ���׼ˢ��һ��ADC����

/*----------------------ADC���-----------------------*/
#define ADC_DEV_NAME      		"adc1"
#define PRESS_OUT_CHANNEL   	14   		// ����ѹ��Ƶ��(PC4) 	
#define FLUX_CHANNEL   				15   		// ��������Ƶ��(PC5)
#define PRESS_IN_CHANNEL   		8    		// ���ѹ��Ƶ��(PB0)
#define PRESS_IN_RANGE_HIGH   3669.0  // ADC������Χ(��Ӧ3V)
#define PRESS_IN_RANGE_LOW		0.0			// ��Ӧ0V
#define PRESS_OUT_RANGE_HIGH  3668.0  // ADC������Χ(��Ӧ3V)
#define PRESS_OUT_RANGE_LOW		0.0			// ��Ӧ0V
#define FLUX_RANGE_HIGH   		3677.0  // ADC������Χ(��Ӧ3V)
#define FLUX_RANGE_LOW				0.0			// ��Ӧ0V
#define MOV_AVG_NUM       		64    		// ����ƽ���˲����õ�ADC���ݸ��� 

/*----------------����������ѹ���������---------------*/
#define PID_UPDATE_CNT    			10    // PID_UPDATE_CNT ��ʱ���׼����һ��PID����

/*----------------������������---------------*/
void DataInitialize(void);					// savepos�̴߳�EEPROME��ȡ���� �Լ� �ӵ����ȡ����
void FeedLenEnc(int32_t enc_val);    // �˶����ض��ĵ������λ�� 
void FeedPosValve(float pos);				// �˶����ض��ķ�оλ��
void CaliPresentADC(uint32_t now_enc_val_addr, uint32_t userSet_phy_val_addr, uint32_t phy_upperBound_addr, uint32_t phy_lowerBound_addr, uint32_t EncHigh_eeaddr, uint32_t PhyHigh_eeaddr, uint32_t EncLow_eeaddr, uint32_t PhyLow_eeaddr, uint32_t coeff_addr, float* EncHigh, float* PhyHigh, float* EncLow, float* PhyLow, float* convert_coeff);
int CaliSensor(uint32_t bound_reg_addr, uint32_t bound_ee_addr, uint32_t convert_coeff_ee_addr, float* old_lower_bound, float* old_upper_bound, float* convert_coeff, int type); // �궨��������������
void SetThreshold(uint32_t reg_addr, uint32_t ee_addr, float* threshold);

/*----------------����������-----------------*/
#define SPLITE_POINT 		0.4						// �������궨�����޵ķָ��
#define CHUAN_DONG_BI		62						// �����ϸ˴�����
#define DAO_CHENG				4							// ˿�ܵ���
#define EVC_RESOLUTION  4000					// ���תһȦ�������ߵĲ���
#define IP_TO_EP				5							// ������������ӳ��ֵĴ����ȣ��� FL �� FP ��Ч

#endif

