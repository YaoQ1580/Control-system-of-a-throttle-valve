#include "miscellaneousTools.h"
#include "GlobalVariable.h" 

/*------------------��������-------------------*/
void StrToHex(char* src, uint32_t* dst)
{
	uint8_t i;
	uint8_t tmp = 0;
	uint32_t tmp2;
	uint8_t len = rt_strlen((const char*)src);
	
	tmp2 = 0;
	
	if(len != 0)
	{
		for(i = 0; i < len - 1; i++)
		{
			if(src[i] >= '0' && src[i] <= '9')
				tmp = src[i] - '0';
			else
				tmp = src[i] - 'A' + 10;
			
			tmp2 |= tmp;
			tmp2 <<= 4;
		}
		
		if(src[i] >= '0' && src[i] <= '9')
			tmp = src[i] - '0';
		else
			tmp = src[i] - 'A' + 10;
		
		tmp2 |= tmp;	
		*dst = tmp2;
	}
}

// src��ʾһ��uint8_t�����飬cnt��ʾ�������ĳ��ȣ��ú�����src���������ת����hex��ʽ���ַ���������src[] = "0x01, 0x02, 0x03"����ôdst[] = "01 02 03"
void Uint8ArrToHexCharStr(uint8_t* src, int cnt, char* dst)
{
	int index = 0;
	int i;
	uint8_t tmp;
	if(cnt == 0)
	{
		dst[0] = '\0';
		return;
	}
	
	for(i = 0; i < cnt; i++)
	{
		tmp = (src[i] & 0xF0) >> 4;
		if(tmp <= 9)
		{
			dst[index++] = tmp + '0';
		}
		else
		{
			dst[index++] = tmp - 10 + 'A';
		}
		tmp = (src[i] & 0x0F);
		if(tmp <= 9)
		{
			dst[index++] = tmp + '0';
		}
		else
		{
			dst[index++] = tmp - 10 + 'A';
		}
		dst[index++] = ' ';
	}
	dst[index] = '\0';
}
	
	

void DecToHex(uint8_t* src, uint8_t* trg, uint16_t len)
{	
		int i;
		uint8_t tmp;
		for(i = 0; i < len; i++)
		{
				tmp = (src[i] & 0xF0) >> 4;
				if(tmp < 10)
						*trg ++ = tmp + '0';
				else
						*trg ++ = tmp - 10 + 'A';
				
				tmp = src[i] & 0x0F;
				if(tmp < 10)
						*trg ++ = tmp + '0';
				else
						*trg ++ = tmp - 10 + 'A';
				
				*trg ++ = ' ';
		}
		
		*trg = '\0';
}

// ��32λ���ݷָ��4��8λ���ݣ�������src = 0x12345678����dst[] = {0x78, 0x56, 0x34, 0x12};
void itob(int32_t src, uint8_t* dst)
{
	dst[0] = src & 0xFF;
	dst[1] = (src >> 8) & 0xFF;
	dst[2] = (src >> 16) & 0xFF;
	dst[3] = (src >> 24) & 0xFF;
}

// ��4��8λ����ת����һ��32λ���ݣ�����src[] = {0x78, 0x56, 0x34, 0x12}����������0x12345678;
int32_t btoi(uint8_t* src)
{
	int32_t tmp = 0;
	
	tmp |= src[3];
	tmp <<= 8;
	tmp |= src[2];
	tmp <<= 8;
	tmp |= src[1];
	tmp <<= 8;
	tmp |= src[0];
	
	return tmp;
}

/*---------------------------I2C-----------------------------*/
static struct rt_i2c_bus_device *i2c_bus;

rt_err_t i2c_init()
{
	i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(I2C_BUS_NAME); // ����I2C�豸�������ؾ��
	
	if(i2c_bus != RT_NULL)
	{
		return RT_EOK;
	}
	else
	{
		rt_kprintf("find i2c device fail!\n");
		return -RT_ERROR;
	}
}




// ��int������д��EEPROME�����Ӧ���Ƕ��̰߳�ȫ��
int WriteInt32ToEEPROME(uint32_t addr, int32_t num)
{
	int32_t read_data;
	
	// ͨѶ֮ǰ�û��������ò����Ļ���Զ�ȴ�����Ϊ I2C һ����ͨѶ����
	rt_mutex_take(I2CBus, RT_WAITING_FOREVER);
	
	// ������д��EEPROME
	write_AT24C02(addr, num);
	// �ȴ�����������д��
	rt_thread_mdelay(I2C_WRITE_DELAY);
	// ��ȡEEPROME���Ƿ�д��ȥ��
	read_AT24C02(addr, &read_data);
	
	// �����ͷŵ�
	rt_mutex_release(I2CBus);
	
	if(read_data == num)  // д��ȥ��
	{
		return 1;
	}
	else									// ����д����
	{
		return 0;
	}
}





// ��float������д��EEPROME�����Ӧ���Ƕ��̰߳�ȫ��
int WriteFloatToEEPROME(uint32_t addr, float num)
{
	int32_t tmp = *((int*)&num); // ��float���ݵĶ����Ʊ���ת�͵�һ��int�����У�����֮����bit����������ݲ���
	
	return WriteInt32ToEEPROME(addr, tmp);
}




// �� EEPROME �ж�ȡ int ���ݣ����Ӧ���Ƕ��̰߳�ȫ��
int ReadInt32FromEEPROME(uint32_t addr, int32_t* num)
{
	rt_err_t rt_ret;
	
	// ͨѶ֮ǰ�û��������ò����Ļ���Զ�ȴ�����Ϊ I2C һ����ͨѶ�ɹ���
	rt_mutex_take(I2CBus, RT_WAITING_FOREVER);
	
	// ��ȡ����
	rt_ret = read_AT24C02(addr, num);
	
	// ����֮���ͷ���
	rt_mutex_release(I2CBus);
	
	if(rt_ret == RT_EOK)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}




// �� EEPROME �ж�ȡ float ���ݣ����Ӧ���Ƕ��̰߳�ȫ��
int ReadFloatFromEEPROME(uint32_t addr, float* num)
{
	int ee_ret;
	int32_t recev_data;
	
	ee_ret = ReadInt32FromEEPROME(addr, &recev_data);
	// ������ֵ���ͳ�float
	*num = *((float*)&recev_data);
	
	return ee_ret;
}




// ��int�͵���ֵnumд����ʼ��ַΪaddr��EEPROME�洢����least significant�ֽ�����
// ע��: ������صı�����λ�����ַ�����ʽ�����Ե��ô˺���֮ǰ��Ҫ�Ƚ���ת����Int��֮��ſ��Դ洢��EEPROME
rt_err_t write_AT24C02(uint32_t addr, int32_t num)
{
	uint8_t data[5];
	struct rt_i2c_msg msgs;
	
	// ��int��ת����4�ֽ�����
	data[0] = addr;      // EEPROME�еĵ�ַ
	itob(num, data + 1);
	
	// ����I2C��Ϣ
	msgs.addr = AT24C02_ADDR;
	msgs.flags = RT_I2C_WR;
  msgs.buf = data;
  msgs.len = 5;
	
	// ��������
	if(rt_i2c_transfer(i2c_bus, &msgs, 1) == 1)
  {
    return RT_EOK;
  }
  else
  {
    return -RT_ERROR;
  }
}




// ��float�͵���ֵnumд����ʼ��ַΪaddr��EEPROME�洢����least significant�ֽ�����
rt_err_t write_AT24C02_float(uint32_t addr, float num)
{
	int32_t tmp = *((int*)&num); // ��float���ݵĶ����Ʊ���ת�͵�һ��int�����У�����֮����bit����������ݲ���
	
	return write_AT24C02(addr, tmp); // ����write
}




// ����ʼ��ַΪaddr��EEPROME�洢����ȡ�ĸ��ֽڲ����䰴��int�ͶԴ�
rt_err_t read_AT24C02(uint32_t addr, int32_t* dst)
{
	uint8_t ee_addr = addr;
	struct rt_i2c_msg msgs[2];
	uint8_t data[4];
	
	// ���ȷ���һ���������EEPROME���ǽ����׵�ַaddr���ж�����
	msgs[0].addr = AT24C02_ADDR;
	msgs[0].flags = RT_I2C_WR;
  msgs[0].buf = &ee_addr;
  msgs[0].len = 1;
	
	// ֮������������ȡ�ĸ��ֽ�
	msgs[1].addr = AT24C02_ADDR;
	msgs[1].flags = RT_I2C_RD;
  msgs[1].buf = data;
  msgs[1].len = 4;
	
	if(rt_i2c_transfer(i2c_bus, msgs, 2) == 2)
  {
		*dst = btoi(data); // ��ȡ�ɹ��Ļ��������ֽ�ƴ�ճ�int�ͷ���
		
    return RT_EOK;
  }
  else
  {
    return -RT_ERROR;
  }
}




// ����ʼ��ַΪaddr��EEPROME�洢����ȡ�ĸ��ֽڲ����䰴��float�ͶԴ�
rt_err_t read_AT24C02_float(uint32_t addr, float* dst)
{
	rt_err_t ack;
	int32_t tmp;
	
	ack = read_AT24C02(addr, &tmp); // �ȴ�EEPROME�ж�ȡint������
	if(ack == RT_EOK)
	{
		*dst = *((float*)&tmp);       // ����int�Ͷ����Ʊ��뵱��float���д���
	}
	
	return ack;
}




// ��modbus�Ĵ����е�float��ֵд����ʼ��ַΪaddr��EEPROME�洢��
int WriteFloatRegToEeprome(uint32_t reg_addr, uint32_t ee_addr)
{
	float data;
	data = GetFloatFromReg(reg_addr);    // �ӼĴ����ж�ȡfloat��ֵ
	
	return WriteFloatToEEPROME(ee_addr, data);  // �� float ��ֵд�� EEPROME
}




/*-------------------���ͨѶ���------------------*/
// �����ݴ�ָ��д���̼߳�ͨѶ����
void SendTxtMail(rt_mailbox_t mail, uint8_t* src)
{
	rt_ubase_t* recev;
	char* to_sent;
	rt_err_t uwRet = RT_EOK;
	
	// ���ȼ��ͨѶ�����Ƿ���֮ǰû�ж�ȡ�������ʼ�
	uwRet = rt_mb_recv(mail, recev, 0);
	if(uwRet == RT_EOK) 
	{
		rt_kprintf("Motor Thread find some garbage mail !! \n");
		rt_free(recev); // ����������ʼ����ͷ��ʼ��Ķѿռ�
	}
	
	// ���û�������ʼ�����ô����Ҫ���͵����ݷŵ���������
	to_sent = rt_strdup((const char*)src);
	rt_mb_send(mail, (rt_uint32_t)to_sent);
}




// ��ָ��͸����Ȼ��ȥ����ȴ�����ķ�������
// ���� CHECK_MAIL �Σ����ܵ����⣺WAIT_MAIL ʱ��̫�̵������·�����Ϣ
int SafelySendCmdAndReadAck(char* cmd, char* dst) // don't forget to free heap
{
	int check = CHECK_MAIL;
	int str_len;
	rt_err_t uwRet = RT_EOK;
	uint8_t* tmp;
	int index, i;
	
	// ���ȼ��ͨѶ�����Ƿ���֮ǰû�ж�ȡ�������ʼ�
	uwRet = rt_mb_recv(ack, (rt_ubase_t*)&tmp, 0); 
	if(uwRet == RT_EOK) // there exists some garbage
	{
		rt_kprintf("ReadMotorReturn find some garbage mail!! \n");
		rt_free(tmp);  // ����������ʼ����ͷ��ʼ��Ķѿռ�
	}

	while(check > 0)
	{
		rt_device_write(motor, 0, cmd, rt_strlen((const char*)cmd));  // ������������
		uwRet = rt_mb_recv(ack, (rt_ubase_t*)&tmp, WAIT_MAIL);  			// �ȴ���������ʼ����ȴ���ʱʱ��Ϊ WAIT_MAIL
		if(uwRet == RT_EOK) // ����ڳ�ʱʱ�����յ��ʼ�
		{
			str_len = rt_strlen((const char*)tmp);
			rt_memcpy(dst, tmp, str_len); // ��Ӧ���ʼ����ݿ�����dst
			dst[str_len] = '\0';
			rt_free(tmp);                        // �ͷ��ʼ��Ķѿռ�
			
			// ��Ϊ������Ҫ�����ｫ dst ���͵��Ĵ������棬��Ϊֻ����������Щ�Ĵ�������д���������ԾͲ��� critical section ��
			// ע�⣺Ҫ��д�� 8 λ
			index = MOTOR_RET_STR;
			for(i = 0; i < str_len; i += 2)
			{
				Reg[index] = 0;
				Reg[index] |= dst[i];
				Reg[index] <<= 8;
				Reg[index] |= dst[i + 1];
				index++;
			}
			Reg[index] = '\0';
			return 1;                            // ����1��ʾ��ȡ�ʼ��ɹ�
		}
		check--; // ���Ӧ��������ٴγ���
		
		rt_kprintf("Send Cmd: %s. fail to receive the %dnd ack, try again.\n", cmd, CHECK_MAIL-check);
	}
	
	dst[0] = '\0';
	Reg[MOTOR_RET_STR] = '\0';  // ��Ϊ reg �� short ���͵ģ���������ʵ���������������� '\0'
	return 0; // ͨѶ���󣬷���0
}




// ��ָ��͸��������Ӧ���޲�����
// ָ���Ҫ�Ӵӻ���ַ������
int ToMotorNoReturnNoParameter(char* cmd)
{
	rt_err_t uwRet = RT_EOK; 	// ȷ���Ƿ��õ�������
	int uwRet_cmu = 0;  			// ȷ���Ƿ�ͨѶ�ɹ�
	int ret_val = 0;         	// ��������ֵ
	char final_cmd[32];		// �� cmd �����ټӹ�
	int cmd_len = rt_strlen((const char*)cmd); // cmd �ĳ���
	char check[5];            // ���Ӧ��
	
	// �� cmd ���� head �� tail
	final_cmd[0] = MOTOR_ADDR;
	rt_memcpy(final_cmd + 1, cmd, cmd_len);
	final_cmd[1 + cmd_len] = 0x0D;
	final_cmd[2 + cmd_len] = '\0';
	
	// ͨѶ֮ǰ�û��������ȴ���ʱΪ WAIT_485
	uwRet = rt_mutex_take(MotorBus, WAIT_485);
	
	// ����ɹ��õ�������
	if(uwRet == RT_EOK)
	{
		uwRet_cmu = SafelySendCmdAndReadAck(final_cmd, check); // ������������
		
		if(uwRet_cmu == 0)  // ͨѶʧ��
		{
			rt_kprintf("Fatal communication failure!!!! \n");
			ret_val = 0;
		}
		else		// ͨѶ�ɹ������������ֵ
		{
			if(check[0] == MOTOR_ADDR)
			{
				switch(check[1])
				{
					case '%':
						//rt_kprintf("Communication Success!! \n");
						ret_val = 1;
						break;
					
					case '*':
						rt_kprintf("Motor is Busy, but communication is OK. \n");
						ret_val = 2;
						break;
					
					case '?':
						rt_kprintf("Motor Didn't Recognize the Cmd. \n");
						ret_val = 0;
						break;
					
					default:
						rt_kprintf("Return Value Is Wrong!! \n");
						ret_val = 0;
						break;
				}
			}
			else
			{
				rt_kprintf("Return Value Is Wrong!! \n");
				ret_val = 0;
			}
		}
		
		rt_mutex_release(MotorBus);		// �ͷŻ�����
	}
	else   // û�гɹ��õ����������ȴ���ʱ
	{
		rt_kprintf("ToMotorNoReturn: Waiting 485 time out !!! \n");
		ret_val = 0;
	}
	
	// ���¼Ĵ�����ֵ
	WriteShortToReg(COMMUNICATION_STA, ret_val);
	return ret_val;
}




// ��ָ��͸��������Ӧ���в�����
// ָ���Ҫ�Ӵӻ���ַ������
int ToMotorNoReturnWithParameter(char* cmd, char* parameter)
{
	char combined_cmd[32];
	int cmd_len = rt_strlen((const char*)cmd);
	int para_len = rt_strlen((const char*)parameter);
	
	// �� cmd �� ���� ���кϲ�
	rt_memcpy(combined_cmd, cmd, cmd_len);
	rt_memcpy(combined_cmd + cmd_len, parameter, para_len);
	combined_cmd[cmd_len + para_len] = '\0';
	
	return ToMotorNoReturnNoParameter(combined_cmd);
}




// ��ָ��͸��������Ӧ���޲�����
// ָ���Ҫ�Ӵӻ���ַ������
int ToMotorWithReturn(char* cmd, char* dst)
{
	rt_err_t uwRet = RT_EOK;    // ȷ���Ƿ��õ�������
	int ret_val = 0;						// ����ֵ
	int uwRet_cmu = 0;					// ȷ���Ƿ�ͨѶ�ɹ�
	char final_cmd[32];			// ���շ��������ָ��
	int cmd_len = rt_strlen((const char*)cmd);
	
	// �� cmd ���� head �� tail
	final_cmd[0] = MOTOR_ADDR;
	rt_memcpy(final_cmd + 1, cmd, cmd_len);
	final_cmd[1 + cmd_len] = 0x0D;
	final_cmd[2 + cmd_len] = '\0';
	
	// ͨѶ֮ǰ�û��������ȴ���ʱΪ WAIT_485
	uwRet = rt_mutex_take(MotorBus, WAIT_485);
	
	// ����ɹ��õ�������
	if(uwRet == RT_EOK)
	{
		uwRet_cmu = SafelySendCmdAndReadAck(final_cmd, dst); // ���� SafelySendCmdAndReadAck ����������ָ��
		
		if(uwRet_cmu == 0)  // ͨѶʧ��
		{
			rt_kprintf("Fatal communication failure!!!! \n");
			ret_val = 0;
		}
		else								// ͨѶ�ɹ�
		{
			// �����Ӧ���Ƿ���ȷ
			// ������־���SCLָ������ԣ�����ֻ��Ҫ���ָ���ǰ3���ַ�
			if(!rt_memcmp(dst, final_cmd, 3))   // ���ָ����ȷ
			{
				//rt_kprintf("Get Return From Motor!! \n");
				// ��ָ���ͷ��ȥ������1CC=4�� ȥ�� ��1CC=����Ȼ�����ķ��ػ����� 0x0D�����Ի���Ҫ��1
				uint8_t data_len = rt_strlen((const char*)(dst + 4)) - 1;
				rt_memcpy(dst, dst + 4, data_len);
				dst[data_len] = '\0';
				
				ret_val = 1;
			}
			else																// ���س���
			{
				rt_kprintf("Return String Is Wrong!! \n");
				ret_val = 0;
			}
		}
		
		rt_mutex_release(MotorBus);           // �ͷŻ�����
	}
	else    // û�гɹ��õ����������ȴ���ʱ
	{
		rt_kprintf("With Ack Type: Waiting 485 time out !!! \n");
		ret_val = 0;
	}
	
	// ���¼Ĵ�����ֵ
	WriteShortToReg(COMMUNICATION_STA, ret_val);
	return ret_val;
}




// ����һ��ָ���ָ��ķ���ֵ���� float ����
int GetFloatFromMotor(char* cmd, float* f_value)
{
	int ret_val;								// ���ͨѶ�Ƿ�ɹ���Ӧ��
	char recved_data[32]; 		// �ӵ����õķ����ַ���
	
	// ��������ָ�����ȡ�����Ӧ��
	ret_val = ToMotorWithReturn(cmd, recved_data);
	if(!ret_val) // ���ͨѶ��������
	{
		rt_kprintf("Try to get float from motor, but failed !!! \n");
		*f_value =  -1;
		return 0;
	}
	else				 // ͨѶ�ɹ��Ļ�
	{
		sscanf((const char*)recved_data, "%f", f_value);
		return 1;
	}
}





/*-----------------������������λ�����----------------*/
// ��EEPROME��ȡ��������ֵ��д����
// �ú����������ϵ��ĳ�ʼ������ִ��һ��
void ReadEncoderDataToMotor()
{
	char encoder_value_str[32];
	int ret_val;
	
	// ������intֵ
	int32_t encoder_value = 0;  
	
	// ��EEPROME�ж�ȡ��ֵ
	ReadInt32FromEEPROME(ENCODER_ADDR, &encoder_value);
	
	// ��int����ת�����ַ���
	sprintf(encoder_value_str, "%d", encoder_value);
	
	// ������������Ϊ��ֹ����������ֻ����һ�� 
	ret_val = ToMotorNoReturnWithParameter("EP", encoder_value_str);
	if(!ret_val)
	{
		rt_kprintf("Send EP data from Eeprome to Motor FAIL!!! \n");
	}
	else
	{
		rt_kprintf("Send EP data: %s  To Motor. \n", encoder_value_str);
	}
}



// �ӵ����ȡint�͵ı�������ֵ������� env_data
// ���� 1 ��ʾ��ȡ�ɹ������� 0 ��ʾ��ȡʧ��
int GetEnvFromMotor(int32_t* env_data)
{
	int ret_val = 0;						// ���ͨѶ�Ƿ�ɹ�
	char recved_data[32]; 			// �ӵ����õ�Ӧ��
	
	// �������͡�EP��������ȡ�����Ӧ��
	ret_val = ToMotorWithReturn("EP", recved_data);
	if(!ret_val) // ���Ӧ��ʧ��
	{
		rt_kprintf("Get encoder value from motor fail !!! \n");
		*env_data = 0;
		return 0;
	}
	else				 // ��ȡ�ɹ�
	{
		sscanf(recved_data, "%d", env_data);
		return 1;
	}
}



// �������������ֵ�����EEPROME
void SaveEncoderData()
{
	int32_t encoder_value;
	int ret_val;
	int ee_val;
	
	// �ӵ����ȡint�͵ı�������ֵ
	ret_val = GetEnvFromMotor(&encoder_value);
	if(ret_val) // �����ȡ�ɹ�
	{
		rt_kprintf("EP: %d is gonna saved into eeprome. \n", encoder_value);
		
		// �����ݴ���EEPROME
		ee_val = WriteInt32ToEEPROME(ENCODER_ADDR, encoder_value);
		if(ee_val)
		{
			rt_kprintf("Sava Encoder Value Succeed !!!  \n");
		}
		else
		{
			rt_kprintf("Sava Encoder Value to EEPROME FAIL !!!  \n");
		}
	}
	else
	{
		rt_kprintf("Try to save encoder, But Read Encoder Value From Motor FAIL !!!  \n");
	}
}



/*-----------------Modbus RTU--------------------*/
// CRCУ��λ���ɺ���
uint16_t CheckCRC(uint8_t *pData, uint8_t siLen)
{
	int i,j = 0;
	uint16_t u16CRC = 0xFFFF;

	for (i = 0; i < siLen; i++)
	{
		u16CRC ^= (uint16_t)(pData[i]);
		for(j = 0; j <= 7; j++)
		{
			if (u16CRC & 0x0001)
			{
				u16CRC = (u16CRC >> 1) ^ 0xA001;
			}
			else
			{
				u16CRC = u16CRC >> 1; 
			}
		}
	}
	
	uint16_t siRet = 0;
	siRet = (u16CRC & 0x00FF) << 8; 
	siRet |= u16CRC >> 8;
	
	return siRet;
}

// ����ʼ����Ϊindex��modbus�Ĵ�����д��short���͵���
void WriteShortToReg(uint8_t index, int16_t num)
{
	rt_enter_critical();
	Reg[index] = num;
	rt_exit_critical();
}

// ����ʼ����Ϊindex��modbus�Ĵ�����д��int���͵�����Ҫд�����Ĵ�����
void WriteIntToReg(uint8_t index, int32_t num)
{
	rt_enter_critical();
	Reg[index] = num >> 16; // ��д��16λ
	Reg[index + 1] = num;   // ��д��16λ
	rt_exit_critical();
}

// ����ʼ����Ϊindex��modbus�Ĵ�����д��float���͵�����Ҫд�����Ĵ�����
void WriteFloatToReg(uint8_t index, float num)
{
	uint32_t tmp = *((uint32_t*)&num);
	
	rt_enter_critical();
	Reg[index] = tmp >> 16; // ��д��16λ
	Reg[index + 1] = tmp;   // ��д��16λ
	rt_exit_critical();
}

// ����ʼ����Ϊindex��modbus�Ĵ����ж�ȡshort���͵���
uint16_t GetShortFromReg(uint8_t index)
{
	uint16_t tmp;
	
	rt_enter_critical();
	tmp = Reg[index];
	rt_exit_critical();
	
	return tmp;
}

// ����ʼ����Ϊindex��modbus�Ĵ����ж�ȡint���͵�����Ҫ�������Ĵ�����
uint32_t GetIntFromReg(uint8_t index)
{
	uint32_t tmp = 0;
	
	rt_enter_critical();
	tmp |= Reg[index];     // �ȶ���16λ
	tmp <<= 16;
	tmp |= Reg[index + 1]; // �ٶ���16λ
	rt_exit_critical();
	
	return tmp;
}

// ����ʼ����Ϊindex��modbus�Ĵ����ж�ȡfloat���͵�����Ҫ�������Ĵ�����
float GetFloatFromReg(uint8_t index)
{
	float dst;
	uint32_t tmp = 0;
	
	rt_enter_critical();
	tmp |= Reg[index];     // �ȶ���16λ
	tmp <<= 16;
	tmp |= Reg[index + 1]; // �ٶ���16λ
	rt_exit_critical();
	
	dst = *((float*)&tmp); // ��intת��Ϊfloat
	
	return dst;
}

// ����modbusЭ����ж�д��Ȧ�ͼĴ����Ĳ���
void Process(uint8_t* src, uint8_t len, rt_device_t handle)
{
	uint8_t i;
	
	uint16_t start_addr = 0;
	uint16_t read_len = 0;
	uint16_t cnt = 0;
	uint8_t ack_str[200];
	uint32_t mask, mask2;
	uint32_t read_coil;
	uint16_t CRC_code;
	
	switch(src[1]) // ����modbus������
	{
		case 0x01:   // ����Ȧ��ע��ֻ�������� 16 �� bit ������
			start_addr |= src[2];
			start_addr <<= 8;
			start_addr |= src[3]; // Ҫ������Ȧ����ʼ��ַ��������Ȧ�ĵڼ�λ��ʼ����
			read_len |= src[4];
			read_len <<= 8;
			read_len |= src[5];   // Ҫ���ĳ��ȣ��������ٸ�bit��
		
			// �����ٽ���֮���ڽ��ж�����
			rt_enter_critical();
			mask = (0x01 << (start_addr + read_len)) - 1;
			read_coil = (coil & mask) >> start_addr;
			rt_exit_critical();
		
			// ����Ӧ���ַ���
			ack_str[0] = 0x01; ack_str[1] = 0x01; ack_str[2] = read_len % 8? read_len/8 + 1: read_len/8;
			ack_str[3] = (uint8_t)read_coil; 
			if(ack_str[2] >= 2)
			{
				ack_str[4] = read_coil >> 8;
				cnt = 5;
			}
			else
				cnt = 4;
			
			// ����CRCУ��
			CRC_code = CheckCRC(ack_str, cnt);
			ack_str[cnt] = CRC_code >> 8; ack_str[cnt + 1] = CRC_code & 0x00FF;
			
			// ����������Ӧ��󷵻ظ���λ��
			rt_device_write(handle, 0, ack_str, cnt + 2);
			break;
		
		case 0x03:   // �����ּĴ���
			start_addr |= src[2];
			start_addr <<= 8;
			start_addr |= src[3]; // Ҫ���ļĴ�������ʼ����
			read_len |= src[4];
			read_len <<= 8;
			read_len |= src[5];   // Ҫ���ļĴ�������
		
			// ����Ӧ��֡��֡ͷ
			ack_str[0] = 0x01; ack_str[1] = 0x03; ack_str[2] = 2 * read_len; cnt = 3;
		
			// ����Ҫ���ļĴ����������Ĵ�����ֵд��Ӧ��
			rt_enter_critical();
			for(i = 0; i < read_len; i++)
			{
				// ��д��8λ
				ack_str[cnt] = Reg[start_addr + i] >> 8;
				
				// ��д��8λ
				ack_str[cnt + 1] = Reg[start_addr + i] & 0x00FF;
				cnt += 2;
			}
			rt_exit_critical();
			
			// ����CRCУ��
			CRC_code = CheckCRC(ack_str, cnt);
			ack_str[cnt] = CRC_code >> 8; ack_str[cnt + 1] = CRC_code & 0x00FF;
			
			// ����������Ӧ��󷵻ظ���λ��
			rt_device_write(handle, 0, ack_str, cnt + 2);
			break;
		
		case 0x05: 	 // д������Ȧ
			start_addr |= src[2];
			start_addr <<= 8;
			start_addr |= src[3]; // ��ȡҪд����ʼ��Ȧ��ַ
		
			rt_enter_critical();
			if(src[4] == 0xFF) // ����bit�� 1
			{
				coil |= (0x01 << start_addr);
			}
			else 							 // ����bit�� 0
			{
				coil &= ~(1 << start_addr);
			}
			rt_exit_critical();
			
			// ����modbusЭ����Ӧ���������ͬ����ֱ�ӷ�������ͺ�
			rt_device_write(handle, 0, src, len);
			break;
		
		case 0x06:   // д�����Ĵ���
			start_addr |= src[2];
			start_addr <<= 8;
			start_addr |= src[3]; 	// ��ȡҪд�ļĴ�������ʼ����
		
			mask = 0;
			mask |= src[4];
			mask <<= 8;
			mask |= src[5];       	// ��ȡҪд�����ֵ
			
			rt_enter_critical();
			Reg[start_addr] = mask; // д��Ĵ���
			rt_exit_critical();
			
			rt_device_write(handle, 0, src, len);
			break;
		
		case 0x0F:   // д�����Ȧ
			start_addr |= src[2];
			start_addr <<= 8;
			start_addr |= src[3]; // ��ȡҪд����ʼ��Ȧ��ַ
			read_len |= src[4];
			read_len <<= 8;
			read_len |= src[5];   // ��ȡҪд��bit����
		
			// ��������
			mask = ~((0x01 << (start_addr + read_len)) - 1);
			mask |= (0x01 << start_addr) - 1;
			
			mask2 = 0;
			if(src[6] >= 2) // ���Ҫд����Ȧbit������1���ֽ�
			{
				mask2 |= src[8];
				mask2 <<= 8;
			}
			mask2 |= src[7];
			mask2 <<= start_addr;
			
			// д��Ȧ
			rt_enter_critical();
			coil &= mask;   // ��addr��addr+len����Ȧ��0   
			coil |= mask2;  // ��addr��addr+len����Ȧ��Ϊ��Ҫ���õ���
			rt_exit_critical();
			
			// ����Ӧ��
			ack_str[0] = 0x01; ack_str[1] = 0x0F; ack_str[2] = src[2]; ack_str[3] = src[3]; ack_str[4] = src[4]; ack_str[5] = src[5];
			CRC_code = CheckCRC(src, 6);
			ack_str[6] = CRC_code >> 8; ack_str[7] = CRC_code & 0x00FF;
			
			// ����Ӧ��
			rt_device_write(handle, 0, ack_str, 8);
			break;
		
		case 0x10:   // д����Ĵ���
			start_addr |= src[2];
			start_addr <<= 8;
			start_addr |= src[3]; // ��ȡҪд�ļĴ�������ʼ����
			read_len |= src[4];
			read_len <<= 8;
			read_len |= src[5]; 	// ��ȡҪд�ļĴ�������
		
			// ����Ҫд�ļĴ�������ȥд�Ĵ���
			rt_enter_critical();
			for(i = 0; i < read_len; i++)
			{
				// ��д��8λ
				Reg[start_addr + i] = 0;
				Reg[start_addr + i] |= src[7 + 2*i];
				Reg[start_addr + i] <<= 8;
				
				// ��д��8λ
				Reg[start_addr + i] |= src[8 + 2*i];
			}
			rt_exit_critical();
			
			// ����Ӧ��
			ack_str[0] = 0x01; ack_str[1] = 0x10; ack_str[2] = src[2]; ack_str[3] = src[3]; ack_str[4] = src[4]; ack_str[5] = src[5];
			CRC_code = CheckCRC(src, 6);
			ack_str[6] = CRC_code >> 8; ack_str[7] = CRC_code & 0x00FF;
			
			// ����Ӧ��
			rt_device_write(handle, 0, ack_str, 8);
			break;
		
		default:
			break;
	}
}




// ���Ĵ�������Ȧֵ��������Ӧ������
void PostProcess(uint8_t* src, uint8_t len)
{
	 uint16_t start_addr = 0;
	 rt_uint32_t recved_event;
	
	switch(src[1])  // ����modbus������
	{
		case 0x05:  // д��Ȧ�ĵ���bit
			start_addr |= src[2];
			start_addr <<= 8;
			start_addr |= src[3]; // ��ȡ��ʼ��ַ
		
			switch(start_addr) // ������ʼ��ַ
			{
				case OPS_POWER_ACTION: // ���ػ�ָ��
					if(src[4] == 0xFF) // ����
					{
						rt_event_send(Schedule, POWER_ON);
						rt_kprintf("RTU Set POWER_ON event. \n");
					}
					else               // �ػ�
					{
						// Ӧ��Ҫ�ȿ������ȵ� 100 ��ʱ��ſ��Թػ�
						uint16_t power_up_progress = GetShortFromReg(POWER_UP_PROGRESS);
						if(power_up_progress == 100)
						{
							rt_event_send(Schedule, POWER_OFF);
							rt_kprintf("RTU Set POWER_OFF event. \n");
						}
						else
						{
							rt_kprintf("Please Wait until System finish Sartup. \n");
						}
					}
					break;
					
					
					
					
				case OPS_MOTOR_ENABLE: // �������ʹ��ָ��
					if(src[4] == 0xFF)
					{
						rt_event_send(Schedule, RESET_MOTOR);
						rt_kprintf("RTU Set RESET_MOTOR event. \n");
					}
					break;
					
					
					
					
				case OPS_PRESENT_POS: 	// �趨��ǰλ��ָ��
					rt_kprintf("Send CALI_POS Event. \n");
					rt_event_send(Schedule, CALI_POS); // ���� CALI_POS �¼���������Ҫ���±궨��ǰλ��
					break;
				
				
				
				
				case OPS_MANUALLY_VALVE_UP: // �ֶ����ڷ�оλ������
					if(src[4] == 0xFF) // ��������λ״̬
					{
						rt_kprintf("Send MAN_MOVE_UP Event. \n");
						rt_event_send(Schedule, MAN_MOVE_UP);
					}
					else               // �����ڸ�λ״̬
					{
						rt_kprintf("Send MAN_MOVE_STOP Event. \n");
						rt_event_send(Schedule, MAN_MOVE_STOP);
					}
					break;
				
					
					
					
				case OPS_MANUALLY_VALVE_DOWN: // �ֶ����ڷ�оλ������
					if(src[4] == 0xFF) // ��������λ״̬
					{
						rt_kprintf("Send MAN_MOVE_DOWN Event. \n");
						rt_event_send(Schedule, MAN_MOVE_DOWN);
					}
					else               // �����ڸ�λ״̬
					{
						rt_kprintf("Send MAN_MOVE_STOP Event. \n");
						rt_event_send(Schedule, MAN_MOVE_STOP);
					}
					break;
				
					
					
					
				case OPS_OUTLET_PRESS_MIN: // �趨ѹ������������ָ��
					rt_kprintf("RTU Set SET_PRESS_SENSOR_MIN event. \n");
					rt_event_send(Schedule, SET_PRESS_SENSOR_MIN);
					break;
				
				
				
				
				case OPS_OUTLET_PRESS_MAX: // �趨ѹ������������ָ��
					rt_kprintf("RTU Set SET_PRESS_SENSOR_MAX event. \n");
					rt_event_send(Schedule, SET_PRESS_SENSOR_MAX);
					break;
				
				
				
				
				case OPS_OUTLET_FLUX_MIN: // �趨������������
					rt_kprintf("RTU Set SET_FLUX_SENSOR_MIN event. \n");
					rt_event_send(Schedule, SET_FLUX_SENSOR_MIN);
					break;
				
				
				
				
				case OPS_OUTLET_FLUX_MAX: // �趨������������
					rt_kprintf("RTU Set SET_FLUX_SENSOR_MAX event. \n");
					rt_event_send(Schedule, SET_FLUX_SENSOR_MAX);
					break;
				
				
				
				
				case OPS_TRG_OUTLET_PRESS: // �趨Ŀ�����ѹ��ָ��
					rt_kprintf("RTU Set SET_PRESS_OUT event. \n");
					rt_event_send(Schedule, SET_PRESS_OUT); // ���� SET_PRESS_OUT �¼���������ʼ����Ŀ�����ѹ��
					break;
				
				
				
				
				case OPS_TRG_OUTLET_FLUX:  // �趨Ŀ���������ָ��
					rt_kprintf("RTU Set SET_FLUX event. \n");
					rt_event_send(Schedule, SET_FLUX);     // ���� SET_FLUX �¼���������ʼ����Ŀ���������
					break;
				
				
				
				
				case OPS_OUTLET_PRESS_PID: // �趨����ѹ������PIDָ��
					rt_kprintf("RTU Set SET_PRESS_PID event. \n");
					rt_event_send(Schedule, SET_PRESS_PID);
					break;
				
				
				
				
				case OPS_OUTLET_FLUX_PID:  // �趨������������PIDָ��
					rt_kprintf("RTU Set SET_FLUX_PID event. \n");
					rt_event_send(Schedule, SET_FLUX_PID);
					break;
				
				
				
				
				case OPS_QUIT_PID:	// �˳�PID����ģʽָ��
					rt_kprintf("RTU Set QUIT_PID event. \n");
					rt_event_send(Schedule, QUIT_PID);    // ���� QUIT_PID �¼��������˳�PID����ģʽ
					break;
				
				
				
				
				case OPS_PRESENT_OUTLET_PRESS: // �趨��ǰ����ѹ��ָ��
					rt_kprintf("RTU Set CALI_PRESENT_OUT_PRESS event. \n");
					rt_event_send(Schedule, CALI_PRESENT_OUT_PRESS);
					break;
				
				
				
				
				case OPS_PRESENT_OUTLET_FLUX:  // �趨��ǰ��������ָ��
					rt_kprintf("RTU Set CALI_PRESENT_FLUX event. \n");
					rt_event_send(Schedule, CALI_PRESENT_FLUX);
					break;
					
				
				
				
				case OPS_PRESENT_INLET_PRESS:  // �趨��ǰ���ѹ��ָ��
					rt_kprintf("RTU Set CALI_PRESENT_IN_PRESS event. \n");
					rt_event_send(Schedule, CALI_PRESENT_IN_PRESS);
					break;
				
				
				
				
				case OPS_MANUAL_SPEED:         // �趨����ֶ�����ʱ���ٶ�
					rt_kprintf("RTU Set CHANGE_MAN_MOVE_SPEED event. \n");
					rt_event_send(Schedule, CHANGE_MAN_MOVE_SPEED);
					break;
				
					
					
					
				case OPS_FEED_SPEED:           // �趨���PIDģʽ������ʱ���ٶ�
					rt_kprintf("RTU Set CHANGE_FEED_MODE_SPEED event. \n");
					rt_event_send(Schedule, CHANGE_FEED_MODE_SPEED);
					break;
				
					
					
					
				case OPS_CURRENT:              // �趨������е���
					rt_kprintf("RTU Set CHANGE_CURRENT event. \n");
					rt_event_send(Schedule, CHANGE_CURRENT);
					break;
				
					
					
				case OPS_MANUAL_ACC:           // �趨����ֶ�����ʱ�ļ��ٶ�
					rt_kprintf("RTU Set CHANGE_MAN_ACC event. \n");
					rt_event_send(Schedule, CHANGE_MAN_ACC);
					break;
				
					
					
				case OPS_FEED_ACC:              // �趨���PIDģʽ������ʱ�ļ��ٶ�
					rt_kprintf("RTU Set CHANGE_FEED_ACC event. \n");
					rt_event_send(Schedule, CHANGE_FEED_ACC);
					break;
				
					
					
					
				case OPS_SET_PRESS_THRESHOLD:				// �趨ѹ��������ֵ
					rt_kprintf("RTU Set CHANGE_PRESS_THRESHOLD event. \n");
					rt_event_send(Schedule, CHANGE_PRESS_THRESHOLD);
					break;
				
				
				
				
				case OPS_SET_FLUX_THRESHOLD:        // �趨����������ֵ	
					rt_kprintf("RTU Set CHANGE_FLUX_THRESHOLD event. \n");
					rt_event_send(Schedule, CHANGE_FLUX_THRESHOLD);
					break;
				
				
				
				case OPS_TRG_POS:               // ���е��ض���оλ��
					rt_kprintf("RTU Set MOVE_TO_TRG_POS event. \n");
					rt_event_send(Schedule, MOVE_TO_TRG_POS);
					break;
				
					
					
				case OPS_TEST:									// ��������˶����Ȳ���
					if(src[4] == 0xFF)
					{
						rt_event_send(Schedule2, START_TEST);
					}
					else
					{
						rt_event_recv(Schedule2, START_TEST, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, 0, &recved_event);
					}
					break;
				
				
				case OPS_SHUT_DOWN:							// �����ͣ
					rt_kprintf("RTU Set SHUT_DOWN_EVENT event. \n");
					rt_event_send(Schedule, SHUT_DOWN_EVENT);
					break;
					
				
				case OPS_MOTOR_DEBUG_MODE:			// ���͸��ģʽ��ֱ�Ӹ���������ַ���ָ��
					if(src[4] == 0xFF)
					{
						rt_kprintf("Start Motor Debug Mode. \n");
						// ���ȹر��߳�����
						rt_kprintf("Shut Down Thread. \n");
						rt_event_recv(Schedule, THREAD_GET_WORK, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, 0, &recved_event);
					}
					else
					{
						rt_kprintf("Back to Normal Mode, Thread Get Work. \n");
						rt_event_send(Schedule, THREAD_GET_WORK);
					}
					break;
				
					
				case OPS_SEND_DIR_MOTOR_CMD:
					if(src[4] == 0xFF)
					{
						rt_kprintf("RTU Set MOTOR_DIRECT_CMD event. \n");
						rt_event_send(Schedule2, MOTOR_DIRECT_CMD);
					}
					break;
					
				default:
					break;
			}
			
			break;

		default:
			break;
	}
}


// �궨ADC����
/*
����˵����
	now_enc_val_addr�� 			�������̶߳�����ԭʼ ADC ��������ֵ��ŵļĴ�����ַ
	userSet_phy_val_addr��	�û����õĵ�ǰ ADC ����������ŵļĴ�����ַ
	phy_upperBound_addr��		��ǰ ADC �����������������޼Ĵ�����ַ
	phy_lowerBound_addr��   ��ǰ ADC �����������������޼Ĵ�����ַ
	EncHigh��								�ϱ궨�㴦�� ADC ����ֵ
	PhyHigh��								�ϱ궨�㴦�� ADC ������ֵ
	EncLow��								�±궨�㴦�� ADC ����ֵ
	PhyLow��								�±궨�㴦�� ADC ������ֵ
*/
void CaliPresentADC(uint32_t now_enc_val_addr, uint32_t userSet_phy_val_addr, uint32_t phy_upperBound_addr, uint32_t phy_lowerBound_addr, uint32_t EncHigh_eeaddr, uint32_t PhyHigh_eeaddr, uint32_t EncLow_eeaddr, uint32_t PhyLow_eeaddr, uint32_t coeff_addr, float* EncHigh, float* PhyHigh, float* EncLow, float* PhyLow, float* convert_coeff)
{
	int ee_ret;
	
	float raw_enc = GetFloatFromReg(now_enc_val_addr);						// ��ǰADC����ֵ���� 
	float cali_phy = GetFloatFromReg(userSet_phy_val_addr);				// �û�����ĵ�ǰҪ�궨��ADC��������ֵ
	
	float min = GetFloatFromReg(phy_lowerBound_addr);				// ADC����������
	float max = GetFloatFromReg(phy_upperBound_addr);				// ADC����������
	float range = max - min;																	// ������������������
		
	if((cali_phy - min) > SPLITE_POINT*range)	// �����趨�ķ�Χ����ô��˵���궨�����ϱ궨��
	{			
		rt_kprintf("Now Calibrating ADC Seensor Upper Point.  \n");
		
		// ����ǰ�� ADC ����ֵ ���� �ϱ궨��ı���ֵ
		*EncHigh = raw_enc;
		ee_ret = WriteFloatToEEPROME(EncHigh_eeaddr, raw_enc);
		if(ee_ret)
			rt_kprintf("Write Current ADC Enc Val(int cast): %d To EEPROME Successfully!! \n", (int)raw_enc);
		else
			rt_kprintf("Write Current ADC Enc Val To EEPROME Fail!! \n");
			
		// ����ǰ�� ADC ������ֵ ���� �ϱ궨���������ֵ
		*PhyHigh = cali_phy;
		ee_ret = WriteFloatToEEPROME(PhyHigh_eeaddr, cali_phy);
		if(ee_ret)
			rt_kprintf("Write Current ADC Phy Val(int cast): %d To EEPROME Successfully!! \n", (int)cali_phy);
		else
			rt_kprintf("Write Current ADC Phy Val To EEPROME Fail!! \n");
	}
	else																	// û�г����趨�ķ�Χ���궨�����±궨��
	{
		rt_kprintf("Now Calibrating ADC Seensor Lower Point.  \n");
		
		// ����ǰ�� ADC ����ֵ ���� �±궨��ı���ֵ
		*EncLow = raw_enc;
		ee_ret = WriteFloatToEEPROME(EncLow_eeaddr, raw_enc);
		if(ee_ret)
			rt_kprintf("Write Current ADC Enc Val(int cast): %d To EEPROME Successfully!! \n", (int)raw_enc);
		else
			rt_kprintf("Write Current ADC Enc Val To EEPROME Fail!! \n");
			
		// ����ǰ�� ADC ������ֵ ���� �±궨���������ֵ
		*PhyLow = cali_phy;
		ee_ret = WriteFloatToEEPROME(PhyLow_eeaddr, cali_phy);
		if(ee_ret)
			rt_kprintf("Write Current ADC Phy Val(int cast): %d To EEPROME Successfully!! \n", (int)cali_phy);
		else
			rt_kprintf("Write Current ADC Phy Val To EEPROME Fail!! \n");
	}
	
	// ����ת��ϵ��
	*convert_coeff = (*EncHigh - *EncLow) / (*PhyHigh - *PhyLow);
	ee_ret = WriteFloatToEEPROME(coeff_addr, *convert_coeff);
	if(ee_ret)
		rt_kprintf("Update convert_coeff(int cast): %d To EEPROME Successfully!! \n", (int)*convert_coeff);
	else
		rt_kprintf("Update convert_coeff To EEPROME Fail!! \n");
}




void DataInitialize()
{
		float tmp;
		int ret_val;
	
		ReadEncoderDataToMotor();                        								// ����������ֵ������
			
		ret_val = GetFloatFromMotor("JS", &tmp);																	// �ӵ����ȡ �ֶ����ڷ�оλ��ʱ���ٶ�
		if(ret_val)
			WriteFloatToReg(MANUAL_SPEED, tmp);
		else
			rt_kprintf("Read manual speed fail !! \n");
				
		ret_val = GetFloatFromMotor("VE", &tmp);																	// �ӵ����ȡ ���PID����ģʽ�µ��ٶ�
		if(ret_val)
			WriteFloatToReg(PID_SPEED, tmp);
		else
			rt_kprintf("Read PID mode speed fail !! \n");
				
		ret_val = GetFloatFromMotor("CC", &tmp);																	// �ӵ����ȡ ������е���
		if(ret_val)
			WriteFloatToReg(CURRENT, tmp);
		else
			rt_kprintf("Read current fail !! \n");
				
		ret_val = GetFloatFromMotor("JA", &tmp);																	// �ӵ����ȡ �ֶ����ڷ�оλ��ʱ�ļ��ٶ�
		if(ret_val)
			WriteFloatToReg(MANUAL_ACC, tmp);
		else
			rt_kprintf("Read manual acceleration fail !! \n");
				
		ret_val = GetFloatFromMotor("AC", &tmp);																	// �ӵ����ȡ ���PID����ģʽ�µļ��ٶ�
		if(ret_val)
			WriteFloatToReg(PID_ACC, tmp);
		else
			rt_kprintf("Read PID mode acceleration fail !! \n");
				
		ReadFloatFromEEPROME(OUTLET_PRESS_MIN_ADDR, &tmp); 								// ��ȡ����ѹ������
		last_press_min = tmp;
		WriteFloatToReg(DEF_OUTLET_PRESS_MIN, tmp);
		//printf("Read outlet pressure min: %f \n", tmp);
		ReadFloatFromEEPROME(OUTLET_PRESS_MAX_ADDR, &tmp); 								// ��ȡ����ѹ������
		last_press_max = tmp;
		WriteFloatToReg(DEF_OUTLET_PRESS_MAX, tmp);
		//printf("Read outlet pressure max: %f \n", tmp);
		ReadFloatFromEEPROME(OUTLET_FLUX_MIN_ADDR, &tmp);  								// ��ȡ������������
		last_flux_min = tmp;
		WriteFloatToReg(DEF_OUTLET_FLUX_MIN, tmp);
		//printf("Read outlet flux min: %f \n", tmp);
		ReadFloatFromEEPROME(OUTLET_FLUX_MAX_ADDR, &tmp);  								// ��ȡ������������
		last_flux_max = tmp;
		WriteFloatToReg(DEF_OUTLET_FLUX_MAX, tmp);
		//printf("Read outlet flux max: %f \n", tmp);
		ReadFloatFromEEPROME(OUTPRESS_P, &tmp);														// ��ȡ����ѹ������PID��P
		WriteFloatToReg(OUTLET_PRESS_P, tmp);
		//printf("Read outlet pressure PID's P: %f \n", tmp);
		ReadFloatFromEEPROME(OUTPRESS_I, &tmp);														// ��ȡ����ѹ������PID��I
		WriteFloatToReg(OUTLET_PRESS_I, tmp);
		//printf("Read outlet pressure PID's I: %f \n", tmp);
		ReadFloatFromEEPROME(OUTPRESS_D, &tmp);														// ��ȡ����ѹ������PID��D
		WriteFloatToReg(OUTLET_PRESS_D, tmp);
		//printf("Read outlet pressure PID's D: %f \n", tmp);
		ReadFloatFromEEPROME(OUTFLUX_P, &tmp);														// ��ȡ������������PID��P
		WriteFloatToReg(OUTLET_FLUX_P, tmp);
		//printf("Read outlet flux PID's P: %f \n", tmp);
		ReadFloatFromEEPROME(OUTFLUX_I, &tmp);														// ��ȡ������������PID��I
		WriteFloatToReg(OUTLET_FLUX_I, tmp);
		//printf("Read outlet flux PID's I: %f \n", tmp);
		ReadFloatFromEEPROME(OUTFLUX_D, &tmp);														// ��ȡ������������PID��D
		WriteFloatToReg(OUTLET_FLUX_D, tmp);
		//printf("Read outlet flux PID's D: %f \n", tmp);
		ReadFloatFromEEPROME(EE_PRESS_CTR_THRESHOLD, &tmp);								// ��ȡ����ѹ��������ֵ
		WriteFloatToReg(OUT_PRESS_CTR_THRESHOLD, tmp);
		press_threshold = tmp;
		//printf("Read outlet pressure control threshold: %f \n", press_threshold);
		ReadFloatFromEEPROME(EE_FLUX_CTR_THRESHOLD, &tmp);								// ��ȡ��������������ֵ
		WriteFloatToReg(OUT_FLUX_CTR_THRESHOLD, tmp);
		flux_threshold = tmp;
		//printf("Read outlet flux control threshold: %f \n", flux_threshold);
		
		ReadFloatFromEEPROME(ENC_TO_POS, &coeff_env_to_valve); 						// ��ȡ �ӱ�����ֵת������оλ�õ�ת��ϵ��
		rt_kprintf("Read coeff_env_to_valve(int cast): %d \n", (int)coeff_env_to_valve);
		ReadFloatFromEEPROME(ADC_TO_PRESS_OUTLET, &adc_to_press_outlet);  // ��ȡ ��ADC�Ķ���ת��������ѹ����ת��ϵ��
		rt_kprintf("Read adc_to_press_outlet(int cast): %d \n", (int)adc_to_press_outlet);
		ReadFloatFromEEPROME(ADC_TO_FLUX_OUTLET, &adc_to_flux_outlet);    // ��ȡ ��ADC�Ķ���ת��������������ת��ϵ��
		rt_kprintf("Read adc_to_flux_outlet(int cast): %d \n", (int)adc_to_flux_outlet);
		ReadFloatFromEEPROME(ADC_TO_PRESS_INLET, &adc_to_press_inlet);    // ��ȡ ��ADC�Ķ���ת�������ѹ����ת��ϵ��
		rt_kprintf("Read adc_to_press_inlet(int cast): %d \n", (int)adc_to_press_inlet);
		
		ReadFloatFromEEPROME(PRESS_IN_PHY_LOW, &press_inlet_phy_low); 
		rt_kprintf("Read press_inlet_phy_low(int cast): %d \n", (int)press_inlet_phy_low);
		ReadFloatFromEEPROME(PRESS_IN_PHY_HIGH, &press_inlet_phy_high); 
		rt_kprintf("Read press_inlet_phy_high(int cast): %d \n", (int)press_inlet_phy_high);
		ReadFloatFromEEPROME(PRESS_OUT_PHY_LOW, &press_outlet_phy_low);   
		rt_kprintf("Read press_outlet_phy_low(int cast): %d \n", (int)press_outlet_phy_low);
		ReadFloatFromEEPROME(PRESS_OUT_PHY_HIGH, &press_outlet_phy_high); 
		rt_kprintf("Read press_outlet_phy_high(int cast): %d \n", (int)press_outlet_phy_high);
		ReadFloatFromEEPROME(FLUX_PHY_LOW, &flux_phy_low);    						
		rt_kprintf("Read flux_phy_low(int cast): %d \n", (int)flux_phy_low);
		ReadFloatFromEEPROME(FLUX_PHY_HIGH, &flux_phy_high);    					
		rt_kprintf("Read flux_phy_high(int cast): %d \n", (int)flux_phy_high);
		ReadFloatFromEEPROME(PRESS_IN_ENC_LOW, &press_inlet_enc_low);    				
		rt_kprintf("Read press_inlet_enc_low(int cast): %d \n", (int)press_inlet_enc_low);
		ReadFloatFromEEPROME(PRESS_IN_ENC_HIGH, &press_inlet_enc_high);    			
		rt_kprintf("Read press_inlet_enc_high(int cast): %d \n", (int)press_inlet_enc_high);
		ReadFloatFromEEPROME(PRESS_OUT_ENC_LOW, &press_outlet_enc_low);    			
		rt_kprintf("Read press_outlet_enc_low(int cast): %d \n", (int)press_outlet_enc_low);
		ReadFloatFromEEPROME(PRESS_OUT_ENC_HIGH, &press_outlet_enc_high);
		rt_kprintf("Read press_outlet_enc_high(int cast): %d \n", (int)press_outlet_enc_high);
		ReadFloatFromEEPROME(FLUX_ENC_LOW, &flux_enc_low);
		rt_kprintf("Read flux_enc_low(int cast): %d \n", (int)flux_enc_low);
		ReadFloatFromEEPROME(FLUX_ENC_HIGH, &flux_enc_high);
		rt_kprintf("Read flux_enc_high(int cast): %d \n", (int)flux_enc_high);
}

void FeedLenEnc(int32_t enc_val)
{
	char enc_val_str[32];
	int ret_val;
	
	// �� int ����ֵת�����ַ�����ʽ
	sprintf(enc_val_str, "%d", enc_val);
	
	// �� FL ��������
	ret_val = ToMotorNoReturnWithParameter("FL", enc_val_str);
	if(ret_val == 1)
		rt_kprintf("Send FL %s Successfully !! \n", enc_val_str);
	else if(ret_val == 2)
		rt_kprintf("Send FL %s, but Motor Is Busy, This shouldn't happend because we have MOV_STA event, Weired!! \n", enc_val_str);
	else
		rt_kprintf("Send FL command Fail!! \n");
}



// ������оλ�ã�Ȼ��ͨ������λ��ֵ����
void FeedPosValve(float pos)
{
	char feed_val_str[32];
	int ret_val;
	
	// ����Ŀ��λ������ȡĿ��λ�õı���ֵ
	rt_kprintf("The Trg Pos(int cast): %d \n", (int)pos);
	float enc_f = (pos * coeff_env_to_valve);
	
	// ע������ IE �� FP �ĵ�λ��ͬ���������ﻹ��Ҫ��һ��ת������ IE ��ֵתΪ FP ��ֵ��
	int32_t feed_val = (int32_t)(enc_f * IP_TO_EP);
	
	// ����ֵת��Ϊ�ַ�����ʽ
	sprintf(feed_val_str, "%d", feed_val);
	
	// �ȷ���һ��SKDָ��Ӷ���֤���������ִ������� FP ָ��
	ret_val = ToMotorNoReturnNoParameter("SKD");
	if(ret_val)
		rt_kprintf("Send SKD Successfully !! \n");
	else
		rt_kprintf("Send SKD Fail !! \n");
	
	// ���� FP ����õ���˶����ض�λ��
	ret_val = ToMotorNoReturnWithParameter("FP", feed_val_str);
	if(ret_val)
		rt_kprintf("Send FP %s Successfully !! \n", feed_val_str);
	else
		rt_kprintf("Send FP command Fail !! \n");
}



// �궨������������
int CaliSensor(uint32_t bound_reg_addr, uint32_t bound_ee_addr, uint32_t convert_coeff_ee_addr, float* old_lower_bound, float* old_upper_bound, float* convert_coeff, int type)
{
	float err;
	float* old_val;
	int ee_ret;
	int ret_val;
	
	if(type)	// �궨��������
	{
		old_val = old_upper_bound;
		rt_kprintf("Calibrating Sensor Upper Bound. \n");
	}
	else			// �궨��������
	{
		old_val = old_lower_bound;
		rt_kprintf("Calibrating Sensor Lower Bound. \n");
	}
	
	// �Ƚ��趨ֵ�;ɵ�ֵ���бȽϣ�������߲���С�Ļ����ܿ������û�����˼���ȷ������ô�Ͳ��ظ��趨��
	// �൱���ṩ��һ�� short circuit ����
//	err = *old_val - GetFloatFromReg(bound_reg_addr);
//	err = err < 0? -err : err;
//	if(err < 0.01) // С��0.01�Ļ�����Ϊ�趨��ֵ����һ�μ�¼��ֵһ��
//	{
//		// �ָ��Ĵ�����ֵ
//		rt_kprintf("New Value is too close to the old value. Abandon it!!  \n");
//		WriteFloatToReg(bound_reg_addr, *old_val);
//		ret_val = 0;
//	}
//	else
	{
		// ���� old_value Ϊ���µ�ֵ
		*old_val = GetFloatFromReg(bound_reg_addr);
					
		// �� �����������޶�Ӧ�� �Ĵ�����������ֵ����EEPROME
		ee_ret = WriteFloatToEEPROME(bound_ee_addr, *old_val);
		if(ee_ret)
		{
			rt_kprintf("Write New Value(int cast): %d to EEPROME Successfully!! \n", (int)(*old_val));
			ret_val = 1;
		}
		else
		{
			rt_kprintf("Write New Value to EEPROME Fail!! \n");
			ret_val = 0;
		}
		
		//�������̸��ˣ����Ը��� ���±궨�� �Ӷ��������ı궨��� ADC ������Ӱ��
		switch(bound_reg_addr)
		{
			case DEF_OUTLET_PRESS_MAX:
				press_inlet_phy_high = *old_val;
				WriteFloatToEEPROME(PRESS_IN_PHY_HIGH, press_inlet_phy_high);
				press_inlet_enc_high = PRESS_IN_RANGE_HIGH;
				WriteFloatToEEPROME(PRESS_IN_ENC_HIGH, press_inlet_enc_high);
				press_outlet_phy_high = *old_val;
				WriteFloatToEEPROME(PRESS_OUT_PHY_HIGH, press_outlet_phy_high);
				press_outlet_enc_high = PRESS_OUT_RANGE_HIGH;
				WriteFloatToEEPROME(PRESS_OUT_ENC_HIGH, press_outlet_enc_high);
				break;
			
			case DEF_OUTLET_PRESS_MIN:
				press_inlet_phy_low = *old_val;
				WriteFloatToEEPROME(PRESS_IN_PHY_LOW, press_inlet_phy_low);
				press_inlet_enc_low = PRESS_IN_RANGE_LOW;
				WriteFloatToEEPROME(PRESS_IN_ENC_LOW, press_inlet_enc_low);
				press_outlet_phy_low = *old_val;
				WriteFloatToEEPROME(PRESS_OUT_PHY_LOW, press_outlet_phy_low);
				press_outlet_enc_low = PRESS_OUT_RANGE_LOW;
				WriteFloatToEEPROME(PRESS_OUT_ENC_LOW, press_outlet_enc_low);
				break;
			
			case DEF_OUTLET_FLUX_MAX:
				flux_phy_high = *old_val;
				WriteFloatToEEPROME(FLUX_PHY_HIGH, flux_phy_high);
				flux_enc_high = FLUX_RANGE_HIGH;
				WriteFloatToEEPROME(FLUX_ENC_HIGH, flux_enc_high);
				break;
			
			case DEF_OUTLET_FLUX_MIN:
				flux_phy_low = *old_val;
				WriteFloatToEEPROME(FLUX_PHY_LOW, flux_phy_low);
				flux_enc_low = FLUX_RANGE_LOW;
				WriteFloatToEEPROME(FLUX_ENC_LOW, flux_enc_low);
				break;
		}
		
		// ����ת��ϵ��һ�����
		adc_to_press_inlet = (press_inlet_enc_high - press_inlet_enc_low) / (press_inlet_phy_high - press_inlet_phy_low);
		adc_to_press_outlet = (press_outlet_enc_high - press_outlet_enc_low) / (press_outlet_phy_high - press_outlet_phy_low);
		adc_to_flux_outlet = (flux_enc_high - flux_enc_low) / (flux_phy_high - flux_phy_low);
		WriteFloatToEEPROME(ADC_TO_PRESS_INLET, adc_to_press_inlet);
		WriteFloatToEEPROME(ADC_TO_PRESS_OUTLET, adc_to_press_outlet);
		WriteFloatToEEPROME(ADC_TO_FLUX_OUTLET, adc_to_flux_outlet);
		rt_kprintf("adc_to_press_inlet: %d \n", (int)adc_to_press_inlet);
		rt_kprintf("adc_to_press_outlet: %d \n", (int)adc_to_press_outlet);
		rt_kprintf("adc_to_flux_outlet: %d \n", (int)adc_to_flux_outlet);
		
//		// ����ת��ϵ��
//		*convert_coeff = (float)(RANGE_HIGH - RANGE_LOW) / (*old_upper_bound - *old_lower_bound);

//		// �����º�� ת��ϵ�� ��ֵ���浽EEPROME
//		ee_ret = WriteFloatToEEPROME(convert_coeff_ee_addr, *convert_coeff);
//		if(ee_ret)
//		{
//			rt_kprintf("Save Convert Coefficient(int cast): %d To EEPROME Successfully\n", (int)*convert_coeff);
//			ret_val = 1;
//		}
//		else
//		{
//			rt_kprintf("Save Convert Coefficient To EEPROME Fail\n");
//			ret_val = 0;
//		}
	}
	
	return ret_val;
}


void SetThreshold(uint32_t reg_addr, uint32_t ee_addr, float* threshold)
{
	int ee_ret;
	
	// ���Ĵ����е���ֵ��ȫ�ֱ���
	*threshold = GetFloatFromReg(reg_addr);
			
	// ����ֵ����EEPROME�������´ζ���
	ee_ret = WriteFloatToEEPROME(ee_addr, *threshold);
	if(ee_ret)
		rt_kprintf("Save threshold Successfully !! \n");
	else
		rt_kprintf("Save threshold Fail !! \n");
}

