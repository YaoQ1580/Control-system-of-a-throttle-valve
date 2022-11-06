#include "miscellaneousTools.h"
#include "GlobalVariable.h" 

/*------------------辅助函数-------------------*/
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

// src表示一个uint8_t的数组，cnt表示这个数组的长度，该函数将src里面的内容转化成hex形式的字符串，例：src[] = "0x01, 0x02, 0x03"，那么dst[] = "01 02 03"
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

// 将32位数据分割成4个8位数据，举例：src = 0x12345678，则dst[] = {0x78, 0x56, 0x34, 0x12};
void itob(int32_t src, uint8_t* dst)
{
	dst[0] = src & 0xFF;
	dst[1] = (src >> 8) & 0xFF;
	dst[2] = (src >> 16) & 0xFF;
	dst[3] = (src >> 24) & 0xFF;
}

// 将4个8位数据转化成一个32位数据，举例src[] = {0x78, 0x56, 0x34, 0x12}，则函数返回0x12345678;
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
	i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(I2C_BUS_NAME); // 查找I2C设备，并返回句柄
	
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




// 将int型数据写入EEPROME，这个应该是多线程安全的
int WriteInt32ToEEPROME(uint32_t addr, int32_t num)
{
	int32_t read_data;
	
	// 通讯之前拿互锁量，拿不到的话永远等待，因为 I2C 一定会通讯返回
	rt_mutex_take(I2CBus, RT_WAITING_FOREVER);
	
	// 将数据写入EEPROME
	write_AT24C02(addr, num);
	// 等待缓冲区数据写完
	rt_thread_mdelay(I2C_WRITE_DELAY);
	// 读取EEPROME看是否写进去了
	read_AT24C02(addr, &read_data);
	
	// 把锁释放掉
	rt_mutex_release(I2CBus);
	
	if(read_data == num)  // 写进去了
	{
		return 1;
	}
	else									// 出现写错误
	{
		return 0;
	}
}





// 将float型数据写入EEPROME，这个应该是多线程安全的
int WriteFloatToEEPROME(uint32_t addr, float num)
{
	int32_t tmp = *((int*)&num); // 将float数据的二进制编码转送到一个int变量中，方便之后在bit层面进行数据操作
	
	return WriteInt32ToEEPROME(addr, tmp);
}




// 从 EEPROME 中读取 int 数据，这个应该是多线程安全的
int ReadInt32FromEEPROME(uint32_t addr, int32_t* num)
{
	rt_err_t rt_ret;
	
	// 通讯之前拿互锁量，拿不到的话永远等待，因为 I2C 一定会通讯成功的
	rt_mutex_take(I2CBus, RT_WAITING_FOREVER);
	
	// 读取数据
	rt_ret = read_AT24C02(addr, num);
	
	// 读完之后释放锁
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




// 从 EEPROME 中读取 float 数据，这个应该是多线程安全的
int ReadFloatFromEEPROME(uint32_t addr, float* num)
{
	int ee_ret;
	int32_t recev_data;
	
	ee_ret = ReadInt32FromEEPROME(addr, &recev_data);
	// 将返回值解释成float
	*num = *((float*)&recev_data);
	
	return ee_ret;
}




// 将int型的数值num写入起始地址为addr的EEPROME存储区，least significant字节在先
// 注意: 电机返回的编码器位置是字符串形式，所以调用此函数之前需要先将其转化成Int型之后才可以存储到EEPROME
rt_err_t write_AT24C02(uint32_t addr, int32_t num)
{
	uint8_t data[5];
	struct rt_i2c_msg msgs;
	
	// 将int型转化成4字节数组
	data[0] = addr;      // EEPROME中的地址
	itob(num, data + 1);
	
	// 生成I2C消息
	msgs.addr = AT24C02_ADDR;
	msgs.flags = RT_I2C_WR;
  msgs.buf = data;
  msgs.len = 5;
	
	// 发送数据
	if(rt_i2c_transfer(i2c_bus, &msgs, 1) == 1)
  {
    return RT_EOK;
  }
  else
  {
    return -RT_ERROR;
  }
}




// 将float型的数值num写入起始地址为addr的EEPROME存储区，least significant字节在先
rt_err_t write_AT24C02_float(uint32_t addr, float num)
{
	int32_t tmp = *((int*)&num); // 将float数据的二进制编码转送到一个int变量中，方便之后在bit层面进行数据操作
	
	return write_AT24C02(addr, tmp); // 调用write
}




// 从起始地址为addr的EEPROME存储区读取四个字节并将其按照int型对待
rt_err_t read_AT24C02(uint32_t addr, int32_t* dst)
{
	uint8_t ee_addr = addr;
	struct rt_i2c_msg msgs[2];
	uint8_t data[4];
	
	// 首先发送一个命令告诉EEPROME我们将对首地址addr进行读操作
	msgs[0].addr = AT24C02_ADDR;
	msgs[0].flags = RT_I2C_WR;
  msgs[0].buf = &ee_addr;
  msgs[0].len = 1;
	
	// 之后我们连续读取四个字节
	msgs[1].addr = AT24C02_ADDR;
	msgs[1].flags = RT_I2C_RD;
  msgs[1].buf = data;
  msgs[1].len = 4;
	
	if(rt_i2c_transfer(i2c_bus, msgs, 2) == 2)
  {
		*dst = btoi(data); // 读取成功的话，将四字节拼凑成int型返回
		
    return RT_EOK;
  }
  else
  {
    return -RT_ERROR;
  }
}




// 从起始地址为addr的EEPROME存储区读取四个字节并将其按照float型对待
rt_err_t read_AT24C02_float(uint32_t addr, float* dst)
{
	rt_err_t ack;
	int32_t tmp;
	
	ack = read_AT24C02(addr, &tmp); // 先从EEPROME中读取int型数据
	if(ack == RT_EOK)
	{
		*dst = *((float*)&tmp);       // 将该int型二进制编码当作float进行处理
	}
	
	return ack;
}




// 将modbus寄存器中的float数值写入起始地址为addr的EEPROME存储区
int WriteFloatRegToEeprome(uint32_t reg_addr, uint32_t ee_addr)
{
	float data;
	data = GetFloatFromReg(reg_addr);    // 从寄存器中读取float数值
	
	return WriteFloatToEEPROME(ee_addr, data);  // 将 float 数值写入 EEPROME
}




/*-------------------电机通讯相关------------------*/
// 将数据串指针写入线程间通讯邮箱
void SendTxtMail(rt_mailbox_t mail, uint8_t* src)
{
	rt_ubase_t* recev;
	char* to_sent;
	rt_err_t uwRet = RT_EOK;
	
	// 首先检查通讯邮箱是否有之前没有读取的垃圾邮件
	uwRet = rt_mb_recv(mail, recev, 0);
	if(uwRet == RT_EOK) 
	{
		rt_kprintf("Motor Thread find some garbage mail !! \n");
		rt_free(recev); // 如果有垃圾邮件，释放邮件的堆空间
	}
	
	// 如果没有垃圾邮件，那么将所要发送的数据放到邮箱里面
	to_sent = rt_strdup((const char*)src);
	rt_mb_send(mail, (rt_uint32_t)to_sent);
}




// 将指令发送给电机然后去邮箱等待电机的返回数据
// 尝试 CHECK_MAIL 次，可能的问题：WAIT_MAIL 时间太短导致重新发送消息
int SafelySendCmdAndReadAck(char* cmd, char* dst) // don't forget to free heap
{
	int check = CHECK_MAIL;
	int str_len;
	rt_err_t uwRet = RT_EOK;
	uint8_t* tmp;
	int index, i;
	
	// 首先检查通讯邮箱是否有之前没有读取的垃圾邮件
	uwRet = rt_mb_recv(ack, (rt_ubase_t*)&tmp, 0); 
	if(uwRet == RT_EOK) // there exists some garbage
	{
		rt_kprintf("ReadMotorReturn find some garbage mail!! \n");
		rt_free(tmp);  // 如果有垃圾邮件，释放邮件的堆空间
	}

	while(check > 0)
	{
		rt_device_write(motor, 0, cmd, rt_strlen((const char*)cmd));  // 向电机发送命令
		uwRet = rt_mb_recv(ack, (rt_ubase_t*)&tmp, WAIT_MAIL);  			// 等待邮箱接收邮件，等待超时时间为 WAIT_MAIL
		if(uwRet == RT_EOK) // 如果在超时时间内收到邮件
		{
			str_len = rt_strlen((const char*)tmp);
			rt_memcpy(dst, tmp, str_len); // 将应答邮件数据拷贝到dst
			dst[str_len] = '\0';
			rt_free(tmp);                        // 释放邮件的堆空间
			
			// 因为调试需要，这里将 dst 发送到寄存器上面，因为只有这里会对这些寄存器进行写操作，所以就不进 critical section 了
			// 注意：要先写高 8 位
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
			return 1;                            // 返回1表示读取邮件成功
		}
		check--; // 如果应答错误，则再次尝试
		
		rt_kprintf("Send Cmd: %s. fail to receive the %dnd ack, try again.\n", cmd, CHECK_MAIL-check);
	}
	
	dst[0] = '\0';
	Reg[MOTOR_RET_STR] = '\0';  // 因为 reg 是 short 类型的，所以这里实际上是设置了两个 '\0'
	return 0; // 通讯错误，返回0
}




// 将指令发送给电机（无应答，无参数）
// 指令不需要加从机地址！！！
int ToMotorNoReturnNoParameter(char* cmd)
{
	rt_err_t uwRet = RT_EOK; 	// 确认是否拿到互锁量
	int uwRet_cmu = 0;  			// 确认是否通讯成功
	int ret_val = 0;         	// 函数返回值
	char final_cmd[32];		// 对 cmd 进行再加工
	int cmd_len = rt_strlen((const char*)cmd); // cmd 的长度
	char check[5];            // 检查应答
	
	// 将 cmd 加上 head 和 tail
	final_cmd[0] = MOTOR_ADDR;
	rt_memcpy(final_cmd + 1, cmd, cmd_len);
	final_cmd[1 + cmd_len] = 0x0D;
	final_cmd[2 + cmd_len] = '\0';
	
	// 通讯之前拿互锁量，等待超时为 WAIT_485
	uwRet = rt_mutex_take(MotorBus, WAIT_485);
	
	// 如果成功拿到互锁量
	if(uwRet == RT_EOK)
	{
		uwRet_cmu = SafelySendCmdAndReadAck(final_cmd, check); // 向电机发送命令
		
		if(uwRet_cmu == 0)  // 通讯失败
		{
			rt_kprintf("Fatal communication failure!!!! \n");
			ret_val = 0;
		}
		else		// 通讯成功，看电机返回值
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
		
		rt_mutex_release(MotorBus);		// 释放互锁量
	}
	else   // 没有成功拿到互锁量，等待超时
	{
		rt_kprintf("ToMotorNoReturn: Waiting 485 time out !!! \n");
		ret_val = 0;
	}
	
	// 更新寄存器的值
	WriteShortToReg(COMMUNICATION_STA, ret_val);
	return ret_val;
}




// 将指令发送给电机（无应答，有参数）
// 指令不需要加从机地址！！！
int ToMotorNoReturnWithParameter(char* cmd, char* parameter)
{
	char combined_cmd[32];
	int cmd_len = rt_strlen((const char*)cmd);
	int para_len = rt_strlen((const char*)parameter);
	
	// 将 cmd 和 参数 进行合并
	rt_memcpy(combined_cmd, cmd, cmd_len);
	rt_memcpy(combined_cmd + cmd_len, parameter, para_len);
	combined_cmd[cmd_len + para_len] = '\0';
	
	return ToMotorNoReturnNoParameter(combined_cmd);
}




// 将指令发送给电机（有应答，无参数）
// 指令不需要加从机地址！！！
int ToMotorWithReturn(char* cmd, char* dst)
{
	rt_err_t uwRet = RT_EOK;    // 确认是否拿到互锁量
	int ret_val = 0;						// 返回值
	int uwRet_cmu = 0;					// 确认是否通讯成功
	char final_cmd[32];			// 最终发给电机的指令
	int cmd_len = rt_strlen((const char*)cmd);
	
	// 将 cmd 加上 head 和 tail
	final_cmd[0] = MOTOR_ADDR;
	rt_memcpy(final_cmd + 1, cmd, cmd_len);
	final_cmd[1 + cmd_len] = 0x0D;
	final_cmd[2 + cmd_len] = '\0';
	
	// 通讯之前拿互锁量，等待超时为 WAIT_485
	uwRet = rt_mutex_take(MotorBus, WAIT_485);
	
	// 如果成功拿到互锁量
	if(uwRet == RT_EOK)
	{
		uwRet_cmu = SafelySendCmdAndReadAck(final_cmd, dst); // 调用 SafelySendCmdAndReadAck 来向电机发送指令
		
		if(uwRet_cmu == 0)  // 通讯失败
		{
			rt_kprintf("Fatal communication failure!!!! \n");
			ret_val = 0;
		}
		else								// 通讯成功
		{
			// 检查电机应答是否正确
			// 根据鸣志电机SCL指令的特性，我们只需要检查指令的前3个字符
			if(!rt_memcmp(dst, final_cmd, 3))   // 如果指令正确
			{
				//rt_kprintf("Get Return From Motor!! \n");
				// 将指令的头部去掉：“1CC=4” 去掉 “1CC=”，然后电机的返回还包括 0x0D，所以还需要减1
				uint8_t data_len = rt_strlen((const char*)(dst + 4)) - 1;
				rt_memcpy(dst, dst + 4, data_len);
				dst[data_len] = '\0';
				
				ret_val = 1;
			}
			else																// 返回出错
			{
				rt_kprintf("Return String Is Wrong!! \n");
				ret_val = 0;
			}
		}
		
		rt_mutex_release(MotorBus);           // 释放互锁量
	}
	else    // 没有成功拿到互锁量，等待超时
	{
		rt_kprintf("With Ack Type: Waiting 485 time out !!! \n");
		ret_val = 0;
	}
	
	// 更新寄存器的值
	WriteShortToReg(COMMUNICATION_STA, ret_val);
	return ret_val;
}




// 发送一条指令，将指令的返回值按照 float 处理
int GetFloatFromMotor(char* cmd, float* f_value)
{
	int ret_val;								// 电机通讯是否成功的应答
	char recved_data[32]; 		// 从电机获得的返回字符串
	
	// 向电机发送指令，并获取电机的应答
	ret_val = ToMotorWithReturn(cmd, recved_data);
	if(!ret_val) // 电机通讯出现问题
	{
		rt_kprintf("Try to get float from motor, but failed !!! \n");
		*f_value =  -1;
		return 0;
	}
	else				 // 通讯成功的话
	{
		sscanf((const char*)recved_data, "%f", f_value);
		return 1;
	}
}





/*-----------------保存电机编码器位置相关----------------*/
// 从EEPROME读取编码器数值并写入电机
// 该函数仅仅在上电后的初始化当中执行一次
void ReadEncoderDataToMotor()
{
	char encoder_value_str[32];
	int ret_val;
	
	// 编码器int值
	int32_t encoder_value = 0;  
	
	// 从EEPROME中读取数值
	ReadInt32FromEEPROME(ENCODER_ADDR, &encoder_value);
	
	// 将int型数转换成字符串
	sprintf(encoder_value_str, "%d", encoder_value);
	
	// 将命令发给电机，为防止死机，仅仅只发送一次 
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



// 从电机读取int型的编码器数值将其存在 env_data
// 返回 1 表示读取成功，返回 0 表示读取失败
int GetEnvFromMotor(int32_t* env_data)
{
	int ret_val = 0;						// 电机通讯是否成功
	char recved_data[32]; 			// 从电机获得的应答
	
	// 向电机发送“EP”，并获取电机的应答
	ret_val = ToMotorWithReturn("EP", recved_data);
	if(!ret_val) // 电机应答失败
	{
		rt_kprintf("Get encoder value from motor fail !!! \n");
		*env_data = 0;
		return 0;
	}
	else				 // 读取成功
	{
		sscanf(recved_data, "%d", env_data);
		return 1;
	}
}



// 将电机编码器数值保存进EEPROME
void SaveEncoderData()
{
	int32_t encoder_value;
	int ret_val;
	int ee_val;
	
	// 从电机读取int型的编码器数值
	ret_val = GetEnvFromMotor(&encoder_value);
	if(ret_val) // 如果读取成功
	{
		rt_kprintf("EP: %d is gonna saved into eeprome. \n", encoder_value);
		
		// 将数据存入EEPROME
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
// CRC校验位生成函数
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

// 从起始索引为index的modbus寄存器中写入short类型的数
void WriteShortToReg(uint8_t index, int16_t num)
{
	rt_enter_critical();
	Reg[index] = num;
	rt_exit_critical();
}

// 从起始索引为index的modbus寄存器中写入int类型的数（要写两个寄存器）
void WriteIntToReg(uint8_t index, int32_t num)
{
	rt_enter_critical();
	Reg[index] = num >> 16; // 先写高16位
	Reg[index + 1] = num;   // 再写低16位
	rt_exit_critical();
}

// 从起始索引为index的modbus寄存器中写入float类型的数（要写两个寄存器）
void WriteFloatToReg(uint8_t index, float num)
{
	uint32_t tmp = *((uint32_t*)&num);
	
	rt_enter_critical();
	Reg[index] = tmp >> 16; // 先写高16位
	Reg[index + 1] = tmp;   // 再写低16位
	rt_exit_critical();
}

// 从起始索引为index的modbus寄存器中读取short类型的数
uint16_t GetShortFromReg(uint8_t index)
{
	uint16_t tmp;
	
	rt_enter_critical();
	tmp = Reg[index];
	rt_exit_critical();
	
	return tmp;
}

// 从起始索引为index的modbus寄存器中读取int类型的数（要读两个寄存器）
uint32_t GetIntFromReg(uint8_t index)
{
	uint32_t tmp = 0;
	
	rt_enter_critical();
	tmp |= Reg[index];     // 先读高16位
	tmp <<= 16;
	tmp |= Reg[index + 1]; // 再读低16位
	rt_exit_critical();
	
	return tmp;
}

// 从起始索引为index的modbus寄存器中读取float类型的数（要读两个寄存器）
float GetFloatFromReg(uint8_t index)
{
	float dst;
	uint32_t tmp = 0;
	
	rt_enter_critical();
	tmp |= Reg[index];     // 先读高16位
	tmp <<= 16;
	tmp |= Reg[index + 1]; // 再读低16位
	rt_exit_critical();
	
	dst = *((float*)&tmp); // 将int转化为float
	
	return dst;
}

// 根据modbus协议进行读写线圈和寄存器的操作
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
	
	switch(src[1]) // 解析modbus操作码
	{
		case 0x01:   // 读线圈，注意只能连续读 16 个 bit ！！！
			start_addr |= src[2];
			start_addr <<= 8;
			start_addr |= src[3]; // 要读的线圈的起始地址（即从线圈的第几位开始读）
			read_len |= src[4];
			read_len <<= 8;
			read_len |= src[5];   // 要读的长度（即读多少个bit）
		
			// 进入临界区之后在进行读操作
			rt_enter_critical();
			mask = (0x01 << (start_addr + read_len)) - 1;
			read_coil = (coil & mask) >> start_addr;
			rt_exit_critical();
		
			// 生成应答字符串
			ack_str[0] = 0x01; ack_str[1] = 0x01; ack_str[2] = read_len % 8? read_len/8 + 1: read_len/8;
			ack_str[3] = (uint8_t)read_coil; 
			if(ack_str[2] >= 2)
			{
				ack_str[4] = read_coil >> 8;
				cnt = 5;
			}
			else
				cnt = 4;
			
			// 生成CRC校验
			CRC_code = CheckCRC(ack_str, cnt);
			ack_str[cnt] = CRC_code >> 8; ack_str[cnt + 1] = CRC_code & 0x00FF;
			
			// 生成完整的应答后返回给上位机
			rt_device_write(handle, 0, ack_str, cnt + 2);
			break;
		
		case 0x03:   // 读保持寄存器
			start_addr |= src[2];
			start_addr <<= 8;
			start_addr |= src[3]; // 要读的寄存器的起始索引
			read_len |= src[4];
			read_len <<= 8;
			read_len |= src[5];   // 要读的寄存器个数
		
			// 生成应答帧的帧头
			ack_str[0] = 0x01; ack_str[1] = 0x03; ack_str[2] = 2 * read_len; cnt = 3;
		
			// 根据要读的寄存器数量将寄存器数值写入应答
			rt_enter_critical();
			for(i = 0; i < read_len; i++)
			{
				// 先写高8位
				ack_str[cnt] = Reg[start_addr + i] >> 8;
				
				// 再写低8位
				ack_str[cnt + 1] = Reg[start_addr + i] & 0x00FF;
				cnt += 2;
			}
			rt_exit_critical();
			
			// 生成CRC校验
			CRC_code = CheckCRC(ack_str, cnt);
			ack_str[cnt] = CRC_code >> 8; ack_str[cnt + 1] = CRC_code & 0x00FF;
			
			// 生成完整的应答后返回给上位机
			rt_device_write(handle, 0, ack_str, cnt + 2);
			break;
		
		case 0x05: 	 // 写单个线圈
			start_addr |= src[2];
			start_addr <<= 8;
			start_addr |= src[3]; // 获取要写的起始线圈地址
		
			rt_enter_critical();
			if(src[4] == 0xFF) // 将该bit置 1
			{
				coil |= (0x01 << start_addr);
			}
			else 							 // 将该bit置 0
			{
				coil &= ~(1 << start_addr);
			}
			rt_exit_critical();
			
			// 由于modbus协议中应答和命令相同，故直接返回命令就好
			rt_device_write(handle, 0, src, len);
			break;
		
		case 0x06:   // 写单个寄存器
			start_addr |= src[2];
			start_addr <<= 8;
			start_addr |= src[3]; 	// 获取要写的寄存器的起始索引
		
			mask = 0;
			mask |= src[4];
			mask <<= 8;
			mask |= src[5];       	// 获取要写入的数值
			
			rt_enter_critical();
			Reg[start_addr] = mask; // 写入寄存器
			rt_exit_critical();
			
			rt_device_write(handle, 0, src, len);
			break;
		
		case 0x0F:   // 写多个线圈
			start_addr |= src[2];
			start_addr <<= 8;
			start_addr |= src[3]; // 获取要写的起始线圈地址
			read_len |= src[4];
			read_len <<= 8;
			read_len |= src[5];   // 获取要写的bit个数
		
			// 生成掩码
			mask = ~((0x01 << (start_addr + read_len)) - 1);
			mask |= (0x01 << start_addr) - 1;
			
			mask2 = 0;
			if(src[6] >= 2) // 如果要写的线圈bit数超过1个字节
			{
				mask2 |= src[8];
				mask2 <<= 8;
			}
			mask2 |= src[7];
			mask2 <<= start_addr;
			
			// 写线圈
			rt_enter_critical();
			coil &= mask;   // 将addr到addr+len的线圈置0   
			coil |= mask2;  // 将addr到addr+len的线圈置为想要设置的数
			rt_exit_critical();
			
			// 生成应答
			ack_str[0] = 0x01; ack_str[1] = 0x0F; ack_str[2] = src[2]; ack_str[3] = src[3]; ack_str[4] = src[4]; ack_str[5] = src[5];
			CRC_code = CheckCRC(src, 6);
			ack_str[6] = CRC_code >> 8; ack_str[7] = CRC_code & 0x00FF;
			
			// 返回应答
			rt_device_write(handle, 0, ack_str, 8);
			break;
		
		case 0x10:   // 写多个寄存器
			start_addr |= src[2];
			start_addr <<= 8;
			start_addr |= src[3]; // 获取要写的寄存器的起始索引
			read_len |= src[4];
			read_len <<= 8;
			read_len |= src[5]; 	// 获取要写的寄存器个数
		
			// 根据要写的寄存器个数去写寄存器
			rt_enter_critical();
			for(i = 0; i < read_len; i++)
			{
				// 先写高8位
				Reg[start_addr + i] = 0;
				Reg[start_addr + i] |= src[7 + 2*i];
				Reg[start_addr + i] <<= 8;
				
				// 再写低8位
				Reg[start_addr + i] |= src[8 + 2*i];
			}
			rt_exit_critical();
			
			// 生成应答
			ack_str[0] = 0x01; ack_str[1] = 0x10; ack_str[2] = src[2]; ack_str[3] = src[3]; ack_str[4] = src[4]; ack_str[5] = src[5];
			CRC_code = CheckCRC(src, 6);
			ack_str[6] = CRC_code >> 8; ack_str[7] = CRC_code & 0x00FF;
			
			// 返回应答
			rt_device_write(handle, 0, ack_str, 8);
			break;
		
		default:
			break;
	}
}




// 将寄存器与线圈值解析成相应的命令
void PostProcess(uint8_t* src, uint8_t len)
{
	 uint16_t start_addr = 0;
	 rt_uint32_t recved_event;
	
	switch(src[1])  // 解析modbus操作码
	{
		case 0x05:  // 写线圈的单个bit
			start_addr |= src[2];
			start_addr <<= 8;
			start_addr |= src[3]; // 获取起始地址
		
			switch(start_addr) // 解析起始地址
			{
				case OPS_POWER_ACTION: // 开关机指令
					if(src[4] == 0xFF) // 开机
					{
						rt_event_send(Schedule, POWER_ON);
						rt_kprintf("RTU Set POWER_ON event. \n");
					}
					else               // 关机
					{
						// 应该要等开机进度到 100 的时候才可以关机
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
					
					
					
					
				case OPS_MOTOR_ENABLE: // 电机重新使能指令
					if(src[4] == 0xFF)
					{
						rt_event_send(Schedule, RESET_MOTOR);
						rt_kprintf("RTU Set RESET_MOTOR event. \n");
					}
					break;
					
					
					
					
				case OPS_PRESENT_POS: 	// 设定当前位置指令
					rt_kprintf("Send CALI_POS Event. \n");
					rt_event_send(Schedule, CALI_POS); // 设置 CALI_POS 事件，表明需要重新标定当前位置
					break;
				
				
				
				
				case OPS_MANUALLY_VALVE_UP: // 手动调节阀芯位置上移
					if(src[4] == 0xFF) // 若处于置位状态
					{
						rt_kprintf("Send MAN_MOVE_UP Event. \n");
						rt_event_send(Schedule, MAN_MOVE_UP);
					}
					else               // 若处于复位状态
					{
						rt_kprintf("Send MAN_MOVE_STOP Event. \n");
						rt_event_send(Schedule, MAN_MOVE_STOP);
					}
					break;
				
					
					
					
				case OPS_MANUALLY_VALVE_DOWN: // 手动调节阀芯位置下移
					if(src[4] == 0xFF) // 若处于置位状态
					{
						rt_kprintf("Send MAN_MOVE_DOWN Event. \n");
						rt_event_send(Schedule, MAN_MOVE_DOWN);
					}
					else               // 若处于复位状态
					{
						rt_kprintf("Send MAN_MOVE_STOP Event. \n");
						rt_event_send(Schedule, MAN_MOVE_STOP);
					}
					break;
				
					
					
					
				case OPS_OUTLET_PRESS_MIN: // 设定压力传感器下限指令
					rt_kprintf("RTU Set SET_PRESS_SENSOR_MIN event. \n");
					rt_event_send(Schedule, SET_PRESS_SENSOR_MIN);
					break;
				
				
				
				
				case OPS_OUTLET_PRESS_MAX: // 设定压力传感器上限指令
					rt_kprintf("RTU Set SET_PRESS_SENSOR_MAX event. \n");
					rt_event_send(Schedule, SET_PRESS_SENSOR_MAX);
					break;
				
				
				
				
				case OPS_OUTLET_FLUX_MIN: // 设定出口流量下限
					rt_kprintf("RTU Set SET_FLUX_SENSOR_MIN event. \n");
					rt_event_send(Schedule, SET_FLUX_SENSOR_MIN);
					break;
				
				
				
				
				case OPS_OUTLET_FLUX_MAX: // 设定出口流量上限
					rt_kprintf("RTU Set SET_FLUX_SENSOR_MAX event. \n");
					rt_event_send(Schedule, SET_FLUX_SENSOR_MAX);
					break;
				
				
				
				
				case OPS_TRG_OUTLET_PRESS: // 设定目标出口压力指令
					rt_kprintf("RTU Set SET_PRESS_OUT event. \n");
					rt_event_send(Schedule, SET_PRESS_OUT); // 设置 SET_PRESS_OUT 事件，表明开始调节目标出口压力
					break;
				
				
				
				
				case OPS_TRG_OUTLET_FLUX:  // 设定目标出口流量指令
					rt_kprintf("RTU Set SET_FLUX event. \n");
					rt_event_send(Schedule, SET_FLUX);     // 设置 SET_FLUX 事件，表明开始调节目标出口流量
					break;
				
				
				
				
				case OPS_OUTLET_PRESS_PID: // 设定出口压力调节PID指令
					rt_kprintf("RTU Set SET_PRESS_PID event. \n");
					rt_event_send(Schedule, SET_PRESS_PID);
					break;
				
				
				
				
				case OPS_OUTLET_FLUX_PID:  // 设定出口流量调节PID指令
					rt_kprintf("RTU Set SET_FLUX_PID event. \n");
					rt_event_send(Schedule, SET_FLUX_PID);
					break;
				
				
				
				
				case OPS_QUIT_PID:	// 退出PID调节模式指令
					rt_kprintf("RTU Set QUIT_PID event. \n");
					rt_event_send(Schedule, QUIT_PID);    // 设置 QUIT_PID 事件，表明退出PID控制模式
					break;
				
				
				
				
				case OPS_PRESENT_OUTLET_PRESS: // 设定当前出口压力指令
					rt_kprintf("RTU Set CALI_PRESENT_OUT_PRESS event. \n");
					rt_event_send(Schedule, CALI_PRESENT_OUT_PRESS);
					break;
				
				
				
				
				case OPS_PRESENT_OUTLET_FLUX:  // 设定当前出口流量指令
					rt_kprintf("RTU Set CALI_PRESENT_FLUX event. \n");
					rt_event_send(Schedule, CALI_PRESENT_FLUX);
					break;
					
				
				
				
				case OPS_PRESENT_INLET_PRESS:  // 设定当前入口压力指令
					rt_kprintf("RTU Set CALI_PRESENT_IN_PRESS event. \n");
					rt_event_send(Schedule, CALI_PRESENT_IN_PRESS);
					break;
				
				
				
				
				case OPS_MANUAL_SPEED:         // 设定电机手动调节时的速度
					rt_kprintf("RTU Set CHANGE_MAN_MOVE_SPEED event. \n");
					rt_event_send(Schedule, CHANGE_MAN_MOVE_SPEED);
					break;
				
					
					
					
				case OPS_FEED_SPEED:           // 设定电机PID模式下运行时的速度
					rt_kprintf("RTU Set CHANGE_FEED_MODE_SPEED event. \n");
					rt_event_send(Schedule, CHANGE_FEED_MODE_SPEED);
					break;
				
					
					
					
				case OPS_CURRENT:              // 设定电机运行电流
					rt_kprintf("RTU Set CHANGE_CURRENT event. \n");
					rt_event_send(Schedule, CHANGE_CURRENT);
					break;
				
					
					
				case OPS_MANUAL_ACC:           // 设定电机手动调节时的加速度
					rt_kprintf("RTU Set CHANGE_MAN_ACC event. \n");
					rt_event_send(Schedule, CHANGE_MAN_ACC);
					break;
				
					
					
				case OPS_FEED_ACC:              // 设定电机PID模式下运行时的加速度
					rt_kprintf("RTU Set CHANGE_FEED_ACC event. \n");
					rt_event_send(Schedule, CHANGE_FEED_ACC);
					break;
				
					
					
					
				case OPS_SET_PRESS_THRESHOLD:				// 设定压力控制阈值
					rt_kprintf("RTU Set CHANGE_PRESS_THRESHOLD event. \n");
					rt_event_send(Schedule, CHANGE_PRESS_THRESHOLD);
					break;
				
				
				
				
				case OPS_SET_FLUX_THRESHOLD:        // 设定流量控制阈值	
					rt_kprintf("RTU Set CHANGE_FLUX_THRESHOLD event. \n");
					rt_event_send(Schedule, CHANGE_FLUX_THRESHOLD);
					break;
				
				
				
				case OPS_TRG_POS:               // 运行到特定阀芯位置
					rt_kprintf("RTU Set MOVE_TO_TRG_POS event. \n");
					rt_event_send(Schedule, MOVE_TO_TRG_POS);
					break;
				
					
					
				case OPS_TEST:									// 电机往返运动精度测试
					if(src[4] == 0xFF)
					{
						rt_event_send(Schedule2, START_TEST);
					}
					else
					{
						rt_event_recv(Schedule2, START_TEST, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, 0, &recved_event);
					}
					break;
				
				
				case OPS_SHUT_DOWN:							// 电机急停
					rt_kprintf("RTU Set SHUT_DOWN_EVENT event. \n");
					rt_event_send(Schedule, SHUT_DOWN_EVENT);
					break;
					
				
				case OPS_MOTOR_DEBUG_MODE:			// 电机透传模式，直接给电机发送字符串指令
					if(src[4] == 0xFF)
					{
						rt_kprintf("Start Motor Debug Mode. \n");
						// 首先关闭线程运作
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


// 标定ADC读数
/*
参数说明：
	now_enc_val_addr： 			读数据线程读到的原始 ADC 编码器数值存放的寄存器地址
	userSet_phy_val_addr：	用户设置的当前 ADC 物理量所存放的寄存器地址
	phy_upperBound_addr：		当前 ADC 传感器的物理量上限寄存器地址
	phy_lowerBound_addr：   当前 ADC 传感器的物理量下限寄存器地址
	EncHigh：								上标定点处的 ADC 编码值
	PhyHigh：								上标定点处的 ADC 物理量值
	EncLow：								下标定点处的 ADC 编码值
	PhyLow：								下标定点处的 ADC 物理量值
*/
void CaliPresentADC(uint32_t now_enc_val_addr, uint32_t userSet_phy_val_addr, uint32_t phy_upperBound_addr, uint32_t phy_lowerBound_addr, uint32_t EncHigh_eeaddr, uint32_t PhyHigh_eeaddr, uint32_t EncLow_eeaddr, uint32_t PhyLow_eeaddr, uint32_t coeff_addr, float* EncHigh, float* PhyHigh, float* EncLow, float* PhyLow, float* convert_coeff)
{
	int ee_ret;
	
	float raw_enc = GetFloatFromReg(now_enc_val_addr);						// 当前ADC编码值读数 
	float cali_phy = GetFloatFromReg(userSet_phy_val_addr);				// 用户输入的当前要标定的ADC物理量的值
	
	float min = GetFloatFromReg(phy_lowerBound_addr);				// ADC传感器下限
	float max = GetFloatFromReg(phy_upperBound_addr);				// ADC传感器上限
	float range = max - min;																	// 传感器的物理量量程
		
	if((cali_phy - min) > SPLITE_POINT*range)	// 超过设定的范围，那么就说明标定的是上标定点
	{			
		rt_kprintf("Now Calibrating ADC Seensor Upper Point.  \n");
		
		// 将当前的 ADC 编码值 存入 上标定点的编码值
		*EncHigh = raw_enc;
		ee_ret = WriteFloatToEEPROME(EncHigh_eeaddr, raw_enc);
		if(ee_ret)
			rt_kprintf("Write Current ADC Enc Val(int cast): %d To EEPROME Successfully!! \n", (int)raw_enc);
		else
			rt_kprintf("Write Current ADC Enc Val To EEPROME Fail!! \n");
			
		// 将当前的 ADC 物理量值 存入 上标定点的物理量值
		*PhyHigh = cali_phy;
		ee_ret = WriteFloatToEEPROME(PhyHigh_eeaddr, cali_phy);
		if(ee_ret)
			rt_kprintf("Write Current ADC Phy Val(int cast): %d To EEPROME Successfully!! \n", (int)cali_phy);
		else
			rt_kprintf("Write Current ADC Phy Val To EEPROME Fail!! \n");
	}
	else																	// 没有超过设定的范围，标定的是下标定点
	{
		rt_kprintf("Now Calibrating ADC Seensor Lower Point.  \n");
		
		// 将当前的 ADC 编码值 存入 下标定点的编码值
		*EncLow = raw_enc;
		ee_ret = WriteFloatToEEPROME(EncLow_eeaddr, raw_enc);
		if(ee_ret)
			rt_kprintf("Write Current ADC Enc Val(int cast): %d To EEPROME Successfully!! \n", (int)raw_enc);
		else
			rt_kprintf("Write Current ADC Enc Val To EEPROME Fail!! \n");
			
		// 将当前的 ADC 物理量值 存入 下标定点的物理量值
		*PhyLow = cali_phy;
		ee_ret = WriteFloatToEEPROME(PhyLow_eeaddr, cali_phy);
		if(ee_ret)
			rt_kprintf("Write Current ADC Phy Val(int cast): %d To EEPROME Successfully!! \n", (int)cali_phy);
		else
			rt_kprintf("Write Current ADC Phy Val To EEPROME Fail!! \n");
	}
	
	// 更新转换系数
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
	
		ReadEncoderDataToMotor();                        								// 将编码器数值载入电机
			
		ret_val = GetFloatFromMotor("JS", &tmp);																	// 从电机读取 手动调节阀芯位置时的速度
		if(ret_val)
			WriteFloatToReg(MANUAL_SPEED, tmp);
		else
			rt_kprintf("Read manual speed fail !! \n");
				
		ret_val = GetFloatFromMotor("VE", &tmp);																	// 从电机读取 电机PID运行模式下的速度
		if(ret_val)
			WriteFloatToReg(PID_SPEED, tmp);
		else
			rt_kprintf("Read PID mode speed fail !! \n");
				
		ret_val = GetFloatFromMotor("CC", &tmp);																	// 从电机读取 电机运行电流
		if(ret_val)
			WriteFloatToReg(CURRENT, tmp);
		else
			rt_kprintf("Read current fail !! \n");
				
		ret_val = GetFloatFromMotor("JA", &tmp);																	// 从电机读取 手动调节阀芯位置时的加速度
		if(ret_val)
			WriteFloatToReg(MANUAL_ACC, tmp);
		else
			rt_kprintf("Read manual acceleration fail !! \n");
				
		ret_val = GetFloatFromMotor("AC", &tmp);																	// 从电机读取 电机PID运行模式下的加速度
		if(ret_val)
			WriteFloatToReg(PID_ACC, tmp);
		else
			rt_kprintf("Read PID mode acceleration fail !! \n");
				
		ReadFloatFromEEPROME(OUTLET_PRESS_MIN_ADDR, &tmp); 								// 读取出口压力下限
		last_press_min = tmp;
		WriteFloatToReg(DEF_OUTLET_PRESS_MIN, tmp);
		//printf("Read outlet pressure min: %f \n", tmp);
		ReadFloatFromEEPROME(OUTLET_PRESS_MAX_ADDR, &tmp); 								// 读取出口压力上限
		last_press_max = tmp;
		WriteFloatToReg(DEF_OUTLET_PRESS_MAX, tmp);
		//printf("Read outlet pressure max: %f \n", tmp);
		ReadFloatFromEEPROME(OUTLET_FLUX_MIN_ADDR, &tmp);  								// 读取出口流量下限
		last_flux_min = tmp;
		WriteFloatToReg(DEF_OUTLET_FLUX_MIN, tmp);
		//printf("Read outlet flux min: %f \n", tmp);
		ReadFloatFromEEPROME(OUTLET_FLUX_MAX_ADDR, &tmp);  								// 读取出口流量上限
		last_flux_max = tmp;
		WriteFloatToReg(DEF_OUTLET_FLUX_MAX, tmp);
		//printf("Read outlet flux max: %f \n", tmp);
		ReadFloatFromEEPROME(OUTPRESS_P, &tmp);														// 读取出口压力调节PID的P
		WriteFloatToReg(OUTLET_PRESS_P, tmp);
		//printf("Read outlet pressure PID's P: %f \n", tmp);
		ReadFloatFromEEPROME(OUTPRESS_I, &tmp);														// 读取出口压力调节PID的I
		WriteFloatToReg(OUTLET_PRESS_I, tmp);
		//printf("Read outlet pressure PID's I: %f \n", tmp);
		ReadFloatFromEEPROME(OUTPRESS_D, &tmp);														// 读取出口压力调节PID的D
		WriteFloatToReg(OUTLET_PRESS_D, tmp);
		//printf("Read outlet pressure PID's D: %f \n", tmp);
		ReadFloatFromEEPROME(OUTFLUX_P, &tmp);														// 读取出口流量调节PID的P
		WriteFloatToReg(OUTLET_FLUX_P, tmp);
		//printf("Read outlet flux PID's P: %f \n", tmp);
		ReadFloatFromEEPROME(OUTFLUX_I, &tmp);														// 读取出口流量调节PID的I
		WriteFloatToReg(OUTLET_FLUX_I, tmp);
		//printf("Read outlet flux PID's I: %f \n", tmp);
		ReadFloatFromEEPROME(OUTFLUX_D, &tmp);														// 读取出口流量调节PID的D
		WriteFloatToReg(OUTLET_FLUX_D, tmp);
		//printf("Read outlet flux PID's D: %f \n", tmp);
		ReadFloatFromEEPROME(EE_PRESS_CTR_THRESHOLD, &tmp);								// 读取出口压力控制阈值
		WriteFloatToReg(OUT_PRESS_CTR_THRESHOLD, tmp);
		press_threshold = tmp;
		//printf("Read outlet pressure control threshold: %f \n", press_threshold);
		ReadFloatFromEEPROME(EE_FLUX_CTR_THRESHOLD, &tmp);								// 读取出口流量控制阈值
		WriteFloatToReg(OUT_FLUX_CTR_THRESHOLD, tmp);
		flux_threshold = tmp;
		//printf("Read outlet flux control threshold: %f \n", flux_threshold);
		
		ReadFloatFromEEPROME(ENC_TO_POS, &coeff_env_to_valve); 						// 读取 从编码器值转换到阀芯位置的转换系数
		rt_kprintf("Read coeff_env_to_valve(int cast): %d \n", (int)coeff_env_to_valve);
		ReadFloatFromEEPROME(ADC_TO_PRESS_OUTLET, &adc_to_press_outlet);  // 读取 从ADC的读数转换到出口压力的转换系数
		rt_kprintf("Read adc_to_press_outlet(int cast): %d \n", (int)adc_to_press_outlet);
		ReadFloatFromEEPROME(ADC_TO_FLUX_OUTLET, &adc_to_flux_outlet);    // 读取 从ADC的读数转换到出口流量的转换系数
		rt_kprintf("Read adc_to_flux_outlet(int cast): %d \n", (int)adc_to_flux_outlet);
		ReadFloatFromEEPROME(ADC_TO_PRESS_INLET, &adc_to_press_inlet);    // 读取 从ADC的读数转换到入口压力的转换系数
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
	
	// 将 int 型数值转化成字符串形式
	sprintf(enc_val_str, "%d", enc_val);
	
	// 将 FL 命令发给电机
	ret_val = ToMotorNoReturnWithParameter("FL", enc_val_str);
	if(ret_val == 1)
		rt_kprintf("Send FL %s Successfully !! \n", enc_val_str);
	else if(ret_val == 2)
		rt_kprintf("Send FL %s, but Motor Is Busy, This shouldn't happend because we have MOV_STA event, Weired!! \n", enc_val_str);
	else
		rt_kprintf("Send FL command Fail!! \n");
}



// 给定阀芯位置，然后通过绝对位置值进给
void FeedPosValve(float pos)
{
	char feed_val_str[32];
	int ret_val;
	
	// 根据目标位置来获取目标位置的编码值
	rt_kprintf("The Trg Pos(int cast): %d \n", (int)pos);
	float enc_f = (pos * coeff_env_to_valve);
	
	// 注意由于 IE 和 FP 的单位不同，所以这里还需要做一个转化（将 IE 的值转为 FP 的值）
	int32_t feed_val = (int32_t)(enc_f * IP_TO_EP);
	
	// 将数值转化为字符串形式
	sprintf(feed_val_str, "%d", feed_val);
	
	// 先发送一个SKD指令，从而保证电机会立即执行下面的 FP 指令
	ret_val = ToMotorNoReturnNoParameter("SKD");
	if(ret_val)
		rt_kprintf("Send SKD Successfully !! \n");
	else
		rt_kprintf("Send SKD Fail !! \n");
	
	// 发送 FP 命令，让电机运动到特定位置
	ret_val = ToMotorNoReturnWithParameter("FP", feed_val_str);
	if(ret_val)
		rt_kprintf("Send FP %s Successfully !! \n", feed_val_str);
	else
		rt_kprintf("Send FP command Fail !! \n");
}



// 标定传感器上下限
int CaliSensor(uint32_t bound_reg_addr, uint32_t bound_ee_addr, uint32_t convert_coeff_ee_addr, float* old_lower_bound, float* old_upper_bound, float* convert_coeff, int type)
{
	float err;
	float* old_val;
	int ee_ret;
	int ret_val;
	
	if(type)	// 标定的是上限
	{
		old_val = old_upper_bound;
		rt_kprintf("Calibrating Sensor Upper Bound. \n");
	}
	else			// 标定的是下限
	{
		old_val = old_lower_bound;
		rt_kprintf("Calibrating Sensor Lower Bound. \n");
	}
	
	// 先将设定值和旧的值进行比较，如果二者差别很小的话，很可能是用户多点了几下确定，那么就不重复设定了
	// 相当于提供了一个 short circuit 机制
//	err = *old_val - GetFloatFromReg(bound_reg_addr);
//	err = err < 0? -err : err;
//	if(err < 0.01) // 小于0.01的话就认为设定的值和上一次记录的值一样
//	{
//		// 恢复寄存器的值
//		rt_kprintf("New Value is too close to the old value. Abandon it!!  \n");
//		WriteFloatToReg(bound_reg_addr, *old_val);
//		ret_val = 0;
//	}
//	else
	{
		// 更新 old_value 为最新的值
		*old_val = GetFloatFromReg(bound_reg_addr);
					
		// 将 传感器上下限对应的 寄存器的最新数值存入EEPROME
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
		
		//由于量程改了，所以更新 上下标定点 从而避免错误的标定点对 ADC 读数的影响
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
		
		// 三个转换系数一起更新
		adc_to_press_inlet = (press_inlet_enc_high - press_inlet_enc_low) / (press_inlet_phy_high - press_inlet_phy_low);
		adc_to_press_outlet = (press_outlet_enc_high - press_outlet_enc_low) / (press_outlet_phy_high - press_outlet_phy_low);
		adc_to_flux_outlet = (flux_enc_high - flux_enc_low) / (flux_phy_high - flux_phy_low);
		WriteFloatToEEPROME(ADC_TO_PRESS_INLET, adc_to_press_inlet);
		WriteFloatToEEPROME(ADC_TO_PRESS_OUTLET, adc_to_press_outlet);
		WriteFloatToEEPROME(ADC_TO_FLUX_OUTLET, adc_to_flux_outlet);
		rt_kprintf("adc_to_press_inlet: %d \n", (int)adc_to_press_inlet);
		rt_kprintf("adc_to_press_outlet: %d \n", (int)adc_to_press_outlet);
		rt_kprintf("adc_to_flux_outlet: %d \n", (int)adc_to_flux_outlet);
		
//		// 更新转换系数
//		*convert_coeff = (float)(RANGE_HIGH - RANGE_LOW) / (*old_upper_bound - *old_lower_bound);

//		// 将更新后的 转换系数 数值保存到EEPROME
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
	
	// 将寄存器中的数值给全局变量
	*threshold = GetFloatFromReg(reg_addr);
			
	// 将数值存入EEPROME，方便下次读出
	ee_ret = WriteFloatToEEPROME(ee_addr, *threshold);
	if(ee_ret)
		rt_kprintf("Save threshold Successfully !! \n");
	else
		rt_kprintf("Save threshold Fail !! \n");
}

