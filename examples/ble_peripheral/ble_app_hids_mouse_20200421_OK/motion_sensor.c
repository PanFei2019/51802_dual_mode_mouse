#include <stdint.h>
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_drv_spi.h"
#include "nrf_log.h"
#include "configuration.h"

#define Paw_3205_REG_PRODUCT_ID1                0x00            /**< Product ID register */  
#define Paw_3205_REG_PRODUCT_ID2                0x01 
#define Paw_3065_REG_VALUE_PRODUCT_ID1         	0x31
#define Paw_3205_REG_VALUE_PRODUCT_ID1         	0x30
#define Paw_3205_REG_VALUE_PRODUCT_ID2_MASK		  0xF0
#define Paw_3205_REG_VALUE_PRODUCT_ID2		      0x50

enum
{
	PAW_3205_CPI_600=0x00,
	PAW_3205_CPI_800=0x01,
	PAW_3205_CPI_1000=0x02,
	PAW_3205_CPI_1200=0x03,
	PAW_3205_CPI_1600=0x04
};

enum
{
	ChipSet_3205=0x01,
	ChipSet_3065=0x02,
	ChipSet_3065_XY = 0x03,
	ChipSet_3212=0x04,
	ChipSet_KA8 =0x05
};

enum
{
	BT_mode    = 0x00,
	RF_2_point_4_mode = 0x01
};

//extern uint8_t       m_rx_buf[5];    /**< RX buffer. */
extern const nrf_drv_spi_t spi;
//extern nrf_drv_spi_config_t spi_config;
extern void spi_event_handler(nrf_drv_spi_evt_t const * p_event);

extern uint8_t work_mode;
extern bool spi_xfer_done;
extern bool m_spi_is_enable;
extern bool USING_HW_SPI;
bool deviceFound = 0;
uint8_t ChipVersion = 0;
uint8_t ChipSet = 0;
uint8_t setCPIvalue;

#if 1
void CustomSensorReSynchronousSPI(void)
{
	//uint16_t i =0;
	nrf_gpio_cfg_output(SPI_SCK_PIN);
	nrf_gpio_pin_write(SPI_SCK_PIN, 1);
	nrf_delay_us(100);  
	//nrf_delay_ms(20);  
	nrf_gpio_pin_write(SPI_SCK_PIN, 0);
	nrf_delay_us(100); 
	//nrf_delay_ms(30);  
	nrf_gpio_pin_write(SPI_SCK_PIN, 1);
	nrf_delay_ms(1);
	nrf_delay_us(300); 
	//nrf_delay_us(100);
}

void usrPaw3205ReSynchronousSPI(void)
{
	//uint16_t i =0;
	nrf_gpio_cfg_output(SPI_SCK_PIN);
	nrf_gpio_pin_write(SPI_SCK_PIN, 1);
	//nrf_delay_us(100);  
	nrf_delay_ms(20);  
	nrf_gpio_pin_write(SPI_SCK_PIN, 0);
	//nrf_delay_us(100); 
	nrf_delay_ms(20);  
	nrf_gpio_pin_write(SPI_SCK_PIN, 1);
	nrf_delay_us(3000); 
}
#endif



#if 0
void writeReg(uint8_t regAddress, int8_t val)
{
		uint8_t       m_rx_buf[2];    /**< RX buffer. */
		int8_t buf[2];

		//'??Paw3205  SWKINT ???????
		if(regAddress==0x06)
		{
			//val|=0x40;  //Trigger only if motion when sleeping
			val &= ~0x40;  //Try Level triggered when motion is available
		}

		// Create spi command. Ensure write bit is set in the address byte
		buf[0] = regAddress | 0x80;
		buf[1] = val;
			
		spi_xfer_done = false;
		APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &buf[0], 2, m_rx_buf, 2));
		while(!spi_xfer_done);
}

///////////////////////////////////////////////////////////////////////////
//
int8_t readReg(uint8_t regAddress)
{
		uint8_t val;
		uint8_t txbuf[2];
		uint8_t rxbuf[2];
	
    txbuf[0]=regAddress;
    txbuf[1]=0xFF;
		
		spi_xfer_done = false;
		APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, txbuf, 2, rxbuf, 2));
		while(!spi_xfer_done);
    val=rxbuf[1];

    return val;
}
#else

void driver_sensor_two_spi_send_byte(uint8_t value)
{	
		uint8_t i;
    nrf_gpio_cfg_output(SPI_SCK_PIN);
		nrf_gpio_cfg_output(SPI_MISO_PIN);
		nrf_gpio_cfg_input(SPI_MOSI_PIN,NRF_GPIO_PIN_NOPULL);
		//nrf_gpio_cfg_output(SPI_MISO_PIN);
		//nrf_gpio_pin_write(SPI_MISO_PIN, 0);

    for(i=0; i<8; i++)
    {
				nrf_gpio_pin_write(SPI_SCK_PIN, 0);
				//nrf_delay_us(1);
        if(value & 0x80)
						nrf_gpio_pin_write(SPI_MISO_PIN, 1);
        else
						nrf_gpio_pin_write(SPI_MISO_PIN, 0);
        value <<= 1;
				//nrf_delay_us(1);
				nrf_gpio_pin_write(SPI_SCK_PIN, 1);
				//nrf_delay_us(1);
    }
		nrf_gpio_pin_write(SPI_MISO_PIN, 0);
		//nrf_delay_us(5);
		//nrf_gpio_pin_write(SPI_SCK_PIN, 0);

	#if 0
	//release SDIO BUS
	for(i=0;i<2;i++)
	{
		nrf_delay_us(1);
	}
	#endif
}


/*! \fn UINT8 driver_sensor_two_spi_read_byte(void)
    \brief Read a byte from paw3204.

    \param void
    \return data read
*/
int8_t driver_sensor_two_spi_read_byte(void)
{
    int8_t i, ret;

    ret = 0;
		nrf_gpio_cfg_output(SPI_SCK_PIN);
		nrf_gpio_cfg_input(SPI_MOSI_PIN,NRF_GPIO_PIN_NOPULL);
		nrf_gpio_cfg_input(SPI_MISO_PIN,NRF_GPIO_PIN_NOPULL);

    for(i=0; i<8; i++)
    {
				nrf_gpio_pin_write(SPI_SCK_PIN, 0);
				//nrf_delay_us(1);
				nrf_gpio_pin_write(SPI_SCK_PIN, 1);
				//nrf_delay_us(1);
        ret <<= 1;
        if(nrf_gpio_pin_read(SPI_MISO_PIN))
            ret |= 0x01;
				//nrf_delay_us(1);
    }
		//nrf_gpio_pin_write(SPI_SCK_PIN, 0);
    return ret;
}

void writeReg(uint8_t regAddress , uint8_t value)
{
		uint8_t m_rx_buf[2];    /**< RX buffer. */
		int8_t buf[2];
		
		if(regAddress==0x06)
		{
			value &= ~0x40;  //Try Level triggered when motion is available
		}

		// Create spi command. Ensure write bit is set in the address byte
		buf[0] = regAddress | 0x80;
		buf[1] = value;
	
		if(USING_HW_SPI)
		{
			#if 1	
			APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, buf, 2, m_rx_buf, 2));
			#else
			APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &buf[0], 1, m_rx_buf, 1));
			APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &buf[1], 1, &m_rx_buf[1], 1));
			#endif
		}
		else
		{
			driver_sensor_two_spi_send_byte(buf[0]);
			driver_sensor_two_spi_send_byte(buf[1]);
			//nrf_gpio_cfg_default(SPI_SCK_PIN);
			//nrf_gpio_cfg_default(SPI_MOSI_PIN);
			//nrf_gpio_cfg_default(SPI_MISO_PIN);
		}
}

/*! \fn void driver_sensor_two_spi_read_data(UINT8 addr, UINT8 *value, UINT8 len)
    \brief Read \a len bytes data to the pointer \a value from the address \addr in paw3204.

    \param addr - the address of written
    \param value - the pointer stored the read data
    \param len - the length of data read
    \return void
*/
int8_t readReg(uint8_t regAddress)
{
		uint8_t val;
		uint8_t txbuf[2];
		uint8_t rxbuf[2];
	
    txbuf[0]=regAddress;
    txbuf[1]=0xFF;
		if(USING_HW_SPI)
		{
			#if 0	
			APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, txbuf, 2, rxbuf, 2));
			#else
			APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, txbuf, 1, rxbuf, 1));
			//nrf_delay_us(50); 
			APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &txbuf[1], 1, &rxbuf[1], 1));
			#endif
			//nrf_delay_ms(2); 
			val=rxbuf[1];
		}
		else
		{
			driver_sensor_two_spi_send_byte(regAddress);
			val = driver_sensor_two_spi_read_byte();
			//nrf_gpio_cfg_default(SPI_SCK_PIN);
			//nrf_gpio_cfg_default(SPI_MOSI_PIN);
			//nrf_gpio_cfg_default(SPI_MISO_PIN);
		}
    return val;
}
#endif


///////////////////////////////////////////////////////////////////////////
/// 
bool verifyProductId(void)
{	
	int8_t value;
	value=readReg(Paw_3205_REG_PRODUCT_ID1);
	#if DEBUG_TRACE_ENABLE
	//NRF_LOG_INFO("P_ID1=%02x\r\n",value);
	#endif
	if(value==Paw_3205_REG_VALUE_PRODUCT_ID1||value==Paw_3065_REG_VALUE_PRODUCT_ID1)
	{
		ChipVersion = readReg(Paw_3205_REG_PRODUCT_ID2);
		//NRF_LOG_INFO("P_ID2=%02x\r\n",ChipVersion);
		return 1;
	}
	else 
	{
		#if 0//DEBUG_TRACE_ENABLE
		NRF_LOG_RAW_INFO("@");
		#endif
		return 0;
	}
}

bool ChenkSensorSync(void)
{	
	int8_t value;
	value=readReg(Paw_3205_REG_PRODUCT_ID1);
	if(value==Paw_3205_REG_VALUE_PRODUCT_ID1||value==Paw_3065_REG_VALUE_PRODUCT_ID1)
	{
		#if 0
		nrf_gpio_cfg_output(29);
		nrf_gpio_pin_write(29, 1);	
		#endif
		return 1;
	}
	else 
	{
		#if 0//DEBUG_TRACE_ENABLE
		NRF_LOG_RAW_INFO("@");
		#endif
		#if 0
		nrf_gpio_cfg_output(29);
		nrf_gpio_pin_write(29, 0);
		#endif
		return 0;
	}
}


void CheckSensor(void)
{
	int8_t i = 0;
	
	deviceFound = ChenkSensorSync();
	while((!deviceFound)&&(i++<3))
	{
		#if DEBUG_TRACE_ENABLE
		NRF_LOG_INFO("i=%d\r\n",i);
		#endif
	
		if(USING_HW_SPI)
		{
			nrf_drv_spi_uninit(&spi);
		}

		switch(ChipSet)
		{
			case 0:
			case ChipSet_3065:
			case ChipSet_3065_XY:
			case ChipSet_KA8:
				usrPaw3205ReSynchronousSPI();
				break;
			
			case ChipSet_3205:			
			case ChipSet_3212:
				CustomSensorReSynchronousSPI();
				break;

			default: break;
		}
		
		if(USING_HW_SPI)
		{
			nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
			spi_config.ss_pin   = SPI_SS_PIN;
			spi_config.miso_pin = SPI_MISO_PIN;
			spi_config.mosi_pin = SPI_MOSI_PIN;
			spi_config.sck_pin  = SPI_SCK_PIN;
			APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
		}
		deviceFound = ChenkSensorSync();	
	}
}


bool getMotion(int16_t *x, int16_t *y, int8_t maxReads)
{	
	uint8_t i = 0;
	uint8_t MotionE;
  //int8_t x;
	//int8_t y;
	*x=0;
	*y=0;

	CheckSensor();

	// If we have a sensor connected
	if(deviceFound)
	{
			#if 0
			while(readReg(0x02)&0x80)
			{
				*x +=readReg(0x03);
				*y +=readReg(0x04);
				//NRF_LOG_INFO("%02x, %02x\r\n",*x,*y);
				//if((*x>=maxReads)||(*y>=maxReads)) break;
				//return true;
			}
			if(*x|*y)
			{
				return true;
			}
			else
			{
				return false;
			}
			#else
			
			#if 1
			#if 0
			if(ChipSet==ChipSet_3205)
			{
				if(!nrf_gpio_pin_read(23))
				{
					for(i=0;i<4;i++)
					{
						readReg(0x02);						
						*x +=readReg(0x03);
						*y +=readReg(0x04);
						if((*x!=0)||(*y!=0)) break;
					}
				}
			}
			else
			#endif
			{
				if(!nrf_gpio_pin_read(23))
				{
					for(i=0;i<4;i++)
					{
						readReg(0x02);						
						*x +=readReg(0x03);
						*y +=readReg(0x04);
						if((*x!=0)||(*y!=0)) break;
					}
				}
			}
			#else
			readReg(0x02);						
			*x +=readReg(0x03);
			*y +=readReg(0x04);
			#endif
			
			if((*x!=0)||(*y!=0))
			{
				return true;
			}
			else
			{
				return false;
			}
			#endif		
	}
}




///////////////////////////////////////////////////////////////////////////
//turn on
void turnOn(void)
{
	//Full chip reset	
	writeReg(0x06 , 0x80);			//double call it ..................
	//nrf_delay_us(5000);

	do
	{
		//Full chip reset	
		//writeReg(0x06 , 0x80);			//double call it ..................
		deviceFound = verifyProductId();
		nrf_delay_us(500);
	}while(!deviceFound);
	
	#if DEBUG_TRACE_ENABLE
	NRF_LOG_INFO("motion deviceFound\r\n");
	NRF_LOG_INFO("PRODUCT_ID1= %02x\r\n",readReg(Paw_3205_REG_PRODUCT_ID1));
	NRF_LOG_INFO("ChipVersion= %02x\r\n",readReg(Paw_3205_REG_PRODUCT_ID2));
	//NRF_LOG_INFO("ChipVersion= %02x\r\n",ChipVersion);
	#endif
	
	#if 1
	writeReg(0x09,0x5A);
	if((ChipVersion&0xF0)==0xD0||ChipVersion==0||ChipVersion==0x70||ChipVersion==0x54)      //PAW3205 or PAW3065 or PAW3065-XY or KA8
	{	
		if((ChipVersion&0xF0)==0xD0)		
		{	
			#if DEBUG_TRACE_ENABLE
			NRF_LOG_INFO("ChipSet= PAW3205\r\n");
			#endif
			ChipSet = ChipSet_3205;
			writeReg(0x0d,0x0a);	 // 
			writeReg(0x1b,0x35);	 //
			writeReg(0x1d,0xdb);	 //
			writeReg(0x28,0xb4);
			writeReg(0x29,0x46);
			writeReg(0x2a,0x96);
			writeReg(0x2b,0x8c);
			writeReg(0x2c,0x6e);
			writeReg(0x2d,0x64);
			writeReg(0x38,0x5f);
			writeReg(0x39,0x0f);
			writeReg(0x3a,0x32);
			writeReg(0x3b,0x47);
			writeReg(0x42,0x10);
			writeReg(0x43,0x09);
			writeReg(0x54,0x2e);
			writeReg(0x55,0xf2);
			writeReg(0x61,0xf4);
			writeReg(0x63,0x70);
			writeReg(0x75,0x52);
			writeReg(0x76,0x41);
			writeReg(0x77,0xed);
			writeReg(0x78,0x23);
			writeReg(0x79,0x46);
			writeReg(0x7a,0xe5);
			writeReg(0x7c,0x48);
			writeReg(0x7d,0x80);
			writeReg(0x7e,0x77);

			writeReg(0x7F,0x01);
			writeReg(0x0B,0x00);
			writeReg(0x7F,0x00);
			
			#if NOT_USE_HW_SPI_IN_2_point_4_mode_PAW3205
			if(work_mode==RF_2_point_4_mode)
			{
				USING_HW_SPI = false;
			}
			#endif
			
			#if NOT_USE_HW_SPI_IN_BT_mode_PAW3205
			if(work_mode==BT_mode)
			{
				USING_HW_SPI = false;
			}
			#endif
			
			//USING_HW_SPI = false;
			
			if(!USING_HW_SPI)
			{
				if(m_spi_is_enable)
				{
					nrf_drv_spi_uninit(&spi);
					m_spi_is_enable = false;
					#if DEBUG_TRACE_ENABLE
					NRF_LOG_INFO("disable spi\r\n");
					#endif
				}
				
				do
				{
					deviceFound = verifyProductId();
					nrf_delay_us(500);
				}while(!deviceFound);
			}
			
		}
		
		setCPIvalue=PAW_3205_CPI_1200;	
		//set cpi
		if(ChipVersion==0x70)      //PAW3065-XY
		{
			#if DEBUG_TRACE_ENABLE
			NRF_LOG_INFO("ChipSet= PAW3065-XY\r\n");
			#endif
			ChipSet = ChipSet_3065_XY;
			
			#if DefaultHighDPI
			writeReg(0x06 , 0x03); //1600
			#else
			writeReg(0x06 , 0x02); //1200
			#endif
			
		}
		else
		{
			#if DefaultHighDPI
			writeReg(0x06 , 0x04);	//1600
			#else
			writeReg(0x06 , PAW_3205_CPI_1200);				
			#endif
		}
		
		if(ChipVersion==0x54)      //KA8
		{
			#if DEBUG_TRACE_ENABLE
			NRF_LOG_INFO("ChipSet= KA8\r\n");
			#endif
			ChipSet = ChipSet_KA8;
		}
		
		if(ChipVersion==0x00)      //PAW3065
		{
			#if DEBUG_TRACE_ENABLE
			NRF_LOG_INFO("ChipSet= PAW3065\r\n");
			#endif
			ChipSet = ChipSet_3065;
		}
	}
	else if(ChipVersion==0x02)
	{	
		#if DEBUG_TRACE_ENABLE
		NRF_LOG_INFO("ChipSet= PAW3212\r\n");
		#endif
		ChipSet = ChipSet_3212;
		//setCPIvalue = readReg(0x0D);
		//NRF_LOG_INFO("CPI_X:%02x\r\n",setCPIvalue);
		//PAW3212
		#if DefaultHighDPI
		writeReg(0x0D , 42);    //X 1600
		writeReg(0x0E , 42);    //Y 1600
		#else
		writeReg(0x0D , 32);    //X 1200
		writeReg(0x0E , 32);    //Y 1200
		#endif
		//setCPIvalue = readReg(0x0D);
		//NRF_LOG_INFO("CPI_X1:%02x\r\n",setCPIvalue);
	}
	else
	{
		//NRF_LOG_INFO("CIP:%02x\r\n",readReg(0x06));
		//while(1);
	}
	#endif
	writeReg(0x09,0x00);
}

void usrPaw3205setCPI(uint8_t CPIValue)
{
	uint8_t tmp;
	writeReg(0x09,0x5A);
	tmp = readReg(0x06);
	tmp &= ~0x07;
	#if 0
	writeReg(0x06 , tmp|CPIValue);
	//hiddcfa_eepromWrite(&CPIValue, 1, 1);
	#else
	switch(CPIValue)
	{
		case 1:
			#if DEBUG_TRACE_ENABLE
			NRF_LOG_RAW_INFO("1\r\n");
			#endif
			//printk("ChipVersion=%x\r\n",ChipVersion);
			if((ChipVersion&0xF0)==0xD0||ChipVersion==0||ChipVersion==0x54)        //PAW3205 or PAW3065
			{
				//NRF_LOG_RAW_INFO("1\r\n");				
				writeReg(0x06 , tmp|0x01); // 800
			}
			else if(ChipVersion==0x02)                        //PAW3212
			{
				writeReg(0x0D , 21);      //X 800
				writeReg(0x0E , 21);      //Y 800
			}
			else if(ChipVersion==0x70)                        //PAW3065-XY
			{
				writeReg(0x06 , tmp|0x00);// 800
			}
			break;

		case 2:
			#if DEBUG_TRACE_ENABLE
			NRF_LOG_RAW_INFO("2\r\n");	
			#endif
			//printk("ChipVersion=%x\r\n",ChipVersion);
			if((ChipVersion&0xF0)==0xD0||ChipVersion==0||ChipVersion==0x54)        //PAW3205 or PAW3065
			{
				//printk("2\r\n");				
				writeReg(0x06 , tmp|0x03); // 1300
			}
			else if(ChipVersion==0x02)                        //PAW3212
			{
				writeReg(0x0D , 32);       //X 1200
				writeReg(0x0E , 32);       //Y 1200
			}
			else if(ChipVersion==0x70)                        //PAW3065-XY
			{
				writeReg(0x06 , tmp|0x02); // 1300
			}
			break;

		case 3:
			#if DEBUG_TRACE_ENABLE
			NRF_LOG_RAW_INFO("3\r\n");
			#endif
			//printk("ChipVersion=%x\r\n",ChipVersion);
			if((ChipVersion&0xF0)==0xD0||ChipVersion==0||ChipVersion==0x54)        //PAW3205 or PAW3065
			{
				//printk("3\r\n");				
				writeReg(0x06 , tmp|0x04); // 1600
			}
			else if(ChipVersion==0x02)                        //PAW3212
			{
				writeReg(0x0D , 42);      //X 1600
				writeReg(0x0E , 42);      //Y 1600
			}
			else if(ChipVersion==0x70)
			{
				writeReg(0x06 , tmp|0x03); // 1600              //PAW3065-XY
			}
			break;

		case 4:
			#if DEBUG_TRACE_ENABLE
			NRF_LOG_RAW_INFO("4\r\n");
			#endif
			if(ChipVersion==0x02)                        //PAW3212
			{
				writeReg(0x0D , 63);      //X 2400
				writeReg(0x0E , 63);      //Y 2400
			}
			
			break;

		default:
			break;
	}
	#endif
	writeReg(0x09,0x00);
}

void enterOrExitSleepmode(uint8_t operation)
{
	uint8_t tmp;
	#if SensorJustSleep
	//printk("sensor sleep\r\n");
	writeReg(0x09,0x5A);	 //Disable write protect	
	writeReg(0x05,operation);	 //force enter sleep 2 mode...
	writeReg(0x09,0x00);	 //enable write protect
	#else
	//printk("sensor power down\r\n");
	//printk("operation=%2x\r\n",operation);
	writeReg(0x09,0x5A);	 //Disable write protect	
	tmp = readReg(0x06);
	tmp &= 0x07;
	if(operation==0x13)
	{
		writeReg(0x06,0x08|tmp);	 //force enter sleep 2 mode...
	}
	else
	{
		writeReg(0x06,tmp);	 
	}
	writeReg(0x4B,operation);
	writeReg(0x09,0x00);	 //enable write protect
	#endif
}

void enterOrExitPowerDownMode(uint8_t operation)
{
	uint8_t tmp;
	writeReg(0x09,0x5A);	 //Disable write protect	
	tmp = readReg(0x06);
	tmp &= 0x07;
	if(operation==0x13)
	{
		writeReg(0x06,0x08|tmp);	 //force enter sleep 2 mode...
	}
	else
	{
		writeReg(0x06,tmp);	 
	}
	writeReg(0x4B,operation);
	writeReg(0x09,0x00);	 //enable write protect
}

void enterOrExitjustSleepmode(uint8_t operation)
{
	#if 0
	uint8_t tmp;
	writeReg(0x09,0x5A);	 //Disable write protect	
	tmp = readReg(0x06);
	tmp &= 0x07;
	if(operation==0x13)
	{
		writeReg(0x06,0x08|tmp);	 //force enter sleep 2 mode...
	}
	else
	{
		writeReg(0x06,tmp);	 
	}
	writeReg(0x4B,operation);
	writeReg(0x09,0x00);	 //enable write protect
	#else
	writeReg(0x09,0x5A);	 //Disable write protect	
	writeReg(0x05,operation);	 //force enter sleep 2 mode...
	writeReg(0x09,0x00);	 //enable write protect
	#endif
}





