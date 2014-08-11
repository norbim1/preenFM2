/*
 * Copyright 2013
 *
 * Author: Xavier Hosxe (xavier . hosxe (at) gmail . com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "PreenFM.h"
#include "Common.h"

#define STM32F4D_I2S_MCLK_ENABLE  1
// supported by STM32F4Discovery codec chip:
// I2S_Standard_Phillips, I2S_Standard_MSB, I2S_Standard_LSB,
// I2S_Standard_PCMShort, I2S_Standard_PCMLong
// note that "Philips" is written incorrectly (typo in STM32 driver)
#define STM32F4D_I2S_STANDARD     I2S_Standard_Phillips
// supported by STM32F4Discovery codec chip:
// I2S_DataFormat_16b, I2S_DataFormat_16bextended, I2S_DataFormat_24b, I2S_DataFormat_32b
#define STM32F4D_I2S_DATA_FORMAT  I2S_DataFormat_24b; //I2S_DataFormat_16b;
// the sample rate in hertz
#ifndef OVERCLOCK
// 168000000 / 1105 / 4
#define STM32F4D_I2S_AUDIO_FREQ   38009
#else
// 192000000 / 1116 / 4  :
#define STM32F4D_I2S_AUDIO_FREQ  	43010
#endif

/////////////////////////////////////////////////////////////////////////////
// STM32F4 I2S Pin definitions
/////////////////////////////////////////////////////////////////////////////

#define STM32F4D_I2S_WS_PORT   GPIOA
#define STM32F4D_I2S_WS_PIN    GPIO_Pin_4
#define STM32F4D_I2S_WS_PINSRC GPIO_PinSource4

#define STM32F4D_I2S_CK_PORT   GPIOC
#define STM32F4D_I2S_CK_PIN    GPIO_Pin_10
#define STM32F4D_I2S_CK_PINSRC GPIO_PinSource10

#define STM32F4D_I2S_SD_PORT   GPIOC
#define STM32F4D_I2S_SD_PIN    GPIO_Pin_12
#define STM32F4D_I2S_SD_PINSRC GPIO_PinSource12

#define STM32F4D_I2S_MCLK_PORT GPIOC
#define STM32F4D_I2S_MCLK_PIN  GPIO_Pin_7
#define STM32F4D_I2S_MCLK_PINSRC GPIO_PinSource7

// I2C configuration
// Thanks to http://www.mind-dump.net/configuring-the-stm32f4-discovery-for-audio for this info!

#define CODEC_I2C I2C1
#define CODEC_I2C_SCL_PORT GPIOB
#define CODEC_I2C_SCL_PIN  GPIO_Pin_6
#define CODEC_I2C_SDA_PORT GPIOB
#define CODEC_I2C_SDA_PIN  GPIO_Pin_9
#define CODEC_RESET_PORT   GPIOD
#define CODEC_RESET_PIN    GPIO_Pin_4


#define CORE_I2C_ADDRESS 0x33
#define CODEC_I2C_ADDRESS 0x94

#define CODEC_MAPBYTE_INC 0x80

//register map bytes for CS42L22 (see page 35)
#define CODEC_MAP_CHIP_ID 0x01
#define CODEC_MAP_PWR_CTRL1 0x02
#define CODEC_MAP_PWR_CTRL2 0x04
#define CODEC_MAP_CLK_CTRL  0x05
#define CODEC_MAP_IF_CTRL1  0x06
#define CODEC_MAP_IF_CTRL2  0x07
#define CODEC_MAP_PASSTHROUGH_A_SELECT 0x08
#define CODEC_MAP_PASSTHROUGH_B_SELECT 0x09
#define CODEC_MAP_ANALOG_SET 0x0A
#define CODEC_MAP_PASSTHROUGH_GANG_CTRL 0x0C
#define CODEC_MAP_PLAYBACK_CTRL1 0x0D
#define CODEC_MAP_MISC_CTRL 0x0E
#define CODEC_MAP_PLAYBACK_CTRL2 0x0F
#define CODEC_MAP_PASSTHROUGH_A_VOL 0x14
#define CODEC_MAP_PASSTHROUGH_B_VOL 0x15
#define CODEC_MAP_PCMA_VOL 0x1A
#define CODEC_MAP_PCMB_VOL 0x1B
#define CODEC_MAP_BEEP_FREQ_ONTIME 0x1C
#define CODEC_MAP_BEEP_VOL_OFFTIME 0x1D
#define CODEC_MAP_BEEP_TONE_CFG 0x1E
#define CODEC_MAP_TONE_CTRL 0x1F
#define CODEC_MAP_MASTER_A_VOL 0x20
#define CODEC_MAP_MASTER_B_VOL 0x21
#define CODEC_MAP_HP_A_VOL 0x22
#define CODEC_MAP_HP_B_VOL 0x23
#define CODEC_MAP_SPEAK_A_VOL 0x24
#define CODEC_MAP_SPEAK_B_VOL 0x25
#define CODEC_MAP_CH_MIX_SWAP 0x26
#define CODEC_MAP_LIMIT_CTRL1 0x27
#define CODEC_MAP_LIMIT_CTRL2 0x28
#define CODEC_MAP_LIMIT_ATTACK 0x29
#define CODEC_MAP_OVFL_CLK_STATUS 0x2E
#define CODEC_MAP_BATT_COMP 0x2F
#define CODEC_MAP_VP_BATT_LEVEL 0x30
#define CODEC_MAP_SPEAK_STATUS 0x31
#define CODEC_MAP_CHARGE_PUMP_FREQ 0x34

static void codec_init();
static void send_codec_ctrl(uint8_t controlBytes[], uint8_t numBytes);
static uint8_t read_codec_register(uint8_t mapByte);

int sample_buffer[SAMPLE_BUFFER_SIZE]; // sample buffer used for DMA

void strobePin(uint8_t count, uint32_t rate) {
	GPIO_ResetBits(GPIOD, LEDBPIN);
	uint32_t c;
	while (count-- > 0) {
		for (c = rate; c > 0; c--) {
			asm volatile ("nop");
		}
		GPIO_SetBits(GPIOD, LEDBPIN);
		for (c = rate; c > 0; c--) {
			asm volatile ("nop");
		}
		GPIO_ResetBits(GPIOD, LEDBPIN);
	}
}


/**
 * @brief  Configures the USART Peripheral.
 * @param  None
 * @retval None
 */
void USART_Config() {

	/* --------------------------- System Clocks Configuration -----------------*/
	/* USART3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* GPIOB clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	/*-------------------------- GPIO Configuration ----------------------------*/
	// TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Connect USART pins to AF */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

	// Init USART
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 31250;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	/* Here the USART3 receive interrupt is enabled
	 * and the interrupt controller is configured
	 * to jump to the USART3_IRQHandler() function
	 * if the USART3 receive interrupt occurs
	 */
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // enable the USART3 receive interrupt
	USART_ITConfig(USART3, USART_IT_TXE, DISABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; // we want to configure the USART3 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10; // this sets the priority group of the USART3 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 10; // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // the USART3 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure); // the properties are passed to the NVIC_Init function which takes care of the low level stuff

	USART_Cmd(USART3, ENABLE);
}

void LED_Config() {
	// GPIOG Periph clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Configure PD15 in output mode */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = (LEDBPIN |LEDGPIN |LEDOPIN |LEDRPIN);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void CS43L22_Config() {
	  // configure STM32F4Discovery I2S pins

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	  GPIO_StructInit(&GPIO_InitStructure);
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

	  GPIO_InitStructure.GPIO_Pin = STM32F4D_I2S_WS_PIN;
	  GPIO_Init(STM32F4D_I2S_WS_PORT, &GPIO_InitStructure);
	  GPIO_PinAFConfig(STM32F4D_I2S_WS_PORT, STM32F4D_I2S_WS_PINSRC, GPIO_AF_SPI3);

	  GPIO_InitStructure.GPIO_Pin = STM32F4D_I2S_CK_PIN;
	  GPIO_Init(STM32F4D_I2S_CK_PORT, &GPIO_InitStructure);
	  GPIO_PinAFConfig(STM32F4D_I2S_CK_PORT, STM32F4D_I2S_CK_PINSRC, GPIO_AF_SPI3);

	  GPIO_InitStructure.GPIO_Pin = STM32F4D_I2S_SD_PIN;
	  GPIO_Init(STM32F4D_I2S_SD_PORT, &GPIO_InitStructure);
	  GPIO_PinAFConfig(STM32F4D_I2S_SD_PORT, STM32F4D_I2S_SD_PINSRC, GPIO_AF_SPI3);

	  GPIO_InitStructure.GPIO_Pin = STM32F4D_I2S_MCLK_PIN;
	  GPIO_Init(STM32F4D_I2S_MCLK_PORT, &GPIO_InitStructure);
	  GPIO_PinAFConfig(STM32F4D_I2S_MCLK_PORT, STM32F4D_I2S_MCLK_PINSRC, GPIO_AF_SPI3);

	  // configure I2C pins to access the CS43L22 configuration registers

	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	  GPIO_InitStructure.GPIO_Pin = CODEC_I2C_SCL_PIN;
	  GPIO_Init(CODEC_I2C_SCL_PORT, &GPIO_InitStructure);
	  GPIO_PinAFConfig(CODEC_I2C_SCL_PORT, GPIO_PinSource6, GPIO_AF_I2C1);

	  GPIO_InitStructure.GPIO_Pin = CODEC_I2C_SDA_PIN;
	  GPIO_Init(CODEC_I2C_SDA_PORT, &GPIO_InitStructure);
	  GPIO_PinAFConfig(CODEC_I2C_SDA_PORT, GPIO_PinSource9, GPIO_AF_I2C1);

	  // CS43L22 reset pin
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Pin = CODEC_RESET_PIN;
	  GPIO_Init(CODEC_RESET_PORT, &GPIO_InitStructure);

	  GPIO_ResetBits(CODEC_RESET_PORT, CODEC_RESET_PIN); // activate reset

	  // I2S initialisation
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
	  RCC_PLLI2SCmd(ENABLE); // new for STM32F4: enable I2S PLL
	  SPI_I2S_DeInit(SPI3);
	  I2S_InitTypeDef I2S_InitStructure;
	  I2S_StructInit(&I2S_InitStructure);
	  I2S_InitStructure.I2S_Standard = STM32F4D_I2S_STANDARD;
	  I2S_InitStructure.I2S_DataFormat = STM32F4D_I2S_DATA_FORMAT;
	  I2S_InitStructure.I2S_MCLKOutput = STM32F4D_I2S_MCLK_ENABLE ? I2S_MCLKOutput_Enable : I2S_MCLKOutput_Disable;
	  I2S_InitStructure.I2S_AudioFreq  = (u16)(STM32F4D_I2S_AUDIO_FREQ);
	  I2S_InitStructure.I2S_CPOL = I2S_CPOL_Low; // configuration required as well?
	  I2S_InitStructure.I2S_Mode = I2S_Mode_MasterTx;
	  I2S_Init(SPI3, &I2S_InitStructure);
	  I2S_Cmd(SPI3, ENABLE);

	  // DMA Configuration for SPI Tx Event
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	  DMA_InitTypeDef DMA_InitStructure;
	  DMA_StructInit(&DMA_InitStructure);

	  DMA_Cmd(DMA1_Stream5, DISABLE);
	  DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_TEIF5 | DMA_FLAG_HTIF5 | DMA_FLAG_FEIF5);
	  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI3->DR;
	  //  DMA_InitStructure.DMA_MemoryBaseAddr = ...; // configured in CS43L22_Start
	  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	  //  DMA_InitStructure.DMA_BufferSize = ...; // configured in CS43L22_Start
	  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	  DMA_Init(DMA1_Stream5, &DMA_InitStructure);
	  // DMA_Cmd(DMA1_Stream5, ENABLE); // done on CS43L22_Start

	  DMA_ITConfig(DMA1_Stream5, DMA_IT_TE | DMA_IT_FE, DISABLE);
	  // trigger interrupt when transfer half complete/complete
	  DMA_ITConfig(DMA1_Stream5, DMA_IT_HT | DMA_IT_TC, ENABLE);

	  // enable SPI interrupts to DMA
	  SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, ENABLE);

	  // Configure and enable DMA interrupt
	  /* Configure the DMA IRQ handler priority */
	  NVIC_SetPriority(DMA1_Stream5_IRQn, 0x0);
	  NVIC_EnableIRQ(DMA1_Stream5_IRQn);

	  // configure I2C
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	  I2C_DeInit(CODEC_I2C);
	  I2C_InitTypeDef I2C_InitStructure;
	  I2C_StructInit(&I2C_InitStructure);
	  I2C_InitStructure.I2C_ClockSpeed = 100000;
	  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	  I2C_InitStructure.I2C_OwnAddress1 = CORE_I2C_ADDRESS;
	  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;

	  I2C_Cmd(CODEC_I2C, ENABLE);
	  I2C_Init(CODEC_I2C, &I2C_InitStructure);

	  codec_init();
}

///////////////////////////////////////////////////////////////////////////////
////! Stops DMA driven I2S transfers
///////////////////////////////////////////////////////////////////////////////
void CS43L22_Stop(void)
{
  // disable DMA
  DMA_Cmd(DMA1_Stream5, DISABLE);
}

///////////////////////////////////////////////////////////////////////////////
////! Starts DMA driven I2S transfers
////! \param[in] *buffer pointer to sample buffer (contains L/R halfword)
////! \param[in] len size of audio buffer
///////////////////////////////////////////////////////////////////////////////
void CS43L22_Start(u32 *buffer, u16 len)
{
	// start I2S DMA transfers

	RCC_PLLI2SCmd(DISABLE);
#ifndef OVERCLOCK
	RCC_PLLI2SConfig((uint32_t)214,(uint32_t)2); //I2S PLL dividers for 38009 calculated from stm32f407 datasheet!!!
#else
	RCC_PLLI2SConfig((uint32_t)198,(uint32_t)2); //I2S PLL dividers for 43010 calculated from stm32f407 datasheet!!!
#endif
	CS43L22_Config();

	// reload DMA source address and counter
	DMA_Cmd(DMA1_Stream5, DISABLE);

	// ensure that IRQ flag is cleared (so that DMA IRQ isn't invoked by accident while this function is called)
	DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_TEIF5 | DMA_FLAG_HTIF5 | DMA_FLAG_FEIF5);

	// take over new buffer pointer/length
	DMA1_Stream5->M0AR = (u32)buffer;
	DMA1_Stream5->NDTR = 2*len;

	DMA_Cmd(DMA1_Stream5, ENABLE);
}

void RNG_Config(void)
{
 /* Enable RNG clock source */
  RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);

  /* RNG Peripheral enable */
  RNG_Cmd(ENABLE);
}

// I2C configuration
static void codec_init()
{
  //uint32_t delaycount;
  uint8_t CodecCommandBuffer[5];

  uint8_t regValue = 0xFF;

  GPIO_SetBits(CODEC_RESET_PORT, CODEC_RESET_PIN);
  //delaycount = 10000000;
  //while (delaycount > 0)
  //{
  //	  delaycount--;
  //}
  PreenFM2_uDelay(100);

  //keep codec OFF
  CodecCommandBuffer[0] = CODEC_MAP_PLAYBACK_CTRL1;
  CodecCommandBuffer[1] = 0x01;
  send_codec_ctrl(CodecCommandBuffer, 2);

  //begin initialization sequence (p. 32)
  CodecCommandBuffer[0] = 0x00;
  CodecCommandBuffer[1] = 0x99;
  send_codec_ctrl(CodecCommandBuffer, 2);

  CodecCommandBuffer[0] = 0x47;
  CodecCommandBuffer[1] = 0x80;
  send_codec_ctrl(CodecCommandBuffer, 2);

  regValue = read_codec_register(0x32);

  CodecCommandBuffer[0] = 0x32;
  CodecCommandBuffer[1] = regValue | 0x80;
  send_codec_ctrl(CodecCommandBuffer, 2);

  regValue = read_codec_register(0x32);

  CodecCommandBuffer[0] = 0x32;
  CodecCommandBuffer[1] = regValue & (~0x80);
  send_codec_ctrl(CodecCommandBuffer, 2);

  CodecCommandBuffer[0] = 0x00;
  CodecCommandBuffer[1] = 0x00;
  send_codec_ctrl(CodecCommandBuffer, 2);
  //end of initialization sequence

  CodecCommandBuffer[0] = CODEC_MAP_PWR_CTRL2;
  CodecCommandBuffer[1] = 0xAF;
  send_codec_ctrl(CodecCommandBuffer, 2);

  CodecCommandBuffer[0] = CODEC_MAP_PLAYBACK_CTRL1;
  CodecCommandBuffer[1] = 0x70;
  send_codec_ctrl(CodecCommandBuffer, 2);

  CodecCommandBuffer[0] = CODEC_MAP_CLK_CTRL;
  CodecCommandBuffer[1] = 0x81; //auto detect clock
  send_codec_ctrl(CodecCommandBuffer, 2);

  CodecCommandBuffer[0] = CODEC_MAP_IF_CTRL1;
  CodecCommandBuffer[1] = 0x05; //24bit
  send_codec_ctrl(CodecCommandBuffer, 2);

  CodecCommandBuffer[0] = 0x0A;
  CodecCommandBuffer[1] = 0x00;
  send_codec_ctrl(CodecCommandBuffer, 2);

  CodecCommandBuffer[0] = 0x27;
  CodecCommandBuffer[1] = 0x00;
  send_codec_ctrl(CodecCommandBuffer, 2);

  CodecCommandBuffer[0] = 0x1A | CODEC_MAPBYTE_INC;
  CodecCommandBuffer[1] = 0x0A;
  CodecCommandBuffer[2] = 0x0A;
  send_codec_ctrl(CodecCommandBuffer, 3);

  CodecCommandBuffer[0] = 0x1F;
  CodecCommandBuffer[1] = 0x0F;
  send_codec_ctrl(CodecCommandBuffer, 2);

  CodecCommandBuffer[0] = CODEC_MAP_PWR_CTRL1;
  CodecCommandBuffer[1] = 0x9E;
  send_codec_ctrl(CodecCommandBuffer, 2);
}

static void send_codec_ctrl(uint8_t controlBytes[], uint8_t numBytes)
{
  uint8_t bytesSent=0;

  while (I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BUSY)) {
    //just wait until no longer busy
  }

  I2C_GenerateSTART(CODEC_I2C, ENABLE);
  while (!I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_SB)) {
    //wait for generation of start condition
  }
  I2C_Send7bitAddress(CODEC_I2C, CODEC_I2C_ADDRESS, I2C_Direction_Transmitter);
  while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
    //wait for end of address transmission
  }
  while (bytesSent < numBytes) {
    I2C_SendData(CODEC_I2C, controlBytes[bytesSent]);
    bytesSent++;
    while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING)) {
      //wait for transmission of byte
    }
  }
  while(!I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BTF)) {
    //wait until it's finished sending before creating STOP
  }
  I2C_GenerateSTOP(CODEC_I2C, ENABLE);

}

static uint8_t read_codec_register(uint8_t mapbyte)
{
  uint8_t receivedByte = 0;

  while (I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BUSY)) {
    //just wait until no longer busy
  }

  I2C_GenerateSTART(CODEC_I2C, ENABLE);
  while (!I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_SB)) {
    //wait for generation of start condition
  }

  I2C_Send7bitAddress(CODEC_I2C, CODEC_I2C_ADDRESS, I2C_Direction_Transmitter);
  while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
    //wait for end of address transmission
  }

  I2C_SendData(CODEC_I2C, mapbyte); //sets the transmitter address
  while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING)) {
    //wait for transmission of byte
  }

  I2C_GenerateSTOP(CODEC_I2C, ENABLE);

  while (I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BUSY)) {
    //just wait until no longer busy
  }

  I2C_AcknowledgeConfig(CODEC_I2C, DISABLE);

  I2C_GenerateSTART(CODEC_I2C, ENABLE);
  while (!I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_SB)) {
    //wait for generation of start condition
  }

  I2C_Send7bitAddress(CODEC_I2C, CODEC_I2C_ADDRESS, I2C_Direction_Receiver);
  while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
    //wait for end of address transmission
  }

  while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
    //wait until byte arrived
  }
  receivedByte = I2C_ReceiveData(CODEC_I2C);

  I2C_GenerateSTOP(CODEC_I2C, ENABLE);

  return receivedByte;
}


void LCD_InitChars(LiquidCrystal *lcd) {
    unsigned char row0[8] = {
            0b11111,
            0b11111,
            0b11111,
            0b11111,
            0b11111,
            0b11111,
            0b11111,
            0b11111,
    };
    unsigned char row1[8] = {
            0b01111,
            0b01111,
            0b01111,
            0b01111,
            0b01111,
            0b01111,
            0b01111,
            0b01111,
    };
    unsigned char row2[8] = {
            0b00111,
            0b00111,
            0b00111,
            0b00111,
            0b00111,
            0b00111,
            0b00111,
            0b01111,
    };
    unsigned char row3[8] = {
            0b00011,
            0b00011,
            0b00011,
            0b00011,
            0b00011,
            0b00011,
            0b00011,
            0b00011,
    };
    unsigned char row4[8] = {
            0b00001,
            0b00001,
            0b00001,
            0b00001,
            0b00001,
            0b00001,
            0b00001,
            0b00001,
    };

    unsigned char row5[8] = {
            0b00000,
            0b00000,
            0b00000,
            0b00000,
            0b00000,
            0b00000,
            0b00000,
            0b00000,
    };


    lcd->createChar(0, row0);
    lcd->createChar(1, row1);
    lcd->createChar(2, row2);
    lcd->createChar(3, row3);
    lcd->createChar(4, row4);
    lcd->createChar(5, row5);

}



