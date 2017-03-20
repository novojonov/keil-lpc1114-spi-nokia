#include "LPC11xx.h"

#include <stdbool.h>

volatile unsigned long sys_ticks = 0;
volatile unsigned long t1, t2;

volatile unsigned char c;
volatile unsigned char l;

volatile int n = 0;

void SysTick_Handler(void);
void delay(int);

#define SPI_TNF (0x1 << 1)
#define SPI_BSY (0x1 << 4)

#define SPI_DC_BIT (0x1 << 6)
#define SPI_RST_BIT (0x1 << 7)

void spi_enable(void);
void spi_disable(void);
void spi_set_command_mode(void);
void spi_set_data_mode(void);
void spi_wait_while_busy(void);
void spi_wait_fifo_not_full(void);
void spi_transmit_byte(unsigned char);

#define BLINK
#undef BLINK

int main(void)
{
	int i, j, k;
	
	SystemCoreClockUpdate();
	
	SysTick_Config(SystemCoreClock / 1000 - 1);
	
	NVIC_SetPriority(SysTick_IRQn, 1);
	NVIC_EnableIRQ(SysTick_IRQn);
	
	LPC_SYSCON->SYSAHBCLKCTRL |= 0x1 << 11 | 0x1 << 6 | 0x1 << 16;
	
	//DC cmd - low, data - high, RST - active low
	LPC_IOCON->PIO2_6 = 0xC0;
	LPC_IOCON->PIO2_7 = 0xC0;
#ifdef BLINK
	LPC_IOCON->PIO0_7 = 0xC0;
#endif
	LPC_GPIO2->DIR |= SPI_DC_BIT | SPI_RST_BIT;
#ifdef BLINK
	LPC_GPIO0->DIR |= (0x1 << 7);
#endif
	
	//Perform reset
	LPC_SYSCON->PRESETCTRL &= ~(0x1 << 0);
	LPC_GPIO2->DATA &= ~SPI_RST_BIT;
	delay(10);
	LPC_SYSCON->PRESETCTRL |= (0x1 << 0);
	LPC_GPIO2->DATA |= SPI_RST_BIT;
	
	//SPI
	LPC_IOCON->PIO0_2 = 0xC1;
	LPC_IOCON->PIO0_6 = 0xC2;
	LPC_IOCON->PIO0_8 = 0xC1;
	LPC_IOCON->PIO0_9 = 0xC1;
	LPC_IOCON->SCK_LOC = 0x02;
	LPC_SYSCON->SSP0CLKDIV = SystemCoreClock / (4000000UL * 2);
	LPC_SSP0->CPSR = 0x02;
	LPC_SSP0->CR0 = 0x07;
	
	spi_enable();

	spi_wait_while_busy();

	spi_set_command_mode();
	
	//Extended mode
	spi_transmit_byte(0x21);
  //Set Vop (contrast)
	spi_transmit_byte(0xBF);
  //Set bias
	spi_transmit_byte(0x14);
  //T coefficient
	spi_transmit_byte(0x04);
  //Normal mode
	spi_transmit_byte(0x20);
  //Set not inversed mode
	spi_transmit_byte(0x0C);
	
	spi_wait_while_busy();

	//Go normal mode
	spi_transmit_byte(0x20);
	//Set row 0
	spi_transmit_byte(0x40);
	//Set column 0
	spi_transmit_byte(0x80);
		
	spi_wait_while_busy();

	spi_set_data_mode();

	for (i = 0; i < 504; i++)
	{
		spi_transmit_byte(0);
	}
	
	spi_wait_while_busy();
		
	c = 0xF0;
	l = 6;
		
	while (true)
	{
		c = ~c;
		
		if (n > 3)
		{
			c = ((c == 0xF0) | (c == 0x0F)) ? 0xCC : 0xF0;
			l = l == 6 ? 3 : 6;
			n = 0;
		}
		
#ifdef BLINK
		if (LPC_GPIO0->DATA & (0x1 << 7))
		{
			LPC_GPIO0->DATA &= ~(0x1 << 7);
		}
		else
		{
			LPC_GPIO0->DATA |= (0x1 << 7);
		}
#endif
		
		spi_enable();
		
		t1 = sys_ticks;

		for (k = 0; k < 6; k++)
		{
			spi_set_command_mode();

			// Normal command
			spi_transmit_byte(0x20);
			// Row k
			spi_transmit_byte(0x40 | (unsigned char)(k & 0x07));
			// Column 0
			spi_transmit_byte(0x80);
			
			spi_wait_while_busy();
			
			spi_set_data_mode();

			for (j = 0; j < 28 * 3 / l; j++)
			{
				for (i = 0; i < l; i++)
				{
					spi_transmit_byte(c);
				}
				c = ~c;
			}
		
			spi_wait_while_busy();
		}
		
		t2 = sys_ticks;
		
		spi_disable();
		
		n++;
		
		delay(500);
	}
}

void SysTick_Handler(void)
{
	sys_ticks++;
}

void delay(int ms)
{
	unsigned long ts = sys_ticks;
	while ((sys_ticks - ts) < ms) {}
}

void spi_enable(void)
{
	LPC_SSP0->CR1 = 0x2;
}

void spi_disable(void)
{
	LPC_SSP0->CR1 = 0x0;
}

void spi_wait_while_busy(void)
{
	while ((LPC_SSP0->SR & SPI_BSY) != 0x0) {}
}

void spi_set_command_mode(void)
{
	spi_wait_while_busy();
	LPC_GPIO2->DATA &= ~SPI_DC_BIT;
}

void spi_set_data_mode(void)
{
	spi_wait_while_busy();
	LPC_GPIO2->DATA |= SPI_DC_BIT;
}

void spi_transmit_byte(unsigned char data)
{
	while ((LPC_SSP0->SR & SPI_TNF) != SPI_TNF) {}
	LPC_SSP0->DR = data;
}
