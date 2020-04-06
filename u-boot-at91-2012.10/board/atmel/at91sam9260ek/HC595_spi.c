

#include <common.h>
#include <spi.h>

#include <asm/io.h>
#include <asm/arch/at91_common.h>
#include <asm/arch/at91_pmc.h>
#include <asm/arch/at91sam9260.h>
#include <asm/arch/at91_pio.h>
#include <asm/arch/at91_spi.h>
#include <asm/arch/clk.h>
#include <asm/arch/gpio.h>
#include <asm/arch/hardware.h>


#include "HC595_spi.h"


#ifdef CONFIG_AT91_GPIO_PULLUP
# define PUP CONFIG_AT91_GPIO_PULLUP
#else
# define PUP 0
#endif



#define BOARD_HC595_NPCS             	2

#define	BOARD_HC595_NPCS_PIN				AT91_PIN_PC16	/* this is  NPCS pin*/
#define	BOARD_HC595_OE_PIN					AT91_PIN_PC4/* this is the OE pin */
#define	BOARD_HC595_SPI0_MOSI_PIN			AT91_PIN_PA1/* this is the OE pin */
#define	BOARD_HC595_SPI0_MISO_PIN			AT91_PIN_PA0/* this is the OE pin */
#define	BOARD_HC595_SPI0_SPCK_PIN			AT91_PIN_PA2/* this is the OE pin */


#define ENABLE_CE()        at91_set_gpio_value(BOARD_HC595_NPCS_PIN, 0)//PIO_Clear(pin)  //do{}while(0)//
#define DISABLE_CE()       at91_set_gpio_value(BOARD_HC595_NPCS_PIN, 1)//PIO_Set(pin)		 //do{}while(0)//

#define ENABLE_OE()        at91_set_gpio_value(BOARD_HC595_OE_PIN, 1)//PIO_Set(pin)  //do{}while(0)//
#define DISABLE_OE()       at91_set_gpio_value(BOARD_HC595_OE_PIN, 0)//PIO_Clear(pin)		 //do{}while(0)//



/// Calculate the PCS field value given the chip select NPCS value
#define SPI_PCS(npcs)       ((~(1 << npcs) & 0xF) << 16)

/// Calculates the value of the CSR SCBR field given the baudrate and MCK.
#define SPI_SCBR(baudrate, masterClock) \
            ((unsigned int) (masterClock / baudrate) << 8)

/// Calculates the value of the CSR DLYBS field given the desired delay (in ns)
#define SPI_DLYBS(delay, masterClock) \
            ((unsigned int) (((masterClock / 1000000) * delay) / 1000) << 16)

/// Calculates the value of the CSR DLYBCT field given the desired delay (in ns)
#define SPI_DLYBCT(delay, masterClock) \
            ((unsigned int) (((masterClock / 1000000) * delay) / 32000) << 16)


static unsigned int HC595_data =  HC595_BIT_DM9161_RESET | HC595_BIT_LED11_RED | HC595_BIT_LED12_RED | HC595_BIT_LED16;

void HC595_SPI_Init(void)
{
	at91_pmc_t	*pmc	= (at91_pmc_t *) ATMEL_BASE_PMC;

	//配置管脚
	/* Clock is enabled in board_early_init_f() */
	at91_set_gpio_output(BOARD_HC595_NPCS_PIN, 1);
	at91_set_gpio_output(BOARD_HC595_OE_PIN, 1);
	
	at91_set_a_periph(AT91_PIO_PORTA, 0, PUP);	/* SPI0_MISO */
	at91_set_a_periph(AT91_PIO_PORTA, 1, PUP);	/* SPI0_MOSI */
	at91_set_a_periph(AT91_PIO_PORTA, 2, PUP);	/* SPI0_SPCK */
	
	/* Enable spi clock */
	writel(1 << ATMEL_ID_SPI0, &pmc->pcer);

	//disable SPI
	writel(AT91_SPI_SPIDIS, ATMEL_BASE_SPI0+AT91_SPI_CR);
	// Execute a software reset of the SPI twice
	writel(AT91_SPI_SWRST, ATMEL_BASE_SPI0 + AT91_SPI_CR);
	writel(AT91_SPI_SWRST, ATMEL_BASE_SPI0 + AT91_SPI_CR);
	//配置模式寄存器
	writel(AT91_SPI_MSTR | AT91_SPI_MODFDIS | AT91_SPI_PCS, ATMEL_BASE_SPI0+AT91_SPI_MR);
	//配置片选寄存器
	writel(AT91_SPI_CPOL| AT91_SPI_BITS_16 |SPI_SCBR(1000000,get_mck_clk_rate()), ATMEL_BASE_SPI0 + AT91_SPI_CSR(BOARD_HC595_NPCS));
	/* SPI_Enable */
	writel(AT91_SPI_SPIEN, ATMEL_BASE_SPI0 + AT91_SPI_CR);
	//关闭spi时钟
	writel(1 << ATMEL_ID_SPI0, &pmc->pcdr);
	//HC595输出使能
	ENABLE_OE();	
	SPI_Send(HC595_data);
}
void SPI_Send(unsigned int data)
{
	unsigned int spiMr;
	at91_pmc_t	*pmc	= (at91_pmc_t *) ATMEL_BASE_PMC;
	/* Enable clock */
	writel(1 << ATMEL_ID_SPI0, &pmc->pcer);
  	ENABLE_CE();	
	
	spiMr = readl(ATMEL_BASE_SPI0 + AT91_SPI_MR);
	spiMr |= AT91_SPI_PCS;
	spiMr &= ~((1 << BOARD_HC595_NPCS) << 16);	
	writel(spiMr, ATMEL_BASE_SPI0 + AT91_SPI_MR);
   
	writel(AT91_SPI_SPIEN, ATMEL_BASE_SPI0 + AT91_SPI_CR);
	//写入高16bit
	while((readl(ATMEL_BASE_SPI0 + AT91_SPI_SR)&AT91_SPI_TDRE) == 0);
	//WRITE_SPI(BOARD_HC595_SPI_BASE, SPI_TDR, (data>>16)&0xffff);
	writel((data>>16)&0xffff, ATMEL_BASE_SPI0 + AT91_SPI_TDR);
	//写入低16bit
   	while((readl(ATMEL_BASE_SPI0 + AT91_SPI_SR)&AT91_SPI_TDRE) == 0);
	//WRITE_SPI(BOARD_HC595_SPI_BASE, SPI_TDR, data&0xffff);
	writel(data&0xffff, ATMEL_BASE_SPI0 + AT91_SPI_TDR);

	//while((READ_SPI(BOARD_HC595_SPI_BASE, SPI_SR)&AT91C_SPI_TXEMPTY) == 0);
	while((readl(ATMEL_BASE_SPI0 + AT91_SPI_SR)&AT91_SPI_TXEMPTY) == 0);

	//WRITE_SPI(BOARD_HC595_SPI_BASE, SPI_CR, AT91C_SPI_SPIDIS);
	writel(AT91_SPI_SPIDIS, ATMEL_BASE_SPI0 + AT91_SPI_CR);

	spiMr = readl(ATMEL_BASE_SPI0 + AT91_SPI_MR);
	spiMr |= AT91_SPI_PCS;
	writel(spiMr, ATMEL_BASE_SPI0 + AT91_SPI_MR); 
	
	//WRITE_PMC(AT91C_BASE_PMC, PMC_PCDR, (1 <<BOARD_HC595_SPI_ID));
	//at91_sys_write(AT91_PMC_PCDR, 1 << AT91SAM9260_ID_SPI0);
	writel(1 << ATMEL_ID_SPI0, &pmc->pcdr);
	DISABLE_CE();
}

void HC595_set_all(unsigned int data, unsigned char updateFlag)
{
	HC595_data = data;
 	if(1 == updateFlag)
	{
	 	SPI_Send(HC595_data);
	} 
}
void HC595_set_bits(unsigned int bits, unsigned char updateFlag)
{
	HC595_data |= bits;
 	if(1 == updateFlag)
	{
	 	SPI_Send(HC595_data);
	} 
}
void HC595_clr_bits(unsigned int bits, unsigned char updateFlag)
{
	HC595_data &= (~bits);
 	if(1 == updateFlag)
	{
	 	SPI_Send(HC595_data);
	} 
}

