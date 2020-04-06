/*
 * (C) Copyright 2007-2008
 * Stelian Pop <stelian@popies.net>
 * Lead Tech Design <www.leadtechdesign.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/at91sam9260_matrix.h>
#include <asm/arch/at91sam9_smc.h>
#include <asm/arch/at91_common.h>
#include <asm/arch/at91_pmc.h>
#include <asm/arch/at91_rstc.h>
#include <asm/arch/gpio.h>

#if defined(CONFIG_RESET_PHY_R) && defined(CONFIG_MACB)
# include <net.h>
#endif
#include <netdev.h>

//#include "HC595_spi.h"



DECLARE_GLOBAL_DATA_PTR;

/* ------------------------------------------------------------------------- */
/*
 * Miscelaneous platform dependent initialisations
 */

#ifdef CONFIG_CMD_NAND
static void at91sam9260ek_nand_hw_init(void)
{
	struct at91_smc *smc = (struct at91_smc *)ATMEL_BASE_SMC;
	struct at91_matrix *matrix = (struct at91_matrix *)ATMEL_BASE_MATRIX;
	unsigned long csa;

	/* Assign CS3 to NAND/SmartMedia Interface */
	csa = readl(&matrix->ebicsa);
	csa |= AT91_MATRIX_CS3A_SMC_SMARTMEDIA;
	writel(csa, &matrix->ebicsa);

	/* Configure SMC CS3 for NAND/SmartMedia */
	writel(AT91_SMC_SETUP_NWE(1) | AT91_SMC_SETUP_NCS_WR(0) |
		AT91_SMC_SETUP_NRD(1) | AT91_SMC_SETUP_NCS_RD(0),
		&smc->cs[3].setup);
	writel(AT91_SMC_PULSE_NWE(3) | AT91_SMC_PULSE_NCS_WR(3) |
		AT91_SMC_PULSE_NRD(3) | AT91_SMC_PULSE_NCS_RD(3),
		&smc->cs[3].pulse);
	writel(AT91_SMC_CYCLE_NWE(5) | AT91_SMC_CYCLE_NRD(5),
		&smc->cs[3].cycle);
	writel(AT91_SMC_MODE_RM_NRD | AT91_SMC_MODE_WM_NWE |
		AT91_SMC_MODE_EXNW_DISABLE |
#ifdef CONFIG_SYS_NAND_DBW_16
		AT91_SMC_MODE_DBW_16 |
#else /* CONFIG_SYS_NAND_DBW_8 */
		AT91_SMC_MODE_DBW_8 |
#endif
		AT91_SMC_MODE_TDF_CYCLE(2),
		&smc->cs[3].mode);

	/* Configure RDY/BSY */
	at91_set_gpio_input(CONFIG_SYS_NAND_READY_PIN, 1);
	/*configure wp pin*/
	at91_set_gpio_output(CONFIG_SYS_NAND_WP_PIN, 1);

	/* Enable NandFlash */
	at91_set_gpio_output(CONFIG_SYS_NAND_ENABLE_PIN, 1);

}
#endif

#ifdef CONFIG_MACB
static void at91sam9260ek_macb_hw_init(void)
{
	struct at91_pmc *pmc = (struct at91_pmc *)ATMEL_BASE_PMC;
	struct at91_port *pioa = (struct at91_port *)ATMEL_BASE_PIOA;
	struct at91_rstc *rstc = (struct at91_rstc *)ATMEL_BASE_RSTC;
	unsigned long erstl;
	int i = 0xffff;
	
	at91_set_pio_output(AT91_PIO_PORTB, 26, 1); 

	/* Enable EMAC clock */
	writel(1 << ATMEL_ID_EMAC0, &pmc->pcer);

	/*
	 * Disable pull-up on:
	 *	ERX0 (PA14) => MODE0
	 *	ERX1 (PA15) => MODE1
	 *	RXDV (PA17) => MODE2
	 *    RESET(PA18) => reset
	 * PHY has internal pull-down
	 */
	writel(pin_to_mask(AT91_PIN_PA14) |
		pin_to_mask(AT91_PIN_PA15) |
		pin_to_mask(AT91_PIN_PA17) ,
		&pioa->pudr);

	/*set The power up reset latch PIO for PHY*/
	//
	at91_set_gpio_output(AT91_PIN_PA17, 1);
	at91_set_gpio_output(AT91_PIN_PA14, 1);
	at91_set_gpio_output(AT91_PIN_PA15, 1);
	//RESET
	at91_set_gpio_output(AT91_PIN_PB23, 1);

	erstl = readl(&rstc->mr) & AT91_RSTC_MR_ERSTL_MASK;
	

	/* Need to reset PHY -> 500ms reset */
	writel(AT91_RSTC_KEY | AT91_RSTC_MR_ERSTL(13) |
		AT91_RSTC_MR_URSTEN, &rstc->mr);

	writel(AT91_RSTC_KEY | AT91_RSTC_CR_EXTRST, &rstc->cr);
			
	at91_set_gpio_output(AT91_PIN_PB23, 0);

	/* Wait for end hardware reset */
	while (!(readl(&rstc->sr) & AT91_RSTC_SR_NRSTL))
		;
	
	at91_set_pio_output(AT91_PIO_PORTB, 23, 0);	
	while(i--);
	at91_set_pio_output(AT91_PIO_PORTB, 23, 1);	
	at91_set_pio_output(AT91_PIO_PORTB, 26, 0);

	/* Restore NRST value */
	writel(AT91_RSTC_KEY | erstl | AT91_RSTC_MR_URSTEN,
		&rstc->mr);

	/* Re-enable pull-up */
	writel(pin_to_mask(AT91_PIN_PA14) |
		pin_to_mask(AT91_PIN_PA15) |
		pin_to_mask(AT91_PIN_PA17) ,
		&pioa->puer);
	
	/* Initialize EMAC=MACB hardware */
	at91_macb_hw_init();
}
#endif

int board_early_init_f(void)
{
	struct at91_pmc *pmc = (struct at91_pmc *)ATMEL_BASE_PMC;

	/* Enable clocks for all PIOs */
	writel((1 << ATMEL_ID_PIOA) | (1 << ATMEL_ID_PIOB) |
		(1 << ATMEL_ID_PIOC),
		&pmc->pcer);

	return 0;
}

int board_init(void)
{
#ifdef CONFIG_AT91SAM9G20EK
	/* arch number of AT91SAM9260EK-Board */
	gd->bd->bi_arch_number = MACH_TYPE_AT91SAM9G20EK;
#else
	/* arch number of AT91SAM9260EK-Board */
	gd->bd->bi_arch_number = MACH_TYPE_AT91SAM9260EK;
#endif
	/* adress of boot parameters */
	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;

	at91_seriald_hw_init();
#ifdef CONFIG_CMD_NAND
	at91sam9260ek_nand_hw_init();
#endif
#ifdef CONFIG_HAS_DATAFLASH
	at91_spi0_hw_init((1 << 0) | (1 << 1));
#endif
	//HC595_SPI_Init();
#ifdef CONFIG_MACB
	at91sam9260ek_macb_hw_init();
#endif

	return 0;
}

int dram_init(void)
{
	gd->ram_size = get_ram_size(
		(void *)CONFIG_SYS_SDRAM_BASE,
		CONFIG_SYS_SDRAM_SIZE);
	return 0;
}

#ifdef CONFIG_RESET_PHY_R
void reset_phy(void)
{
	int i = 0xfff;
	at91_set_pio_output(AT91_PIO_PORTB, 19, 0);	
	while(i--);
	at91_set_pio_output(AT91_PIO_PORTB, 19, 1);	
}
#endif

int board_eth_init(bd_t *bis)
{
	int rc = 0;
#ifdef CONFIG_MACB
	rc = macb_eth_initialize(0, (void *)ATMEL_BASE_EMAC0, 0x00);
#endif
	return rc;
}
