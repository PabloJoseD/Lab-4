/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2014 Chuck McManis <cmcmanis@mcmanis.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <math.h>
#include "clock.h"
#include "console.h"
#include "sdram.h"
#include "lcd-spi.h"
#include "gfx.h"

 
#include </home/pablo/git/libopencm3-examples/libopencm3/include/libopencm3/stm32/rcc.h>
#include </home/pablo/git/libopencm3-examples/libopencm3/include/libopencm3/stm32/usart.h>
#include </home/pablo/git/libopencm3-examples/libopencm3/include/libopencm3/stm32/spi.h>
#include </home/pablo/git/libopencm3-examples/libopencm3/include/libopencm3/stm32/gpio.h>

#define LBLUE GPIOE, GPIO8
#define LRED GPIOE, GPIO9
#define LORANGE GPIOE, GPIO10
#define LGREEN GPIOE, GPIO11
#define LBLUE2 GPIOE, GPIO12
#define LRED2 GPIOE, GPIO13
#define LORANGE2 GPIOE, GPIO14
#define LGREEN2 GPIOE, GPIO15

#define LD4 GPIOE, GPIO8
#define LD3 GPIOE, GPIO9
#define LD5 GPIOE, GPIO10
#define LD7 GPIOE, GPIO11
#define LD9 GPIOE, GPIO12
#define LD10 GPIOE, GPIO13
#define LD8 GPIOE, GPIO14
#define LD6 GPIOE, GPIO15 

/* Convert degrees to radians */
#define d2r(d) ((d) * 6.2831853 / 360.0)

/*
 * This is our example, the heavy lifing is actually in lcd-spi.c but
 * this drives that code.
 */
 

static void spi_setup(void)
{
	rcc_periph_clock_enable(RCC_SPI1);
	/* For spi signal pins */
	rcc_periph_clock_enable(RCC_GPIOA);
	/* For spi mode select on the l3gd20 */
	rcc_periph_clock_enable(RCC_GPIOE);

	/* Setup GPIOE3 pin for spi mode l3gd20 select. */
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3);
	/* Start with spi communication disabled */
	gpio_set(GPIOE, GPIO3);

	/* Setup GPIO pins for AF5 for SPI1 signals. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
			GPIO5 | GPIO6 | GPIO7);
	gpio_set_af(GPIOA, GPIO_AF5, GPIO5 | GPIO6 | GPIO7);

	//spi initialization;
	spi_set_master_mode(SPI1);
	spi_set_baudrate_prescaler(SPI1, SPI_CR1_BR_FPCLK_DIV_64);
	spi_set_clock_polarity_0(SPI1);
	spi_set_clock_phase_0(SPI1);
	spi_set_full_duplex_mode(SPI1);
	spi_set_unidirectional_mode(SPI1); /* bidirectional but in 3-wire */
	spi_set_data_size(SPI1, SPI_CR2_DS_8BIT);
	spi_enable_software_slave_management(SPI1);
	spi_send_msb_first(SPI1);
	spi_set_nss_high(SPI1);
	//spi_enable_ss_output(SPI1);
	spi_fifo_reception_threshold_8bit(SPI1);
	SPI_I2SCFGR(SPI1) &= ~SPI_I2SCFGR_I2SMOD;
	spi_enable(SPI1);
}

static void usart_setup(void)
{
	/* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2| GPIO3);

	/* Setup UART parameters. */
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART2);
}

static void gpio_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOE);
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
		GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 |
		GPIO14 | GPIO15);
}

static void my_usart_print_int(uint32_t usart, int32_t value)
{
	int8_t i;
	int8_t nr_digits = 0;
	char buffer[25];

	if (value < 0) {
		usart_send_blocking(usart, '-');
		value = value * -1;
	}

	if (value == 0) {
		usart_send_blocking(usart, '0');
	}

	while (value > 0) {
		buffer[nr_digits++] = "0123456789"[value % 10];
		value /= 10;
	}

	for (i = nr_digits-1; i >= 0; i--) {
		usart_send_blocking(usart, buffer[i]);
	}

	usart_send_blocking(usart, '\r');
	usart_send_blocking(usart, '\n');
}

static void clock_setup(void)
{
	rcc_clock_setup_hsi(&rcc_hsi_configs[RCC_CLOCK_HSI_64MHZ]);
}

#define GYR_RNW			(1 << 7) /* Write when zero */
#define GYR_MNS			(1 << 6) /* Multiple reads when 1 */
#define GYR_WHO_AM_I		0x0F
#define GYR_OUT_TEMP		0x26
#define GYR_STATUS_REG		0x27
#define GYR_CTRL_REG1		0x20
#define GYR_CTRL_REG1_PD	(1 << 3)
#define GYR_CTRL_REG1_XEN	(1 << 1)
#define GYR_CTRL_REG1_YEN	(1 << 0)
#define GYR_CTRL_REG1_ZEN	(1 << 2)
#define GYR_CTRL_REG1_BW_SHIFT	4
#define GYR_CTRL_REG4		0x23
#define GYR_CTRL_REG4_FS_SHIFT	4

#define GYR_OUT_X_L		0x28
#define GYR_OUT_X_H		0x29


int main(void)
{
	//int p1, p2, p3;

	uint8_t temp;
	int16_t gyr_x;
	clock_setup();
	gpio_setup();
	usart_setup();
	spi_setup();

	gpio_clear(GPIOE, GPIO3);
	spi_send8(SPI1, GYR_CTRL_REG1);
	spi_read8(SPI1);
	spi_send8(SPI1, GYR_CTRL_REG1_PD | GYR_CTRL_REG1_XEN |
			GYR_CTRL_REG1_YEN | GYR_CTRL_REG1_ZEN |
			(3 << GYR_CTRL_REG1_BW_SHIFT));
	spi_read8(SPI1);
	gpio_set(GPIOE, GPIO3);

	gpio_clear(GPIOE, GPIO3);
	spi_send8(SPI1, GYR_CTRL_REG4);
	spi_read8(SPI1);
	spi_send8(SPI1, (1 << GYR_CTRL_REG4_FS_SHIFT));
	spi_read8(SPI1);
	gpio_set(GPIOE, GPIO3);

	clock_setup();
	console_setup(115200);
	sdram_init();
	lcd_spi_init();
	console_puts("LCD Initialized\n");
	console_puts("Should have a checker pattern, press any key to proceed\n");
	msleep(2000);
/*	(void) console_getc(1); */
	gfx_init(lcd_draw_pixel, 240, 320);
	gfx_fillScreen(LCD_GREY);
	gfx_fillRoundRect(10, 10, 220, 220, 5, LCD_WHITE);
	gfx_drawRoundRect(10, 10, 220, 220, 5, LCD_RED);
	gfx_fillCircle(20, 250, 10, LCD_RED);
	gfx_fillCircle(120, 250, 10, LCD_GREEN);
	gfx_fillCircle(220, 250, 10, LCD_BLUE);
	gfx_setTextSize(2);
	gfx_setCursor(15, 25);
	gfx_puts("STM32F4-DISCO");
	gfx_setTextSize(1);
	gfx_setCursor(15, 49);
	gfx_puts("Simple example to put some");
	gfx_setCursor(15, 60);
	gfx_puts("stuff on the LCD screen.");
	lcd_show_frame();
	console_puts("Now it has a bit of structured graphics.\n");
	console_puts("Press a key for some simple animation.\n");
	msleep(2000);
/*	(void) console_getc(1); */
	gfx_setTextColor(LCD_YELLOW, LCD_BLACK);
	gfx_setTextSize(3);
	// p1 = 0;
	// p2 = 45;
	// p3 = 90;
	while (1) {
		gfx_fillScreen(LCD_BLACK);
		gfx_setCursor(15, 150);
		gfx_puts("PLANETS!");


		// gfx_fillCircle(120, 160, 40, LCD_YELLOW);
		// gfx_drawCircle(120, 160, 55, LCD_GREY);
		// gfx_drawCircle(120, 160, 75, LCD_GREY);
		// gfx_drawCircle(120, 160, 100, LCD_GREY);

		// gfx_fillCircle(120 + (sin(d2r(p1)) * 55),
		// 	       160 + (cos(d2r(p1)) * 55), 5, LCD_RED);
		// gfx_fillCircle(120 + (sin(d2r(p2)) * 75),
		// 	       160 + (cos(d2r(p2)) * 75), 10, LCD_WHITE);
		// gfx_fillCircle(120 + (sin(d2r(p3)) * 100),
		// 	       160 + (cos(d2r(p3)) * 100), 8, LCD_BLUE);
		// p1 = (p1 + 3) % 360;
		// p2 = (p2 + 2) % 360;
		// p3 = (p3 + 1) % 360;
		lcd_show_frame();

		gpio_clear(GPIOE, GPIO3);
		spi_send8(SPI1, GYR_WHO_AM_I | GYR_RNW);
		spi_read8(SPI1);
		spi_send8(SPI1, 0);
		temp=spi_read8(SPI1);
		my_usart_print_int(USART2, (temp));
		gpio_set(GPIOE, GPIO3);

		gpio_clear(GPIOE, GPIO3);
		spi_send8(SPI1, GYR_STATUS_REG | GYR_RNW);
		spi_read8(SPI1);
		spi_send8(SPI1, 0);
		temp=spi_read8(SPI1);
		my_usart_print_int(USART2, (temp));
		gpio_set(GPIOE, GPIO3);

		gpio_clear(GPIOE, GPIO3);
		spi_send8(SPI1, GYR_OUT_TEMP | GYR_RNW);
		spi_read8(SPI1);
		spi_send8(SPI1, 0);
		temp=spi_read8(SPI1);
		my_usart_print_int(USART2, (temp));
		gpio_set(GPIOE, GPIO3);

		gpio_clear(GPIOE, GPIO3);
		spi_send8(SPI1, GYR_OUT_X_L | GYR_RNW);
		spi_read8(SPI1);
		spi_send8(SPI1, 0);
		gyr_x=spi_read8(SPI1);
		gpio_set(GPIOE, GPIO3);

		gpio_clear(GPIOE, GPIO3);
		spi_send8(SPI1, GYR_OUT_X_H | GYR_RNW);
		spi_read8(SPI1);
		spi_send8(SPI1, 0);
		gyr_x|=spi_read8(SPI1) << 8;
		my_usart_print_int(USART2, (gyr_x));
		gpio_set(GPIOE, GPIO3);

		int i;
		for (i = 0; i < 80000; i++)    /* Wait a bit. */
			__asm__("nop");



	}
}