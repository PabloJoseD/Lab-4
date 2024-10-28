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

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "clock.h"
#include "console.h"
#include "sdram.h"
#include "lcd-spi.h"
#include "gfx.h"

 
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>

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


// Registros del giroscopio, tabla 17 hoja de datos
#define GYR_OUT_X_L		0x28
#define GYR_OUT_X_H		0x29
#define GYR_OUT_Y_L		0x2A
#define GYR_OUT_Y_H		0x2B
#define GYR_OUT_Z_L		0x2C
#define GYR_OUT_Z_H		0x2D


// Sensibilidad de la pantalla
#define L3GD20_SENSITIVITY_250DPS  (0.00875F)     



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
	rcc_periph_clock_enable(RCC_GPIOE | RCC_GPIOG);
	// GPIO pantalla
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
		GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 |
		GPIO14 | GPIO15);
	// GPIO botones
	gpio_mode_setup(GPIOG, GPIO_MODE_OUTPUT,
			GPIO_PUPD_NONE, GPIO13 | GPIO14);
}

static void button_setup(void)
{
	/* Enable GPIOA clock. */
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Set GPIO0 (in GPIO port A) to 'input open-drain'. */
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);
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


static void spi_setup(void)
{
	rcc_periph_clock_enable(RCC_SPI5);
	/* For spi signal pins */
	rcc_periph_clock_enable(RCC_GPIOC);
	/* For spi mode select on the l3gd20 */
	rcc_periph_clock_enable(RCC_GPIOF);

	/* Setup GPIOE3 pin for spi mode l3gd20 select. */
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);
	/* Start with spi communication disabled */
	gpio_set(GPIOC, GPIO1);

	/* Setup GPIO pins for AF5 for SPI1 signals. */
	gpio_mode_setup(GPIOF, GPIO_MODE_AF, GPIO_PUPD_NONE,
			GPIO7 | GPIO8 | GPIO9);
	gpio_set_af(GPIOF, GPIO_AF5, GPIO7 | GPIO8 | GPIO9);

	// Inicializacion del SPI;
	spi_set_master_mode(SPI5);
	spi_set_baudrate_prescaler(SPI5, SPI_CR1_BR_FPCLK_DIV_64);
	spi_set_clock_polarity_0(SPI5);
	spi_set_clock_phase_0(SPI5);
	spi_set_full_duplex_mode(SPI5);
	spi_set_unidirectional_mode(SPI5); /* bidirectional but in 3-wire */
	spi_enable_software_slave_management(SPI5);
	spi_send_msb_first(SPI5);
	spi_set_nss_high(SPI5);

	SPI_I2SCFGR(SPI5) &= ~SPI_I2SCFGR_I2SMOD;
	spi_enable(SPI5);
	
    gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_CTRL_REG1); 
	spi_read(SPI5);
	spi_send(SPI5, GYR_CTRL_REG1_PD | GYR_CTRL_REG1_XEN |
			GYR_CTRL_REG1_YEN | GYR_CTRL_REG1_ZEN |
			(3 << GYR_CTRL_REG1_BW_SHIFT));
	spi_read(SPI5);
	gpio_set(GPIOC, GPIO1); 
    gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_CTRL_REG4);
	spi_read(SPI5);
	spi_send(SPI5, (1 << GYR_CTRL_REG4_FS_SHIFT));
	spi_read(SPI5);
	gpio_set(GPIOC, GPIO1);
	
}


static void coordenadas(void){
	char eje_x[20];
	char eje_y[20];
	char eje_z[20];

	uint8_t temp;

	int16_t gyr_x;
	int16_t gyr_y;
	int16_t gyr_z;


	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_WHO_AM_I | 0x80);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	spi_read(SPI5);
	//temp=spi_read(SPI5);
	//my_usart_print_int(USART2, (temp));
	gpio_set(GPIOC, GPIO1);

	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_STATUS_REG | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	spi_read(SPI5);
	//temp=spi_read(SPI5);
	//my_usart_print_int(USART2, (temp));
	gpio_set(GPIOC, GPIO1);

	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_OUT_TEMP | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	spi_read(SPI5);
	//temp=spi_read(SPI5);
	//my_usart_print_int(USART2, (temp));
	gpio_set(GPIOC, GPIO1);


	// EJE X

	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_OUT_X_L | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	gyr_x=spi_read(SPI5);
	gpio_set(GPIOC, GPIO1);

	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_OUT_X_H | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	gyr_x|=spi_read(SPI5) << 8;
	//my_usart_print_int(USART2, (gyr_x));
	gpio_set(GPIOC, GPIO1);


	// EJE Y

	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_OUT_Y_L | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	gyr_y=spi_read(SPI5);
	gpio_set(GPIOC, GPIO1);

	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_OUT_Y_H | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	gyr_y|=spi_read(SPI5) << 8;
	//my_usart_print_int(USART2, (gyr_x));
	gpio_set(GPIOC, GPIO1);


	// EJE Z

	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_OUT_Z_L | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	gyr_z=spi_read(SPI5);
	gpio_set(GPIOC, GPIO1);

	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_OUT_Z_H | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	gyr_z|=spi_read(SPI5) << 8;
	//my_usart_print_int(USART2, (gyr_x));
	gpio_set(GPIOC, GPIO1);


	gyr_x = gyr_x*L3GD20_SENSITIVITY_250DPS;
	gyr_y = gyr_y*L3GD20_SENSITIVITY_250DPS;
	gyr_z = gyr_z*L3GD20_SENSITIVITY_250DPS;

	sprintf(eje_x, "%d", gyr_x);
	sprintf(eje_y, "%d", gyr_y);
	sprintf(eje_z, "%d", gyr_z);

	gfx_setCursor(15, 90);
	gfx_puts("x: ");
	gfx_puts(eje_x);

	gfx_setCursor(15, 135);
	gfx_puts("y: ");
	gfx_puts(eje_y);

	gfx_setCursor(15, 185);
	gfx_puts("z: ");
	gfx_puts(eje_z);

}



int main(void)
{

	clock_setup();
	button_setup();
	gpio_setup();
	usart_setup();
	spi_setup();
	console_setup(115200);
	sdram_init();
	lcd_spi_init();
 	gfx_init(lcd_draw_pixel, 240, 320);
	gfx_setTextColor(LCD_BLACK, LCD_WHITE);
	gfx_setTextSize(2);

	int estado_led = 0;
	int estado_ultimo_boton = 0;
	
	while (1) {

		int estado_boton = gpio_get(GPIOA, GPIO0);
		int i;
		gfx_fillScreen(LCD_WHITE);
		gfx_setCursor(30, 40);
		gfx_puts("Sismografo");
		coordenadas();

		//Detectar flanco de subida con antirrebote
        if (estado_boton && !estado_ultimo_boton) {
			if (gpio_get(GPIOA, GPIO0)) {  // Verificar nuevamente el estado del botón
			estado_led = !estado_led;  // Cambiar el estado del LED

				if (estado_led) {
					gpio_set(GPIOG, GPIO13 | GPIO14);
				} 
				else {
					gpio_clear(GPIOG, GPIO13 | GPIO14);
				} 
		    }
		}
		
		estado_ultimo_boton = estado_boton;  // Actualizar el último estado del botón
		lcd_show_frame();

		
		for (i = 0; i < 800000; i++)    /* Wait a bit. */
			__asm__("nop");


	}
}
