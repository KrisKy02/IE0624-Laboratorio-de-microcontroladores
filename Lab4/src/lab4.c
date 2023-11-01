// SE DECALRAN LAS BIBLIOTECAS Y ARCHIVOS REQUERIDOS
// Bibliotecas estándar de C
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <errno.h>
#include <unistd.h>
// Librerías específicas del proyecto
#include "clock.h"
#include "console.h"
#include "sdram.h"
#include "lcd-spi.h"
#include "gfx.h"
#include "rcc.h"
#include "adc.h"
#include "dac.h"
#include "gpio.h"
#include "spi.h"
#include "usart.h"
// // Librerías del microcontrolador
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>



// Definiciones relacionadas con el giroscopio
#define GYR_RNW			(1 << 7) /* Permite escribir cuando es cero */
#define GYR_MNS			(1 << 6) /* Habilita lecturas múltiples cuando es 1 */
#define GYR_WHO_AM_I		0x0F    // Registro que identifica el dispositivo
#define GYR_OUT_TEMP		0x26    // Registro de temperatura de salida
#define GYR_STATUS_REG		0x27    // Registro de estado

// Definiciones para configurar el giroscopio
#define GYR_CTRL_REG1		0x20    // Registro de control 1
#define GYR_CTRL_REG1_PD	(1 << 3) // Modo de encendido
#define GYR_CTRL_REG1_XEN	(1 << 1) // Habilitar eje X
#define GYR_CTRL_REG1_YEN	(1 << 0) // Habilitar eje Y
#define GYR_CTRL_REG1_ZEN	(1 << 2) // Habilitar eje Z
#define GYR_CTRL_REG1_BW_SHIFT	4    // Cambio de ancho de banda
#define GYR_CTRL_REG4		0x23    // Registro de control 4
#define GYR_CTRL_REG4_FS_SHIFT	4    // Cambio de escala completa

// Direcciones de registros de datos de giroscopio
#define GYR_OUT_X_L		0x28
#define GYR_OUT_X_H		0x29
#define GYR_OUT_Y_L		0x2A
#define GYR_OUT_Y_H		0x2B
#define GYR_OUT_Z_L		0x2C
#define GYR_OUT_Z_H		0x2D

#define L3GD20_SENSITIVITY_250DPS  (0.00875F)  // Sensibilidad del giroscopio

// Estructura para almacenar las lecturas de los ejes X, Y, y Z del giroscopio
typedef struct Gyro {
  int16_t x;
  int16_t y;
  int16_t z;
} gyro;

void spi_transaction(uint16_t reg, uint16_t val);
int16_t read_axis(uint8_t lsb_command, uint8_t msb_command);
uint8_t spi_communication(uint8_t command);
void send_data(gyro lectura, float bateria_lvl);
void display_data(gyro lectura, float bateria_lvl, bool enviar);
void initialize_system(void);
void delay(void);
void handle_leds(float bateria_lvl, bool enviar);
int print_decimal(int num);
gyro read_xyz(void);     // Función para leer valores de los ejes X, Y y Z

// Función para realizar transacciones SPI con el giroscopio
void spi_transaction(uint16_t reg, uint16_t val) {
    gpio_clear(GPIOC, GPIO1);  // Bajar CS (Chip Select) para comenzar transacción
    spi_send(SPI5, reg);       // Enviar registro al giroscopio
    spi_read(SPI5);            // Leer respuesta del giroscopio
    spi_send(SPI5, val);       // Enviar valor al giroscopio
    spi_read(SPI5);            // Leer respuesta del giroscopio
    gpio_set(GPIOC, GPIO1);    // Subir CS para finalizar transacción
}


// Función para configurar el módulo SPI5 y GPIOs relacionados
static void spi_setup(void)
{
    rcc_periph_clock_enable(RCC_SPI5);    // Habilita el reloj para SPI5
    rcc_periph_clock_enable(RCC_GPIOC);   // Habilita el reloj para el puerto GPIOC
    rcc_periph_clock_enable(RCC_GPIOF);   // Habilita el reloj para el puerto GPIOF

    // Configura el pin GPIO1 de GPIOC como salida
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,GPIO1);
    gpio_set(GPIOC, GPIO1);

    // Configura los pines GPIO7, GPIO8 y GPIO9 de GPIOF para funciones alternas (probablemente SCK, MISO, MOSI de SPI)
    gpio_mode_setup(GPIOF, GPIO_MODE_AF, GPIO_PUPD_NONE,GPIO7 | GPIO8 | GPIO9);   
    gpio_set_af(GPIOF, GPIO_AF5, GPIO7 | GPIO8 | GPIO9);

    // Configuración de SPI5
    spi_set_master_mode(SPI5);   // Establece SPI5 en modo maestro
    spi_set_baudrate_prescaler(SPI5, SPI_CR1_BR_FPCLK_DIV_64);  // Configura la velocidad de baudios de SPI5
    spi_set_clock_polarity_0(SPI5);   // Configura la polaridad del reloj a 0
    spi_set_clock_phase_0(SPI5);      // Configura la fase del reloj a 0
    spi_set_full_duplex_mode(SPI5);   // Establece SPI5 en modo full duplex
    spi_set_unidirectional_mode(SPI5); // Establece SPI5 en modo unidireccional (pero con 3 cables)
    spi_enable_software_slave_management(SPI5); // Habilita la gestión de esclavo por software
    spi_send_msb_first(SPI5);   // Establece la transmisión de bits empezando por el más significativo
    spi_set_nss_high(SPI5);     // Establece el pin NSS (Chip Select) en alto
    SPI_I2SCFGR(SPI5) &= ~SPI_I2SCFGR_I2SMOD; // Configuración adicional para SPI5
    spi_enable(SPI5);           // Habilita SPI5

    // Transacciones SPI para configurar un dispositivo (probablemente un giroscopio)
    spi_transaction(GYR_CTRL_REG1, GYR_CTRL_REG1_PD | GYR_CTRL_REG1_XEN | GYR_CTRL_REG1_YEN | GYR_CTRL_REG1_ZEN | (3 << GYR_CTRL_REG1_BW_SHIFT));
    spi_transaction(GYR_CTRL_REG4, (1 << GYR_CTRL_REG4_FS_SHIFT));
}

// Función para configurar USART1 y el GPIO relacionado
static void usart_setup(void)
{
    // Configura el pin GPIO9 de GPIOA para transmitir en USART1
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);	
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9);
	
    // Configuración de USART1
    usart_set_baudrate(USART1, 115200);   // Establece la velocidad de baudios en 115200
    usart_set_databits(USART1, 8);        // Establece 8 bits de datos
    usart_set_stopbits(USART1, USART_STOPBITS_1); // Establece 1 bit de parada
    usart_set_mode(USART1, USART_MODE_TX);  // Establece USART1 en modo de transmisión
    usart_set_parity(USART1, USART_PARITY_NONE); // Sin paridad
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE); // Sin control de flujo

    // Habilita USART1
    usart_enable(USART1);
}


// Función para realizar una comunicación SPI y obtener una respuesta
uint8_t spi_communication(uint8_t command) {
    gpio_clear(GPIOC, GPIO1);          // Desactiva el pin GPIO1 de GPIOC (probablemente CS o Chip Select)
    spi_send(SPI5, command);           // Envia el comando por SPI5
    spi_read(SPI5);                    // Lee una respuesta desde SPI5, aunque esta respuesta no se usa
    spi_send(SPI5, 0);                 // Envía un byte en 0 por SPI5
    uint8_t result = spi_read(SPI5);   // Lee el resultado desde SPI5
    gpio_set(GPIOC, GPIO1);            // Activa el pin GPIO1 de GPIOC (termina la comunicación con el dispositivo)
    return result;                     // Devuelve el resultado leído
}

// Función para leer un eje del giroscopio, combinando el byte menos significativo (LSB) y el más significativo (MSB)
int16_t read_axis(uint8_t lsb_command, uint8_t msb_command) {
    int16_t result;
    result = spi_communication(lsb_command);                // Lee el byte menos significativo
    result |= spi_communication(msb_command) << 8;          // Lee el byte más significativo y lo combina con el LSB
    return result;                                          // Devuelve el valor combinado
}

// Función para leer los valores de los ejes X, Y, Z del giroscopio
gyro read_xyz(void) {
    gyro lectura;

    spi_communication(GYR_WHO_AM_I | 0x80);                 // Lee el registro WHO_AM_I
    spi_communication(GYR_STATUS_REG | GYR_RNW);            // Lee el registro STATUS_REG
    spi_communication(GYR_OUT_TEMP | GYR_RNW);              // Lee el registro OUT_TEMP

    // Lee y escala los valores de los ejes
    lectura.x = read_axis(GYR_OUT_X_L | GYR_RNW, GYR_OUT_X_H | GYR_RNW) * L3GD20_SENSITIVITY_250DPS;
    lectura.y = read_axis(GYR_OUT_Y_L | GYR_RNW, GYR_OUT_Y_H | GYR_RNW) * L3GD20_SENSITIVITY_250DPS;
    lectura.z = read_axis(GYR_OUT_Z_L | GYR_RNW, GYR_OUT_Z_H | GYR_RNW) * L3GD20_SENSITIVITY_250DPS;

    return lectura; // Devuelve la lectura de los 3 ejes
}

// Función para configurar algunos pines GPIO
static void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOG);                     // Habilita el reloj para el puerto GPIOG
    rcc_periph_clock_enable(RCC_GPIOA);                     // Habilita el reloj para el puerto GPIOA

    // Configura el pin GPIO0 de GPIOA como entrada
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);
    // Configura el pin GPIO13 de GPIOG como salida
    gpio_mode_setup(GPIOG, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);
    // Configura el pin GPIO14 de GPIOG como salida
    gpio_mode_setup(GPIOG, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14);
}

// Función para imprimir un número decimal en la consola
int print_decimal(int num)
{
    char buf[12]; // Buffer suficientemente grande para contener un entero de 32 bits con signo
    char *p = buf + sizeof(buf) - 1; // Apuntamos al final del buffer
    int len = 0;
    bool is_negative = false;

    if (num == 0) { // Caso especial para cero
        console_putc('0');
        return 1;
    }

    if (num < 0) {
        is_negative = true;
        num = -num; // Convierte a positivo
    }

    *p = '\0'; // Finaliza la cadena con un carácter nulo

    // Conversión de número a cadena
    while (num > 0) {
        p--; 
        *p = '0' + (num % 10);
        num /= 10;
        len++;
    }

    if (is_negative) {
        p--;
        *p = '-';
        len++;
    }

    console_puts(p); // Imprime toda la cadena
    return len;
}

// Configura el ADC
static void adc_setup(void)
{
    // Configura el pin PA3 (probablemente conectado a la batería) en modo análogo
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO3);
    adc_power_off(ADC1);                              // Apaga el ADC1
    adc_disable_scan_mode(ADC1);                      // Desactiva el modo de escaneo
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_3CYC);  // Establece el tiempo de muestreo para todos los canales
    adc_power_on(ADC1);                               // Enciende el ADC1
}

// Lee el valor del ADC del canal especificado
static uint16_t read_adc_naiive(uint8_t channel)
{
    uint8_t channel_array[16];
    channel_array[0] = channel;                       // Establece el canal a leer en el array
    adc_set_regular_sequence(ADC1, 1, channel_array); // Configura la secuencia de conversión regular del ADC
    adc_start_conversion_regular(ADC1);               // Inicia la conversión
    while (!adc_eoc(ADC1));                           // Espera hasta que la conversión haya finalizado
    uint16_t reg16 = adc_read_regular(ADC1);          // Lee el valor convertido
    return reg16;                                     // Devuelve el valor
}