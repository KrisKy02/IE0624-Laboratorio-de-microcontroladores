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
