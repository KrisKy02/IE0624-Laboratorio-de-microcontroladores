#include <avr/io.h>
#include <avr/interrupt.h>

/* VARIABLES GLOBALES */
int pressed = 0;

/* INTERRUPCIONES */
// Interrupción para el botón
ISR(INT0_vect){
  pressed = 1;
}

// Interrupción del temporizador
ISR (TIMER0_OVF_vect){

}
/* MAIN */
// Bucle principal
int main(void) {
  setup();  // Inicializar
  sei();    // Habilitar interrupciones

  while (1) {
  }
}

/* FUNCIONES */

// Función de configuración inicial
void setup(){
  // Configurar los pines PB0, PB1, PB2 y PB3 como salidas
  DDRB |= (1 << PB3)|(1 << PB2)|(1 << PB1)|(1 << PB0);

  // Inicializar los estados de los pines: PB3 en LOW, PB2 en HIGH, PB1 en LOW, PB0 en HIGH
  PORTB = (0<<PB3)|(1<<PB2)|(0<<PB1)|(1<<PB0);

  // Establecer el estado inicial de la máquina de estados como PASS_VEHICLE
  estado = PASS_VEHICLE;

  // Inicializar la variable que registra si se ha pulsado un botón
  pressed = 0;

  // Inicializar la variable de segundos
  seconds = 0;

  // Habilitar la interrupción externa en el pin INT0
  GIMSK |= (1<<INT0);

  // Configurar la interrupción externa INT0 para que se active en el flanco de subida
  MCUCR |= (1 << ISC00) | (1 << ISC01);

  // Establecer el modo del Timer0 (modo normal)
  TCCR0A = 0x00;

  // Establecer el preescalador del Timer0 a 1024 (dividir la frecuencia del reloj entre 1024)
  TCCR0B = (1 << CS00) | (1 << CS02);

  // Inicializar el valor del contador del Timer0
  TCNT0 = 0;

  // Habilitar la interrupción por desbordamiento del Timer0
  TIMSK |= (1 << TOIE0);
}
