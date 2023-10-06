#include <PCD8544.h>
#include <math.h>
//Constantes con valores de los pines
static const byte Volt_1 = 0;
static const byte Volt_2 = 1;
static const byte Volt_3 = 2;
static const byte Volt_4 = 3;
static const byte ledPin = 13;
static const byte SerialPin = 12;
static const byte ACDCPin = 4;

static PCD8544 lcd;
//Variables de Habilitación

int Habilitar_Datos;
int Habilitar_Datos_ACDC;
//Resistencias para conversión
float r1 = 1000000;
float r2 = 263150;

void setup() {
  lcd.begin();
  Serial.begin(9600); //Se da inicio a la comunicación con el puerto serial
  //Se define la dirección de los pines digitales
  pinMode(ledPin, OUTPUT);
  pinMode(SerialPin, INPUT);

}

void loop() {

  //lcd.clear(); //Se limpia la pantalla
  Habilitar_Datos_ACDC  = analogRead(ACDCPin);

  if(Habilitar_Datos_ACDC == LOW) //Al estar en bajo se verán las tensiones DC y en alto las AC rms
  {
    //Tensiones en DC

    //Voltimetro #1
    float voltimetro_1 = (analogRead(Volt_1) * 5) / 1023.0; 
    float r_volt_1 = (voltimetro_1 / (r2 / (r1 + r2))); //Se realiza la conversión de vuelta para obtener el valor original
    //Mostra los valores en la pantalla:
    lcd.setCursor(0, 0); //Se va al inicio de la pantalla
    lcd.print(" V en DC:");
    lcd.setCursor(0, 1); //Se mueven unicamente las filas
    lcd.print(" V1: ");
    lcd.print(r_volt_1);
    lcd.print(" V ");
    
    //Voltimetro #2
    float voltimetro_2 = (analogRead(Volt_2) * 5) / 1023.0;
    float r_volt_2 = (voltimetro_2 / (r2 / (r1 + r2)));
    //Mostra los valores en la pantalla:
    lcd.setCursor(0, 2);
    lcd.print(" V2: ");
    lcd.print(r_volt_2);
    lcd.print(" V ");
    
    //Voltimetro #3
    float voltimetro_3 = (analogRead(Volt_3) * 5) / 1023.0;
    float r_volt_3 = (voltimetro_3 / (r2 / (r1 + r2)));
    //Muestra los valores en la pantalla:
    lcd.setCursor(0, 3);
    lcd.print(" V3: ");
    lcd.print(r_volt_3);
    lcd.print(" V ");

    //Voltimetro #4
    float voltimetro_4 = (analogRead(Volt_4) * 5) / 1023.0;
    float r_volt_4 = -(voltimetro_4 / (r2 / (r1 + r2)));
    //Mostra los valores en la pantalla:
    lcd.setCursor(0, 4);
    lcd.print(" V4: ");
    lcd.print(r_volt_4);
    lcd.print(" V ");
    
    delay(1000); 

    Habilitar_Datos = digitalRead(SerialPin); 
    
    if(Habilitar_Datos == HIGH) //Permite pasar datos mediante USART
    {
      // Mostrar los valores en el puerto serial de salida:
      Serial.print(" V en DC: "); //se muestran los valores en el monitor serial y en el csv
      Serial.println();
      Serial.print(" V1: ");
      Serial.print(r_volt_1);
      Serial.print(" V ");
      Serial.println();

      Serial.print(" V2: ");
      Serial.print(r_volt_2);
      Serial.print(" V ");
      Serial.println();

      Serial.print(" V3: ");
      Serial.print(r_volt_3);
      Serial.print(" V ");
      Serial.println();

      Serial.print(" V4: ");
      Serial.print(r_volt_4);
      Serial.print(" V ");
      Serial.println();
    }
    else
    {
      ;
    } 
    //Se encienden los bits si las tensiones son menores a -20 V o mayores a 20
    if (((r_volt_1> 20) | (r_volt_2> 20) | (r_volt_3> 20) | (r_volt_4 > 20)) )  //no creo que funcione pero pa probar
    {
      digitalWrite(ledPin, HIGH);
    } 
    else if ((r_volt_1< -20 | r_volt_2  < -20| r_volt_3 < -20| r_volt_4< -20)){
      digitalWrite(ledPin, HIGH);
    }
    else 
    {
      digitalWrite(ledPin, LOW);
    }
  }
  else
  {
    //Tensiones en AC

    //Voltimetro #1
    float voltimetro_1 = (analogRead(Volt_1) * 5) / 1023.0;
    float r_volt_1 = (voltimetro_1 / (r2 / (r1 + r2)));
    r_volt_1 = r_volt_1/sqrt(2); //se obtiene el valor rms
    //Mostra los valores en la pantalla:
    lcd.setCursor(0, 0);
    lcd.print(" Vrms en AC: ");
    lcd.setCursor(0, 1);
    lcd.print(" V1: ");
    lcd.print(r_volt_1);
    lcd.print(" V ");
    
    //Voltimetro #2
    float voltimetro_2 = (analogRead(Volt_2) * 5) / 1023.0;
    float r_volt_2 = (voltimetro_2 / (r2 / (r1 + r2)));
    r_volt_2 = r_volt_2/sqrt(2);
    //Mostra los valores en la pantalla:
    lcd.setCursor(0, 2);
    lcd.print(" V2: ");
    lcd.print(r_volt_2);
    lcd.print(" V");
    
    //Voltimetro #3
    float voltimetro_3 = (analogRead(Volt_3) * 5) / 1023.0;
    float r_volt_3 = (voltimetro_3 / (r2 / (r1 + r2)));
    r_volt_3 = r_volt_3/sqrt(2);
    //Muestra los valores en la pantalla:
    lcd.setCursor(0, 3);
    lcd.print(" V3: ");
    lcd.print(r_volt_3);
    lcd.print(" V ");

    //Voltimetro #4
    float voltimetro_4 = (analogRead(Volt_4) * 5) / 1023.0;
    float r_volt_4 = -(voltimetro_4 / (r2 / (r1 + r2)));
    r_volt_4 = r_volt_4/sqrt(2);
    //Mostra los valores en la pantalla:
    lcd.setCursor(0, 4);
    lcd.print(" V4: ");
    lcd.print(r_volt_4);
    lcd.print(" V ");
    
    delay(1000);

    Habilitar_Datos = digitalRead(SerialPin);
    
    if(Habilitar_Datos == HIGH)
    {
      // Mostrar los valores en el puerto serial de salida:
      Serial.print(" Vrms en AC: ");
      Serial.println();
      Serial.print(" V1: ");
      Serial.print(r_volt_1);
      Serial.print(" V ");
      Serial.println();

      Serial.print(" V2: ");
      Serial.print(r_volt_2);
      Serial.print(" V ");
      Serial.println();

      Serial.print(" V3: ");
      Serial.print(r_volt_3);
      Serial.print(" V ");
      Serial.println();

      Serial.print(" V4: ");
      Serial.print(r_volt_4);
      Serial.print(" V ");
      Serial.println();
    }
    else
    {
      ;
    }
    if (((r_volt_1> 20) | (r_volt_2> 20) | (r_volt_3> 20) | (r_volt_4 > 20)) ) 
    {
      digitalWrite(ledPin, HIGH);
    } 
    else if ((r_volt_1< -20 | r_volt_2  < -20| r_volt_3 < -20| r_volt_4< -20)){
      digitalWrite(ledPin, HIGH);
    }
    else 
    {
      digitalWrite(ledPin, LOW);
    }
  }
  
}