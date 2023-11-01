# IE0624-Laboratorio-de-microcontroladores

# Instrucciones para Lab4

1. **Ubicación del archivo `src`:**  
   El archivo `src` se encuentra en la carpeta `Lab4`. Asegurese de que esté dentro de la siguiente ruta:
   ```
   ~/libopencm3/libopencm3-examples/examples/stm32/f4/stm32f429i-discovery
   ```

2. **Compilación:**  
   Ejecute el siguiente comando para compilar. Es importante tener en cuenta que la dirección del directorio `OPENCM3_DIR` puede variar según la ubicación de su archivo `libopencm3`:
   ```
   make OPENCM3_DIR=~/libopencm3
   ```

3. **Conversión a formato binario:**  
   Luego de la compilación, convierte el archivo `.elf` a formato `.bin` con:
   ```
   arm-none-eabi-objcopy -O binary lab4.elf lab4.bin
   ```

4. **Flasheo a la tarjeta:**  
   Finalmente, transfiere el archivo binario a tu dispositivo con el siguiente comando:
   ```
   st-flash --reset write lab4.bin 0x8000000
   ```
