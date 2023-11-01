import time
import json
import serial
import paho.mqtt.client as mqtt

class IOTClient:

    def __init__(self, broker, port, topic, token, serial_port, baudrate):
        self.broker = broker
        self.port = port
        self.topic = topic
        self.token = token
        self.serial_port = serial_port
        self.baudrate = baudrate

        self.client = mqtt.Client("SensorClient")
        self.data = self.setup_serial()
        self.setup_mqtt()

    def setup_serial(self):
        try:
            data = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            print("¡Conexión serial establecida con éxito!")
            return data
        except serial.SerialException as e:
            print("Error de conexión serial:", str(e))
            return None

    def setup_mqtt(self):
        self.client.on_connect = self.on_connect
        self.client.username_pw_set(self.token)
        self.client.connect(self.broker, self.port)

    def on_connect(self, client, userdata, flags, rc):
        status = "¡Conexion con el servidor establecida!" if rc == 0 else f"Ups, hubo un problema al conectar. Codigo: {rc}"
        print(status)

    def read_sensor_data(self):
        if not self.data:
            return {"Mensaje": "No hay conexion serial."}
        raw_data = self.data.readline().decode('utf-8').rstrip()

        cleaned_data = raw_data.replace("\r", "").replace("\n", "").split('\t')
        if len(cleaned_data) == 4:
            return {
                "Eje X": cleaned_data[0],
                "Eje Y": cleaned_data[1],
                "Eje Z": cleaned_data[2],
                "Nivel de Bateria": cleaned_data[3],
                "Alerta Bateria": "Baja" if float(cleaned_data[3]) < 7 else "Normal"
            }
        return {"Mensaje": "No se estan recibiendo datos."}

    def run(self):
        self.client.loop_start()
        while True:
            sensor_data = self.read_sensor_data()
            if sensor_data:
                message = json.dumps(sensor_data)
                print(f"Datos del sensor: {message}")
                self.client.publish(self.topic, message, qos=1)
                time.sleep(0.8)

if __name__ == "__main__":
    client = IOTClient(
        broker="iot.eie.ucr.ac.cr",
        port=1883,
        topic="v1/devices/me/telemetry",
        token="rfbmp4g2hrexkmf845b0",
        serial_port="/dev/ttyACM0",
        baudrate=115200
    )
    client.run()
