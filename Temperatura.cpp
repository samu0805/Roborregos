const int sensorPin = 34;  // Pin ADC en el ESP32

void setup() {
  // Iniciar la comunicación serial para enviar datos al ordenador
  Serial.begin(115200);  // Velocidad mayor para ESP32
  Serial.println("Thermometer initialized");
}

void loop() {
  // Leer el valor analógico del sensor en el pin asignado
  int sensorValue = analogRead(sensorPin);

  // Mapear el valor leído a un rango de temperatura (por ejemplo, 0 a 300 grados Celsius)
  int temperature = map(sensorValue, 200, 1023, 0, 300);

  // Mostrar el valor crudo del sensor y la temperatura calculada
  Serial.print("Sensor Value: ");
  Serial.println(sensorValue);  // Valor crudo leído por el ADC
  Serial.print("Temperature (Deg C): ");
  Serial.println(temperature);  // Temperatura calculada

  // Breve retraso para evitar saturación en la lectura serial
  delay(500);  // 500 ms de espera para obtener lecturas cada medio segundo (ajustable)
}
