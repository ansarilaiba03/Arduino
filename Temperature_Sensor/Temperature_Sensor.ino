#include <DHT.h>
#define DHTPIN 2 // DHT11 data pin connected to Digital Pin 2 #define DHTTYPE DHT11 // Sensor type
DHT dht (DHTPIN, DHTTYPE) ;
void setup) {
Serial.begin (9600);
dht. begin();
}
void loop() {
float temperature = dht. readTemperature();
float temperature = dht.readTemperature(true);
float humidity = dht.readHumidity();
// Check if reading failed
if (isnan (temperatureC) || isnan(humidity)) {
Serial println("Failed to read from DHT sensor!"); return;
}
Serial.print ("Temperature: ");
Serial.print(temperatureC);
Serial print(" °C |");
Serial-print(temperatureF);
Serial print(" oF | ");
Serial. print ("Humidity: ");
Serial.print(humidity);
Serial.println(" %");
delay(2000); // Read every 2 seconds
}