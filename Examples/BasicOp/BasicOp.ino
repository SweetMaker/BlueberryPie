
#define MPU6050_5V_PIN (17)
#define MPU6050_GND_PIN (16)

void setup()
{
	Serial.begin(115200);
	Serial.println("Hello World");

	pinMode(MPU6050_GND_PIN, OUTPUT);
	digitalWrite(MPU6050_GND_PIN, LOW);
	pinMode(MPU6050_5V_PIN, OUTPUT);
	digitalWrite(MPU6050_5V_PIN, HIGH);
}


void loop()
{
	delay(1000);
	Serial.println("Hello Again");
}