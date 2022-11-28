const int left_bumper_pin = A1;
const int right_bumper_pin = A2;

int left_value = 0; 
int right_value = 0;

void setup()
{
	Serial.begin(9600);
}

void loop()
{
	left_value = analogRead(left_bumper_pin);
	Serial.print("Left bumper value: ");
	Serial.println(left_value);
	
	right_value = analogRead(right_bumper_pin);
	Serial.print("Right bumper value: ");
	Serial.println(right_value);
	delay(200);
}