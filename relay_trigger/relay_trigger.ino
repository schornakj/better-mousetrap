
int ledPin = 12;                 // LED connected to digital pin 13

void setup()
{
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);      // sets the digital pin as output
}

void loop()
{
  String input = Serial.readString();
  Serial.println(input);

  if (input.equals("go")) {
      digitalWrite(ledPin, HIGH);
      delay(1000);
      digitalWrite(ledPin, LOW);
      delay(1000);  
  }
}
