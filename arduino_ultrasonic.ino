#define TRIG1 4
#define ECHO1 5
#define TRIG2 6
#define ECHO2 7
#define TRIG3 8
#define ECHO3 9

void setup() {
    Serial.begin(9600);
    pinMode(TRIG1, OUTPUT);
    pinMode(ECHO1, INPUT);
    pinMode(TRIG2, OUTPUT);
    pinMode(ECHO2, INPUT);
    pinMode(TRIG3, OUTPUT);
    pinMode(ECHO3, INPUT);
}

long medirDistancia(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    return pulseIn(echoPin, HIGH) * 0.034 / 2; // Conversión a cm
}

void loop() {
    delay(10); // Cada 500 ms
    long distancia1 = medirDistancia(TRIG1, ECHO1);
    delay(10);
    long distancia2 = medirDistancia(TRIG2, ECHO2);
    delay(10);
    long distancia3 = medirDistancia(TRIG3, ECHO3);
    
    Serial.print("[");
    Serial.print(distancia1);
    Serial.print(",");
    Serial.print(distancia2);
    Serial.print(",");
    Serial.print(distancia3);
    Serial.println("]");
}
