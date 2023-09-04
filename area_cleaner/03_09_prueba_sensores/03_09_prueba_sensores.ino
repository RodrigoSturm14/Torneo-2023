#define SHARP1 25
//#define SHARP_1 35  // LEFT SIDE
//#define SHARP_2 32  // LEFT
//#define SHARP_3 26  // CNTER
//#define SHARP_4 27  // RIGHT
//#define SHARP_5 25  // RIGHT SIDE

float distancia1(int sample, int pin_sharp) {
  long suma = 0;
  for (int i = 0; i < sample; i++) {
    suma = analogRead(pin_sharp);
  }
  float adc = suma / sample;
  float distancia_cm = 17569.7 * pow(adc, -1.2062);
  return (distancia_cm);
}

void setup() {
  Serial.begin(115200);
  pinMode(SHARP1, INPUT);
}

void loop() {
  int D_cm = distancia1(20, SHARP1);
  Serial.print("distancia: ");
  Serial.println(D_cm);
  delay(100);
}