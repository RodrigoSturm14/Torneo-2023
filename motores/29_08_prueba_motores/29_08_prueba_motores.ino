#define LEFT 1
#define RIGHT 0

#define BACKWARD 0
#define FORWARD 1

// CHANNELS
#define CH_LEFT_1 12
#define CH_LEFT_2 11
#define CH_RIGHT_1 14
#define CH_RIGHT_2 13

// LEFT
#define M1A 19
#define M1B 21

// RIGHT
#define M2A 22
#define M2B 23

const int frequencia = 1000; /* 1 KHz */
const int resolucion = 8;

void motor(int motor, int direccion, int velocidad) {

  if (motor == LEFT) {
    if (direccion == 1) {
      ledcWrite(CH_LEFT_1, velocidad);
      ledcWrite(CH_LEFT_2, 0);
      Serial.println("motor izquierdo adelante");
    } else {
      ledcWrite(CH_LEFT_1, 0);
      ledcWrite(CH_LEFT_2, velocidad);
      Serial.println("motor izquierdo atras");
    }

  }

  else if (motor == RIGHT) {
    if (direccion == 1) {
      ledcWrite(CH_RIGHT_1, velocidad);
      ledcWrite(CH_RIGHT_2, 0);
      Serial.println("motor derecho adelante");
    } else {
      ledcWrite(CH_RIGHT_1, 0);
      ledcWrite(CH_RIGHT_2, velocidad);
      Serial.println("motor derecho atras");
    }
  }
}

void pines_setup() {
  ledcSetup(CH_LEFT_1, frequencia, resolucion);
  ledcSetup(CH_LEFT_2, frequencia, resolucion);
  ledcSetup(CH_RIGHT_1, frequencia, resolucion);
  ledcSetup(CH_RIGHT_2, frequencia, resolucion);

  ledcAttachPin(M1A, CH_LEFT_1);
  ledcAttachPin(M1B, CH_LEFT_2);
  ledcAttachPin(M2A, CH_RIGHT_1);
  ledcAttachPin(M2B, CH_RIGHT_2);

  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);
}

void setup() {

  Serial.begin(115200);
  pines_setup();
}

void loop() {

  motor(LEFT, FORWARD, 100);
  motor(RIGHT, FORWARD, 100);
  delay(5000);

  motor(LEFT, FORWARD, 0);
  motor(RIGHT, FORWARD, 0);
  delay(2000);

  motor(LEFT, BACKWARD, 100);
  motor(RIGHT, BACKWARD, 100);
  delay(5000);

  motor(LEFT, FORWARD, 0);
  motor(RIGHT, FORWARD, 0);
  delay(2000);

  motor(LEFT, BACKWARD, 100);
  motor(RIGHT, FORWARD, 100);
  delay(5000);

  motor(LEFT, FORWARD, 0);
  motor(RIGHT, FORWARD, 0);
  delay(2000);
}
