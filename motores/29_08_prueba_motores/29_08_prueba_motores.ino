/*
#define LEFT 1
#define RIGHT 0

#define BACKWARD 0
#define FORWARD 1
*/

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

// const int FRECUENCIA = 1000; /* 1 KHz */
// const int RESOLUCION = 8;

class Motor {
private:
  int pin_1;
  int pin_2;
  int ch_1;
  int ch_2;
  int frecuencia = 1000;
  int resolucion = 8;

public:
  Motor(int pin_a, int pin_b, int ch_a, int ch_b);
  void Forward(int vel);
  void Backward(int vel);
  void Stop();
};

// constructor
Motor::Motor(int pin_a, int pin_b, int ch_a, int ch_b){
  pin_1 = pin_a;
  pin_2 = pin_b;
  ch_1 = ch_a;
  ch_2 = ch_b;

  ledcSetup(ch_1, frecuencia, resolucion);
  ledcSetup(ch_2, frecuencia, resolucion);

  ledcAttachPin(pin_1, ch_1);
  ledcAttachPin(pin_2, ch_2);

  pinMode(pin_1, OUTPUT);
  pinMode(pin_2, OUTPUT);
}

void Motor::Forward(int vel){
  ledcWrite(ch_1, vel);
  ledcWrite(ch_2, 0);
}

void Motor::Backward(int vel){
  ledcWrite(ch_1, 0);
  ledcWrite(ch_2, vel);
}

void Motor::Stop(){
  ledcWrite(ch_1, 0);
  ledcWrite(ch_2, 0);
}

Motor *rightmotor = new Motor(M1A, M1B, CH_RIGHT_1, CH_RIGHT_2);
Motor *leftmotor = new Motor(M2A, M2B, CH_LEFT_1, CH_LEFT_2);

/*
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
*/
void setup() {

  Serial.begin(115200);
  // pines_setup();
}

void loop() {

  /*
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
  */

  rightmotor->Forward(100);
  leftmotor->Forward(100);
  delay(5000);

  rightmotor->Stop();
  leftmotor->Stop();
  delay(2000);

  rightmotor->Backward(100);
  leftmotor->Backward(100);
  delay(5000);

  rightmotor->Stop();
  leftmotor->Stop();
  delay(2000);

  rightmotor->Forward(100);
  leftmotor->Backward(120);
  delay(5000);

  rightmotor->Stop();
  leftmotor->Stop();
  delay(2000);
}
