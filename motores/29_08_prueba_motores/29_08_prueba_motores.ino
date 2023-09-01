// CHANNELS
#define CH_LEFT_1 12
#define CH_LEFT_2 11
#define CH_RIGHT_1 14
#define CH_RIGHT_2 13

// LEFT
#define M1A 19
#define M1B 21

// RIGHT
#define M2A 23
#define M2B 22

class Motor {
private:
  // atributos/variables usados por los metodos/funciones de la clase
  int pin_1;
  int pin_2;
  int ch_1;
  int ch_2;
  int frecuencia = 1000;
  int resolucion = 8;

public:
  // metodos/funciones
  Motor(int pin_a, int pin_b, int ch_a, int ch_b);
  void Forward(int vel);
  void Backward(int vel);
  void Stop();
};

// funcion constructor de ambos motores; funcion para setear y agregar los parametros (pines y chnls) de los motores en la creacion de objetos
Motor::Motor(int pin_a, int pin_b, int ch_a, int ch_b) {
  pin_1 = pin_a;
  pin_2 = pin_b;
  ch_1 = ch_a;
  ch_2 = ch_b;

  ledcSetup(ch_1, frecuencia, resolucion);
  ledcSetup(ch_2, frecuencia, resolucion);

  ledcAttachPin(pin_1, ch_1);
  ledcAttachPin(pin_2, ch_2);
}

void Motor::Forward(int vel) {
  ledcWrite(ch_2, vel);
  ledcWrite(ch_1, 0);
}

void Motor::Backward(int vel) {
  ledcWrite(ch_2, 0);
  ledcWrite(ch_1, vel);
}

void Motor::Stop() {
  ledcWrite(ch_1, 0);
  ledcWrite(ch_2, 0);
}

// crear objeto motor derecho e izquierdo con sus pines y chnls, con la funcion constructor
Motor *rightmotor = new Motor(M1A, M1B, CH_RIGHT_1, CH_RIGHT_2);
Motor *leftmotor = new Motor(M2A, M2B, CH_LEFT_1, CH_LEFT_2);

void setup() {
  Serial.begin(115200);
}

void loop() {

  rightmotor->Forward(100);
  leftmotor->Forward(100);
  delay(4000);

  rightmotor->Forward(150);
  leftmotor->Forward(150);
  delay(2000);

  rightmotor->Stop();
  leftmotor->Stop();
  delay(2000);
}
