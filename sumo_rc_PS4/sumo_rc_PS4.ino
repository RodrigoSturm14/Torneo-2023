#include <PS4Controller.h>

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

int velocity, turn_velocity;

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

void sumo_rc_loop() {
  int y_axis_value = PS4.LStickY();
  int x_axis_value = PS4.RStickX();
  int l2 = PS4.L2Value();
  int r2 = PS4.R2Value();

  if (r2 > 50) {
    velocity = 250;
    turn_velocity = 150;
  }

  if (l2 > 50) {
    velocity = 100;
    turn_velocity = 50;
  }

  if (r2 < 50 && l2 < 50) {
    velocity = 170;
    turn_velocity = 70;
  }

  if (y_axis_value >= 50) {

    leftmotor->Forward(velocity);
    rightmotor->Forward(velocity);

    if (x_axis_value >= 50) {
      leftmotor->Forward(velocity);
      rightmotor->Forward(turn_velocity);
    }
    if (x_axis_value <= -50) {
      leftmotor->Forward(turn_velocity);
      rightmotor->Forward(velocity);
    }
  } else if (y_axis_value <= -50) {

    leftmotor->Backward(velocity);
    rightmotor->Backward(velocity);

    if (x_axis_value >= 50) {
      leftmotor->Backward(velocity);
      rightmotor->Backward(turn_velocity);
    }
    if (x_axis_value <= -50) {
      leftmotor->Backward(turn_velocity);
      rightmotor->Backward(velocity);
    }
  }

  else if (x_axis_value >= 50) {
    leftmotor->Forward(velocity);
    rightmotor->Forward(turn_velocity);
  }

  else if (x_axis_value <= -50) {
    leftmotor->Forward(turn_velocity);
    rightmotor->Forward(velocity);
  }

  else {
    leftmotor->Stop();
    rightmotor->Stop();
  }
}

void setup() {
  PS4.begin();
  while (!PS4.isConnected())
    ;
  PS4.attach(sumo_rc_loop);
}

void loop() {
  // put your main code here, to run repeatedly:
}
