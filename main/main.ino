#define DEBUG 1

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

#define PUSH_1 16
#define PUSH_2 17

// Sensores
#define SENSOR_2 27
#define SENSOR_3 26
#define SENSOR_1 13
#define SENSOR_4 25
#define SENSOR_5 34
#define SENSOR_6 35
#define SENSOR_7 32
#define SENSOR_8 33

#define CNY70_CANT 8

int Cny70_pins[] = { SENSOR_1, SENSOR_2, SENSOR_3, SENSOR_4, SENSOR_5, SENSOR_6, SENSOR_7, SENSOR_8 };

// Constantes

#define WHITE 0
#define BLACK 1
#define NO 0
#define YES 1
#define LEFT 1
#define RIGHT 2

#define MODES 4

int black_side = BLACK;

class Button {
private:
  int pin;

public:
  Button(int pin_a);
  int IsPressed();
  void WaitPressing();
};

Button::Button(int pin_a) {
  pin = pin_a;
}

int Button::IsPressed() {
  return digitalRead(pin);
}

void Button::WaitPressing() {
  while (digitalRead(pin) == YES)
    continue;
}

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

#define LIMIT_LEN 8

class CNY {
private:
  int map_max = 1000;
  int map_min = 0;
  int cant_pins;
  int value;
  int values_read[LIMIT_LEN];
  int max_value_read[LIMIT_LEN];
  int min_value_read[LIMIT_LEN];
  int Cny70_pins[LIMIT_LEN];

public:
  CNY(int Cny70_pins_a[], int cant_pins_a);
  void InitilizeReadings();
  void Calibrate();
  int ReadFloor();
};

CNY::CNY(int Cny70_pins_a[LIMIT_LEN], int cant_pins_a) {
  cant_pins = cant_pins_a;

  for (int i = 0; i < cant_pins; i++) {
    Cny70_pins[i] = Cny70_pins_a[i];
  }
}

void CNY::InitilizeReadings() {
  for (int i = 0; i < cant_pins; i++) {
    max_value_read[i] = -10000;
    min_value_read[i] = 10000;
  }
}

void CNY::Calibrate() {
  for (int i = 0; i < cant_pins; i++) {
    value = analogRead(Cny70_pins[i]);
    if (value < max_value_read[i]) max_value_read[i] = value;
    else if (value > min_value_read[i]) min_value_read[i] = value;
  }
}

int CNY::ReadFloor() {
  for (int i = 0; i < cant_pins / 2; i++) {
    values_read[i] = analogRead(Cny70_pins[i]);
    values_read[i] = map(values_read[i], min_value_read[i], max_value_read[i], map_min, map_max);
    values_read[i] = constrain(values_read[i], map_min, map_max);
  }

  for (int i = cant_pins / 2; i < cant_pins; i++) {
    values_read[i] = analogRead(Cny70_pins[i]);
    values_read[i] = map(values_read[i], min_value_read[i], max_value_read[i], map_max, map_min);
    values_read[i] = constrain(values_read[i], map_min, map_max);
  }

  return ((-16 * values_read[0]) + (-8 * values_read[1]) + (-4 * values_read[2]) + (-2 * values_read[3]) + (2 * values_read[4]) + (4 * values_read[5]) + (8 * values_read[6]) + (16 * values_read[7]));
}

// crear objeto motor derecho e izquierdo con sus pines y chnls, con la funcion constructor
Motor *rightmotor = new Motor(M1A, M1B, CH_RIGHT_1, CH_RIGHT_2);
Motor *leftmotor = new Motor(M2A, M2B, CH_LEFT_1, CH_LEFT_2);
Button *count_button = new Button(PUSH_1);
Button *confirm_button = new Button(PUSH_2);
int line_folower_array[] = { SENSOR_1, SENSOR_2, SENSOR_3, SENSOR_4, SENSOR_5, SENSOR_6, SENSOR_7, SENSOR_8 };
CNY *line_folower = new CNY(line_folower_array, 8);

void initialize(void (*mode_loop_function)()) {

  if (DEBUG) Serial.println("Esperando confirmacion");
  confirm_button->WaitPressing();
  if (DEBUG) Serial.println("Confirmado.. iniciando..");

  while (true) mode_loop_function();
}

///////////////////////////////       VELOCISTA       //////////////////////////////////////////////////

#define VEL_WHITE_FLOOR 100
#define VEL_MIN 80
#define PID_VEL_MIN 110

bool all_white = false;

int right_vel = 0;
int left_vel = 0;


void Motor_control(int value_a) {
  if (black_side == LEFT) {  // se evalua q velocidad debe tener ambos motores segun el calculo pid obtenido, para corregir el error
    right_vel = PID_VEL_MIN - value_a;
    left_vel = PID_VEL_MIN + value_a;
  } else {
    right_vel = PID_VEL_MIN + value_a;
    left_vel = PID_VEL_MIN - value_a;
  }

  right_vel = constrain(right_vel, 0, 255);  // por mas de q el pid_calc de un valor negativo, la velocidad va a pasar a ser 0
  left_vel = constrain(left_vel, 0, 255);
  if (right_vel > 255) right_vel = 255;
  if (left_vel > 255) left_vel = 255;

  if (all_white) {
    if (black_side == RIGHT) {
      leftmotor->Forward(VEL_WHITE_FLOOR);
      rightmotor->Forward(0);
      if (DEBUG) {
        Serial.print("Motor IZQUIERDO : ADELANTE - ");
        Serial.print(VEL_WHITE_FLOOR);
        Serial.print("Motor DERECHO : QUIETO");
      }
    } else {
      leftmotor->Forward(0);
      rightmotor->Forward(VEL_WHITE_FLOOR);
      if (DEBUG) {
        Serial.print("Motor IZQUIERDO : QUIETO");
        Serial.print("Motor DERECHO : ADELANTE - ");
        Serial.print(VEL_WHITE_FLOOR);
      }
    }
  } else {
    if (right_vel < VEL_MIN) {
      rightmotor->Backward(VEL_MIN + (VEL_MIN - right_vel));
      if (DEBUG) {
        Serial.print("Motor DERECHO : ATRAS - ");
        Serial.println(VEL_MIN + (VEL_MIN - right_vel));
      }
    } else {
      rightmotor->Forward(right_vel);
      if (DEBUG) {
        Serial.print("Motor DERECHO : ADELANTE - ");
        Serial.println(right_vel);
      }
    }

    if (left_vel < VEL_MIN) {
      leftmotor->Backward(VEL_MIN + (VEL_MIN - left_vel));
      if (DEBUG) {
        Serial.print("Motor IZQUIERDO : ATRAS - ");
        Serial.println(VEL_MIN + (VEL_MIN - left_vel));
      }
    } else {
      leftmotor->Forward(left_vel);
      if (DEBUG) {
        Serial.print("Motor IZQUIERDO : ADELANTE - ");
        Serial.println(left_vel);
      }
    }
  }
}
void line_folower_setup() {

  if (DEBUG) Serial.println("Velocista setup: ");
  if (DEBUG) Serial.println("  calibrando...  ");

  line_folower->InitilizeReadings();

  delay(2000);

  while (count_button->IsPressed() == YES) {
    line_folower->Calibrate();
  }
  if (DEBUG) Serial.println("Calibradoo");

  initialize(line_folower_loop);
}

float kp = 10;
float ki = 0;
float kd = 0;
int error = 0;
int last_error = 0;
int actual_value = 0;
int integral = 0;
int PID_calc;
float gain = 120;
float pid_max_average = ((kp * 15600) + (ki * 0) + (kd * (15600 - -15600)));
float pid_min_average = ((kp * 15600) + (ki * 0) + (kd * (-15600 - 15600)));

void line_folower_loop() {
  actual_value = line_folower->ReadFloor();
  error = actual_value;
  integral += error;
  constrain(integral, -100000, 100000);

  all_white = actual_value < -26000;

  PID_calc = (kp * error + ki * integral + kd * (error - last_error));
  PID_calc = map(PID_calc, pid_min_average, pid_max_average, -1 * gain, gain);
  if (DEBUG) {
    Serial.print("PID_calc ");
    Serial.println(PID_calc);
  }

  last_error = error;

  if (actual_value < pid_min_average) pid_min_average = actual_value;
  if (actual_value > pid_max_average) pid_max_average = actual_value;

  Motor_control(PID_calc);

  if(DEBUG) delay(1000);
}
////////////////////////////////////////////////////////////////////////////////////

void mode_selection() {

  int counter = 0;

  while (confirm_button->IsPressed() == YES) {
    if (count_button->IsPressed() == NO) {
      counter++;
      if (counter > MODES) counter = 1;
      delay(250);
      if(DEBUG) Serial.println(counter);
    }
  }
  switch (counter) {
    case 1:
      black_side = LEFT;
      Serial.println("Velocista; lado negro izquierda");
      line_folower_setup();
      break;

    case 2:
      black_side = RIGHT;
      Serial.println("Velocista; lado negro derecho");
      line_folower_setup();
      break;

    case 3:
      Serial.println("Despejar area");
      // area_cleaner_setup();
      break;

    case 4:
      Serial.println("Sumo RC");
      // sumo_rc_setup();
      break;
  }
}


void setup() {
  Serial.begin(115200);
  if (DEBUG) Serial.println("Seleccion modos ");
  mode_selection();
}

void loop() {
}
