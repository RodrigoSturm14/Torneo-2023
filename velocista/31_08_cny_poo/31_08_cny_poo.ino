//--- Pines INPUT / OUTPUT ---
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

// Sensores --> CNY70
#define PIN_CNY_1 13
#define PIN_CNY_2 27
#define PIN_CNY_3 26
#define PIN_CNY_4 25
#define PIN_CNY_5 34
#define PIN_CNY_6 35
#define PIN_CNY_7 32
#define PIN_CNY_8 33

#define PIN_BOTON 16

//--- Cantidad sensores CNY70 ---
#define CNY_CANT 8

// --- Estados logicos ---
#define DEBUG 1
#define LEFT 1
#define RIGHT 2
#define BACKWARD 0
#define FORWARD 1
#define QUIETO 0

// --- Velocidades ---
#define VEL_MIN_PID 110
#define VEL_MIN 80
#define VEL_WHITE_FLOOR 100

// --- Calculo Proporcional ---
#define KP 10
int error, calc_p;
float gain = 120;
float error_prom_max = (KP * 15600);
float error_prom_min = (KP * -15600);

// --- Posicion lado negro ---
// bool black_side = RIGHT;
bool black_side = RIGHT;

int vel_der, vel_izq;

int pines_CNY70[CNY_CANT] = { PIN_CNY_1, PIN_CNY_2, PIN_CNY_3, PIN_CNY_4, PIN_CNY_5, PIN_CNY_6, PIN_CNY_7, PIN_CNY_8 };

// --- Calibracion de sensores ---
int valores_max_leidos[CNY_CANT];
int valores_min_leidos[CNY_CANT];
int valor;

// --- Lecturas de sensores ---
int valores_actuales[CNY_CANT];
bool all_white = false;

bool estado = true;

const int PWMFreq = 1000;
const int PWMResolution = 8;

// --------------------------- Funciones ---------------------------
bool boton_on_off() {
  return estado = digitalRead(PIN_BOTON);
}

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

Motor::Motor(int pin_a, int pin_b, int ch_a, int ch_b) {
  pin_1 = pin_a;
  pin_2 = pin_b;
  ch_1 = ch_a;
  ch_2 = ch_b;

  ledcSetup(ch_1, frecuencia, resolucion);
  ledcSetup(ch_2, frecuencia, resolucion);

  ledcAttachPin(pin_1 ,ch_1);
  ledcAttachPin(pin_2 ,ch_2);
}

void Motor::Forward(int vel) {
  ledcWrite(ch_1, vel);
  ledcWrite(ch_2, 0);
}

void Motor::Backward(int vel) {
  ledcWrite(ch_1, 0);
  ledcWrite(ch_2, vel);
}

void Motor::Stop() {
  ledcWrite(ch_1, 0);
  ledcWrite(ch_2, 0);
}

Motor *rightmotor = new Motor(M1A, M1B, CH_RIGHT_1, CH_RIGHT_2);
Motor *leftmotor = new Motor(M2A, M2B, CH_LEFT_1, CH_LEFT_2);
/*
void pins_and_PWM_setup() {
  // --- Pines Motores (RPWM & LPWM) ---
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);
  // --- CONFIG. PWM ---
  ledcSetup(LEFT_1, PWMFreq, PWMResolution);
  ledcSetup(LEFT_2, PWMFreq, PWMResolution);
  ledcSetup(RIGHT_1, PWMFreq, PWMResolution);
  ledcSetup(RIGHT_2, PWMFreq, PWMResolution);
  // --- Enlace canales PWM a pines motores ---
  ledcAttachPin(M1A, LEFT_1);
  ledcAttachPin(M1B, LEFT_2);
  ledcAttachPin(M2A, RIGHT_1);
  ledcAttachPin(M2B, RIGHT_2);
}

void motor(int motor, int direccion, int velocidad) {

  if (motor == 0) {

    if (direccion == 0) {  // ---> gira en un sentido
      ledcWrite(LEFT_1, velocidad);
      ledcWrite(LEFT_2, 0);

    } else {  // direccion == 1 ---> gira en el otro sentido
      ledcWrite(LEFT_1, 0);
      ledcWrite(LEFT_2, velocidad);
    }
  }

  else if (motor == 1) {

    if (direccion == 0) {  // ---> gira en un sentido
      ledcWrite(RIGHT_1, 0);
      ledcWrite(RIGHT_2, velocidad);

    } else {  // direccion == 1 ---> gira en el otro sentido
      ledcWrite(RIGHT_1, velocidad);
      ledcWrite(RIGHT_2, 0);
    }
  }
}
*/
void setup() {
  for (int i = 0; i < CNY_CANT; i++) {
    pinMode(pines_CNY70[i], INPUT);
  }

  pinMode(PIN_BOTON, INPUT);
  Serial.begin(115200);
  // pins_and_PWM_setup();

  for (int i = 0; i < CNY_CANT; i++) {
    valores_min_leidos[i] = 100000;
    valores_max_leidos[i] = -100000;
  }

  //calibrar lecturas minimas y maximas
  Serial.println("Calibrando.. en while");

  while (boton_on_off() == true) {

    for (int i = 0; i < CNY_CANT; i++) {

      valor = analogRead(pines_CNY70[i]);

      valores_min_leidos[i] = constrain(valores_min_leidos[i], -100000, valor);
      valores_max_leidos[i] = constrain(valores_max_leidos[i], valor, 100000);

      /*Serial.print("Valores mininimos CNY ");
        Serial.print(i);
        Serial.print(" :");
        Serial.println(valores_min_leidos[i]);
        Serial.println();
        delay(100);

        Serial.print("Valores maximos CNY ");
        Serial.print(i);
        Serial.print(" :");
        Serial.println(valores_max_leidos[i]);
        Serial.println();
        delay(100);*/
    }
    /*Serial.print("Valores mininimos CNY 1: ");
      Serial.println(valores_min_leidos[0]);
      Serial.print("Valores mininimos CNY 2: ");
      Serial.println(valores_min_leidos[1]);
      Serial.println();
      Serial.print("Valores maximos CNY 1: ");
      Serial.println(valores_max_leidos[0]);
      Serial.print("Valores maximos CNY 2: ");
      Serial.println(valores_max_leidos[1]);
      delay(1000);*/
  }
  for (int i = 0; i < CNY_CANT; i++) {
    Serial.print("Valores FINALES mininimos CNY ");
    Serial.print(i);
    Serial.print(" :");
    Serial.println(valores_min_leidos[i]);
    Serial.println();
    delay(100);

    Serial.print("Valores FINALES maximos CNY ");
    Serial.print(i);
    Serial.print(" :");
    Serial.println(valores_max_leidos[i]);
    Serial.println();
    delay(100);
  }

  /*Serial.print("Valores mininimos CNY 1 (FINALES): ");
    Serial.println(valores_min_leidos[0]);
    Serial.print("Valores mininimos CNY 2 (FINALES): ");
    Serial.println(valores_min_leidos[1]);

    Serial.print("Valores maximos CNY 1 (FINALES): ");
    Serial.println(valores_max_leidos[0]);
    Serial.print("Valores maximos CNY 2 (FINALES): ");
    Serial.println(valores_max_leidos[1]);*/

  delay(5000);
}

void loop() {

  all_white = false;
  // obtener lecturas de sensores y establecer logica para la cuenta de error

  for (int i = 0; i < CNY_CANT / 2; i++) { // 3-0 sensores izquierdos devuelven valores de 1000 a 0. lado negro izquierda: 0 (los sensores 3-0 izquierdos devuelven valores cercanos a 4095) ------- 3-0 sensores izquierdos devuelven valores de 1000 a 0. lado blanco izquierda: 1000 (los sensores 3-0 izquierdos devuelven valores cercanos a 90)
    valores_actuales[i] = analogRead(pines_CNY70[i]);
    valores_actuales[i] = map(valores_actuales[i], valores_min_leidos[i], valores_max_leidos[i], 1000, 0);
    valores_actuales[i] = constrain(valores_actuales[i], 0, 1000);
  }

  for (int i = CNY_CANT / 2; i < CNY_CANT; i++) { // 7-4 sensores derechos devuelven valores de 0 a 1000. lado blanco derecha: 1000 (los sensores 7-4 derechos devuelven valores cercanos a 0) ------- 7-4 sensores derechos devuelven valores de 0 a 1000. lado negro derecha: 4095 (los sensores 7-4 derechos devuelven valores cercanos a 4095)
    valores_actuales[i] = analogRead(pines_CNY70[i]);
    valores_actuales[i] = map(valores_actuales[i], valores_min_leidos[i], valores_max_leidos[i], 0, 1000);
    valores_actuales[i] = constrain(valores_actuales[i], 0, 1000);
  }

  error = (-16 * valores_actuales[0]) + (-8 * valores_actuales[1]) + (-4 * valores_actuales[2]) + (-2 * valores_actuales[3]) + (2 * valores_actuales[4]) + (4 * valores_actuales[5]) + (8 * valores_actuales[6]) + (16 * valores_actuales[7]);
  // error = (-16 * valores_actuales[7]) + (-8 * valores_actuales[6]) + (-4 * valores_actuales[5]) + (-2 * valores_actuales[4]) + (2 * valores_actuales[3]) + (4 * valores_actuales[2]) + (8 * valores_actuales[1]) + (16 * valores_actuales[0]);

  // 7-4 sensores derechos negativisados __ 3-0 sensores izquierdos positivos

  // POR QUE SE MULTIPLICA EN LA CUENTA DE ERROR?

  Serial.println();
  Serial.print("Error: ");
  Serial.println(error);

  calc_p = KP * error;
  Serial.println();
  Serial.print("Calculo proporcional (P): ");
  Serial.println(calc_p);

  calc_p = map(calc_p, error_prom_min, error_prom_max, -1 * gain, gain);
  Serial.println();
  Serial.print("Calculo proporcional (P): ");
  Serial.println(calc_p);
  Serial.println();

  all_white = error < -26000;

  if (error < error_prom_min) error_prom_min = error;
  if (error > error_prom_max) error_prom_max = error;

  //////////////////////////
  if (black_side == LEFT) {
    vel_der = VEL_MIN_PID - calc_p;
    vel_izq = VEL_MIN_PID + calc_p;
  } else {
    vel_der = VEL_MIN_PID + calc_p;
    vel_izq = VEL_MIN_PID - calc_p;
  }

  Serial.println(vel_izq);
  Serial.println(vel_der);
  //////////////////////////

  vel_der = constrain(vel_der, 0, 255);
  vel_izq = constrain(vel_izq, 0, 255);
  if(vel_der > 255) vel_der = 255;
  if(vel_izq > 255) vel_izq = 255;

  //////////////////////////
  if (all_white) {

    if (black_side == RIGHT) {
      // motor(LEFT, FORWARD, QUIETO);
      leftmotor->Forward(0);
      Serial.println("Motor izquierdo: Quieto - 0");

      // motor(RIGHT, FORWARD, VEL_WHITE_FLOOR);
      rightmotor->Forward(VEL_WHITE_FLOOR);
      Serial.print("Motor derecho: Adelante - ");
      Serial.println(VEL_WHITE_FLOOR);
    } else {
      // motor(RIGHT, FORWARD, QUIETO);
      rightmotor->Forward(0);
      Serial.println("Motor derecho: Quieto - 0");

      // motor(LEFT, FORWARD, VEL_WHITE_FLOOR);
      leftmotor->Forward(VEL_WHITE_FLOOR);
      Serial.print("Motor izquierdo: Adelante - ");
      Serial.println(VEL_WHITE_FLOOR);
    }
  } else {

    if (vel_der < VEL_MIN) {
      int x = VEL_MIN + (VEL_MIN - vel_der);
      Serial.print("Motor derecho: Atras - ");
      Serial.println(x);
      // motor(RIGHT, BACKWARD, x);
      rightmotor->Backward(x);
    } else {
      Serial.print("Motor derecho: Adelante - ");
      Serial.println(vel_der);
      // motor(RIGHT, FORWARD, vel_der);
      rightmotor->Forward(vel_der);
    }

    if (vel_izq < VEL_MIN) {
      int g = VEL_MIN + (VEL_MIN - vel_izq);
      Serial.print("Motor izquierdo: Atras - ");
      Serial.println(g);
      // motor(LEFT, BACKWARD, g);
      leftmotor->Backward(g);
    } else {
      Serial.print("Motor izquierdo: Adelante - ");
      Serial.println(vel_izq);
      // motor(LEFT, FORWARD, vel_izq);
      leftmotor->Forward(vel_izq);
    }
  }
  //////////////////////////

  delay(1000);
}
