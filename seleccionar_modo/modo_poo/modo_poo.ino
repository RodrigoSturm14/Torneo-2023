#define DEBUG 1

#define PUSH_1 16
#define PUSH_2 17

#define NO 0
#define YES 1
#define LEFT 1
#define RIGHT 2

#define MODES 4

int counter = 0;
int black_side;

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
  while (digitalRead(pin) == NO)
    continue;
}

Button *count_button = new Button(PUSH_1);
Button *confirm_button = new Button(PUSH_2);

void mode_selection() {
  if (DEBUG) Serial.printf("Seleccion modos: ... en while");
  while (confirm_button->IsPressed() == YES) {
    if (count_button->IsPressed() == NO) {
      counter++;
      if (counter > MODES) counter = 1;
      if (DEBUG) Serial.println(counter);
      delay(200);
    }
  }
  switch (counter) {
    case 1:
      black_side = LEFT;
      if (DEBUG) Serial.println("Velocista; lado negro izquierda");
      break;

    case 2:
      black_side = RIGHT;
      if (DEBUG) Serial.println("Velocista; lado negro derecha");
      break;

    case 3:
      if (DEBUG) Serial.println("Despejar area");
      break;

    case 4:
      if (DEBUG) Serial.println("Sumo RC");
      break;

    default:
      if (DEBUG) Serial.println("Something went wrong :]");
      break;
  }
}

void setup() {
  Serial.begin(115200);
  mode_selection();
}

void loop() {
}
