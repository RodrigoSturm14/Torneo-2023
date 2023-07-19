#include "BluetoothSerial.h"

#define DEBUG 1

BluetoothSerial BT;

bool i = false;
int modo;
String incoming;

void bt_reconnect() {
  while (!BT.connected()) {
    if (DEBUG) {
      Serial.println("Conectando blutu..");
      delay(1500);
    }
    if (BT.connected()) {
      if (DEBUG) Serial.println("Bluetooth conectado");
      delay(200);
      BT.println("ESP32 conectado, selecciona el modo");
    }
  }
}

void mode_selection() {
  if (BT.connected()) {
    while (i == false) {
      if (BT.available()) {
        incoming = BT.readStringUntil('\n');
        modo = incoming.toInt();
        if (modo > 4 || modo == 0) {
          delay(200);
          BT.println("Ese modo no existe, intenta de nuevo");
        } else {
          delay(200);
          BT.print("Modo seleccionado: ");
          BT.println(modo);
          BT.println();
          BT.println("Configuracion completada");
          i = true;
          delay(500);
        }
      }
    }

    BT.end();
    delay(500);

    if (DEBUG) {
      Serial.println("Bluetooth desconectado");
      Serial.print("Modo seleccionado: ");
      Serial.println(modo);
    }
  }

  switch (modo) {

    case 1:
      while (true) {
        if (DEBUG) Serial.println("1. Velocista negro izquierda");
        delay(1000);
      }

      break;

    case 2:
      while (true) {
        if (DEBUG) Serial.println("2. Velocista negro derecha");
        delay(1000);
      }

      break;

    case 3:
      while (true) {
        if (DEBUG) Serial.println("3. Despejar area");
        delay(1000);
      }

      break;

    case 4:
      while (true) {
        if (DEBUG) Serial.println("4. Sumo RC");
        delay(1000);
      }

      break;
  }
}

void setup() {
  Serial.begin(115200);
  BT.begin("123456");
  if (DEBUG) Serial.println("Bluetooth iniciado");

  bt_reconnect();
  mode_selection();
}

void loop() {
}