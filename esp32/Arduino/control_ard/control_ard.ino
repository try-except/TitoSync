// =========================
//   FULL-STEP HIGH TORQUE
// =========================

// -------------------------
// Pines del motor 1
// -------------------------
const uint8_t AIN1 = 32;
const uint8_t AIN2 = 30;
const uint8_t BIN1 = 28;
const uint8_t BIN2 = 26;

// -------------------------
// Pines del motor 2
// -------------------------
const uint8_t AIN12 = 33;
const uint8_t AIN22 = 31;
const uint8_t BIN12 = 29;
const uint8_t BIN22 = 27;

// -------------------------
// Pines del motor 3
// -------------------------
const uint8_t AIN13 = 45;
const uint8_t AIN23 = 43;
const uint8_t BIN13 = 41;
const uint8_t BIN23 = 39;

// -------------------------
// Pines de finales de carrera motor 3
// -------------------------
const uint8_t FC3_BOTTOM = 44;   // Llega abajo (activo LOW)
const uint8_t FC3_TOP    = 42;   // Llega arriba (activo LOW)

// -------------------------
// Finales de carrera motor 2
// -------------------------
const uint8_t FC2_BACK  = 40;   
const uint8_t FC2_FRONT = 38;   

// -------------------------
// Finales motor 1
// -------------------------
const uint8_t FC1_DER = 36;   
const uint8_t FC1_IZQ = 34;   

// -------------------------
// Pines del driver
// -------------------------
const uint8_t nSLEEP_1   = 22;
const uint8_t nSLEEP_2   = 23;
const uint8_t nSLEEP_3   = 35;
const uint8_t nFAULT_1 = 24;
const uint8_t nFAULT_2 = 25;
const uint8_t nFAULT_3 = 37;

// -------------------------
// Variables recibidas
// -------------------------
int dc = 0, S1 = 0, S2 = 0, S3 = 0;

int contador = 0, contador2 = 0, contador3 = 0;
bool comando_activo = false, comando2_activo = false, comando3_activo = false;
int sentido = 1, sentido2 = 1, sentido3 = 1;
int pasosObjetivo = 0, pasosObjetivo2 = 0, pasosObjetivo3 = 0;
int idx = 0, idx2 = 0, idx3 = 0;

bool rutinaMotor3 = false;

const int DEAD_TIME_US = 3;

// -----------------------------------------------
int valorDespuesDe(const String &msg, const String &tag) {
  String patron = tag + ",";
  int idx = msg.indexOf(patron);
  if (idx == -1) return -1;
  int inicio = idx + patron.length();
  int fin = msg.indexOf(',', inicio);
  if (fin == -1) fin = msg.length();
  return msg.substring(inicio, fin).toInt();
}

// -----------------------------------------------
void disableAll() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void disableAll2() {
  digitalWrite(AIN12, LOW);
  digitalWrite(AIN22, LOW);
  digitalWrite(BIN12, LOW);
  digitalWrite(BIN22, LOW);
}

void disableAll3() {
  digitalWrite(AIN13, LOW);
  digitalWrite(AIN23, LOW);
  digitalWrite(BIN13, LOW);
  digitalWrite(BIN23, LOW);
}

// -----------------------------------------------
bool leerMensajeControl(int &dc, int &S1, int &S2, int &S3) {
  if (!Serial.available()) return false;

  String linea = Serial.readStringUntil('\n');
  linea.trim();
  if (!linea.startsWith("DC,")) return false;

  dc = valorDespuesDe(linea, "DC");
  S1 = valorDespuesDe(linea, "S1");
  S2 = valorDespuesDe(linea, "S2");
  S3 = valorDespuesDe(linea, "S3");

  Serial.print("ECO,DC,"); Serial.print(dc);
  Serial.print(",S1,"); Serial.print(S1);
  Serial.print(",S2,"); Serial.print(S2);
  Serial.print(",S3,"); Serial.println(S3);

  return true;
}

// -----------------------------------------------
void applyStep(uint8_t s) {
  disableAll();
  delayMicroseconds(DEAD_TIME_US);
  switch (s) {
    case 0: digitalWrite(AIN1, HIGH); digitalWrite(BIN1, HIGH); break;
    case 1: digitalWrite(AIN2, HIGH); digitalWrite(BIN1, HIGH); break;
    case 2: digitalWrite(AIN2, HIGH); digitalWrite(BIN2, HIGH); break;
    case 3: digitalWrite(AIN1, HIGH); digitalWrite(BIN2, HIGH); break;
  }
}

void applyStep2(uint8_t s) {
  disableAll2();
  delayMicroseconds(DEAD_TIME_US);
  switch (s) {
    case 0: digitalWrite(AIN12, HIGH); digitalWrite(BIN12, HIGH); break;
    case 1: digitalWrite(AIN22, HIGH); digitalWrite(BIN12, HIGH); break;
    case 2: digitalWrite(AIN22, HIGH); digitalWrite(BIN22, HIGH); break;
    case 3: digitalWrite(AIN12, HIGH); digitalWrite(BIN22, HIGH); break;
  }
}

void applyStep3(uint8_t s) {
  disableAll3();
  delayMicroseconds(DEAD_TIME_US);
  switch (s) {
    case 0: digitalWrite(AIN13, HIGH); digitalWrite(BIN13, HIGH); break;
    case 1: digitalWrite(AIN23, HIGH); digitalWrite(BIN13, HIGH); break;
    case 2: digitalWrite(AIN23, HIGH); digitalWrite(BIN23, HIGH); break;
    case 3: digitalWrite(AIN13, HIGH); digitalWrite(BIN23, HIGH); break;
  }
}

// -----------------------------------------------
void stepMotor(int dir) {
  idx += dir;
  if (idx < 0) idx = 3;
  if (idx > 3) idx = 0;
  applyStep(idx);
}

void stepMotor2(int dir) {
  idx2 += dir;
  if (idx2 < 0) idx2 = 3;
  if (idx2 > 3) idx2 = 0;
  applyStep2(idx2);
}

void stepMotor3(int dir) {
  idx3 += dir;
  if (idx3 < 0) idx3 = 3;
  if (idx3 > 3) idx3 = 0;
  applyStep3(idx3);
}

// -----------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  pinMode(AIN12, OUTPUT); pinMode(AIN22, OUTPUT);
  pinMode(BIN12, OUTPUT); pinMode(BIN22, OUTPUT);

  pinMode(AIN13, OUTPUT); pinMode(AIN23, OUTPUT);
  pinMode(BIN13, OUTPUT); pinMode(BIN23, OUTPUT);

  pinMode(FC3_BOTTOM, INPUT_PULLUP);
  pinMode(FC3_TOP, INPUT_PULLUP);

  pinMode(FC2_BACK, INPUT_PULLUP);
  pinMode(FC2_FRONT, INPUT_PULLUP);

  pinMode(FC1_DER, INPUT_PULLUP);
  pinMode(FC1_IZQ, INPUT_PULLUP);

  pinMode(nSLEEP_1, OUTPUT);
  pinMode(nSLEEP_2, OUTPUT);
  pinMode(nSLEEP_3, OUTPUT);
  pinMode(nFAULT_1, INPUT);
  pinMode(nFAULT_2, INPUT);
  pinMode(nFAULT_3, INPUT);

  digitalWrite(nSLEEP_1, HIGH);
  digitalWrite(nSLEEP_2, HIGH);
  digitalWrite(nSLEEP_3, HIGH);

  Serial.println("Sistema listo.");
}

// -----------------------------------------------
void loop() {

  // Fallas drivers
  if (digitalRead(nFAULT_1) == LOW) { disableAll();  Serial.println("Falla M1"); return; }
  if (digitalRead(nFAULT_2) == LOW) { disableAll2(); Serial.println("Falla M2"); return; }
  if (digitalRead(nFAULT_3) == LOW) { disableAll3(); Serial.println("Falla M3"); return; }

  // Leer comandos
  if (leerMensajeControl(dc, S1, S2, S3)) {

    if (S1 != 0) {
      sentido = (S1 > 0) ? 1 : -1;
      pasosObjetivo = abs(S1);
      contador = 0;
      comando_activo = true;
    }

    if (S2 != 0) {
      sentido2 = (S2 > 0) ? 1 : -1;
      pasosObjetivo2 = abs(S2);
      contador2 = 0;
      comando2_activo = true;
    }

    if (S3 == 1) {
      rutinaMotor3 = true;
    }
    else if (S3 != 0) {
      sentido3 = (S3 > 0) ? 1 : -1;
      pasosObjetivo3 = abs(S3);
      contador3 = 0;
      comando3_activo = true;
    }
  }

  // -------------------------
  // Motor 1 NORMAL
  // -------------------------
  if (comando_activo) {

    // Detención por FC según sentido
    if (sentido > 0 && digitalRead(FC1_IZQ) == LOW) {
      Serial.println("M1 parado por FC1_IZQ");
      comando_activo = false;
      disableAll();
    }
    else if (sentido < 0 && digitalRead(FC1_DER) == LOW) {
      Serial.println("M1 parado por FC1_DER");
      comando_activo = false;
      disableAll();
    }
    else if (contador < pasosObjetivo) {
      stepMotor(sentido);
      contador++;
      delayMicroseconds(7000);
    }
    else {
      comando_activo = false;
      disableAll();
    }
  }

  // -------------------------
  // Motor 2 NORMAL
  // -------------------------
  if (comando2_activo) {

    if (sentido2 > 0) {
      if (digitalRead(FC2_FRONT) == LOW) {
        Serial.println("M2 parado por FC2_FRONT");
        comando2_activo = false;
        disableAll2();
      }
    }
    else {
      if (digitalRead(FC2_BACK) == LOW) {
        Serial.println("M2 parado por FC2_BACK");
        comando2_activo = false;
        disableAll2();
      }
    }

    if (comando2_activo && contador2 < pasosObjetivo2) {
      stepMotor2(sentido2);
      contador2++;
      delayMicroseconds(7000);
    }
    else if (contador2 >= pasosObjetivo2) {
      comando2_activo = false;
      disableAll2();
    }
  }

  // -------------------------
  // RUTINA ESPECIAL MOTOR 2 y 3
  // -------------------------
  if (rutinaMotor3) {

    // BAJAR
    for (int i = 0; i < 30000; i++) {
      if (digitalRead(FC3_BOTTOM) == LOW) break;
      stepMotor3(1);
      delayMicroseconds(2000);
    }
   disableAll3();
   delay(300);

    // SUBIR
    for (int i = 0; i < 30000; i++) {
      if (digitalRead(FC3_TOP) == LOW) break;
      stepMotor3(-1);
      delayMicroseconds(2000);
    }

   disableAll3();
   delay(300);


    // MOVER MOTOR 2 HACIA ATRÁS
    for (int i = 0; i < 2000; i++) {
      if (digitalRead(FC2_BACK) == LOW) {
        Serial.println("M2 detenido por FC2_BACK durante rutina");
        break;
      }
      stepMotor2(-1);
      delayMicroseconds(7000);
    }

    disableAll2();
    disableAll3();
    rutinaMotor3 = false;
    return;
  }

  // -------------------------
  // Motor 3 NORMAL
  // -------------------------
    if (sentido3 > 0) {
      if (digitalRead(FC3_BOTTOM) == LOW) {
        Serial.println("M3 parado por FC3_BOTTOM");
        comando3_activo = false;
        disableAll3();
      }
    }
    else {
      if (digitalRead(FC3_TOP) == LOW) {
        Serial.println("M3 parado por FC3_TOP");
        comando3_activo = false;
        disableAll3();
      }
    }

    if (comando3_activo && contador3 < pasosObjetivo3) {
      stepMotor3(sentido3);
      contador3++;
      delayMicroseconds(2000);
    }
    else if (contador3 >= pasosObjetivo3) {
      comando3_activo = false;
      disableAll3();
    }
  disableAll();
  disableAll2();
  disableAll3();

}


