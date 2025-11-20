// =========================
//   FULL-STEP HIGH TORQUE
// =========================

// Pines del motor
const uint8_t AIN1   = 9;
const uint8_t AIN2   = 3;
const uint8_t BIN1   = 5;
const uint8_t BIN2   = 6;
const uint8_t nSLEEP  = 12;
const uint8_t nFAULT  = 11;

// Variables recibidas del serial
int dc = 0, S1 = 0, S2 = 0, S3 = 0;
int contador = 0;
bool comando_activo = false;
int sentido = 1;
int pasosObjetivo = 0;

// Parámetros de velocidad
int frecuencia = 120;                       // Hz → AJUSTAR PARA TORQUE
int periodo   = 1000000 / frecuencia;      // μs por paso

// ------------------------
// Funciones para parsing
// ------------------------
int valorDespuesDe(const String &msg, const String &tag) {
  String patron = tag + ",";
  int idx = msg.indexOf(patron);
  if (idx == -1) return -1;

  int inicio = idx + patron.length();
  int fin = msg.indexOf(',', inicio);
  if (fin == -1) fin = msg.length();

  return msg.substring(inicio, fin).toInt();
}


// Dead-time para evitar cortos (datasheet)
const int DEAD_TIME_US = 3;

void disableAll() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

bool leerMensajeControl(int &dc, int &S1, int &S2, int &S3, int &cont) {
  if (!Serial.available()) return false;

  String linea = Serial.readStringUntil('\n');
  linea.trim();

  if (!linea.startsWith("DC,")) return false;

  cont = 0;
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

int idx = 0;

void stepMotor(int dir) {
  idx += dir;
  if (idx < 0) idx = 3;
  if (idx > 3) idx = 0;
  applyStep(idx);
}

// -----------------------------------------
// FULL-STEP HIGH-TORQUE: energiza 2 bobinas
// -----------------------------------------

void applyStep(uint8_t s) {
  disableAll();                 // evitar cortos
  delayMicroseconds(DEAD_TIME_US);

  switch (s) {
    case 0: // A+ B+
      digitalWrite(AIN1, HIGH);
      digitalWrite(BIN1, HIGH);
      break;

    case 1: // A- B+
      digitalWrite(AIN2, HIGH);
      digitalWrite(BIN1, HIGH);
      break;

    case 2: // A- B-
      digitalWrite(AIN2, HIGH);
      digitalWrite(BIN2, HIGH);
      break;

    case 3: // A+ B-
      digitalWrite(AIN1, HIGH);
      digitalWrite(BIN2, HIGH);
      break;
  }
}


void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(nSLEEP, OUTPUT);
  pinMode(nFAULT, INPUT);

  digitalWrite(nSLEEP, HIGH);

  Serial.println("Listo. FULL STEP activo.");
}

void loop() {
  int falla = digitalRead(nFAULT);

  bool recibido = leerMensajeControl(dc, S1, S2, S3, contador);

  if (recibido) {
    if (S1 > 0) {
      sentido = 1;
      pasosObjetivo = S1;
      comando_activo = true;
    }
    else if (S1 < 0) {
      sentido = -1;
      pasosObjetivo = -S1;
      comando_activo = true;
    }
    else {
      comando_activo = false;
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW);
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, LOW);
    }

  }

  if (falla == LOW) {
    comando_activo = false;
    digitalWrite(nSLEEP, LOW);
    Serial.println("FALLA detectada en nFAULT");
    delay(100);
    return;
  }

  if (!comando_activo) { 
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW);
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, LOW);
      return;}
  if (contador >= pasosObjetivo) {
    comando_activo = false;
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    return;
  }

  stepMotor(sentido);
  contador ++;
  delayMicroseconds(1000) //eje z
  // delayMicroseconds(7000); //eje hacia adelante 
}
