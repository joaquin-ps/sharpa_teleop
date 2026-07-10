// Multi-motor vibration driver.
//
// Serial protocol (always 2 bytes per motor update):
//   <motor><level>   e.g. "05" -> motor 0 at PWM level 5
//   <motor>x         e.g. "1x" -> motor 1 off
//
// Python clients broadcast by sending one 2-byte command per motor.

// --- Motor config ---
const int NUM_MOTORS = 2;
const int motorPins[] = {9, 10};

// Three motors: set NUM_MOTORS = 3 and uncomment the third pin.
// const int NUM_MOTORS = 3;
// const int motorPins[] = {9, 10, 11};

const int minPWM = 66;
const int maxPWM = 178;  // ~3.5V
int pwmLevels[10];

int pendingMotor = -1;

void setMotorLevel(int motor, int level) {
  if (motor < 0 || motor >= NUM_MOTORS || level < 0 || level > 9) {
    return;
  }
  analogWrite(motorPins[motor], pwmLevels[level]);
}

void stopMotor(int motor) {
  if (motor < 0 || motor >= NUM_MOTORS) {
    return;
  }
  analogWrite(motorPins[motor], 0);
}

void handleChar(char c) {
  if (pendingMotor < 0) {
    if (c >= '0' && c < '0' + NUM_MOTORS) {
      pendingMotor = c - '0';
    }
    return;
  }

  if (c >= '0' && c <= '9') {
    setMotorLevel(pendingMotor, c - '0');
  } else if (c == 'x') {
    stopMotor(pendingMotor);
  }
  pendingMotor = -1;
}

void setup() {
  for (int m = 0; m < NUM_MOTORS; m++) {
    pinMode(motorPins[m], OUTPUT);
    analogWrite(motorPins[m], 0);
  }
  Serial.begin(115200);

  for (int i = 0; i < 10; i++) {
    pwmLevels[i] = minPWM + ((maxPWM - minPWM) * i) / 9;
  }
}

void loop() {
  while (Serial.available() > 0) {
    handleChar(Serial.read());
  }
}
