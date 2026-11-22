#include <Servo.h>

// ===== 핀 설정 =====
const int PIN_A = 9;    // 그리퍼1 (서보1)
const int PIN_B = 10;   // 그리퍼2 (서보2)

// ===== 안전 설정 (그리퍼 가동 범위 0~90도, 양 끝에 5도 여유) =====
const int SAFE_MIN = 5;         // 이 값보다 작은 각도 명령은 5로 제한
const int SAFE_MAX = 85;        // 이 값보다 큰 각도 명령은 85로 제한
const int STEP_DELAY_MS = 15;   // 1도 이동당 대기시간 (클수록 천천히, 전류 피크 감소)
const int TEST_SWING = 10;      // t 명령: 현재 각도 기준 +-도 왕복

Servo servoA;
Servo servoB;
int curA = -1;   // -1 = 아직 신호를 내보내지 않음
int curB = -1;

int clampAngle(int a) {
  return constrain(a, SAFE_MIN, SAFE_MAX);
}

// 첫 명령: 목표값을 먼저 쓴 뒤 attach 해서 시작 시 90도로 튀는 동작을 방지
void startServo(Servo &s, int pin, int &cur, int target) {
  s.write(target);
  s.attach(pin);
  cur = target;
}

// 서보 하나를 1도씩 천천히 이동
void moveOne(Servo &s, int pin, int &cur, int target) {
  target = clampAngle(target);
  if (cur < 0) {
    startServo(s, pin, cur, target);
    return;
  }
  while (cur != target) {
    cur += (target > cur) ? 1 : -1;
    s.write(cur);
    delay(STEP_DELAY_MS);
  }
}

// 두 서보를 동시에 1도씩 이동 (전원 한계 확인용)
void moveBoth(int ta, int tb) {
  ta = clampAngle(ta);
  tb = clampAngle(tb);
  if (curA < 0) startServo(servoA, PIN_A, curA, ta);
  if (curB < 0) startServo(servoB, PIN_B, curB, tb);
  while (curA != ta || curB != tb) {
    if (curA != ta) {
      curA += (ta > curA) ? 1 : -1;
      servoA.write(curA);
    }
    if (curB != tb) {
      curB += (tb > curB) ? 1 : -1;
      servoB.write(curB);
    }
    delay(STEP_DELAY_MS);
  }
}

bool parseAngle(String s, int &out) {
  s.trim();
  if (s.length() == 0 || s.length() > 3) return false;
  for (unsigned int i = 0; i < s.length(); i++) {
    if (!isDigit(s.charAt(i))) return false;
  }
  out = s.toInt();
  return true;
}

void printHelp() {
  Serial.println(F("--- commands (end with newline) ---"));
  Serial.println(F("a<angle>   move gripper A (D9),  e.g. a90"));
  Serial.println(F("b<angle>   move gripper B (D10), e.g. b90"));
  Serial.println(F("d <A> <B>  move both at once,    e.g. d 60 120"));
  Serial.println(F("t          self-test (+-10 deg around current; set a and b first)"));
  Serial.println(F("range 5..85 (clamped). start near the middle, e.g. a45"));
  Serial.println(F("x          stop signals (release both servos)"));
  Serial.println(F("?          status      h  help"));
}

void printStatus() {
  Serial.print(F("STATUS A="));
  Serial.print(curA);
  Serial.print(F(" B="));
  Serial.println(curB);
}

void runSelfTest() {
  if (curA < 0 || curB < 0) {
    Serial.println(F("ERR set both servos first (a<angle>, b<angle>)"));
    return;
  }
  int a0 = curA;
  int b0 = curB;

  Serial.println(F("TEST A"));
  moveOne(servoA, PIN_A, curA, a0 + TEST_SWING);
  moveOne(servoA, PIN_A, curA, a0 - TEST_SWING);
  moveOne(servoA, PIN_A, curA, a0);

  Serial.println(F("TEST B"));
  moveOne(servoB, PIN_B, curB, b0 + TEST_SWING);
  moveOne(servoB, PIN_B, curB, b0 - TEST_SWING);
  moveOne(servoB, PIN_B, curB, b0);

  Serial.println(F("TEST A+B (simultaneous)"));
  moveBoth(a0 + TEST_SWING, b0 + TEST_SWING);
  moveBoth(a0 - TEST_SWING, b0 - TEST_SWING);
  moveBoth(a0, b0);

  Serial.println(F("DONE"));
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  char cmd = tolower(line.charAt(0));
  String arg = line.substring(1);
  int v;

  if (cmd == 'a' || cmd == 'b') {
    if (!parseAngle(arg, v)) {
      Serial.println(F("ERR angle must be a number, e.g. a90"));
      return;
    }
    int target = clampAngle(v);
    if (target != v) Serial.println(F("NOTE clamped to SAFE range"));
    if (cmd == 'a') {
      moveOne(servoA, PIN_A, curA, target);
      Serial.print(F("OK A "));
      Serial.println(curA);
    } else {
      moveOne(servoB, PIN_B, curB, target);
      Serial.print(F("OK B "));
      Serial.println(curB);
    }
  } else if (cmd == 'd') {
    int ta, tb;
    if (sscanf(arg.c_str(), "%d %d", &ta, &tb) != 2) {
      Serial.println(F("ERR usage: d <A> <B>"));
      return;
    }
    moveBoth(ta, tb);
    Serial.print(F("OK D "));
    Serial.print(curA);
    Serial.print(' ');
    Serial.println(curB);
  } else if (cmd == 't') {
    runSelfTest();
  } else if (cmd == 'x') {
    servoA.detach();
    servoB.detach();
    curA = -1;
    curB = -1;
    Serial.println(F("OK DETACHED"));
  } else if (cmd == '?') {
    printStatus();
  } else {
    printHelp();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  Serial.println(F("READY"));
  printHelp();
}

void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    handleLine(line);
  }
}
