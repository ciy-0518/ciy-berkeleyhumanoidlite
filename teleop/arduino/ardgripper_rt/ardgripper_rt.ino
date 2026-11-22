// ardgripper_rt — 그리퍼 서보 2개 실시간 제어 (REPORT_51, 작업 지시 14-g 명령 대기열, 14-h 워밍업은 w 명령으로,
// 14-i 한 번에 한 서보 + 도착 뒤 기다림). 기존 검증용 ardgrippercode 는 그대로 둔다.
//
// 직렬 115200, 명령은 줄바꿈으로 끝난다:
//   g <A각도> <B각도>   목표 지정 (0~90 으로 자름). 0 = 열림, 90 = 닫힘 (양쪽 같음). 대기열에 넣는다 (아래)
//   w [A각도 B각도]     워밍업 (열림 → 닫힘 → 열림) 을 대기열에 넣는다 (14-h). 같은 속도, 앞 명령이 끝난 뒤
//                       14-n: 닫힘은 붙인 각도까지 (재생기가 config/gripper.json 값을 붙인다). 없으면 ANGLE_WARM_CLOSE (60)
//   x                   서보 신호 끊기 (detach) + 대기열 비우기. 다음 g 에서 마지막 각도를 먼저 쓰고 다시 붙인다
//   ?                   상태 출력:  A <지금> <목표> <붙음 0/1> B <지금> <목표> <붙음 0/1> Q <대기열> W <워밍업 0/1> D <버린 수>
//                                   M <움직이는 서보 0 없음/1 A/2 B> S <도착 뒤 기다리는 중 0/1>   (M·S 는 14-i 에서 뒤에 붙임)
//                       (앞 일곱 칸은 예전과 같다. 지금 = 서보에 마지막으로 쓴 각도. 실제 위치를 재는 것은 아니다.
//                        목표 = 지금 그 서보에 준 목표 — 한 명령에서 차례가 안 온 서보는 아직 앞 목표다)
// A = 9번 핀 = 왼쪽, B = 10번 핀 = 오른쪽.
//
// 켜지면 (14-h): 두 서보를 열림 (0°) 으로 붙이고 움직이지 않은 채 바로 "READY". 판 위 주차 자세에서 켜지므로 여기서는
// 닫았다 열지 않는다 (그리퍼가 판에 닿아 있어 끌린다). 워밍업은 재생기가 첫 자세 (판 위) 에서 w 로 한다.
// (14-g 에서는 켜질 때 워밍업을 했다. 붙는 순간 0° 로 가는 것은 같다 — 닫힌 채 꺼졌으면 켜질 때 열린다)
//
// 명령 대기열 (14-g): 움직이는 도중 받은 g 는 버리지 않고 차례로 쌓았다가, 두 서보가 모두 목표에 닿으면 다음 것을 한다.
//   같은 목표가 연달아 오면 하나로 합친다 (재생기가 같은 목표를 0.2 초마다 다시 보낸다).
//   길이 QMAX. 넘치면 가장 오래된 것을 버리고 D 를 하나 올린다.
//   인계 때 "받는 손 닫기" 가 다 끝난 뒤 "주는 손 열기" 가 시작된다 (두 서보가 동시에 움직이지 않는다 — 전류도 덜 든다).
// 한 번에 한 서보 (14-i): 두 서보가 동시에 움직이면 USB 5 V 전원이 모자라 R 이 닫히다 멈췄다 (2026-11-08 실기 시험).
//   한 명령에서 둘 다 바뀌면 닫는 쪽 (각도가 커지는 쪽) 먼저, 여는 쪽 나중. 둘 다 같은 쪽이면 FIRST_SERVO 먼저. 워밍업 w 도 한 서보씩.
//   한 서보의 명령값이 목표에 닿으면 SETTLE_MS 기다린 뒤 다음 서보·다음 명령 (명령값 도착은 실제 도착보다 빠르다).
// delay 로 막지 않는다: loop 가 매번 직렬을 읽고, STEP_MS 마다 1도씩 목표 쪽으로 간다.
// 명령이 끊겨도 그 자리를 유지한다 (자동으로 열지 않는다 — 쥔 것을 놓치지 않게).

#include <Servo.h>

const int PIN_A = 9;
const int PIN_B = 10;
const int ANGLE_MIN = 0;
const int ANGLE_MAX = 90;
const int ANGLE_OPEN = 0;
const int ANGLE_CLOSE = 90;                  // g 범위 위끝 (닫힘 각도는 PC 가 g 로 정한다)
const int ANGLE_WARM_CLOSE = 60;             // 14-n: w 에 각도가 없을 때 워밍업 닫힘 (2026-11-10 실측: 캔 Ø5.75 cm 55° 부터 안정 → 60)
const unsigned long STEP_MS = 10;            // 1도 가는 데 걸리는 시간 (클수록 천천히, 전류 피크 ↓) — 예전과 같다
const uint8_t QMAX = 8;                      // 명령 대기열 길이
const unsigned long SETTLE_MS = 400;         // 14-i: 한 서보가 (명령값으로) 목표에 닿은 뒤 다음 서보·다음 명령까지 기다림
const uint8_t FIRST_SERVO = 0;               // 14-i: 둘 다 닫거나 둘 다 열 때 먼저 움직일 서보 (0 = A 왼쪽, 1 = B 오른쪽)

struct Grip {
  Servo s;
  int pin;
  int cur;        // 서보에 마지막으로 쓴 각도. -1 = 아직 한 번도 신호를 낸 적 없음
  int target;
  bool attached;
};

Grip ga = {Servo(), PIN_A, -1, -1, false};
Grip gb = {Servo(), PIN_B, -1, -1, false};
unsigned long lastStep = 0;

int qa[QMAX];
int qb[QMAX];
bool qw[QMAX];                 // 워밍업 (w) 으로 넣은 것
bool activeWarm = false;       // 지금 하는 목표가 워밍업인가
int cmdA = ANGLE_OPEN;         // 14-i: 지금 (또는 마지막) 명령의 두 목표 — 합치기 비교용 (차례가 안 온 서보의 target 은 아직 앞 값)
int cmdB = ANGLE_OPEN;
Grip *stageG[2];               // 14-i: 이 명령에서 차례로 움직일 서보
int stageT[2];
uint8_t nStage = 0;            // 0 = 하는 명령 없음
uint8_t iStage = 0;
bool settling = false;
unsigned long settleT0 = 0;
uint8_t qHead = 0;
uint8_t qLen = 0;
unsigned int qDropped = 0;

char buf[40];
uint8_t blen = 0;

int clampAngle(long a) {
  if (a < ANGLE_MIN) return ANGLE_MIN;
  if (a > ANGLE_MAX) return ANGLE_MAX;
  return (int)a;
}

void attachAt(Grip &g, int angle) {       // 각도를 먼저 쓰고 붙인다 (붙는 순간 다른 각도로 튀지 않게)
  g.cur = angle;
  g.s.write(g.cur);
  g.s.attach(g.pin);
  g.attached = true;
}

void setTarget(Grip &g, int t) {
  g.target = clampAngle(t);
  if (g.cur < 0) {            // 워밍업 뒤에는 오지 않는다. 혹시 붙인 적이 없으면 열림에서 시작해 1도씩
    attachAt(g, ANGLE_OPEN);
  } else if (!g.attached) {   // x 뒤: 마지막 각도를 먼저 쓰고 붙인 뒤 1도씩
    attachAt(g, g.cur);
  }
}

bool atTarget(const Grip &g) {
  return !g.attached || g.cur == g.target;
}

void stepOne(Grip &g) {
  if (!g.attached || g.cur < 0 || g.cur == g.target) return;
  g.cur += (g.target > g.cur) ? 1 : -1;
  g.s.write(g.cur);
}

// ---- 대기열
void qLast(int &a, int &b) {               // 대기열 끝 (없으면 지금 명령)
  if (qLen > 0) {
    uint8_t i = (qHead + qLen - 1) % QMAX;
    a = qa[i];
    b = qb[i];
  } else {
    a = cmdA;
    b = cmdB;
  }
}

bool warming() {               // 워밍업이 대기열에 있거나 하는 중 (상태 W)
  if (activeWarm) return true;
  for (uint8_t k = 0; k < qLen; k++) {
    if (qw[(qHead + k) % QMAX]) return true;
  }
  return false;
}

void qPush(int a, int b, bool w) {
  int la, lb;
  qLast(la, lb);
  // 같은 목표가 연달아 → 하나로 (대기열이 비었으면 지금 목표와 비교). 워밍업 것은 합치지 않는다
  if (!w && la == a && lb == b && (qLen > 0 || (ga.attached && gb.attached))) return;
  if (qLen == QMAX) {                    // 넘침 → 가장 오래된 것을 버린다
    qHead = (qHead + 1) % QMAX;
    qLen--;
    qDropped++;
  }
  uint8_t i = (qHead + qLen) % QMAX;
  qa[i] = a;
  qb[i] = b;
  qw[i] = w;
  qLen++;
}

bool needsMove(const Grip &g, int t) {
  return !g.attached || g.cur != t || g.target != t;
}

void startCommand(int a, int b, bool w) {  // 14-i: 바뀌는 서보만, 닫는 쪽 먼저
  cmdA = a;
  cmdB = b;
  activeWarm = w;
  bool mA = needsMove(ga, a);
  bool mB = needsMove(gb, b);
  nStage = 0;
  iStage = 0;
  settling = false;
  if (mA && mB) {
    bool closeA = a > ga.cur;
    bool closeB = b > gb.cur;
    bool aFirst = (closeA != closeB) ? closeA : (FIRST_SERVO == 0);
    stageG[0] = aFirst ? &ga : &gb;
    stageT[0] = aFirst ? a : b;
    stageG[1] = aFirst ? &gb : &ga;
    stageT[1] = aFirst ? b : a;
    nStage = 2;
  } else if (mA || mB) {
    stageG[0] = mA ? &ga : &gb;
    stageT[0] = mA ? a : b;
    nStage = 1;
  }
  if (nStage > 0) setTarget(*stageG[0], stageT[0]);
}

void qNext(unsigned long now) {          // 14-i: 지금 서보가 닿으면 SETTLE_MS 뒤 다음 서보, 다 하면 다음 명령
  if (nStage > 0) {
    if (settling) {
      if (now - settleT0 < SETTLE_MS) return;
      settling = false;
      iStage++;
      if (iStage < nStage) {
        setTarget(*stageG[iStage], stageT[iStage]);
        return;
      }
      nStage = 0;
    } else {
      if (atTarget(*stageG[iStage])) {
        settling = true;
        settleT0 = now;
      }
      return;
    }
  }
  if (qLen == 0) {
    activeWarm = false;
    return;
  }
  int a = qa[qHead];
  int b = qb[qHead];
  bool w = qw[qHead];
  qHead = (qHead + 1) % QMAX;
  qLen--;
  startCommand(a, b, w);
}

void detachAll() {
  if (ga.attached) { ga.s.detach(); ga.attached = false; }
  if (gb.attached) { gb.s.detach(); gb.attached = false; }
}

void printStatus() {
  Serial.print("A "); Serial.print(ga.cur); Serial.print(' '); Serial.print(ga.target); Serial.print(' ');
  Serial.print(ga.attached ? 1 : 0);
  Serial.print(" B "); Serial.print(gb.cur); Serial.print(' '); Serial.print(gb.target); Serial.print(' ');
  Serial.print(gb.attached ? 1 : 0);
  Serial.print(" Q "); Serial.print(qLen);
  Serial.print(" W "); Serial.print(warming() ? 1 : 0);
  Serial.print(" D "); Serial.print(qDropped);
  Serial.print(" M "); Serial.print((nStage > 0 && !settling) ? (stageG[iStage] == &ga ? 1 : 2) : 0);
  Serial.print(" S "); Serial.println(settling ? 1 : 0);
}

void handleLine(char *line) {
  while (*line == ' ') line++;
  if (line[0] == 'g') {
    char *p = line + 1;
    char *end;
    long a = strtol(p, &end, 10);
    if (end == p) { Serial.println("ERR g <A> <B>"); return; }
    p = end;
    long b = strtol(p, &end, 10);
    if (end == p) { Serial.println("ERR g <A> <B>"); return; }
    qPush(clampAngle(a), clampAngle(b), false);
    Serial.println("OK");
  } else if (line[0] == 'w') {
    char *p = line + 1;                       // 14-n: w [A B]
    char *end;
    long a = strtol(p, &end, 10);
    long b = ANGLE_WARM_CLOSE;
    if (end == p) {
      a = ANGLE_WARM_CLOSE;
    } else {
      p = end;
      b = strtol(p, &end, 10);
      if (end == p) b = a;                    // 하나만 주면 둘 다
    }
    qPush(clampAngle(a), clampAngle(b), true);
    qPush(ANGLE_OPEN, ANGLE_OPEN, true);
    Serial.println("OK");
  } else if (line[0] == 'x') {
    detachAll();
    qLen = 0;
    nStage = 0;               // 14-i: 하던 명령도 그만
    settling = false;
    activeWarm = false;       // 워밍업 중 x → 그만두고 풀어 둔다
    Serial.println("OK");
  } else if (line[0] == '?') {
    printStatus();
  } else if (line[0] != 0) {
    Serial.println("ERR");
  }
}

void readSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      buf[blen] = 0;
      if (blen > 0) handleLine(buf);
      blen = 0;
    } else if (blen < sizeof(buf) - 1) {
      buf[blen++] = c;
    } else {
      blen = 0;               // 너무 긴 줄은 버린다
    }
  }
}

void setup() {
  Serial.begin(115200);
  attachAt(ga, ANGLE_OPEN);
  attachAt(gb, ANGLE_OPEN);
  ga.target = ANGLE_OPEN;
  gb.target = ANGLE_OPEN;
  Serial.println("READY");
}

void loop() {
  readSerial();
  unsigned long now = millis();
  if (now - lastStep >= STEP_MS) {
    lastStep = now;
    stepOne(ga);
    stepOne(gb);
  }
  qNext(now);
}
