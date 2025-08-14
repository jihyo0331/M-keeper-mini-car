#include <SoftwareSerial.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <stdio.h>

// ===== 설정 =====
#define ROWS 13
#define COLS 15
#define N    (ROWS*COLS)

// ---- 핀 설정 ----
const uint8_t PIN_L = 6;   // 좌/전진
const uint8_t PIN_R = 7;   // 우/전진
const uint8_t IR_PIN = 3;  // 적외선 센서 입력

// BT-05 (HM-10 호환) SoftwareSerial (UNO/Nano 등)
const uint8_t BT_RX_PIN = 8; // Arduino RX  <- BT-05 TX
const uint8_t BT_TX_PIN = 9; // Arduino TX  <- BT-05 RX
const unsigned long BT_BAUD = 9600;
SoftwareSerial BT(BT_RX_PIN, BT_TX_PIN);

// 타이밍(ms)
const unsigned long STEP_MS = 1000;
const unsigned long TURN_MS = 300;
const unsigned long GAP_MS  = 100;

// ---- 지도 (0=길, 1=벽, 2=시작, 3=목적지) : PROGMEM + uint8_t ----
const uint8_t base_map[ROWS][COLS] PROGMEM = {
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,2},
  {1,0,1,0,0,0,0,1,0,1,1,1,1,1,1},
  {1,0,0,1,1,1,1,0,0,1,1,1,1,1,1},
  {1,0,0,1,1,1,1,0,0,0,0,0,0,1,1},
  {1,0,0,1,1,1,1,0,0,1,1,1,0,1,1},
  {1,0,1,0,0,0,0,1,0,1,1,1,0,1,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,1,0,0,0,0,1,0,1,0,0,1,0,1},
  {1,0,0,1,1,1,1,0,0,0,1,1,0,0,1},
  {1,0,1,0,0,0,0,1,0,1,0,0,1,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
};
inline uint8_t mapAt(uint8_t r, uint8_t c){ return pgm_read_byte(&base_map[r][c]); }

// ---- 동적 장애물 / 경로표시 비트셋 ----
uint8_t blockedBits[(N+7)/8];
uint8_t pathBits[(N+7)/8];

inline void bset(uint8_t* bits, uint16_t i){ bits[i>>3] |=  (1<<(i&7)); }
inline void bclr(uint8_t* bits, uint16_t i){ bits[i>>3] &= ~(1<<(i&7)); }
inline bool bget(const uint8_t* bits, uint16_t i){ return (bits[i>>3] & (1<<(i&7))) != 0; }
inline uint16_t IDX(uint8_t r,uint8_t c){ return (uint16_t)r*COLS + c; }
inline bool inBounds(int8_t r,int8_t c){ return (r>=0 && r<ROWS && c>=0 && c<COLS); }

// ---- BFS용 테이블(경량) ----
uint16_t distv[N];
int16_t  parentIdx[N];
uint8_t  visitedBits[(N+7)/8];
uint8_t  q[N];  uint8_t qh, qt;
uint16_t pathBuf[N];

// ---- 시작/목표 좌표 & 오버라이드 ----
int8_t start_r=-1, start_c=-1;
int8_t goal_r =-1, goal_c =-1;
int8_t goal_override_r=-1, goal_override_c=-1;

// ===== BLE 전송 =====
void sendHello(){       BT.println(F("HELLO,13,15")); }
void sendGoal(int r,int c){ BT.print(F("GOAL,")); BT.print(r); BT.print(','); BT.println(c); }
void sendPose(int r,int c){ BT.print(F("POS,"));  BT.print(r); BT.print(','); BT.println(c); }
void sendTurn(char d){  BT.print(F("TURN,")); BT.println(d); }
void sendMoveStep(){    BT.println(F("MOVE,1")); }
void sendBlocked(int r,int c){ BT.print(F("BLOCK,")); BT.print(r); BT.print(','); BT.println(c); }
void sendReplan(){      BT.println(F("REPLAN")); }
void sendArrival(){     BT.println(F("ARRIVE")); }
void sendWall(int r,int c){ BT.print(F("WALL,")); BT.print(r); BT.print(','); BT.println(c); }
void sendClearPath(){   BT.println(F("CLEARPATH")); }
void sendPathCell(int r,int c){ BT.print(F("PATH,")); BT.print(r); BT.print(','); BT.println(c); }

// ===== 보조 =====
uint8_t currentVal(uint8_t r,uint8_t c){
  uint16_t i = IDX(r,c);
  if (bget(blockedBits, i)) return 1;
  return mapAt(r,c);
}

void findStartGoal(){
  start_r = start_c = -1;
  if (goal_override_r>=0 && goal_override_c>=0) { goal_r=goal_override_r; goal_c=goal_override_c; }
  else { goal_r = goal_c = -1; }

  for(uint8_t r=0;r<ROWS;r++){
    for(uint8_t c=0;c<COLS;c++){
      uint8_t v = mapAt(r,c);
      if(v==2){ start_r=r; start_c=c; }
      if(v==3 && goal_r<0){ goal_r=r; goal_c=c; }
    }
  }
}

void printMapOverlay(int cur_r,int cur_c){
  for(uint8_t r=0;r<ROWS;r++){
    for(uint8_t c=0;c<COLS;c++){
      int outv;
      uint8_t base = mapAt(r,c);
      uint16_t i = IDX(r,c);
      if (r==cur_r && c==cur_c) outv = 2;
      else if (bget(blockedBits, i)) outv = 1;
      else {
        if (base==2) base=0;
        outv = base;
        if (base==0 && bget(pathBits, i)) outv = 4;
      }
      Serial.print(outv); Serial.print(' ');
    }
    Serial.println();
  }
  Serial.println();
}

// 방향: 0=UP,1=RIGHT,2=DOWN,3=LEFT
int stepDir(int dr,int dc){
  if(dr==-1 && dc==0) return 0;
  if(dr== 1 && dc==0) return 2;
  if(dr== 0 && dc==1) return 1;
  if(dr== 0 && dc==-1) return 3;
  return -1;
}

// 센서: HIGH = 신호(장애물) 로 간주
bool isObstacle(){
  uint8_t cnt=0;
  for(uint8_t k=0;k<5;k++){ if(digitalRead(IR_PIN)==HIGH) cnt++; delay(2); }
  return (cnt>=3);
}

// ---- BFS ----
bool bfsFind(uint8_t sr,uint8_t sc, uint8_t gr,uint8_t gc, uint16_t outPath[], uint8_t& outLen){
  for(uint16_t i=0;i<N;i++){ distv[i]=0xFFFF; parentIdx[i]=-1; }
  for(uint8_t i=0;i<sizeof(visitedBits);i++) visitedBits[i]=0;
  const int8_t dr[4]={-1,1,0,0}; const int8_t dc[4]={0,0,-1,1};

  uint16_t s=IDX(sr,sc), g=IDX(gr,gc);
  distv[s]=0; bset(visitedBits,s); qh=qt=0; q[qt++]=(uint8_t)s;

  while(qh!=qt){
    uint16_t u = q[qh++];
    uint8_t ur=u/COLS, uc=u%COLS;
    if(u==g) break;
    for(uint8_t k=0;k<4;k++){
      int8_t nr=ur+dr[k], nc=uc+dc[k];
      if(!inBounds(nr,nc)) continue;
      if(currentVal(nr,nc)==1) continue;
      uint16_t v=IDX(nr,nc);
      if(!bget(visitedBits,v)){
        bset(visitedBits,v);
        distv[v]=distv[u]+1;
        parentIdx[v]=(int16_t)u;
        q[qt++]=(uint8_t)v;
      }
    }
  }
  if(distv[g]==0xFFFF) return false;

  uint8_t len=0;
  for(int16_t cur=(int16_t)g; cur!=-1; cur=parentIdx[cur]){
    outPath[len++]=(uint16_t)cur;
    if(cur==(int16_t)s) break;
  }
  for(uint8_t i=0;i<len/2;i++){ uint16_t t=outPath[i]; outPath[i]=outPath[len-1-i]; outPath[len-1-i]=t; }
  outLen=len; return true;
}

// 경로 비트셋 + PATH 전송
void setPathBitsAndSend(const uint16_t path[], uint8_t len){
  for(uint16_t i=0;i<(N+7)/8;i++) pathBits[i]=0;
  sendClearPath();
  if(len<=2) return;
  for(uint8_t i=1;i<len-1;i++){
    uint16_t idx=path[i];
    uint8_t r=idx/COLS, c=idx%COLS;
    if(mapAt(r,c)==0 && !bget(blockedBits,idx)){
      bset(pathBits,idx);
      sendPathCell(r,c);
    }
  }
}

// 모터 제어
void bothHigh(){ digitalWrite(PIN_L,HIGH); digitalWrite(PIN_R,HIGH); }
void bothLow() { digitalWrite(PIN_L,LOW);  digitalWrite(PIN_R,LOW);  }
void leftTurn(){  digitalWrite(PIN_L,LOW);  digitalWrite(PIN_R,HIGH); delay(TURN_MS); bothLow(); delay(GAP_MS); }
void rightTurn(){ digitalWrite(PIN_L,HIGH); digitalWrite(PIN_R,LOW);  delay(TURN_MS); bothLow(); delay(GAP_MS); }

// 정적 벽 브로드캐스트
void broadcastStaticWalls(){
  for(uint8_t r=0;r<ROWS;r++)
    for(uint8_t c=0;c<COLS;c++)
      if(mapAt(r,c)==1) sendWall(r,c);
}

// ===== 주행 =====
void runPathfindingAndDrive(){
  findStartGoal();
  if(start_r<0){ Serial.println(F("시작점(2) 없음")); return; }
  if(goal_r<0){  Serial.println(F("목적지(3/override) 없음")); return; }

  uint8_t plen=0;
  if(!bfsFind(start_r,start_c,goal_r,goal_c,pathBuf,plen)){
    Serial.println(F("경로 없음")); return;
  }

  sendGoal(goal_r, goal_c);
  sendPose(start_r, start_c);
  setPathBitsAndSend(pathBuf, plen);

  Serial.print(F("최단 거리: ")); Serial.println(distv[IDX(goal_r,goal_c)]);
  int8_t cur_r=start_r, cur_c=start_c;
  printMapOverlay(cur_r,cur_c);
  if(plen<2){ Serial.println(F("움직일 칸 없음")); return; }

  int dir_cur = stepDir((int)(pathBuf[1]/COLS)-(int)cur_r, (int)(pathBuf[1]%COLS)-(int)cur_c);
  if(dir_cur==-1){ Serial.println(F("경로 해석 오류")); return; }

  Serial.println(F("=== 주행 시작 ==="));
  uint8_t i=1;
  while(true){
    if(i>=plen){ Serial.println(F("경로 소진")); break; }
    int8_t next_r=pathBuf[i]/COLS, next_c=pathBuf[i]%COLS;

    int dir_next = stepDir((int)next_r-(int)cur_r, (int)next_c-(int)cur_c);
    if(dir_next==-1){ i++; continue; }

    int diff=(dir_next - dir_cur + 4) % 4;
    if(diff==1){ rightTurn(); sendTurn('R'); dir_cur=dir_next; }
    else if(diff==3){ leftTurn();  sendTurn('L'); dir_cur=dir_next; }
    else if(diff==2){ rightTurn(); rightTurn(); sendTurn('U'); dir_cur=dir_next; }

    if(isObstacle()){
      uint16_t bi=IDX(next_r,next_c);
      bset(blockedBits,bi);
      sendBlocked(next_r,next_c);
      sendWall(next_r,next_c);
      sendReplan();

      if(!bfsFind(cur_r,cur_c,goal_r,goal_c,pathBuf,plen)){
        Serial.println(F("재탐색 실패: 더 이상 경로 없음"));
        bothLow(); break;
      }
      setPathBitsAndSend(pathBuf,plen);
      printMapOverlay(cur_r,cur_c);

      if(plen>=2){
        dir_cur = stepDir((int)(pathBuf[1]/COLS)-(int)cur_r, (int)(pathBuf[1]%COLS)-(int)cur_c);
        i=1; continue;
      }else{
        Serial.println(F("움직일 칸 없음 → 중단"));
        break;
      }
    }

    bothHigh(); delay(STEP_MS); bothLow(); delay(GAP_MS);
    sendMoveStep();
    cur_r=next_r; cur_c=next_c;
    sendPose(cur_r,cur_c);
    printMapOverlay(cur_r,cur_c);

    if(cur_r==goal_r && cur_c==goal_c){
      sendArrival();
      Serial.println(F("=== 목적지 도착 ==="));
      break;
    }
    i++;
  }
  Serial.println(F("=== 주행 종료 ==="));
}

// ===== 명령 처리 (줄 버퍼) =====
char bufUSB[64];  uint8_t usbLen=0;
char bufBTx[64];  uint8_t btLen=0;

// 줄 앞뒤 쓰레기 제거: 첫 영문/숫자부터 해석
const char* skipNoise(const char* s){
  while(*s){
    char c=*s;
    if ((c>='A'&&c<='Z') || (c>='a'&&c<='z') || (c>='0'&&c<='9')) return s;
    s++;
  }
  return s;
}

void handleLine(const char* raw){
  const char* line = skipNoise(raw);      // ← 앞쪽 깨진 바이트 제거
  // 빠른 토큰 검색 (줄 중간이어도 처리)
  const char* p;

  // '1'만 보내면 시작 (공백 허용)
  while(*line==' ') line++;
  if(line[0]=='1' && line[1]=='\0'){
    Serial.println(F(">> 실행 시작 (CMD)"));
    runPathfindingAndDrive();
    Serial.println(F("완료. 다시 실행하려면 '1'"));
    return;
  }

  if( (p=strstr(line,"SETGOAL,")) ){
    int r,c;
    if(sscanf(p+8,"%d,%d",&r,&c)==2 && inBounds(r,c)){
      goal_override_r=(int8_t)r; goal_override_c=(int8_t)c;
      sendGoal(r,c);
      Serial.print(F("새 목표 설정: ")); Serial.print(r); Serial.print(','); Serial.println(c);
    }else{
      Serial.println(F("SETGOAL 파싱/범위 오류"));
    }
    return;
  }

  if( (p=strstr(line,"GOAL,")) ){
    int r,c;
    if(sscanf(p+5,"%d,%d",&r,&c)==2 && inBounds(r,c)){
      goal_override_r=(int8_t)r; goal_override_c=(int8_t)c;
      sendGoal(r,c);
      Serial.print(F("BT 목표 설정: ")); Serial.print(r); Serial.print(','); Serial.println(c);
    }else{
      Serial.println(F("GOAL 파싱/범위 오류"));
    }
    return;
  }

  Serial.print(F("알 수 없는 명령: ")); Serial.println(raw);
}

// 수신 파이프: ASCII만 수용(줄바꿈 제외), 나머지 버림
template<typename TStream>
void pump(TStream& s, char* buf, uint8_t& len){
  while(s.available()){
    char ch = (char)s.read();

    // 개행 없이 '1'만 와도 즉시 시작
    if (ch=='1' && len==0){
      handleLine("1");
      continue;
    }

    // 허용 문자만 버퍼에 저장(줄바꿈은 통과)
    if (ch!='\n' && ch!='\r'){
      if ( (ch < 32) || (ch > 126) ) continue; // 비ASCII/제어문자 무시
    }

    if(ch=='\r') continue;
    if(ch=='\n'){
      buf[len]='\0';
      if(len>0) handleLine(buf);
      len=0;
    }else{
      if(len < 63) buf[len++] = ch;
    }
  }
}

// ===== Arduino 진입점 =====
void setup(){
  Serial.begin(9600);     // 시리얼 모니터도 9600으로 설정
  BT.begin(BT_BAUD);

  pinMode(PIN_L, OUTPUT);
  pinMode(PIN_R, OUTPUT);
  pinMode(IR_PIN, INPUT); // UNO엔 PULLDOWN 없음(외부 회로 권장)

  bothLow();
  for(uint16_t i=0;i<(N+7)/8;i++){ blockedBits[i]=0; pathBits[i]=0; }

  sendHello();
  broadcastStaticWalls();

  Serial.println(F("=== 준비 완료 ==="));
  Serial.println(F("PC/BT: '1' → 주행 시작, 'SETGOAL,r,c' 또는 'GOAL,r,c' → 목적지 설정"));
  printMapOverlay(-1,-1);
}

void loop(){
  pump(Serial,  bufUSB, usbLen);
  pump(BT,      bufBTx, btLen);
}