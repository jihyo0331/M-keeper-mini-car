// ===== 설정 =====
#define ROWS 13
#define COLS 15

// 핀 설정
const int PIN_L = 6;   // 좌/전진
const int PIN_R = 7;   // 우/전진
const int IR_PIN = 3;  // 적외선 센서 입력

// 타이밍(ms)
const unsigned long STEP_MS = 1000;
const unsigned long TURN_MS = 300;
const unsigned long GAP_MS  = 100;

// BLE(UART)
const unsigned long BT_BAUD = 9600;  // HM-10/BT-05 기본

// ---- 원본 지도: 수정 금지 ----
// 0=길, 1=벽, 2=시작, 3=목적지
const int base_map[ROWS][COLS] = {
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

// ---- 작업용 테이블 ----
const int INF = 1000000000;
int dist_tbl[ROWS][COLS];
bool visited[ROWS][COLS];
int prev_r[ROWS][COLS], prev_c[ROWS][COLS];

// 계획 경로/동적 장애물
bool planned[ROWS][COLS];
bool blocked[ROWS][COLS];

// 시작/목표
int start_r=-1, start_c=-1;
int goal_r=-1, goal_c=-1;

// ★ 웹에서 설정하는 목적지 오버라이드
int goal_override_r = -1, goal_override_c = -1;

// ===== BLE 전송 헬퍼 =====
void btBegin() { Serial1.begin(BT_BAUD); delay(100); }
void btSendLine(const String& s) { Serial1.println(s); }

void sendHello()              { btSendLine("HELLO,13,15"); }
void sendGoal(int r, int c)   { btSendLine("GOAL,"  + String(r) + "," + String(c)); }
void sendPose(int r, int c)   { btSendLine("POS,"   + String(r) + "," + String(c)); }
void sendTurn(char dir)       { btSendLine(String("TURN,") + dir); }
void sendMoveStep()           { btSendLine("MOVE,1"); }
void sendBlocked(int r,int c) { btSendLine("BLOCK," + String(r) + "," + String(c)); }
void sendReplan()             { btSendLine("REPLAN"); }
void sendArrival()            { btSendLine("ARRIVE"); }
void sendWall(int r,int c)    { btSendLine("WALL,"  + String(r) + "," + String(c)); }
void sendClearPath()          { btSendLine("CLEARPATH"); }
void sendPathCell(int r,int c){ btSendLine("PATH,"  + String(r) + "," + String(c)); }

// ===== 유틸 =====
bool inBounds(int r,int c){ return (r>=0 && r<ROWS && c>=0 && c<COLS); }
int currentVal(int r,int c){ if(blocked[r][c]) return 1; return base_map[r][c]; }

void findStartGoal(){
  // 시작은 맵에서, 목표는 override 우선
  start_r = start_c = -1;
  if (goal_override_r >= 0 && goal_override_c >= 0) {
    goal_r = goal_override_r; goal_c = goal_override_c;
  } else {
    goal_r = goal_c = -1;
  }
  for(int r=0;r<ROWS;r++){
    for(int c=0;c<COLS;c++){
      int v = currentVal(r,c);
      if(v==2){ start_r=r; start_c=c; }
      if(v==3 && goal_r==-1){ goal_r=r; goal_c=c; } // override 없을 때만 맵의 3 사용
    }
  }
}

void clearPlanned(){ for(int r=0;r<ROWS;r++) for(int c=0;c<COLS;c++) planned[r][c]=false; }

void printMapOverlay(int cur_r,int cur_c){
  for(int r=0;r<ROWS;r++){
    for(int c=0;c<COLS;c++){
      int v;
      if(r==cur_r && c==cur_c){ v = 2; }
      else if(blocked[r][c]){ v = 1; }
      else{
        v = base_map[r][c];
        if(v==2) v=0;
        if(planned[r][c] && v==0) v=4;
      }
      Serial.print(v); Serial.print(' ');
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

// 센서: 신호가 들어오면 장애물
bool isObstacle(){
  int cnt = 0;
  for(int k=0;k<5;k++){ if(digitalRead(IR_PIN)==HIGH) cnt++; delay(2); }
  return (cnt >= 3);
}

// 길찾기
bool pathfindFrom(int s_r,int s_c, int path_r[],int path_c[],int& path_len){
  for(int r=0;r<ROWS;r++) for(int c=0;c<COLS;c++){
    dist_tbl[r][c]=INF; visited[r][c]=false; prev_r[r][c]=-1; prev_c[r][c]=-1;
  }
  dist_tbl[s_r][s_c]=0;
  const int dr[4]={-1,1,0,0}, dc[4]={0,0,-1,1};
  for(int it=0; it<ROWS*COLS; it++){
    int mr=-1, mc=-1, best=INF;
    for(int r=0;r<ROWS;r++) for(int c=0;c<COLS;c++)
      if(!visited[r][c] && dist_tbl[r][c]<best){ best=dist_tbl[r][c]; mr=r; mc=c; }
    if(mr==-1) break;
    if(mr==goal_r && mc==goal_c) break;
    visited[mr][mc]=true;
    for(int k=0;k<4;k++){
      int nr=mr+dr[k], nc=mc+dc[k];
      if(!inBounds(nr,nc)) continue;
      if(currentVal(nr,nc)==1) continue;
      if(dist_tbl[mr][mc]+1 < dist_tbl[nr][nc]){
        dist_tbl[nr][nc]=dist_tbl[mr][mc]+1; prev_r[nr][nc]=mr; prev_c[nr][nc]=mc;
      }
    }
  }
  if(dist_tbl[goal_r][goal_c]==INF) return false;

  int tr[ROWS*COLS], tc[ROWS*COLS], n=0;
  int r=goal_r, c=goal_c;
  while(!(r==s_r && c==s_c)){
    tr[n]=r; tc[n]=c; n++;
    int pr=prev_r[r][c], pc=prev_c[r][c];
    if(pr==-1 && pc==-1) break;
    r=pr; c=pc;
  }
  tr[n]=s_r; tc[n]=s_c; n++;
  path_len=0;
  for(int i=n-1;i>=0;i--){ path_r[path_len]=tr[i]; path_c[path_len]=tc[i]; path_len++; }
  return true;
}

// 계획 경로 오버레이 + BLE PATH 통지
void buildPlannedOverlay(int path_r[],int path_c[],int path_len){
  clearPlanned();
  sendClearPath();
  for(int i=1;i<path_len-1;i++){
    int r=path_r[i], c=path_c[i];
    if(!blocked[r][c] && base_map[r][c]==0){
      planned[r][c] = true;
      sendPathCell(r,c);
    }
  }
}

// 모터
void bothHigh(){ digitalWrite(PIN_L,HIGH); digitalWrite(PIN_R,HIGH); }
void bothLow() { digitalWrite(PIN_L,LOW);  digitalWrite(PIN_R,LOW);  }
void leftTurn(){  digitalWrite(PIN_L,LOW);  digitalWrite(PIN_R,HIGH); delay(TURN_MS); bothLow(); delay(GAP_MS); }
void rightTurn(){ digitalWrite(PIN_L,HIGH); digitalWrite(PIN_R,LOW);  delay(TURN_MS); bothLow(); delay(GAP_MS); }

// 정적 벽 브로드캐스트
void broadcastStaticWalls(){
  for(int r=0;r<ROWS;r++) for(int c=0;c<COLS;c++)
    if(base_map[r][c]==1) sendWall(r,c);
}

// ===== 주행 =====
void runPathfindingAndDrive(){
  findStartGoal();
  if(start_r==-1){ Serial.println("시작점(2) 없음"); return; }
  if(goal_r==-1){  Serial.println("목적지(3/override) 없음"); return; }

  int path_r[ROWS*COLS], path_c[ROWS*COLS], path_len=0;
  if(!pathfindFrom(start_r,start_c,path_r,path_c,path_len)){
    Serial.println("경로 없음"); return;
  }

  sendGoal(goal_r, goal_c);
  sendPose(start_r, start_c);
  buildPlannedOverlay(path_r,path_c,path_len);

  Serial.print("최단 거리: "); Serial.println(dist_tbl[goal_r][goal_c]);
  int cur_r = start_r, cur_c = start_c;
  printMapOverlay(cur_r,cur_c);
  if(path_len<2){ Serial.println("움직일 칸 없음"); return; }

  int dir_cur = stepDir(path_r[1]-path_r[0], path_c[1]-path_c[0]);
  if(dir_cur==-1){ Serial.println("경로 해석 오류"); return; }

  Serial.println("=== 주행 시작 ===");
  int i = 1;
  while(true){
    if(i >= path_len){ Serial.println("경로 소진"); break; }
    int next_r = path_r[i], next_c = path_c[i];
    int dir_next = stepDir(next_r-cur_r, next_c-cur_c);
    if(dir_next==-1){ i++; continue; }

    int diff = (dir_next - dir_cur + 4) % 4;
    if(diff==1){ rightTurn(); sendTurn('R'); dir_cur=dir_next; }
    else if(diff==3){ leftTurn(); sendTurn('L');  dir_cur=dir_next; }
    else if(diff==2){ rightTurn(); rightTurn(); sendTurn('U'); dir_cur=dir_next; }

    if(isObstacle()){
      blocked[next_r][next_c] = true;
      sendBlocked(next_r, next_c);
      sendWall(next_r, next_c);
      sendReplan();

      if(!pathfindFrom(cur_r,cur_c,path_r,path_c,path_len)){
        Serial.println("재탐색 실패: 더 이상 경로 없음"); bothLow(); break;
      }
      buildPlannedOverlay(path_r,path_c,path_len);
      printMapOverlay(cur_r,cur_c);

      if(path_len>=2){
        dir_cur = stepDir(path_r[1]-path_r[0], path_c[1]-path_c[0]);
        i = 1;
        continue;
      } else { Serial.println("움직일 칸 없음"); break; }
    }

    bothHigh(); delay(STEP_MS); bothLow(); delay(GAP_MS);
    sendMoveStep();
    cur_r = next_r; cur_c = next_c;
    sendPose(cur_r, cur_c);
    printMapOverlay(cur_r,cur_c);

    if(cur_r==goal_r && cur_c==goal_c){ sendArrival(); break; }
    i++;
  }
  Serial.println("=== 주행 종료 ===");
}

// ===== 명령 처리(줄 단위) =====
String bufUSB, bufBT;

void handleCmdLine(const String& lineRaw){
  String line = lineRaw; line.trim();
  if(line.length()==0) return;

  if(line=="1"){
    Serial.println(">> 실행 시작 (CMD)");
    runPathfindingAndDrive();
    Serial.println("완료. 다시 실행하려면 '1'");
    return;
  }

  if(line.startsWith("SETGOAL")){
    // SETGOAL,r,c
    int p1 = line.indexOf(',');
    int p2 = (p1>=0)? line.indexOf(',', p1+1) : -1;
    if(p1<0 || p2<0){ Serial.println("SETGOAL 파싱 실패"); return; }
    int r = line.substring(p1+1, p2).toInt();
    int c = line.substring(p2+1).toInt();

    if(!inBounds(r,c)){ Serial.println("SETGOAL: 범위 밖"); return; }
    // if(currentVal(r,c)==1){ Serial.println("SETGOAL: 벽(1) 위치"); return; }

    goal_override_r = r; goal_override_c = c;
    // 현재 목표 갱신 알림
    sendGoal(r,c);
    Serial.print("새 목표 설정: "); Serial.print(r); Serial.print(","); Serial.println(c);
    return;
  }

  Serial.print("알 수 없는 명령: "); Serial.println(line);
}

void pumpSerialToBuffer(HardwareSerial& s, String& buf){
  while(s.available()){
    char ch = s.read();
    if(ch=='\r') continue;
    if(ch=='\n'){ handleCmdLine(buf); buf=""; }
    else { buf += ch; if(buf.length()>100) buf.remove(0); }
  }
}

// ===== Arduino 진입점 =====
void setup(){
  Serial.begin(115200);
  while(!Serial){;}

  pinMode(PIN_L, OUTPUT);
  pinMode(PIN_R, OUTPUT);
  pinMode(IR_PIN, INPUT_PULLDOWN);

  bothLow();
  for(int r=0;r<ROWS;r++) for(int c=0;c<COLS;c++){ blocked[r][c]=false; planned[r][c]=false; }

  btBegin();
  sendHello();
  broadcastStaticWalls();

  Serial.println("=== 준비 완료 ===");
  Serial.println("PC/BT에서 '1' → 주행 시작, 'SETGOAL,r,c' → 목적지 설정");
  printMapOverlay(-1,-1);
}

void loop(){
  pumpSerialToBuffer(Serial,  bufUSB);
  pumpSerialToBuffer(Serial1, bufBT);
}
