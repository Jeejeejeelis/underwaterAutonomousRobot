//======== WIRES ===========
// GROUND             BLUE
// Upper limit switch GREEN
// Lower limit switch YELLOW
// Mystery            ORANGE
// Motor down         RED
// Motor up           BROWN
//==========================

// 21781 encoder readings from one cylinder end to another(25 cm)
// 5750 is neutral float

const byte upDir = 48; // motor up pin
const byte downDir = 46; // motor down pin
bool up = false;
bool down = false;
int upLimit; // trigger for UP limit switch
int downLimit; // trigger for DOWN limit switch
const byte sense = 20; // encoder pin
long current = 0; // current position of encoder
int goal = 0; // 50000000; // SET GOAL HERE, later this will be read by serial from RP
bool input; // for serial commands from RP
long run = 0;
int dir = 0; // NEW
bool flag = false;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  Serial.setTimeout(10);
  pinMode(sense, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sense), s1, CHANGE);
}

void(* resetFunc) (void) = 0;

void s1() {
  if( dir == 1) {
    if(current >= goal) {
      digitalWrite(upDir,LOW);
      digitalWrite(downDir,LOW);
      flag = false;
      Serial.print("st");
      delay(1000);
      resetFunc();
      }
      else current += 1;
    }
  else if(dir == -1) {
    if(current <= goal) {
      digitalWrite(upDir,LOW);
      digitalWrite(downDir,LOW);
      flag = false;
      Serial.print("st");
      delay(1000);
      resetFunc();
      }
    else current -= 1;
  }
}

int selectDir() {
  int in;
  bool fla = false;
  if (current == goal) {
    in = 0;
    digitalWrite(upDir, LOW);
    digitalWrite(downDir, LOW);
  }
  else if (current < goal) {
    digitalWrite(upDir, LOW);
    digitalWrite(downDir, HIGH);
    fla = true;
    in = 1;
  }
  else if (current > goal) {
    in = -1;
    digitalWrite(upDir, HIGH);
    digitalWrite(downDir, LOW);
    fla = true;
  }
  flag = fla;
  return in ;
}

int selectGoal() {
  int gol;
  return gol;
}

void loop() {
  delay(10);
  if (Serial.available() > 0) {
    digitalWrite(LED_BUILTIN, HIGH);
    goal = Serial.parseInt();
    dir = selectDir();
  }
  while(flag)
  }
