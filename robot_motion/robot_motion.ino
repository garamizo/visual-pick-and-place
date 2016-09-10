#include <Servo.h>

// set degrees of freedom ==================
const int dof = 4;
// =======================================

Servo joint[dof]; // robot joint servomotors
Servo gripper;    // robot gripper servomotor

void robot_setup() {
  // setup servomotors

  // set servomotor pins ============================
  int jpins[] = {3, 4, 5, 6};   // joint pins
  int gpin = 7;                 // gripper pin
  // ===================================================

  for (int k = 0; k < dof; k++)
    joint[k].attach(jpins[k]);
  gripper.attach(gpin);
}

void robot_move(float *q) {
  // move ee to a pose, given a tolerance norm2. Returns true if success

  // centralize servomotors =============================
  const float jdirection[] = { -1, -1, 1, 1};   // joint direction
  const float joffset[] = {90, 90, 90, 85};     // joint offset
  // ===================================================

  for (int k = 0; k < dof; k++)
    joint[k].write(jdirection[k] * q[k] * 180 / PI + joffset[k]);
}


void gripper_act(int cmd) {
  // command gripper by 0:open to 100:close
  
  // select gripper servo bounds =================
  float open = 70, close = 180;
  // ==============================================

  float ang = map(constrain(cmd, 0, 100), 0, 100, open, close);
  gripper.write(ang);
}

void p(const char *str, float *_mat, size_t m, size_t n) {
  // print matrix _mat of size mxn, with message str
  
  float (*mat)[n] = (float (*)[n]) _mat;
  int i, j;

  Serial.print(String('\n') + str + '\n');
  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++) {
      Serial.print("\t");
      Serial.print(mat[i][j], DEC);
    }
    Serial.print('\n');
  }
}

void setup() {

  Serial.begin(115200);
  Serial.setTimeout(100);
  robot_setup();
}


void loop() {
  // move to pose sent by the serial link and actuate gripper
  
  if(Serial.available() > 0) {

    // retrieve target pose and gripper state
    float q[] = { Serial.parseFloat(),
                  Serial.parseFloat(), 
                  Serial.parseFloat(), 
                  Serial.parseFloat() };  // in radians and meters [x, y, z, pitch]
    float gact = Serial.parseFloat();     // gripper act [0:open - 100:close]

    delay(10);
    if (Serial.read() != '\r' || Serial.read() != '\n') {
      Serial.println("Not following communication protocol");
      while(Serial.available()) Serial.read();    // clear buffer
      
    } else {
      robot_move(q);
      gripper_act(gact);
      Serial.println("ACK");
    }
  }
}

