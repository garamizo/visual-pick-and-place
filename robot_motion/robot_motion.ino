#include "LinearAlgebra.h"
#include "Robotics.h"
#include <Servo.h>

// set degrees of freedom ==================
const int dof = 4;
float home[] = {0.2, 0, 0.3, 0};  // home pose
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

int robot_move(float *pose, float tol) {
  // move ee to a pose, given a tolerance norm2. Returns true if success

  // centralize servomotors =============================
  const float jdirection[] = { -1, -1, 1, 1};   // joint direction
  const float joffset[] = {90, 90, 90, 85};     // joint offset
  const float CONTR = 0.8;                      // contraction factor (for ik solver)
  // ===================================================

  static float q[] = {0, 0, 0, 0};
  float q0[4] = {q[0]*CONTR, q[1]*CONTR, q[2]*CONTR, q[3]*CONTR};
  float error = ik(pose, 4, q0, fk_fcn);

  if (error > tol) return 0; // Serial.println("Could not reach target");

  memcpy(q, q0, sizeof(float) * 4);
  for (int k = 0; k < dof; k++)
    joint[k].write(jdirection[k] * q[k] * 180 / PI + joffset[k]);

  return 1;
}

void fk_fcn(float *q, float *s) {
  // convert joint coords q to space coords s

  // define robot DH kinematics ============================
  float angle[] = {q[0], q[1] - PI/2, q[2] + PI/2, q[3]},
        offset[] = {.105, 0, 0, 0},
        twist[] = {-PI/2, 0, 0, PI/2},
        length[] = {.02, .145, .185, .077};
  // ===================================================

  float T[4][4] = { // ee pose in transform matrix
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1}
  };

  float tmp[4][4];
  int k, i, j, p;

  for (k = 0; k < dof; k++) {

    float cz = cos(angle[k]), sz = sin(angle[k]), cx = cos(twist[k]), sx = sin(twist[k]);

    float A[4][4] = {
      cz,   -sz * cx,   sz * sx,    length[k]*cz,
      sz,   cz * cx,    -cz * sx,   length[k]*sz,
      0,    sx,     cx,     offset[k],
      0,    0,      0,      1
    };

    memcpy((float*) tmp, (float*) T, sizeof(float) * 16);

    for (i = 0; i < 4; i++)
      for (j = 0; j < 4; j++) {
        T[i][j] = 0;
        for (p = 0; p < 4; p++)
          T[i][j] += tmp[i][p] * A[p][j];
      }
  }

  // select coordinates ==============================
  s[0] = T[0][3]; // x
  s[1] = T[1][3]; // y
  s[2] = T[2][3]; // z
//  s[3] = -atan(T[0][1] / T[0][0]); // yaw (ZYX)
  s[3] = asin(-T[2][0]); // pitch (ZYX)
  // ===================================================
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
  robot_move(home, 0.1);
  gripper_act(0);
}


void loop() {
  // move to pose sent by the serial link and actuate gripper
  
  if(Serial.available() > 0) {

    // retrieve target pose and gripper state
    float s[] = { Serial.parseFloat(),
                  Serial.parseFloat(), 
                  Serial.parseFloat(), 
                  Serial.parseFloat() };  // in radians and meters [x, y, z, pitch]
    float gact = Serial.parseFloat();     // gripper act [0:open - 100:close]

    delay(10);
    if (Serial.read() != '\r' || Serial.read() != '\n') {
      Serial.print("Not following communication protocol");
      while(Serial.available()) Serial.read();    // clear buffer
    }

    // move robot and actuate gripper
    else if (robot_move(s, 0.1) == true) {
      gripper_act(gact);
      Serial.print("Success");
    }
    else
      p("Failure", s, 1, dof);

    Serial.println(); // termination character
  }
}

