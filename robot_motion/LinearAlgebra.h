#include <Arduino.h>

void perror(const char *err) {
  Serial.print(String("Error: ") + err + '\n');
}

void bsSolve(float *_u, float *b, size_t n, float *x) {
  // Solve linear system of a Upper triangular matrix U

  float (*u)[n] = (float (*)[n]) _u;

  int i, j, k;
  const float TOL = 1e-5;

  for(i = n-1; i >= 0; i--) {
    if(fabs(u[i][i]) < TOL)
      perror("Nearly singular matrix");

    for(k = n-1; k > i; k--)
      b[i] -= x[k]*u[i][k];
    x[i] = b[i] / u[i][i];
  }
}


void fsSolve(float *_l, float *b, size_t n, float *x) {
  // Solve linear system of a Lower triangular matrix L, with diagonal equal to 1

  float (*l)[n] = (float (*)[n]) _l;

  int i, j, k;
  const float TOL = 1e-5;

  for(i = 0; i < n; i++) {
    if(fabs(l[i][i]) < TOL)
      perror("Nearly singular matrix");

    for(k = 0; k < i; k++)
      b[i] -= x[k]*l[i][k];
    x[i] = b[i];
  }
}


void luFact(float *_a, int *ipivot, size_t n) {

  float (*a)[n] = (float (*)[n]) _a;

  int i, j, k;
  const float TOL = 1e-5;

  for (k = 0; k < n; k++) {
    int ipm = k;

    // find pivot
    int kp = ipivot[k];
    float pv = fabs(a[kp][k]);

    for (i = k+1; i < n; i++) {
      int ip = ipivot[i];
      if (pv < fabs(a[ip][k])) {
        pv = fabs(a[ip][k]);
        ipm = i;
      }
    }

    // Interchange rows via ipivot array, if necessary
    if (ipm != k) {
      float tmp = ipivot[k];
      ipivot[k] = ipivot[ipm];
      ipivot[ipm] = tmp;
    }
    kp = ipivot[k];

    // check singularity
    if (fabs(a[kp][k]) < TOL) {
      perror("Nearly singular matrix");
    }

    // compute and store multipliers
    for (i = k+1; i < n; i++) {
      int ip = ipivot[i];
      a[ip][k] /= a[kp][k];
    }

    // apply transformation to submatrix
    for (j = k+1; j < n; j++)
      for (i = k+1; i < n; i++) {
        int ip = ipivot[i];
        a[ip][j] -= a[ip][k] * a[kp][j];
      }
  }
}

float norm(float *v, size_t n) {
  
  float norm = 0;
  int j;
  
  for(j = 0; j < n; j++)
    norm += v[j]*v[j];
  norm = sqrt(norm);

  return(norm);
}


void solve(float *_a, float *b, size_t n, float *x) {

  float (*a)[n] = (float (*)[n]) _a,
    pb[n], y[n], pa[n][n];
  int ipivot[n];

  int k, j;

  for (k = 0; k < n; k++)
    ipivot[k] = k;

  luFact((float*) a, ipivot, n);

  // flip rows
  for (k = 0; k < n; k++) {
    pb[k] = b[ipivot[k]];

    for (j = 0; j < n; j++)
      pa[k][j] = a[ipivot[k]][j];
  }

  fsSolve((float*) pa, pb, n, y);
  bsSolve((float*) pa, y, n, x);
}
