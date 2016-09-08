
void jacobian(void (*fk)(float*, float*), float *q, size_t m, size_t n, float *_jac) {
  // jacobian(fk, q, 4, 3, (float*) jac);
  float (*jac)[n] = (float (*)[n]) _jac;
  float s1[m], s2[m];
  int i, j;

  const float EPS = 1e-3;

  for(j = 0; j < m; j++) {
    q[j] += EPS;
    fk(q, s2);
    q[j] -= 2*EPS;
    fk(q, s1);
    q[j] += EPS;
    for(i = 0; i < n; i++)
      jac[i][j] = (s2[i] - s1[i])/(2*EPS);
  }
}


float ik(float *t, size_t dof, float *q, void (*fk)(float*, float*)) {
// Apply IK for the fk function
//  t: pointer to target pose
//  q: pointer to joint solution
//  dof: degrees of freedom
// Returns the norm error

  const int MAX_ITER = 30;
  const float MAX_NORM = 1e-3;

  float enorm,
    jac[dof][dof],
    s[dof],
    e[dof],
    dq[dof];

  int k, j;

  fk(q, s);

  for(k = 0; k < MAX_ITER; k++) {

    for(j = 0; j < dof; j++)
      e[j] = t[j] - s[j];
    enorm = norm(e, dof);

    if(enorm < MAX_NORM) break;

    jacobian(fk, q, dof, dof, (float*) jac);
    solve((float*) jac, e, dof, dq);

    // saturate maximum angle increment
    float dqnorm = norm(dq, dof);
    float factor = dqnorm > 3 ? 1/(dqnorm*dqnorm) : 1;

    for (j = 0; j < dof; j++)
      q[j] += dq[j]*factor;

    fk(q, s);
  }
  
  return(enorm);
}
