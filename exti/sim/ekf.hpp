#pragma once
#include <cstdint>


template<uint8_t dim_state, uint8_t dim_obser>
struct Ekf {
  float x_hat[dim_state];

  float P_pre[dim_state * dim_state];
  float P[dim_state * dim_state];
  float Q[dim_state * dim_state];
  float R[dim_obser * dim_obser];

  float K[dim_state * dim_obser];

  float f_xu[dim_state];
  float F[dim_state * dim_state];
  float F_T[dim_state * dim_state];
  float h_x[dim_obser];
  float H[dim_obser * dim_state];
  float H_T[dim_state * dim_obser];

  float temp1[dim_state * dim_state];
  float temp2[dim_state * dim_obser];
  float temp3[dim_obser * dim_state];
  float temp4[dim_obser * dim_obser];
  float temp5[dim_obser];
  float temp6[dim_obser];
  float temp7[dim_obser * dim_obser];

};


template<uint8_t dim_state, uint8_t dim_obser>
int ekf_step(Ekf<dim_state, dim_obser>& ekf, float* z);



