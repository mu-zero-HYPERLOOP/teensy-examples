#pragma once
#include <cstdint>

constexpr bool print = false;

using BaseType = double;

template<uint8_t dim_state, uint8_t dim_obser>
struct Ekf {
  BaseType x_hat[dim_state];

  BaseType P_pre[dim_state * dim_state];
  BaseType P[dim_state * dim_state];
  BaseType Q[dim_state * dim_state];
  BaseType R[dim_obser * dim_obser];

  BaseType K[dim_state * dim_obser];

  BaseType f_xu[dim_state];
  BaseType F[dim_state * dim_state];
  BaseType F_T[dim_state * dim_state];
  BaseType h_x[dim_obser];
  BaseType H[dim_obser * dim_state];
  BaseType H_T[dim_state * dim_obser];

  BaseType temp1[dim_state * dim_state];
  BaseType temp2[dim_state * dim_obser];
  BaseType temp3[dim_obser * dim_state];
  BaseType temp4[dim_obser * dim_obser];
  BaseType temp5[dim_obser];
  BaseType temp6[dim_obser];
  BaseType temp7[dim_obser * dim_obser];

};


template<uint8_t dim_state, uint8_t dim_obser>
int ekf_step(Ekf<dim_state, dim_obser>& ekf, BaseType* z);



