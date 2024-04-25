#include "ekf.hpp"
#include <cmath>
#include <cstdint>
#include <cstring>

template<uint8_t n_row, uint8_t n_col>
void mat_sub(const float* a, const float* b, float* res) {
  for (uint8_t row = 0; row < n_row; row++) {
    for (uint8_t col = 0; col < n_col; col++) {
      res[row * n_row + col] = a[row * n_row + col] - b[row * n_row + col];
    }
  }
}
template<uint8_t n_row, uint8_t n_col>
void mat_add(const float* a, const float* b, float* res) {
  for (uint8_t row = 0; row < n_row; row++) {
    for (uint8_t col = 0; col < n_col; col++) {
      res[row * n_row + col] = a[row * n_row + col] + b[row * n_row + col];
    }
  }
}
template<uint8_t n_1, uint8_t n_2, uint8_t n_3>
void mat_mul(const float* a, const float* b, float* res) {
  memset(res, 0, sizeof(float) * n_1 * n_3);
  for (uint8_t x = 0; x < n_1; x++) {
    for (uint8_t y = 0; y < n_3; y++) {
      for (uint8_t k = 0; k < n_2; k++) {
        res[x * n_3 + y] += a[x * n_2 + k] * b[k * n_3 + y];
      }
    }
  }
}

template<uint8_t n_row, uint8_t n_col>
void mat_mv(const float* a, float* res) {
  memcpy(res, a, sizeof(float) * n_row * n_col);
}

template<uint8_t N>
int choldc1(float* res, float* acc) {
    int i,j,k;
    double sum;
    for (i = 0; i < N; i++) {
        for (j = i; j < N; j++) {
            sum = res[i*N+j];
            for (k = i - 1; k >= 0; k--) {
                sum -= res[i*N+k] * res[j*N+k];
            }
            if (i == j) {
                if (sum <= 0) {
                    return 1; /* error */
                }
                acc[i] = sqrt(sum);
            }
            else {
                res[j*N+i] = sum / acc[i];
            }
        }
    }
    return 0; /* success */
}

template<uint8_t N>
int choldcsl(float* a, float* res, float* acc) 
{
    int i,j,k; double sum;
    for (i = 0; i < N; i++) 
        for (j = 0; j < N; j++) 
            res[i*N+j] = a[i*N+j];
    if (choldc1<N>(res, acc)) return 1;
    for (i = 0; i < N; i++) {
        res[i*N+i] = 1 / acc[i];
        for (j = i + 1; j < N; j++) {
            sum = 0;
            for (k = i; k < j; k++) {
                sum -= res[j*N+k] * res[k*N+i];
            }
            res[j*N+i] = sum / acc[j];
        }
    }

    return 0; /* success */
}

template<uint8_t N>
int mat_inv(float* a, float* res, float* acc) 
{
    int i,j,k;
    if (choldcsl<N>(a,res,acc)) return 1;
    for (i = 0; i < N; i++) {
        for (j = i + 1; j < N; j++) {
            res[i*N+j] = 0.0;
        }
    }
    for (i = 0; i < N; i++) {
        res[i*N+i] *= res[i*N+i];
        for (k = i + 1; k < N; k++) {
            res[i*N+i] += res[k*N+i] * res[k*N+i];
        }
        for (j = i + 1; j < N; j++) {
            for (k = j; k < N; k++) {
                res[i*N+j] += res[k*N+i] * res[k*N+j];
            }
        }
    }
    for (i = 0; i < N; i++) {
        for (j = 0; j < i; j++) {
            res[i*N+j] = res[j*N+i];
        }
    }

    return 0; /* success */
}

template<uint8_t dim_state, uint8_t dim_obser>
void predict_mean(Ekf<dim_state, dim_obser>& ekf) {
  mat_mv<dim_state, 1>(ekf.f_xu, ekf.x_hat);
}

template<uint8_t dim_state, uint8_t dim_obser>
void predict_state_cov(Ekf<dim_state, dim_obser>& ekf) {
  mat_mul<dim_state, dim_state, dim_state>(ekf.F, ekf.P_pre, ekf.temp1);
  mat_mul<dim_state, dim_state, dim_state>(ekf.temp1, ekf.F_T, ekf.P);
  mat_add<dim_state, dim_state>(ekf.P, ekf.Q, ekf.P);
}

template<uint8_t dim_state, uint8_t dim_obser>
int calculate_gain(Ekf<dim_state, dim_obser>& ekf) {
  mat_mul<dim_state, dim_state, dim_obser>(ekf.P, ekf.H_T, ekf.temp2);
  mat_mul<dim_obser, dim_state, dim_state>(ekf.H, ekf.P, ekf.temp3);
  mat_mul<dim_obser, dim_state, dim_obser>(ekf.temp3, ekf.H_T, ekf.temp4);
  mat_add<dim_obser, dim_obser>(ekf.temp4, ekf.R, ekf.temp4);
  if (mat_inv<dim_obser>(ekf.temp4, ekf.temp7, ekf.temp5)) return 1;
  mat_mul<dim_state, dim_obser, dim_obser>(ekf.temp2, ekf.temp7, ekf.K);
  return 0;
}

template<uint8_t dim_state, uint8_t dim_obser>
void update_mean(Ekf<dim_state, dim_obser>& ekf, float* z) {
  mat_sub<dim_obser, 1>(z, ekf.h_x, ekf.temp5);
  mat_mul<dim_state, dim_obser, 1>(ekf.K, ekf.temp5, ekf.temp6);
  mat_add<dim_state, 1>(ekf.x_hat, ekf.temp6, ekf.x_hat);
}

template<uint8_t dim_state, uint8_t dim_obser>
void update_state_cov(Ekf<dim_state, dim_obser>& ekf) {
  mat_mul<dim_state, dim_obser, dim_state>(ekf.K, ekf.H, ekf.temp1);
  mat_mul<dim_state, dim_state, dim_state>(ekf.temp1, ekf.P, ekf.P_pre);
  mat_sub<dim_state, dim_state>(ekf.P, ekf.P_pre, ekf.P_pre);
}

template<uint8_t dim_state, uint8_t dim_obser>
int ekf_step(Ekf<dim_state, dim_obser>& ekf, float* z) {
  predict_mean<dim_state, dim_obser>(ekf);
  predict_state_cov<dim_state, dim_obser>(ekf);
  if (calculate_gain(ekf)) return 1;
  update_mean<dim_state, dim_obser>(ekf, z);
  update_state_cov<dim_state, dim_obser>(ekf);
  return 0;
}

template int ekf_step(Ekf<3, 2> &ekf, float *z);




