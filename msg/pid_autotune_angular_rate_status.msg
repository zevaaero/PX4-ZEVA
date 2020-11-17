uint64 timestamp                # time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

float32[5] coeff                # coefficients of the identified discrete-time model
float32[5] coeff_var            # coefficients' variance of the identified discrete-time model
float32 innov

float32 kc
float32 ki
float32 kd

float32[3] rate_sp

float32 u_filt
float32 y_filt
