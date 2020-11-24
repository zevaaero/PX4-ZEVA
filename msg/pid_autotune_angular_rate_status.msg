uint64 timestamp                # time since system start (microseconds)

float32[5] coeff                # coefficients of the identified discrete-time model
float32[5] coeff_var            # coefficients' variance of the identified discrete-time model
float32 innov

float32 kc
float32 ki
float32 kd

float32[3] rate_sp

float32 u_filt
float32 y_filt

uint8 STATE_IDLE = 0
uint8 STATE_ROLL = 1
uint8 STATE_ROLL_PAUSE = 2
uint8 STATE_PITCH = 3
uint8 STATE_PITCH_PAUSE = 4
uint8 STATE_YAW = 5
uint8 STATE_YAW_PAUSE = 6
uint8 STATE_VERIFICATION = 7
uint8 STATE_COMPLETE = 8
uint8 STATE_FAIL = 9

uint8 state
