uint64 timestamp                # time since system start (microseconds)
uint64 timestamp_sample         # source actuator sample timestamp

uint8 MODE_DISABLED = 0
uint8 MODE_3211 = 1
uint8 MODE_SWEEP = 2

uint8 AXIS_ROLL = 0
uint8 AXIS_PITCH = 1
uint8 AXIS_YAW = 2

uint8 mode                      # SYS_ID_EN
uint8 axis                      # SYS_ID_AXIS

float32 signal                  # injected axis command including trim
float32 excitation              # generated 3211/sweep command before trim
float32 frequency               # current sweep angular frequency [rad/s]
float32 elapsed_time            # time since injection start [s]

float32 pure_delta_lon          # longitudinal cyclic command from controller path before SYS_ID
float32 pure_delta_lat          # lateral cyclic command from controller path before SYS_ID
float32 pure_delta_col          # collective pitch command from controller path before SYS_ID
float32 pure_delta_ped          # tail/pedal command from controller path before SYS_ID

float32 sys_id_delta_lon        # SYS_ID injected longitudinal cyclic command
float32 sys_id_delta_lat        # SYS_ID injected lateral cyclic command
float32 sys_id_delta_col        # SYS_ID injected collective pitch command
float32 sys_id_delta_ped        # SYS_ID injected tail/pedal command

float32 total_delta_lon         # pure_delta_lon + sys_id_delta_lon
float32 total_delta_lat         # pure_delta_lat + sys_id_delta_lat
float32 total_delta_col         # pure_delta_col + sys_id_delta_col
float32 total_delta_ped         # pure_delta_ped + sys_id_delta_ped

float32 pure_servo_tail         # requested tail servo command from controller path before SYS_ID
float32 pure_servo_swash0       # requested swash servo 0 command from controller path before SYS_ID
float32 pure_servo_swash1       # requested swash servo 1 command from controller path before SYS_ID
float32 pure_servo_swash2       # requested swash servo 2 command from controller path before SYS_ID
float32 pure_servo_swash3       # requested swash servo 3 command from controller path before SYS_ID

float32 sys_id_servo_tail       # requested SYS_ID tail servo component before clipping/output reversal
float32 sys_id_servo_swash0     # requested SYS_ID swash servo 0 component before clipping/output reversal
float32 sys_id_servo_swash1     # requested SYS_ID swash servo 1 component before clipping/output reversal
float32 sys_id_servo_swash2     # requested SYS_ID swash servo 2 component before clipping/output reversal
float32 sys_id_servo_swash3     # requested SYS_ID swash servo 3 component before clipping/output reversal

float32 requested_servo_tail    # pure_servo_tail + sys_id_servo_tail before clipping/output reversal
float32 requested_servo_swash0  # pure_servo_swash0 + sys_id_servo_swash0 before clipping/output reversal
float32 requested_servo_swash1  # pure_servo_swash1 + sys_id_servo_swash1 before clipping/output reversal
float32 requested_servo_swash2  # pure_servo_swash2 + sys_id_servo_swash2 before clipping/output reversal
float32 requested_servo_swash3  # pure_servo_swash3 + sys_id_servo_swash3 before clipping/output reversal

float32 servo_tail              # actuator_servos tail command before PWM mapping/output reversal
float32 servo_swash0            # actuator_servos swash servo 0 command before PWM mapping/output reversal
float32 servo_swash1            # actuator_servos swash servo 1 command before PWM mapping/output reversal
float32 servo_swash2            # actuator_servos swash servo 2 command before PWM mapping/output reversal
float32 servo_swash3            # actuator_servos swash servo 3 command before PWM mapping/output reversal

float32[8] pure_servo           # requested actuator_servos command from controller path before SYS_ID
float32[8] sys_id_servo         # requested SYS_ID actuator_servos component per servo channel
float32[8] requested_servo      # pure_servo + sys_id_servo before clipping/output reversal
