uint64 timestamp                # time since system start (microseconds)

uint8 MODE_DISABLED = 0
uint8 MODE_3211 = 1
uint8 MODE_SWEEP = 2

uint8 AXIS_ROLL = 0
uint8 AXIS_PITCH = 1
uint8 AXIS_YAW = 2

uint8 mode                      # SYS_ID_EN
uint8 axis                      # SYS_ID_AXIS

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
