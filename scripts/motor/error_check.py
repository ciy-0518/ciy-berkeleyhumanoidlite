# Motor status check script
# 실행: uv run python motor_status.py -c can0 -i 9

import berkeley_humanoid_lite_lowlevel.recoil as recoil

args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)
dev = args.id

mode  = bus._read_parameter_u32(dev, recoil.Parameter.MODE)
error = bus._read_parameter_u32(dev, recoil.Parameter.ERROR)

print(f"device : {dev}")
print(f"mode   : {hex(mode)}")
print(f"error  : {bin(error)}")

# mode 해석
mode_map = {
    0x00: "DISABLED",
    0x01: "IDLE",
    0x02: "DAMPING",
    0x05: "CALIBRATION",
    0x10: "CURRENT",
    0x11: "TORQUE",
    0x12: "VELOCITY",
    0x13: "POSITION",
    0x20: "VABC_OVERRIDE",
    0x21: "VALPHABETA_OVERRIDE",
    0x22: "VQD_OVERRIDE",
    0x80: "DEBUG",
}
print(f"       → {mode_map.get(mode, 'UNKNOWN')}")

# error 비트 해석
error_map = {
    0:  "GENERAL",
    1:  "ESTOP",
    2:  "INITIALIZATION_ERROR",
    3:  "CALIBRATION_ERROR",
    4:  "POWERSTAGE_ERROR",
    5:  "INVALID_MODE",
    6:  "WATCHDOG_TIMEOUT",
    7:  "OVER_VOLTAGE",
    8:  "OVER_CURRENT",
    9:  "OVER_TEMPERATURE",
    10: "CAN_RX_FAULT",
    11: "CAN_TX_FAULT",
    12: "I2C_FAULT",
    13: "ENCODER_FAULT",
}

if error == 0:
    print("       → NO_ERROR")
else:
    for bit, name in error_map.items():
        if error & (1 << bit):
            print(f"       → {name}")

bus.stop()
