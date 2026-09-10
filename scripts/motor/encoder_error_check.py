import berkeley_humanoid_lite_lowlevel.recoil as recoil

bus = recoil.Bus(channel="can0", bitrate=1000000)
error = bus._read_parameter_u32(7, recoil.Parameter.ERROR)  # 7 자리에 모터 ID 입력
print(f"error: {bin(error)}")
bus.stop()