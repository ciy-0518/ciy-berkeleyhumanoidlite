import berkeley_humanoid_lite_lowlevel.recoil as recoil

bus = recoil.Bus(channel="can0", bitrate=1000000)

# 현재 위치 기준으로 offset 설정
current_pos = bus._read_parameter_f32(9, recoil.Parameter.ENCODER_POSITION)
bus._write_parameter_f32(9, recoil.Parameter.POSITION_CONTROLLER_POSITION_OFFSET, -current_pos)

print(f"offset 설정: {-current_pos:.4f}")
bus.stop()