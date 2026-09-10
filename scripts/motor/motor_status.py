import berkeley_humanoid_lite_lowlevel.recoil as recoil
import time

bus = recoil.Bus(channel="can0", bitrate=1000000)

# 작은 토크 명령
bus.write_torque_limit(9, 0.1)
bus.set_mode(9, recoil.Mode.TORQUE)

# 양의 토크 가했을 때 pos 변화 확인
bus._write_parameter_f32(9, recoil.Parameter.POSITION_CONTROLLER_TORQUE_TARGET, 0.05)

time.sleep(0.5)

pos = bus._read_parameter_f32(9, recoil.Parameter.ENCODER_POSITION)
print(f"pos: {pos:.4f}")  # 증가했으면 정상, 감소했으면 방향 불일치

bus.set_mode(9, recoil.Mode.IDLE)
bus.stop()