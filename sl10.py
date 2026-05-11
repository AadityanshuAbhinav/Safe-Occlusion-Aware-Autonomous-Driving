import can
import os
import time
import math
import struct
import threading

# =============================================================================
#  CONFIGURATION
# =============================================================================
CAN_CHANNEL     = 'can1' if os.path.exists('/sys/class/net/can1') else 'can0'
TARGET_DIST_M   = 10.0     # Target distance in meters
SPEED_1KPH      = 0x1C     # 28 counts ≈ 1 km/h
DRIVE_BYTE0     = 0x11     # Enable, Speed Ctrl Mode, D Gear
BRAKE_60PCT_B1  = 0x58
BRAKE_60PCT_B2  = 0x02

# Chassis Constants
WHEELBASE_M     = 1.9
STEER_MODE_FRONT_ACKERMAN = 0

# CAN IDs
ID_DRIVE_TX = 0x130
ID_BRAKE_TX = 0x131
ID_STEER_TX = 0x132

ID_DRIVE_FB = 0x530
ID_STEER_FB = 0x532

# =============================================================================
#  CAN DECODING HELPERS
# =============================================================================
def s16_le(data, byte_offset):
    return struct.unpack_from('<h', data, byte_offset)[0]

def u8(data, byte_offset):
    return data[byte_offset]

def bits(byte_val, start_bit, length):
    return (byte_val >> start_bit) & ((1 << length) - 1)

# =============================================================================
#  THREAD-SAFE ODOMETRY INTEGRATOR
# =============================================================================
def compute_motion(speed_mps, steer_front_deg, steer_rear_deg, steer_mode, dt):
    """Kinematic model for Hooke Chassis."""
    L = WHEELBASE_M
    sf = math.radians(steer_front_deg)
    sr = math.radians(steer_rear_deg)
    v = speed_mps

    # Default to Front Ackerman for standard straight-line driving
    if steer_mode == STEER_MODE_FRONT_ACKERMAN:
        ds_fwd = v * dt
        dtheta = (v * math.tan(sf) / L * dt) if abs(sf) > 1e-6 else 0.0
    else:
        # Fallback simplified model for other modes
        ds_fwd = v * dt
        dtheta = 0.0

    return ds_fwd, dtheta

class ThreadSafeOdometry:
    def __init__(self):
        self.lock = threading.Lock()
        self.distance_m = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta_rad = math.radians(90.0)
        
        self.steer_front_deg = 0.0
        self.steer_rear_deg = 0.0
        self.steer_mode = STEER_MODE_FRONT_ACKERMAN
        
        self._last_ts = None
        self.running = True

    def update_steer(self, front_deg, rear_deg, mode):
        with self.lock:
            self.steer_front_deg = front_deg
            self.steer_rear_deg = rear_deg
            self.steer_mode = mode

    def update_speed(self, speed_mps, ts):
        with self.lock:
            if self._last_ts is not None:
                dt = ts - self._last_ts
                if 0 < dt <= 0.5:
                    ds_fwd, dtheta = compute_motion(
                        speed_mps, self.steer_front_deg, 
                        self.steer_rear_deg, self.steer_mode, dt
                    )
                    
                    theta_mid = self.theta_rad + dtheta / 2.0
                    self.x += ds_fwd * math.cos(theta_mid)
                    self.y += ds_fwd * math.sin(theta_mid)
                    self.theta_rad += dtheta
                    
                    self.distance_m += abs(ds_fwd)
                    
            self._last_ts = ts

# =============================================================================
#  CAN RX/TX LOGIC
# =============================================================================
def can_rx_thread(bus, odom):
    """Background thread integrating true feedback speed/steer into distance."""
    while odom.running:
        msg = bus.recv(timeout=0.1)
        if msg is None:
            continue
            
        now = time.time()
        data = bytes(msg.data)
        
        if msg.arbitration_id == ID_DRIVE_FB:
            speed_mps = s16_le(data, 1) * 0.01
            odom.update_speed(speed_mps, now)
            
        elif msg.arbitration_id == ID_STEER_FB:
            mode = bits(u8(data, 0), 4, 4)
            front_deg = s16_le(data, 1) / 10.0
            rear_deg = s16_le(data, 3) / 10.0
            odom.update_steer(front_deg, rear_deg, mode)

def build_frame(can_id, payload):
    checksum = 0
    for b in payload[:7]:
        checksum ^= b
    payload.append(checksum)
    return can.Message(arbitration_id=can_id, data=bytearray(payload), is_extended_id=False)

def send_drive_straight(bus, cycle):
    """Drive 1km/h, steer 0 deg, brake released."""
    bus.send(build_frame(ID_DRIVE_TX, [DRIVE_BYTE0, SPEED_1KPH, 0x00, 0x00, 0x01, 0x00, cycle & 0x0F]))
    bus.send(build_frame(ID_BRAKE_TX, [0x01,        0x00,       0x00, 0x02, 0x00, 0x00, cycle & 0x0F]))
    bus.send(build_frame(ID_STEER_TX, [0x01,        0x00,       0x00, 0x00, 0x00, 0x00, cycle & 0x0F]))

def send_stop(bus, cycle):
    """Zero drive, brake 60%."""
    bus.send(build_frame(ID_DRIVE_TX, [0x00, 0x00,           0x00, 0x00, 0x01, 0x00, cycle & 0x0F]))
    bus.send(build_frame(ID_BRAKE_TX, [0x01, BRAKE_60PCT_B1, BRAKE_60PCT_B2, 0x02, 0x00, 0x00, cycle & 0x0F]))
    bus.send(build_frame(ID_STEER_TX, [0x01, 0x00,           0x00, 0x00, 0x00, 0x00, cycle & 0x0F]))

# =============================================================================
#  MAIN CONTROLLER
# =============================================================================
def main():
    print(f"Initializing Closed-Loop Odometry Controller")
    print(f"Target: {TARGET_DIST_M} m  |  Mode: Front Ackerman  |  Speed: 1 km/h")
    print("=" * 70)
    
    input("Press ENTER to authorize motion...\n")
    
    bus = can.interface.Bus(interface='socketcan', channel=CAN_CHANNEL)
    odom = ThreadSafeOdometry()
    cycle = 0
    
    rx_thread = threading.Thread(target=can_rx_thread, args=(bus, odom), daemon=True)
    rx_thread.start()

    try:
        print("[MOVING] Commanding forward trajectory...")
        
        while True:
            # Safely fetch current distance
            with odom.lock:
                current_dist = odom.distance_m
                
            # Check target condition
            if current_dist >= TARGET_DIST_M:
                print(f"\n[TARGET REACHED] Travelled {current_dist:.3f} m. Halting chassis.")
                break

            # Send drive commands at ~50Hz
            send_drive_straight(bus, cycle)
            cycle = (cycle + 1) & 0x0F
            
            print(f"  Progress: {current_dist:5.3f} m / {TARGET_DIST_M} m", end="\r")
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n[ABORT] User triggered manual ESTOP.")

    finally:
        # Guarantee stop state on exit
        print("Applying 60% brake pressure...")
        for _ in range(10): # Send a burst of stop commands to ensure receipt
            send_stop(bus, cycle)
            cycle = (cycle + 1) & 0x0F
            time.sleep(0.02)
            
        odom.running = False
        rx_thread.join(timeout=1.0)
        bus.shutdown()
        print("Done.")

if __name__ == '__main__':
    main()