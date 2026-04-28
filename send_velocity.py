"""
Example: Using motor_driver.py to control a CANopen motion controller.
Make sure:
  pip install python-can canopen
and your CAN interface (e.g., can0) is up (sudo ip link set can0 up type can bitrate 1000000)
"""

from motor_driver import Motor_Driver
import time

# -------------------------------
# 1. Create and connect to the node
# -------------------------------
node_id = 1
drv = Motor_Driver(
    node_id=node_id,
    eds_path="DS301_profile.eds",  # or your EDS file
    bustype="socketcan",
    channel="can0"
)

print("Connecting to node...")
drv.connect()
print(f"NMT state: {drv.state}")




try:
    drv.set_mode_of_operation(mode = 3)
    print(" position control mode entered")
except Exception as e:
    print(f"Error, Not able to set Mode:\n {e}")

# IQ
try:
    drv.set_iq_gains(kp=0.135, ki=510.790359)
    print("Iq gains set")
except canopen.SdoAbortedError as e:
    print(f"Iq gains SDO abort: 0x{e.code:08X} {e}")
except Exception as e:
    print(f"Iq gains skipped (other error): {e}")

# VELOCITY
try:
    drv.set_velocity_gains(kp=int(7.15*100), ki=int(257.44*100))
    print("Velocity gains set successfully")
except canopen.SdoAbortedError as e:
    print(f"Velocity gains SDO abort: 0x{e.code:08X} {e}")
except Exception as e:
    print(f"Velocity gains skipped (other error): {e}")


# POSITION
try:
    drv.set_position_gains(kp=int(16*100))
    print("Position gains set successfully")
except canopen.SdoAbortedError as e:
    print(f"Position gains SDO abort: 0x{e.code:08X} {e}")
except Exception as e:
    print(f"Position Gains skipped (other error): {e}")


try: 
    # The value is in rps2. Multiply my 100 before sending
    drv.set_accel_decel(accel = 10*100, decel = 10*100)
except Exception as e:
    print("Not able to set acc and dec values due to:")
    print(e)


drv.to_operational()
print("Node switched to OPERATIONAL state.")

drv.set_controlword(0x000F)   # Enable voltage, operation enable
print("Controlword set to 0x000F (drive started).")




try:

    while True:
        drv.set_velocity(10)
        time.sleep(1)
except KeyboardInterrupt:
    drv.set_velocity(0)








