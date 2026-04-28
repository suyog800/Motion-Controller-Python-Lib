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






# -------------------------------
# 2. Initialize and start the node
# -------------------------------
# Some drives require heartbeat before NMT start
try:
    drv.set_heartbeat(2000)  # 1000 ms producer heartbeat (0x1017)
    print("Heartbeat configured.")
except Exception as e:
    print(f"Heartbeat skipped: {e}")


try:
    drv.set_mode_of_operation(mode = 4)
    print(" position control mode entered")
except Exception as e:
    print(f"Error, Not able to set Mode:\n {e}")

# IQ
try:
    drv.set_iq_gains(kp=0.35, ki=211.6935484)
    print("Iq gains set")
except canopen.SdoAbortedError as e:
    print(f"Iq gains SDO abort: 0x{e.code:08X} {e}")
except Exception as e:
    print(f"Iq gains skipped (other error): {e}")

# VELOCITY
try:
    drv.set_velocity_gains(kp=int(1007), ki=int(1309))
    print("Velocity gains set successfully")
except canopen.SdoAbortedError as e:
    print(f"Velocity gains SDO abort: 0x{e.code:08X} {e}")
except Exception as e:
    print(f"Velocity gains skipped (other error): {e}")


# POSITION
try:
    drv.set_position_gains(kp=int(3741))
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

# drv.set_controlword(0x000F)   # Enable voltage, operation enable
# print("Controlword set to 0x000F (drive started).")



# position is in rotations (float) and velocity is in RPM (float)
# drv.set_target_position_and_velocity(position=0, velocity=500.0,force_pdo = True)
# print("moving to zero position.")

# try :
#     while True:
#         state = drv.get_motor_state()
#         if state["new"]:
#             print("New data arrived!")
#             print("Position :", state["position"])
#             print("Velocity :", state["velocity"])
#             print("Iq :", state["iq"])
#             print("Id :", state["id"])
#             print("\n")
#         else:
#             print("No new data\n\n")
#         time.sleep(0.1)  # import time

# except KeyboardInterrupt:
#     drv.disconnect()
#     print("Node disconnected. Example finished.")
#     print("exiting")



# t = 1
# time.sleep(t)

# for i in range(0,10):
#     if (i%2 == 0):
#         print("cycle:", i)
#         drv.set_target_position_and_velocity(position=-20.0,velocity=400,force_pdo = True)
#         print("going to 10")
#         t = 20*60/400
#         time.sleep(t+3)
#     else:
#         drv.set_target_position_and_velocity(position=0,velocity=400,force_pdo = True)
#         print("going to 0")
#         t = 20*60/400
#         time.sleep(t+3)



# for i in range(0,20):
#     drv.set_target_position_and_velocity(position=-1*i,velocity=1000.0,force_pdo = True)
#     print("going to ",(i*-1))
#     time.sleep(2)

# drv.set_target_position_and_velocity(position=0,velocity=1000.0,force_pdo = True)





# drv.set_target_position_and_velocity(position=0.0, velocity=500.0)
# print("Return to zero position.")
# time.sleep(5.0)

# -------------------------------
# 8. Stop and disconnect
# -------------------------------
# drv.set_controlword(0x0000)  # disable operation (safe stop)
# drv.to_preoperational()


