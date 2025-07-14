from pymavlink import mavutil

# Create the connection
# From topside computer
master = mavutil.mavlink_connection('COM7', baud=115200)

while True:
    msg = master.recv_match()

    if not msg:
        continue
    #print(msg)
    if msg.get_type() == 'AHRS2':
        print(f"Roll: {msg.roll},     Pitch: {msg.pitch}, Yaw: {msg.yaw}")
