from pymavlink import mavutil
import pymavlink.dialects.v20.all as dialect
from time import sleep

# Hubungkan ke drone melalui serial
# port = "/dev/ttyACM0"  # Pastikan ini sesuai dengan port yang digunakan
baudrate = 57600  # Sesuaikan dengan baudrate dari flight controller


port = "udp:127.0.0.1:14550"
master = mavutil.mavlink_connection(port, baud=baudrate)

# Tunggu heartbeat dari flight controller
print("Menunggu heartbeat dari drone...")
master.wait_heartbeat()
print(f"Heartbeat diterima dari sistem {master.target_system}, komponen {master.target_component}")

# create request message command
request_message_command = dialect.MAVLink_command_long_message(target_system=master.target_system,
                                                               target_component=master.target_component,
                                                               command=dialect.MAV_CMD_REQUEST_MESSAGE,
                                                               confirmation=0,
                                                               param1=dialect.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                                                               param2=0,
                                                               param3=0,
                                                               param4=0,
                                                               param5=0,
                                                               param6=0,
                                                               param7=0)



# Loop untuk membaca GLOBAL_POSITION_INT
while True:

    # send command to the vehicle
    master.mav.send(request_message_command)
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        alt_msl = msg.alt / 1000.0  # Ketinggian di atas Mean Sea Level (MSL) dalam meter
        alt_relative = msg.relative_alt / 1000.0  # Ketinggian relatif dari home position dalam meter
        print(f"Ketinggian MSL: {alt_msl:.2f} m, Ketinggian Relatif: {alt_relative:.2f} m")
        sleep(1)
