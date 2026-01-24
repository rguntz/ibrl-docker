import pyrealsense2 as rs
ctx = rs.context()
for dev in ctx.query_devices():
    print(dev.get_info(rs.camera_info.serial_number))