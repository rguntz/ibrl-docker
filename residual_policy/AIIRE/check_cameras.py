import pyrealsense2 as rs

def list_realsense_cameras():
    ctx = rs.context()
    devices = ctx.query_devices()
    
    if len(devices) == 0:
        print("No RealSense cameras detected.")
        return []
    
    serials = []
    print("Detected RealSense cameras:")
    for i, dev in enumerate(devices):
        serial = dev.get_info(rs.camera_info.serial_number)
        name = dev.get_info(rs.camera_info.name)
        print(f"  [{i}] Name: {name}, Serial: {serial}")
        serials.append(serial)
    
    return serials

if __name__ == "__main__":
    serials = list_realsense_cameras()
    print("\nSerial numbers of connected cameras:", serials)
