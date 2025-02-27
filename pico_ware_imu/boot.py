import usb_cdc
import storage

storage.remount("/", readonly=False)

m = storage.getmount("/")
m.label = "imu"

storage.remount("/", readonly=True)

storage.enable_usb_drive()
# Enable the data USB serial device
usb_cdc.enable(console=True, data=True)