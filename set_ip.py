import depthai as dai

STATIC_IP = "10.1.1.210"
NETMASK   = "255.255.255.0"
GATEWAY   = "10.1.1.1"   # ここは使っているネットワークに合わせる

ok, info = dai.DeviceBootloader.getFirstAvailableDevice()
if not ok:
    raise RuntimeError("Bootloader device not found")

bootloader = dai.DeviceBootloader(info)
config = bootloader.readConfig()
config.setStaticIPv4(STATIC_IP, NETMASK, GATEWAY)
bootloader.flashConfig(config)

print("Flashed static IP:", STATIC_IP)
