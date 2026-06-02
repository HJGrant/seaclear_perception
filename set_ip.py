import depthai as dai

STATIC_IP = "192.168.2.109"
NETMASK   = "255.255.255.0"
GATEWAY   = ""   # ここは使っているネットワークに合わせる

ok, info = dai.DeviceBootloader.getFirstAvailableDevice()
if not ok:
    raise RuntimeError("Bootloader device not found")

bootloader = dai.DeviceBootloader(info)
config = bootloader.readConfig()
config.setStaticIPv4(STATIC_IP, NETMASK, GATEWAY)
bootloader.flashConfig(config)

print("Flashed static IP:", STATIC_IP)
