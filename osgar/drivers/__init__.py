
# dictionary of all available drivers
all_drivers = dict(gps="osgar.drivers.gps:GPS"
    , imu="osgar.drivers.imu:IMU"
    , spider="osgar.drivers.spider:Spider"
    , serial="osgar.drivers.logserial:LogSerial"
    , can="osgar.drivers.canserial:CANSerial"
    , simulator="osgar.drivers.simulator:SpiderSimulator"
    , tcp="osgar.drivers.logsocket:LogTCPStaticIP"
    , tcpdynamic="osgar.drivers.logsocket:LogTCPDynamicIP"
    , tcpserver="osgar.drivers.logsocket:LogTCPServer"
    , udp="osgar.drivers.logsocket:LogUDP"
    , http="osgar.drivers.logsocket:LogHTTP"
    , lidar="osgar.drivers.sicklidar:SICKLidar"
    , eduro="osgar.drivers.eduro:Eduro"
    , cortexpilot="osgar.drivers.cortexpilot:Cortexpilot"
    , usb="osgar.drivers.logusb:LogUSB"
    , replay="osgar.drivers.replay:ReplayDriver"
    , lordimu="osgar.drivers.lord_imu:LordIMU"
    , pcan="osgar.drivers.pcan:PeakCAN"
)

