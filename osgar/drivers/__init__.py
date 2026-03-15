# dictionary of all available drivers
all_drivers = dict(
    gps="osgar.drivers.gps:GPS",
    imu="osgar.drivers.imu:IMU",
    serial="osgar.drivers.logserial:LogSerial",
    can="osgar.drivers.canserial:CANSerial",
    tcp="osgar.drivers.logsocket:LogTCPStaticIP",
    tcpdynamic="osgar.drivers.logsocket:LogTCPDynamicIP",
    tcpserver="osgar.drivers.logsocket:LogTCPServer",
    udp="osgar.drivers.logsocket:LogUDP",
    http="osgar.drivers.logsocket:LogHTTP",
    lidar="osgar.drivers.sicklidar:SICKLidar",
    usb="osgar.drivers.logusb:LogUSB",
    replay="osgar.drivers.replay:ReplayDriver",
    lordimu="osgar.drivers.lord_imu:LordIMU",
    pcan="osgar.drivers.pcan:PeakCAN",
    usbcam="osgar.drivers.usbcam:UsbCam",
    rosmsg="osgar.drivers.rosmsg:ROSMsgParser",
    lora="osgar.drivers.lora:LoRa",
    vesc="osgar.drivers.vesc:MotorDriverVESC",
    opencv="osgar.drivers.opencv:LogOpenCVCamera",
    zeromq="osgar.drivers.logzeromq:LogZeroMQ",
    timer="osgar.drivers.timer:Timer",
    realsense="osgar.drivers.realsense:RealSense",
    realsense_multicam="osgar.drivers.realsense:Multicam",
    velodyne="osgar.drivers.velodyne:Velodyne",
    systemmonitor="osgar.drivers.system_monitor:SystemMonitor",
    resize="osgar.drivers.resize:Resize",
    pozyx="osgar.drivers.pozyx:Pozyx",
    rtk_filter="osgar.drivers.rtk_filter:RTKFilter",
    vanjee="osgar.drivers.vanjee:VanJeeLidar",
)


from osgar.lib import create_load_tests
load_tests = create_load_tests(__file__)
