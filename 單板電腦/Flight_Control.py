#!/usr/bin/env python3
import sys
import signal
import rospy
import serial
import time
from geometry_msgs.msg import Vector3

'''
MSP V1 訊息格式
$M + type + size + command + data + checksum

type -> 消息類型(<, >, !)
command -> 資料類別
size -> 資料長度
data -> 資料內容(小端序)
checksum -> 校驗碼, XOR算法
'''
HEADER = b'$'
MSP1 = b'M'
REQUEST = b'<'
RESPONSE = b'>'
PORT = "/dev/fc"
DISARM = 0
ARM = 1
PREPARE = 2
CONTROL = 3
flag = DISARM

MSP_ACC_CALIBRATION = 205  # Do the calibration
MSP_ACC_TRIM = 240  # Get trim value
MSP_SET_ACC_TRIM = 239  # Order: Pitch、Roll
MSP_SET_RAW_RC = 200  # Order: Roll Pitch Throttle Yaw ...

def setSerial():
    global ser
    ser = serial.Serial()
    ser.port = PORT
    ser.baudrate = 115200
    ser.timeout = 0.01
    ser.writeTimeout = 2
    try:
        ser.open()
    except Exception as error:
        print(error)

    if not ser.isOpen():
        raise ConnectionError("Serial is not open")

def checksum_XOR(*data: 'bytes') -> bytes:
    checksum = 0
    for byte in data:
        if len(byte) > 1:  # byte array
            for b in byte:
                checksum ^= b
        elif byte:
            checksum ^= byte[0]  # byte[0] -> 8 bits to int

    return bytes([checksum])

def sendData(cmd: int, *data):
    data_byte = b''
    if data:
        for d in data:
            if d:
                data_byte += d.to_bytes(2, 'little')

    cmd = bytes([cmd])
    size = bytes([len(data_byte)])
    message = HEADER + MSP1 + REQUEST + size + cmd + data_byte
    message += checksum_XOR(size, cmd, data_byte)
    try:
        ser.write(message)
    except Exception as error:
        print(error)

def getData(cmd: int):
    sendData(cmd, [])
    t0 = time.time()
    timer = 0
    while timer < 2:
        if ser.read(1) == HEADER and ser.read(2) == MSP1+RESPONSE:
            data_describe = ser.read(2)
            data_size = data_describe[0]
            data = ser.read(data_size)
            return data
        timer = time.time() - t0

    print("Failed to get data")

def setting():
    global flag
    print("#####")
    print("ca = calibrate acc, trim = trim acc")
    print("Type \"q\" to exit setting mode")
    print("Type \"exit\" to stop program")
    while True:
        command = input("\nfunction: ")
        if command in ['q', 'Q']:
            flag = ARM
            print("exiting setting mode")
            break

        elif command == "exit":
            rospy.signal_shutdown("stop program")
            sys.exit("stop program")
        
        elif command == "ca":
            sendData(MSP_ACC_CALIBRATION, [])
            print("calibrating accelerator")

        elif command == "trim":
            data = getData(MSP_ACC_TRIM)
            pitch_trim = int.from_bytes(data[0:2], 'little')
            roll_trim = int.from_bytes(data[2:4], 'little')
            print("roll:", roll_trim)
            print("pitch:", pitch_trim)
            confirm = input("Set new trim value? [Y/n]: ")
            if confirm in ['N', 'n'] or confirm not in ['Y', 'y']:
                continue

            print("For each direction, type \"remain\" to keep its value\n")
            roll_trim_buff = input("roll: ")
            if roll_trim_buff.isdigit():
                roll_trim = int(roll_trim_buff)
            elif roll_trim_buff != "remain":
                print("error")
                continue

            pitch_trim_buff = input("pitch: ")
            if pitch_trim_buff.isdigit():
                pitch_trim = int(pitch_trim_buff)
            elif pitch_trim_buff != "remain":
                print("error")
                continue

            sendData(MSP_SET_ACC_TRIM, pitch_trim, roll_trim)
            for i in range(10):
                sendData(MSP_SET_RAW_RC, 2000, 1000, 1000, 1000)
                time.sleep(0.1)
            print("done")

def arm():
    global flag
    print("arming")
    for i in range(10):
        sendData(MSP_SET_RAW_RC, 1500, 1500, 1000, 2000)
        time.sleep(0.1)

    flag = PREPARE

def prepare():  # 讓槳轉一會
    global flag
    for i in range(50):
        sendData(MSP_SET_RAW_RC, 1500, 1500, 1200, 1500)
        time.sleep(0.02)
    
    flag = CONTROL


def disarm():
    print("disarming")
    try:
        for i in range(10):
            sendData(MSP_SET_RAW_RC, 1500, 1500, 1000, 1000)
            time.sleep(0.1)
    except Exception as e:
        print(e)
        print("Connection aborted")
        print("Disarm failed!!!")
        sys.exit()

def signal_handler(signum, frame):
    if signum == signal.SIGINT.value:  # Ctrl-C
        if flag == DISARM:
            ser.close()
        else:
            disarm()
            ser.close()
            rospy.signal_shutdown("stop program")
            sys.exit("stop program")

def display(value_width: 'int | tuple', name: 'tuple[str]', data: 'tuple[int]'):
    if len(name) != len(data):
        raise ValueError("Inconsistent amount of input data")
    if isinstance(value_width, int):
#        print('\r', end="")
        for n, d in zip(name, data):
            format_setting = "{:" + str(value_width) + '}'
            print(n + ": " + format_setting.format(d), end='\t')
        print("")

pitch = 1500
roll = 1500
throttle = 1500

def callback(msg):
    global pitch,roll,throttle
    pitch = int(msg.y)
    roll = int(msg.x)
    throttle = int(msg.z)
    display(4, ["pitch", "roll", "throttle"], [pitch, roll, throttle])

    if flag == ARM:
       arm()
    elif flag == PREPARE:
         prepare()
#    elif flag == CONTROL:
#         sendData(MSP_SET_RAW_RC, roll, pitch, throttle, 1500)


if __name__ == "__main__":
    rospy.init_node("fc")
    rospy.Subscriber("/throttle", Vector3, callback, queue_size=5)
    setSerial()
    signal.signal(signal.SIGINT, signal_handler)
    setting()

while not rospy.is_shutdown():
    if flag == CONTROL:
       sendData(MSP_SET_RAW_RC, roll, pitch, throttle, 1500)
       display(4, ["pitch", "roll", "throttle"], [pitch, roll, throttle])
    #   rospy.signal_shutdown(" ")


#    sendData(MSP_SET_RAW_RC, roll, pitch, throttle, 1500)
#    display(4, ["pitch", "roll", "throttle"], [pitch, roll, throttle])
    print("#########")
    time.sleep(0.1)


disarm()
sys.exit("Stop program")


