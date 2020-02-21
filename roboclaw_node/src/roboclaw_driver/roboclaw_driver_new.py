import random
import serial
import time
import threading

_trystimeout = 3


# Command Enums

class Cmd:
    M1FORWARD = 0
    M1BACKWARD = 1
    SETMINMB = 2
    SETMAXMB = 3
    M2FORWARD = 4
    M2BACKWARD = 5
    M17BIT = 6
    M27BIT = 7
    MIXEDFORWARD = 8
    MIXEDBACKWARD = 9
    MIXEDRIGHT = 10
    MIXEDLEFT = 11
    MIXEDFB = 12
    MIXEDLR = 13
    GETM1ENC = 16
    GETM2ENC = 17
    GETM1SPEED = 18
    GETM2SPEED = 19
    RESETENC = 20
    GETVERSION = 21
    SETM1ENCCOUNT = 22
    SETM2ENCCOUNT = 23
    GETMBATT = 24
    GETLBATT = 25
    SETMINLB = 26
    SETMAXLB = 27
    SETM1PID = 28
    SETM2PID = 29
    GETM1ISPEED = 30
    GETM2ISPEED = 31
    M1DUTY = 32
    M2DUTY = 33
    MIXEDDUTY = 34
    M1SPEED = 35
    M2SPEED = 36
    MIXEDSPEED = 37
    M1SPEEDACCEL = 38
    M2SPEEDACCEL = 39
    MIXEDSPEEDACCEL = 40
    M1SPEEDDIST = 41
    M2SPEEDDIST = 42
    MIXEDSPEEDDIST = 43
    M1SPEEDACCELDIST = 44
    M2SPEEDACCELDIST = 45
    MIXEDSPEEDACCELDIST = 46
    GETBUFFERS = 47
    GETPWMS = 48
    GETCURRENTS = 49
    MIXEDSPEED2ACCEL = 50
    MIXEDSPEED2ACCELDIST = 51
    M1DUTYACCEL = 52
    M2DUTYACCEL = 53
    MIXEDDUTYACCEL = 54
    READM1PID = 55
    READM2PID = 56
    SETMAINVOLTAGES = 57
    SETLOGICVOLTAGES = 58
    GETMINMAXMAINVOLTAGES = 59
    GETMINMAXLOGICVOLTAGES = 60
    SETM1POSPID = 61
    SETM2POSPID = 62
    READM1POSPID = 63
    READM2POSPID = 64
    M1SPEEDACCELDECCELPOS = 65
    M2SPEEDACCELDECCELPOS = 66
    MIXEDSPEEDACCELDECCELPOS = 67
    SETM1DEFAULTACCEL = 68
    SETM2DEFAULTACCEL = 69
    SETPINFUNCTIONS = 74
    GETPINFUNCTIONS = 75
    SETDEADBAND = 76
    GETDEADBAND = 77
    RESTOREDEFAULTS = 80
    GETTEMP = 82
    GETTEMP2 = 83
    GETERROR = 90
    GETENCODERMODE = 91
    SETM1ENCODERMODE = 92
    SETM2ENCODERMODE = 93
    WRITENVM = 94
    READNVM = 95
    SETCONFIG = 98
    GETCONFIG = 99
    SETM1MAXCURRENT = 133
    SETM2MAXCURRENT = 134
    GETM1MAXCURRENT = 135
    GETM2MAXCURRENT = 136
    SETPWMMODE = 148
    GETPWMMODE = 149
    FLAGBOOTLOADER = 255


# Private Functions

def crc_clear():
    global _crc
    _crc = 0
    return


def crc_update(data):
    global _crc
    _crc ^= data << 8
    for bit in range(0, 8):
        if (_crc & 0x8000) == 0x8000:
            _crc = ((_crc << 1) ^ 0x1021)
        else:
            _crc <<= 1
    return


def _sendcommand(address, command,port):
    crc_clear()
    crc_update(address)
    port.write(chr(address))
    crc_update(command)
    port.write(chr(command))
    return


def _readchecksumword(port):
    data = port.read(2)
    if len(data) == 2:
        crc = (ord(data[0]) << 8) | ord(data[1])
        return 1, crc
    return 0, 0


def _readbyte(port):
    data = port.read(1)
    if len(data):
        val = ord(data)
        crc_update(val)
        return 1, val
    return 0, 0


def _readword(port):
    val1 = _readbyte(port)
    if val1[0]:
        val2 = _readbyte(port)
        if val2[0]:
            return 1, val1[1] << 8 | val2[1]
    return 0, 0


def _readlong(port):
    val1 = _readbyte(port)
    if val1[0]:
        val2 = _readbyte(port)
        if val2[0]:
            val3 = _readbyte(port)
            if val3[0]:
                val4 = _readbyte(port)
                if val4[0]:
                    return 1, val1[1] << 24 | val2[1] << 16 | val3[1] << 8 | val4[1]
    return 0, 0


def _readslong(port):
    val = _readlong(port)
    if val[0]:
        if val[1] & 0x80000000:
            return val[0], val[1] - 0x100000000
        return val[0], val[1]
    return 0, 0


def _writebyte(val,port):
    crc_update(val & 0xFF)
    port.write(chr(val & 0xFF))


def _writesbyte(val,port):
    _writebyte(val,port)


def _writeword(val,port):
    _writebyte((val >> 8) & 0xFF,port)
    _writebyte(val & 0xFF,port)


def _writesword(val,port):
    _writeword(val,port)


def _writelong(val,port):
    _writebyte((val >> 24) & 0xFF,port)
    _writebyte((val >> 16) & 0xFF,port)
    _writebyte((val >> 8) & 0xFF,port)
    _writebyte(val & 0xFF,port)


def _writeslong(val,port):
    _writelong(val,port)


def _read1(address, cmd,port):
    global _crc
    trys = _trystimeout
    while 1:
        port.flushInput()
        _sendcommand(address, cmd,port)
        val1 = _readbyte(port)
        if val1[0]:
            crc = _readchecksumword(port)
            if crc[0]:
                if _crc & 0xFFFF != crc[1] & 0xFFFF:
                    return 0, 0
                return 1, val1[1]
        trys -= 1
        if trys == 0:
            break
    return 0, 0


def _read2(address, cmd,port):
    global _crc
    trys = _trystimeout
    while 1:
        port.flushInput()
        _sendcommand(address, cmd,port)
        val1 = _readword(port)
        if val1[0]:
            crc = _readchecksumword(port)
            if crc[0]:
                if _crc & 0xFFFF != crc[1] & 0xFFFF:
                    return 0, 0
                return 1, val1[1]
        trys -= 1
        if trys == 0:
            break
    return 0, 0


def _read4(address, cmd,port):
    global _crc
    trys = _trystimeout
    while 1:
        port.flushInput()
        _sendcommand(address, cmd,port)
        val1 = _readlong(port)
        if val1[0]:
            crc = _readchecksumword(port)
            if crc[0]:
                if _crc & 0xFFFF != crc[1] & 0xFFFF:
                    return 0, 0
                return 1, val1[1]
        trys -= 1
        if trys == 0:
            break
    return 0, 0


def _read4_1(address, cmd,port):
    global _crc
    trys = _trystimeout
    while 1:
        port.flushInput()
        _sendcommand(address, cmd,port)
        val1 = _readslong(port)
        if val1[0]:
            val2 = _readbyte(port)
            if val2[0]:
                crc = _readchecksumword(port)
                if crc[0]:
                    if _crc & 0xFFFF != crc[1] & 0xFFFF:
                        return 0, 0
                    return 1, val1[1], val2[1]
        trys -= 1
        if trys == 0:
            break
    return 0, 0


def _read_n(address, cmd, args,port):
    global _crc
    trys = _trystimeout
    while 1:
        port.flushInput()
        trys -= 1
        if trys == 0:
            break
        failed = False
        _sendcommand(address, cmd,port)
        data = [1, ]
        for i in range(0, args):
            val = _readlong(port)
            if val[0] == 0:
                failed = True
                break
            data.append(val[1])
        if failed:
            continue
        crc = _readchecksumword(port)
        if crc[0]:
            if _crc & 0xFFFF == crc[1] & 0xFFFF:
                return data
    return 0, 0, 0, 0, 0


def _writechecksum(port):
    global _crc
    _writeword(_crc & 0xFFFF,port)
    val = _readbyte(port)
    if val[0]:
        return True
    return False


def _write0(address, cmd,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _write1(address, cmd, val,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writebyte(val,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _write111(address, cmd, val1, val2,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writebyte(val1,port)
        _writebyte(val2,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _write111(address, cmd, val1, val2, val3,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writebyte(val1,port)
        _writebyte(val2,port)
        _writebyte(val3,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _write2(address, cmd, val,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writeword(val,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _writeS2(address, cmd, val,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writesword(val,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _write22(address, cmd, val1, val2,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writeword(val1,port)
        _writeword(val2,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _writeS22(address, cmd, val1, val2,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writesword(val1,port)
        _writeword(val2,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _writeS2S2(address, cmd, val1, val2,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writesword(val1,port)
        _writesword(val2,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _writeS24(address, cmd, val1, val2,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writesword(val1,port)
        _writelong(val2,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _writeS24S24(address, cmd, val1, val2, val3, val4,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writesword(val1,port)
        _writelong(val2,port)
        _writesword(val3,port)
        _writelong(val4,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _write4(address, cmd, val,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writelong(val,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _writeS4(address, cmd, val,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writeslong(val,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _write44(address, cmd, val1, val2,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writelong(val1,port)
        _writelong(val2,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _write4S4(address, cmd, val1, val2,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writelong(val1,port)
        _writeslong(val2,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _writeS4S4(address, cmd, val1, val2,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writeslong(val1,port)
        _writeslong(val2,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _write441(address, cmd, val1, val2, val3,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writelong(val1,port)
        _writelong(val2,port)
        _writebyte(val3,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _writeS441(address, cmd, val1, val2, val3,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writeslong(val1,port)
        _writelong(val2,port)
        _writebyte(val3,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _write4S4S4(address, cmd, val1, val2, val3,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writelong(val1,port)
        _writeslong(val2,port)
        _writeslong(val3,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _write4S441(address, cmd, val1, val2, val3, val4,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writelong(val1,port)
        _writeslong(val2,port)
        _writelong(val3,port)
        _writebyte(val4,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _write4444(address, cmd, val1, val2, val3, val4,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writelong(val1,port)
        _writelong(val2,port)
        _writelong(val3,port)
        _writelong(val4,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _write4S44S4(address, cmd, val1, val2, val3, val4,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writelong(val1,port)
        _writeslong(val2,port)
        _writelong(val3,port)
        _writeslong(val4,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _write44441(address, cmd, val1, val2, val3, val4, val5,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writelong(val1,port)
        _writelong(val2,port)
        _writelong(val3,port)
        _writelong(val4,port)
        _writebyte(val5,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _writeS44S441(address, cmd, val1, val2, val3, val4, val5,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writeslong(val1,port)
        _writelong(val2,port)
        _writeslong(val3,port)
        _writelong(val4,port)
        _writebyte(val5,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _write4S44S441(address, cmd, val1, val2, val3, val4, val5, val6,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writelong(val1,port)
        _writeslong(val2,port)
        _writelong(val3,port)
        _writeslong(val4,port)
        _writelong(val5,port)
        _writebyte(val6,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _write4S444S441(address, cmd, val1, val2, val3, val4, val5, val6, val7,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writelong(val1,port)
        _writeslong(val2,port)
        _writelong(val3,port)
        _writelong(val4,port)
        _writeslong(val5,port)
        _writelong(val6,port)
        _writebyte(val7,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _write4444444(address, cmd, val1, val2, val3, val4, val5, val6, val7,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writelong(val1,port)
        _writelong(val2,port)
        _writelong(val3,port)
        _writelong(val4,port)
        _writelong(val5,port)
        _writelong(val6,port)
        _writelong(val7,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


def _write444444441(address, cmd, val1, val2, val3, val4, val5, val6, val7, val8, val9,port):
    trys = _trystimeout
    while trys:
        _sendcommand(address, cmd,port)
        _writelong(val1,port)
        _writelong(val2,port)
        _writelong(val3,port)
        _writelong(val4,port)
        _writelong(val5,port)
        _writelong(val6,port)
        _writelong(val7,port)
        _writelong(val8,port)
        _writebyte(val9,port)
        if _writechecksum(port):
            return True
        trys -= 1
    return False


# User accessible functions

def SendRandomData(cnt,port):
    for i in range(0, cnt):
        byte = random.getrandbits(8)
        port.write(chr(byte))
    return


def ForwardM1(address, val,port):
    return _write1(address, Cmd.M1FORWARD, val,port)


def BackwardM1(address, val,port):
    return _write1(address, Cmd.M1BACKWARD, val,port)


def SetMinVoltageMainBattery(address, val,port):
    return _write1(address, Cmd.SETMINMB, val,port)


def SetMaxVoltageMainBattery(address, val,port):
    return _write1(address, Cmd.SETMAXMB, val,port)


def ForwardM2(address, val,port):
    return _write1(address, Cmd.M2FORWARD, val,port)


def BackwardM2(address, val,port):
    return _write1(address, Cmd.M2BACKWARD, val,port)


def ForwardBackwardM1(address, val,port):
    return _write1(address, Cmd.M17BIT, val,port)


def ForwardBackwardM2(address, val,port):
    return _write1(address, Cmd.M27BIT, val,port)


def ForwardMixed(address, val,port):
    return _write1(address, Cmd.MIXEDFORWARD, val,port)


def BackwardMixed(address, val,port):
    return _write1(address, Cmd.MIXEDBACKWARD, val,port)


def TurnRightMixed(address, val,port):
    return _write1(address, Cmd.MIXEDRIGHT, val,port)


def TurnLeftMixed(address, val,port):
    return _write1(address, Cmd.MIXEDLEFT, val,port)


def ForwardBackwardMixed(address, val,port):
    return _write1(address, Cmd.MIXEDFB, val,port)


def LeftRightMixed(address, val,port):
    return _write1(address, Cmd.MIXEDLR, val,port)


def ReadEncM1(address,port):
    return _read4_1(address, Cmd.GETM1ENC,port)


def ReadEncM2(address,port):
    return _read4_1(address, Cmd.GETM2ENC,port)


def ReadSpeedM1(address,port):
    return _read4_1(address, Cmd.GETM1SPEED,port)


def ReadSpeedM2(address,port):
    return _read4_1(address, Cmd.GETM2SPEED,port)


def ResetEncoders(address,port):
    return _write0(address, Cmd.RESETENC,port)


def ReadVersion(address,port):
    global _crc
    trys = _trystimeout
    while 1:
        port.flushInput()
        _sendcommand(address, Cmd.GETVERSION,port)
        str = ""
        passed = True
        for i in range(0, 48):
            port.flushInput()
            _sendcommand(address, Cmd.GETVERSION,port)
            data = port.read(1)
            
            if len(data):
                val = ord(data)
                crc_update(val)
                if val == 0:
                    break
                str += data[0]
            else:
                passed = False
                break
        if passed:
            crc = _readchecksumword(port)
            if crc[0]:
                if _crc & 0xFFFF == crc[1] & 0xFFFF:
                    return 1, str
                else:
                    time.sleep(0.01)
        trys -= 1
        if trys == 0:
            break
    return 0, 0


def SetEncM1(address, cnt,port):
    return _write4(address, Cmd.SETM1ENCCOUNT, cnt,port)


def SetEncM2(address, cnt,port):
    return _write4(address, Cmd.SETM2ENCCOUNT, cnt,port)


def ReadMainBatteryVoltage(address,port):
    return _read2(address, Cmd.GETMBATT,port)


def ReadLogicBatteryVoltage(address,port):
    return _read2(address, Cmd.GETLBATT,port)


def SetMinVoltageLogicBattery(address, val,port):
    return _write1(address, Cmd.SETMINLB, val,port)


def SetMaxVoltageLogicBattery(address, val,port):
    return _write1(address, Cmd.SETMAXLB, val,port)


def SetM1VelocityPID(address, p, i, d, qpps,port):
    return _write4444(address, Cmd.SETM1PID, long(d * 65536), long(p * 65536), long(i * 65536), qpps,port)


def SetM2VelocityPID(address, p, i, d, qpps,port):
    return _write4444(address, Cmd.SETM2PID, long(d * 65536), long(p * 65536), long(i * 65536), qpps,port)


def ReadISpeedM1(address,port):
    return _read4_1(address, Cmd.GETM1ISPEED,port)


def ReadISpeedM2(address,port):
    return _read4_1(address, Cmd.GETM2ISPEED,port)


def DutyM1(address, val,port):
    return _simplFunctionS2(address, Cmd.M1DUTY, val,port)


def DutyM2(address, val,port):
    return _simplFunctionS2(address, Cmd.M2DUTY, val,port)


def DutyM1M2(address, m1, m2,port):
    return _writeS2S2(address, Cmd.MIXEDDUTY, m1, m2,port)


def SpeedM1(address, val,port):
    return _writeS4(address, Cmd.M1SPEED, val,port)


def SpeedM2(address, val,port):
    return _writeS4(address, Cmd.M2SPEED, val,port)


def SpeedM1M2(address, m1, m2,port):
    return _writeS4S4(address, Cmd.MIXEDSPEED, m1, m2,port)


def SpeedAccelM1(address, accel, speed,port):
    return _write4S4(address, Cmd.M1SPEEDACCEL, accel, speed,port)


def SpeedAccelM2(address, accel, speed,port):
    return _write4S4(address, Cmd.M2SPEEDACCEL, accel, speed,port)


def SpeedAccelM1M2(address, accel, speed1, speed2,port):
    return _write4S4S4(address, Cmd.M1SPEEDACCEL, accel, speed1, speed2,port)


def SpeedDistanceM1(address, speed, distance, buffer,port):
    return _writeS441(address, Cmd.M1SPEEDDIST, speed, distance, buffer,port)


def SpeedDistanceM2(address, speed, distance, buffer,port):
    return _writeS441(address, Cmd.M2SPEEDDIST, speed, distance, buffer,port)


def SpeedDistanceM1M2(address, speed1, distance1, speed2, distance2, buffer,port):
    return _writeS44S441(address, Cmd.MIXEDSPEEDDIST, speed1, distance1, speed2, distance2, buffer,port)


def SpeedAccelDistanceM1(address, accel, speed, distance, buffer,port):
    return _write4S441(address, Cmd.M1SPEEDACCELDIST, accel, speed, distance, buffer,port)


def SpeedAccelDistanceM2(address, accel, speed, distance, buffer,port):
    return _write4S441(address, Cmd.M2SPEEDACCELDIST, accel, speed, distance, buffer,port)


def SpeedAccelDistanceM1M2(address, accel, speed1, distance1, speed2, distance2, buffer,port):
    return _write4S44S441(address, Cmd.MIXEDSPEED2ACCELDIST, accel, speed1, distance1, speed2, distance2, buffer,port)


def ReadBuffers(address,port):
    val = _read2(address, Cmd.GETBUFFERS,port)
    if val[0]:
        return 1, val[1] >> 8, val[1] & 0xFF
    return 0, 0, 0


def ReadPWMs(address,port):
    val = _read4(address, Cmd.GETPWMS,port)
    if val[0]:
        pwm1 = val[1] >> 16
        pwm2 = val[1] & 0xFFFF
        if pwm1 & 0x8000:
            pwm1 -= 0x10000
        if pwm2 & 0x8000:
            pwm2 -= 0x10000
        return 1, pwm1, pwm2
    return 0, 0, 0


def ReadCurrents(address,port):
    val = _read4(address, Cmd.GETCURRENTS,port)
    if val[0]:
        cur1 = val[1] >> 16
        cur2 = val[1] & 0xFFFF
        if cur1 & 0x8000:
            cur1 -= 0x10000
        if cur2 & 0x8000:
            cur2 -= 0x10000
        return 1, cur1, cur2
    return 0, 0, 0


def SpeedAccelM1M2_2(address, accel1, speed1, accel2, speed2,port):
    return _write4S44S4(address, Cmd.MIXEDSPEED2ACCEL, accel, speed1, accel2, speed2,port)


def SpeedAccelDistanceM1M2_2(address, accel1, speed1, distance1, accel2, speed2, distance2, buffer,port):
    return _write4S444S441(address, Cmd.MIXEDSPEED2ACCELDIST, accel1, speed1, distance1, accel2, speed2, distance2,
                           buffer,port)


def DutyAccelM1(address, accel, duty,port):
    return _writeS24(address, Cmd.M1DUTYACCEL, duty, accel,port)


def DutyAccelM2(address, accel, duty,port):
    return _writeS24(address, Cmd.M2DUTYACCEL, duty, accel,port)


def DutyAccelM1M2(address, accel1, duty1, accel2, duty2,port):
    return _writeS24S24(Cmd.MIXEDDUTYACCEL, duty1, accel1, duty2, accel2,port)


def ReadM1VelocityPID(address,port):
    data = _read_n(address, Cmd.READM1PID, 4,port)
    if data[0]:
        data[1] /= 65536.0
        data[2] /= 65536.0
        data[3] /= 65536.0
        return data
    return 0, 0, 0, 0, 0


def ReadM2VelocityPID(address,port):
    data = _read_n(address, Cmd.READM2PID, 4,port)
    if data[0]:
        data[1] /= 65536.0
        data[2] /= 65536.0
        data[3] /= 65536.0
        return data
    return 0, 0, 0, 0, 0


def SetMainVoltages(address, min, max,port):
    return _write22(address, Cmd.SETMAINVOLTAGES, min, max,port)


def SetLogicVoltages(address, min, max,port):
    return _write22(address, Cmd.SETLOGICVOLTAGES, min, max,port)


def ReadMinMaxMainVoltages(address,port):
    val = _read4(address, Cmd.GETMINMAXMAINVOLTAGES,port)
    if val[0]:
        min = val[1] >> 16
        max = val[1] & 0xFFFF
        return 1, min, max
    return 0, 0, 0


def ReadMinMaxLogicVoltages(address,port):
    val = _read4(address, Cmd.GETMINMAXLOGICVOLTAGES,port)
    if val[0]:
        min = val[1] >> 16
        max = val[1] & 0xFFFF
        return 1, min, max
    return 0, 0, 0


def SetM1PositionPID(address, kp, ki, kd, kimax, deadzone, min, max,port):
    return _write4444444(address, Cmd.SETM1POSPID, long(kd * 1024), long(kp * 1024), long(ki * 1024), kimax, deadzone,
                         min, max,port)


def SetM2PositionPID(address, kp, ki, kd, kimax, deadzone, min, max,port):
    return _write4444444(address, Cmd.SETM2POSPID, long(kd * 1024), long(kp * 1024), long(ki * 1024), kimax, deadzone,
                         min, max,port)


def ReadM1PositionPID(address,port):
    data = _read_n(address, Cmd.READM1POSPID, 7,port)
    if data[0]:
        data[0] /= 1024.0
        data[1] /= 1024.0
        data[2] /= 1024.0
        return data
    return 0, 0, 0, 0, 0, 0, 0, 0


def ReadM2PositionPID(address,port):
    data = _read_n(address, Cmd.READM2POSPID, 7,port)
    if data[0]:
        data[0] /= 1024.0
        data[1] /= 1024.0
        data[2] /= 1024.0
        return data
    return 0, 0, 0, 0, 0, 0, 0, 0


def SpeedAccelDeccelPositionM1(address, accel, speed, deccel, position, buffer,port):
    return _write44441(address, Cmd.M1SPEEDACCELDECCELPOS, accel, speed, deccel, position, buffer,port)


def SpeedAccelDeccelPositionM2(address, accel, speed, deccel, position, buffer,port):
    return _write44441(address, Cmd.M2SPEEDACCELDECCELPOS, accel, speed, deccel, position, buffer,port)


def SpeedAccelDeccelPositionM1M2(address, accel1, speed1, deccel1, position1, accel2, speed2, deccel2, position2,
                                 buffer,port):
    return _write444444441(address, Cmd.MIXEDSPEEDACCELDECCELPOS, accel1, speed1, deccel1, position1, accel2, speed2,
                           deccel2, position2, buffer,port)


def SetM1DefaultAccel(address, accel,port):
    return _write4(address, Cmd.SETM1DEFAULTACCEL, accel,port)


def SetM2DefaultAccel(address, accel,port):
    return _write4(address, Cmd.SETM2DEFAULTACCEL, accel,port)


def SetPinFunctions(address, S3mode, S4mode, S5mode,port):
    return _write111(address, Cmd.SETPINFUNCTIONS, S3mode, S4mode, S5mode,port)


def ReadPinFunctions(address,port):
    global _crc
    trys = _trystimeout
    while 1:
        _sendcommand(address, Cmd.GETPINFUNCTIONS,port)
        val1 = _readbyte(port)
        if val1[0]:
            val2 = _readbyte(port)
            if val1[0]:
                val3 = _readbyte(port)
                if val1[0]:
                    crc = _readchecksumword(port)
                    if crc[0]:
                        if _crc & 0xFFFF != crc[1] & 0xFFFF:
                            return 0, 0
                        return 1, val1[1], val2[1], val3[1]
        trys -= 1
        if trys == 0:
            break
    return 0, 0


def SetDeadBand(address, min, max,port):
    return _write111(address, Cmd.SETDEADBAND, min, max,port)


def GetDeadBand(address,port):
    val = _read2(address, Cmd.GETDEADBAND,port)
    if val[0]:
        return 1, val[1] >> 8, val[1] & 0xFF
    return 0, 0, 0


# Warning(TTL Serial): Baudrate will change if not already set to 38400.  Communications will be lost
def RestoreDefaults(address,port):
    return _write0(address, Cmd.RESTOREDEFAULTS,port)


def ReadTemp(address,port):
    return _read2(address, Cmd.GETTEMP,port)


def ReadTemp2(address,port):
    return _read2(address, Cmd.GETTEMP2,port)


def ReadError(address,port):
    return _read2(address, Cmd.GETERROR,port)


def ReadEncoderModes(address,port):
    val = _read2(address, Cmd.GETENCODERMODE,port)
    if val[0]:
        return 1, val[1] >> 8, val[1] & 0xFF
    return 0, 0, 0


def SetM1EncoderMode(address, mode,port):
    return _write1(address, Cmd.SETM1ENCODERMODE, mode,port)


def SetM2EncoderMode(address, mode,port):
    return _write1(address, Cmd.SETM2ENCODERMODE, mode,port)


# saves active settings to NVM
def WriteNVM(address,port):
    return _write4(address, Cmd.WRITENVM, 0xE22EAB7A,port)


# restores settings from NVM
# Warning(TTL Serial): If baudrate changes or the control mode changes communications will be lost
def ReadNVM(address,port):
    return _write0(address, Cmd.READNVM,port)


# Warning(TTL Serial): If control mode is changed from packet serial mode
# when setting config communications will be lost!
# Warning(TTL Serial): If baudrate of packet serial mode is changed communications will be lost!
def SetConfig(address, config,port):
    return _write2(address, Cmd.SETCONFIG, config,port)


def GetConfig(address,port):
    return _read2(address, Cmd.GETCONFIG,port)


def SetM1MaxCurrent(address, max,port):
    return _write44(address, Cmd.SETM1MAXCURRENT, max, 0,port)


def SetM2MaxCurrent(address, max,port):
    return _write44(address, Cmd.SETM2MAXCURRENT, max, 0,port)


def ReadM1MaxCurrent(address,port):
    data = _read_n(address, Cmd.GETM1MAXCURRENT, 2,port)
    if data[0]:
        return 1, data[1]
    return 0, 0


def ReadM2MaxCurrent(address,port):
    data = _read_n(address, Cmd.GETM2MAXCURRENT, 2,port)
    if data[0]:
        return 1, data[1]
    return 0, 0


def SetPWMMode(address, mode,port):
    return _write1(address, Cmd.SETPWMMODE, mode,port)


def ReadPWMMode(address,port):
    return _read1(address, Cmd.GETPWMMODE,port)


def Open(comport, rate):
    # global port
    port = serial.Serial(comport, baudrate=rate, timeout=0.1, interCharTimeout=0.01)
    return port
