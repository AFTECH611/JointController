import spidev
import time

hostID = 0xfd             #主机ID fd
pi = 3.1415926

# 不可改动
P_MIN = -12.5     #扭矩下限范围
P_MAX = 12.5      #扭矩上限范围
T_MIN = -12.0     #角度(弧度)下限范围
T_MAX = 12.0      #角度(弧度)上限范围
V_MIN = -30.0     #速度下限范围
V_MAX = 30.0      #速度上限范围
KP_MIN = 0.0      #KP下限范围
KP_MAX = 500.0    #KP上限范围
KD_MIN = 0.0      #KD下限范围
KD_MAX = 5.0      #KD上限范围

class CRC8:
    def __init__(self, polynomial=10):
        self.polynomial = polynomial
        self.table = self._build_table()
    
    def _build_table(self):
        table = []
        for i in range(256):
            crc = i
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ self.polynomial) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
            table.append(crc)
        return table
    
    def calculate(self, data):
        crc = 0
        for byte in data:
            crc = self.table[crc ^ byte] & 0xFF
        return crc

crc8_calculator = CRC8(polynomial=7)

#=========================设置CAN总线的波特率=====================
# SPI连接开发板，只需一条SPI总线，和两个CS片选通道，例如树莓派可以使用SPI通道0，片选通道0和1
# 发送和接收的数据结构：固定帧头6个字节(0x45 0x54 0x45 0x54 0xFF 0x01)+CAN总线的编号 2个字节+波特率4个字节+固定帧尾4个字节(0x0D 0x0A 0x0D 0x0A)+预留4个字节(0x00 0x00 0x00 0x00)+CRC1个字节
# spibus:                SPI总线
# cs:                    SPI片选通道
# cannum:                 CAN总线的编号，1，2，3，4
# baudrate:              CAN总线的波特率，仅支持：1000000，500000，250000，125000
#================================================================
def sys_set_can_baudrate(spibus, cs, cannum, baudrate):
    state = 0
    # 检查CAN总线编号是否有效
    if cannum not in [1, 2, 3, 4]:
        raise ValueError("CAN总线编号必须为1、2、3或4")
    # 检查波特率是否有效
    if baudrate not in [1000000, 500000, 250000, 125000]:
        raise ValueError("波特率必须为1000000、500000、250000或125000")
    try:
        spi = spidev.SpiDev()
        # 打开SPI总线
        spi.open(int(spibus), int(cs))
        spi.max_speed_hz = 10000000  # 10 MHz（修正注释）
        spi.mode = 0                # SPI模式0 (CPOL=0, CPHA=0)

        # 固定帧头6个字节(0x45 0x54 0x45 0x54 0xFF 0x01)
        frame_header = [0x45, 0x54, 0x45, 0x54, 0xFF, 0x01]
        # CAN总线的编号 2个字节
        canid_bytes = [cannum, 0x00]
        # 波特率4个字节
        baudrate_bytes = [
            (baudrate >> 24) & 0xFF,
            (baudrate >> 16) & 0xFF,
            (baudrate >> 8) & 0xFF,
            baudrate & 0xFF
        ]
        # 固定帧尾4个字节(0x0D 0x0A 0x0D 0x0A)
        frame_tail = [0x0D, 0x0A, 0x0D, 0x0A]
        # 预留4个字节(使用0x00填充)
        reserved_bytes = [0x00, 0x00, 0x00, 0x00]
        # 合并所有数据
        full_data = frame_header + canid_bytes + baudrate_bytes + frame_tail + reserved_bytes
        # 计算CRC值
        crc = crc8_calculator.calculate(full_data)
        # 添加CRC字节
        full_data.append(crc)
        # print(f"[SPI发送]: {[f'0x{byte:02X}' for byte in full_data]}")

        # 使用xfer2发送数据并接收从设备的响应
        spi.xfer2(full_data)
        # 等待从设备准备数据
        time.sleep(0.1) 
    except KeyboardInterrupt:
        state = -2
        spi.close()
        print("\nSPI连接已关闭")
    except Exception as e:
        state = -2
        print(f"发生错误: {e}")
        if spi is not None:
            spi.close()
    finally:
        if spi is not None:
            spi.close()
    return state

#=========================设置RS485总线的波特率=====================
# SPI连接开发板，只需一条SPI总线，和两个CS片选通道，例如树莓派可以使用SPI通道0，片选通道0和1
# 发送和接收的数据结构：固定帧头6个字节(0x45 0x54 0x45 0x54 0xFF 0x02)+RS485总线的编号 2个字节+波特率4个字节+停止位1个字节+奇偶校验位1个字节+数据位1个字节+固定帧尾4个字节(0x0D 0x0A 0x0D 0x0A)+预留1个字节(0x00)+CRC1个字节
# spibus:                SPI总线
# cs:                    SPI片选通道
# rs485num:              RS485总线的编号，1，2，3，4
# baudrate:              RS485总线的波特率
# format:                停止位，1位停止位，2位停止位，默认为1
# paritytype:            奇偶校验位，0为无校验，1为奇校验，2为偶校验，默认为0
# datatype:              数据位，8位数据位，9位数据位，默认为8
#================================================================
def sys_set_rs485_baudrate(spibus, cs, rs485num, baudrate, format=1, paritytype=0, datatype=8):
    state = 0
    # 检查RS485总线编号是否有效
    if rs485num not in [1, 2, 3, 4]:
        raise ValueError("RS485总线编号必须为1、2、3或4")
    try:
        spi = spidev.SpiDev()
        # 打开SPI总线
        spi.open(int(spibus), int(cs))
        spi.max_speed_hz = 10000000  # 10 MHz（修正注释）
        spi.mode = 0                # SPI模式0 (CPOL=0, CPHA=0)

        # 固定帧头6个字节(0x45 0x54 0x45 0x54 0xFF 0x02)
        frame_header = [0x45, 0x54, 0x45, 0x54, 0xFF, 0x02]
        # RS485总线的编号 2个字节
        rs485id_bytes = [rs485num, 0x00]
        # 波特率4个字节
        baudrate_bytes = [
            (baudrate >> 24) & 0xFF,
            (baudrate >> 16) & 0xFF,
            (baudrate >> 8) & 0xFF,
            baudrate & 0xFF
        ]
        # 固定帧尾4个字节(0x0D 0x0A 0x0D 0x0A)
        frame_tail = [0x0D, 0x0A, 0x0D, 0x0A]
        # 预留1个字节(使用0x00填充)
        reserved_bytes = [0x00]
        # 合并所有数据
        full_data = frame_header + rs485id_bytes + baudrate_bytes + [format, paritytype, datatype] + frame_tail + reserved_bytes
        # 计算CRC值
        crc = crc8_calculator.calculate(full_data)
        # 添加CRC字节
        full_data.append(crc)
        print(f"[SPI发送]: {[f'0x{byte:02X}' for byte in full_data]}")

        # 使用xfer2发送数据并接收从设备的响应
        spi.xfer2(full_data)
        # 等待从设备准备数据
        time.sleep(0.01)
    except KeyboardInterrupt:
        state = -2
        spi.close()
        print("\nSPI连接已关闭")
    except Exception as e:
        state = -2
        print(f"发生错误: {e}")
        if spi is not None:
            spi.close()
    finally:
        if spi is not None:
            spi.close()
    return state

#=========================发送并接受扩展CAN帧=====================
# SPI连接开发板，只需一条SPI总线，和两个CS片选通道，例如树莓派可以使用SPI通道0，片选通道0和1
# 发送和接收的数据结构：固定帧头2个字节(0x45 0x54)+扩展帧CANID 4个字节+数据帧8个字节+固定帧尾2个字节(0x0D 0x0A)+预留4个字节(0x00 0x00 0x00 0x10)+CRC1个字节
# spibus:                SPI总线
# cs:                    SPI片选通道
# arbitration_id:        29位仲裁ID(CyberGear数据结构)
# data:                  CAN数据字段，长度为0-8字节
#================================================================
def send_extended_frame_main(spibus, cs, arbitration_id, data):
    state = 0
    data_part = 0
    try:
        spi = spidev.SpiDev()
        # 打开SPI总线, 默认（总线0，设备0）
        spi.open(int(spibus), int(cs))
        spi.max_speed_hz = 10000000  # 10 MHz（修正注释）
        spi.mode = 0                # SPI模式0 (CPOL=0, CPHA=0)

        # 固定帧头2个字节(0x45 0x54)
        frame_header = [0x45, 0x54]
        # 扩展帧CANID 4个字节
        extended_id = [
            (arbitration_id >> 24) & 0xFF,
            (arbitration_id >> 16) & 0xFF,
            (arbitration_id >> 8) & 0xFF,
            arbitration_id & 0xFF
        ]
        # 数据帧8个字节
        data_bytes = data.copy()
        if len(data_bytes) < 8:
            data_bytes.extend([0x00] * (8 - len(data_bytes)))
        # 固定帧尾2个字节(0x0D 0x0A)
        frame_tail = [0x0D, 0x0A]
        # 预留4个字节(使用0x00填充)
        reserved_bytes = [0x00, 0x00, 0x00, 0x10]
        # 合并所有数据
        full_data = frame_header + extended_id + data_bytes + frame_tail + reserved_bytes
        # 计算CRC值
        crc_value = crc8_calculator.calculate(full_data)
        # spi发送的固定数据长度为21个字节，最后一个字节为CRC值
        full_data.append(crc_value)
        print(f"[SPI发送]: {[f'0x{byte:02X}' for byte in full_data]}")
        # 使用xfer2发送数据并接收从设备的响应
        # xfer2的特点是在同一时钟周期内完成发送和接收
        rx_data = spi.xfer2(full_data)

        # spi接收的固定数据长度为21个字节，最后一个字节为CRC值
        if len(rx_data) != 21:
            print("接收数据长度错误")
            state = -1
            return(state,data_part)
        # 分离数据和CRC
        data_part = rx_data[:20]  # 实际数据部分
        # 从接收到的数据中提取CRC值
        received_crc = rx_data[20]
        # 计算数据部分的CRC
        calculated_crc = crc8_calculator.calculate(data_part)
        # 验证CRC
        is_valid = (calculated_crc == received_crc)
        print(f"[SPI接收]: {[f'0x{byte:02X}' for byte in rx_data]}")
        if is_valid:
            print(f"CRC验证成功: 计算值=0x{calculated_crc:02X}, 接收值=0x{received_crc:02X}")
        else:
            print(f"CRC验证失败: 计算值=0x{calculated_crc:02X}, 接收值=0x{received_crc:02X}")
            state = -1
    except KeyboardInterrupt:
        state = -2
        spi.close()
        print("\nSPI连接已关闭")
    except Exception as e:
        state = -2
        print(f"发生错误: {e}")
        if spi is not None:
            spi.close()
    finally:
        if spi is not None:
            spi.close()
    return(state,data_part) 


#**************************************************************************
# 设置电机零角度功能函数
# spibus:         SPI总线端口号
# cs:             SPI片选通道
# motorID：       电机ID
# state：         返回0,数据正常发送和接收 返回1,通讯故障
#**************************************************************************
def set_motor_angle_zero(spibus, cs, motorID):
    #仲裁帧ID
    #功能码0x0600
    #主ID号0xfd
    #末尾两位从ID      
    arbitration_id = 0x0600fd00
    arbitration_id = arbitration_id + motorID
    data = [0 for i in range(8)] 
    data[0] = 0x01


    (state,rx_data) = send_extended_frame_main(spibus, cs, arbitration_id, data)

    if state == 0:
        print("设置电机ID号:%d 设置负载端零角度OK" % motorID)
    elif state == -1:
        print("设置电机ID号:%d SPI总线返回数据CRC校验错误, 请检查SPI连接" % motorID)
    elif state == -2:
        print("设置电机ID号:%d 设置负载端零角度error, 请检查SPI连接" % motorID)

    return state



#**************************************************************************
# 设置电机模式:   运控模式    
# spibus:        总线端口号
# cs:            SPI片选通道
# motorID:       电机ID
# state:         返回0, 数据正常接收 返回1，通讯故障
# 数据位第[4]位   0：运控模式，1：位置模式，2：速度模式，3：电流模式
#**************************************************************************
def set_motion_mode(spibus, cs, motorID):
    state = 0

    #仲裁帧ID
    #功能码0x1200 
    #主ID号0xfd
    #末尾两位从ID      
    arbitration_id = 0x1200fd00
    arbitration_id = arbitration_id + motorID
    data = [0 for i in range(8)] 
    data[0] = 0x05
    data[1] = 0x70

    (state,rx_data) = send_extended_frame_main(spibus, cs, arbitration_id, data)

    if state == 0:
        print("电机ID号:%d 控制模式(运控模式)OK" % motorID)
    elif state == -1:
        print("电机ID号:%d SPI总线返回数据CRC校验错误, 请检查SPI连接" % motorID)
    elif state == -2:
        print("电机ID号:%d 控制模式(运控模式)error" % motorID)

    return state


#**************************************************************************
# 设置电机模式使能
# spibus:         SPI总线端口号
# cs:             SPI片选通道
# motorID:        电机ID
# state:          返回0，数据正常接收 返回1，通讯故障
#**************************************************************************
def set_motion_enable(spibus, cs, motorID):  #motorID

    state = 0

    #仲裁帧ID
    #功能码0x0300 
    #主ID号0xfd
    #末尾两位从ID      
    arbitration_id = 0x0300fd00
    arbitration_id = arbitration_id + motorID
    data = [0 for i in range(8)] 

    (state,rx_data) = send_extended_frame_main(spibus, cs, arbitration_id, data)

    if state == 0:
        print("电机ID号:%d 电机模式使能OK" % motorID)
    elif state == -1:
        print("电机ID号:%d SPI总线返回数据CRC校验错误, 请检查SPI连接" % motorID)
    elif state == -2:
        print("电机ID号:%d 电机模式使能error" % motorID)

    return state

#**************************************************************************
# 设置电机Stop
# spibus:    SPI总线端口号
# cs:        SPI片选通道
# motorID:   电机ID
# state:     返回0,数据正常接收 返回1,通讯故障
#**************************************************************************
def set_motion_stop(spibus, cs, motorID):  #motorID

    state = 0

    #仲裁帧ID
    #功能码0x0400 
    #主ID号0xfd
    #末尾两位从ID      
    arbitration_id = 0x0400fd00
    arbitration_id = arbitration_id + motorID
    data = [0 for i in range(8)]

    (state,rx_data) = send_extended_frame_main(spibus, cs, arbitration_id, data)

    if state == 0:
        print("电机ID号:%d 电机Stop OK" % motorID)
    elif state == -1:
        print("电机ID号:%d SPI总线返回数据CRC校验错误, 请检查SPI连接" % motorID)
    elif state == -2:
        print("电机ID号:%d 电机Stop error" % motorID)

    return state

#**************************************************************************
# 设置电机运控参数
# spibus:      SPI总线端口号
# motorID:     电机ID号 
# torque:      设定扭矩,                        0~~65535 对应 -12Nm~~12Nm
# radian:      设定角度(弧单位)                  0~~65535 对应 -4Π~~4Π
# speed:       设定速度                         0~~65535 对应 -30rad/s~~30rad/s
# KP:          设定角度-当前角度的 KP值           0~~65535 对应 0.0~~500.0
# KD:          设定速度-当前速度的 KD值           0~~65535 对应 0.0~~5.0

# state:       返回0，数据正常接收 返回1，通讯故障
#**************************************************************************
def motor_set_motion_parameter(spibus, cs, motorID, torque, radian, speed, KP, KD):

    global P_MIN,P_MAX,T_MIN,T_MAX,V_MIN,V_MAX,KP_MIN,KP_MAX,KD_MIN,KD_MAX

    data = [0 for i in range(8)] 

    # 转化扭矩参数
    data_int16 = (float_to_uint16(torque,P_MIN,P_MAX))<<8 
    arbitration_id = 0x01000000 | data_int16 | motorID


    # 转化角度(弧度)参数载入
    data_int16 = (float_to_uint16(radian,T_MIN,T_MAX))
    data[0] = data_int16>>8
    data[1] = data_int16 & 0x00ff

    #转化载入速度参数
    data_int16 = (float_to_uint16(speed,V_MIN,V_MAX))
    data[2] = data_int16>>8
    data[3] = data_int16 & 0x00ff

    #转化载入KP参数
    data_int16 = (float_to_uint16(KP,KP_MIN,KP_MAX))
    data[4] = data_int16>>8
    data[5] = data_int16 & 0x00ff  

    #转化载入KD参数
    data_int16 = (float_to_uint16(KD,KD_MIN,KD_MAX))
    data[6] = data_int16>>8
    data[7] = data_int16 & 0x00ff

    (state,rx_data)  = send_extended_frame_main(spibus, cs, arbitration_id, data)

    return state
 

#**************************************************************************
#有符号浮点型转四位十六进制(输出小端模式：低在左高在右) 适合单个参数写函数
#**************************************************************************
def float_to_P4hex(float_data):
    # 将十进制浮点数转换为十六进制浮点数
    byte_representation = struct.pack('f', float_data)
    return byte_representation


#**************************************************************************
# 四位十六进制转有符号浮点型转(输入小端模式：低在左高在右) 适合单个参数写函数
#**************************************************************************
def P4hex_to_float(P4hex_data):
    bytes_obj = P4hex_data.to_bytes(4, byteorder='big') 
    float_value = struct.unpack('f', bytes_obj)[0]

    return float_value


#**************************************************************************
# 有符号浮点型转十六进制(0~65536)无符号型      适合运控模式参赛转换
# float_data:        要转换的有符号浮点数据
# float_data_min：   下限范围
# float_data_max：   上限范围
# return :           输出0~65535的int型数据
#**************************************************************************
def float_to_uint16(float_data,float_data_min,float_data_max):

    if float_data > float_data_max:
        float_data_s = float_data_max
    elif float_data < float_data_min:
        float_data_s = float_data_min
    else:
        float_data_s = float_data
    
    return int((float_data_s - float_data_min)/(float_data_max - float_data_min) * 65535)


#**************************************************************************
# 十六进制(0~65535)无符号型转有符号浮点型  适合运控模式参赛转换
# uint16_data:       要转换的无符号型数据
# float_data_min：   下限范围
# float_data_max：   上限范围
# return :           输出上下限范围内的比例 有符合浮点型数据
#**************************************************************************
def uint16_to_float(uint16_data,float_data_min,float_data_max):
   return float((uint16_data - 32767)/65535) * (float_data_max - float_data_min)

def decimal_to_hexadecimal(decimal_number):
    hex_chars = "0123456789ABCDEF"
    hexadecimal = ""
    while decimal_number > 0:
        remainder = decimal_number % 16
        hexadecimal = hex_chars[remainder] + hexadecimal
        decimal_number = decimal_number // 16
    return int(('0x'+hexadecimal),16)

############主函数############
def one_leg_motion_test(spibus, cs, motorID1, motorID2, motorID3):
    time_delay = 0.01 #------请测试，小米电机enable/stop等指令需适当设置延迟时间，保证指令能够执行
    # 假设以控制3个小米电机为例(ID号分别为 1、2、3)，ID号需要改为你自己电机的实际ID号
    # motorID1 = 10                     
    #------1：将电机1机械角度归零
    set_motor_angle_zero(spibus, cs, motorID1)
    #------2：再将电机1设置为运控模式
    set_motion_mode(spibus, cs, motorID1)
    #------3：再将电机1设置为运控模式 使能
    set_motion_enable(spibus, cs, motorID1)
    time.sleep(time_delay)
    # motorID2 = 11
    set_motor_angle_zero(spibus, cs, motorID2)
    set_motion_mode(spibus, cs, motorID2)
    set_motion_enable(spibus, cs, motorID2)
    time.sleep(time_delay)
    # motorID3 = 12
    set_motor_angle_zero(spibus, cs, motorID3)
    set_motion_mode(spibus, cs, motorID3)
    set_motion_enable(spibus, cs, motorID3)
    time.sleep(time_delay)
    #------------以下是控制电机来回旋转的案例-----------------------
    rad1 = 0.0           #最小角度也是起始角度必须为0
    rad2 = 0.0           #最小角度也是起始角度必须为0
    rad3 = 0.0           #最小角度也是起始角度必须为0
    fx1= 0               #方向标志位
    fx2= 0               #方向标志位
    fx3= 0               #方向标志位
    rad_max = 0.5        #最大角度
    rad_min = -0.5
    speed_time_delay = 0.01   #速率延时
    while True:
        try:
            if fx1 == 0 and rad1 < rad_max:
                rad1 = rad1 + 0.01
            elif fx1 == 0 and rad1 >= rad_max:
                fx1 = 1
            elif fx1 == 1 and rad1 > 0:
                rad1 = rad1 - 0.01
            elif fx1 == 1 and rad1 <= 0:
                fx1 = 0
            motor_set_motion_parameter(spibus, cs, motorID1, 0, rad1, 0, 10, 0.5)
            time.sleep(speed_time_delay)

            if fx2 == 0 and rad2 < rad_max:
                rad2 = rad2 + 0.01
            elif fx2 == 0 and rad2 >= rad_max:
                fx2 = 1
            elif fx2 == 1 and rad2 > 0:
                rad2 = rad2 - 0.01
            elif fx2 == 1 and rad2 <= 0:
                fx2 = 0
            motor_set_motion_parameter(spibus, cs, motorID2, 0, rad2, 0, 10, 0.5)
            time.sleep(speed_time_delay)

            if fx3 == 0 and rad3 < rad_max:
                rad3 = rad3 + 0.01
            elif fx3 == 0 and rad3 >= rad_max:
                fx3 = 1
            elif fx3 == 1 and rad3 > 0:
                rad3 = rad3 - 0.01
            elif fx3 == 1 and rad3 <= 0:
                fx3 = 0
            motor_set_motion_parameter(spibus, cs, motorID3, 0, rad3, 0, 10, 0.5)
            time.sleep(speed_time_delay)


        # 当用户按下 Ctrl+C 时，会执行这里的代码
        except KeyboardInterrupt:
            set_motion_stop(spibus, cs, motorID1)
            time.sleep(time_delay)
            set_motion_stop(spibus, cs, motorID2)
            time.sleep(time_delay)
            set_motion_stop(spibus, cs, motorID3)
            time.sleep(time_delay)
            print("按下 Ctrl+C,结束----")
            break

def two_leg_motion_test(spibus, cs, motorID1, motorID2, motorID3, motorID4, motorID5, motorID6):
    time_delay = 0.01 #------请测试，小米电机enable/stop等指令需适当设置延迟时间，保证指令能够执行
    # 假设以控制6个小米电机为例(ID号分别为 1、2、3、4、5、6)，ID号需要改为你自己电机的实际ID号                     
    #------1：将电机1机械角度归零
    set_motor_angle_zero(spibus, cs, motorID1)
    #------2：再将电机1设置为运控模式
    set_motion_mode(spibus, cs, motorID1)
    #------3：再将电机1设置为运控模式 使能
    set_motion_enable(spibus, cs, motorID1)
    time.sleep(time_delay)

    set_motor_angle_zero(spibus, cs, motorID2)
    set_motion_mode(spibus, cs, motorID2)
    set_motion_enable(spibus, cs, motorID2)
    time.sleep(time_delay)

    set_motor_angle_zero(spibus, cs, motorID3)
    set_motion_mode(spibus, cs, motorID3)
    set_motion_enable(spibus, cs, motorID3)
    time.sleep(time_delay)

    set_motor_angle_zero(spibus, cs, motorID4)
    set_motion_mode(spibus, cs, motorID4)
    set_motion_enable(spibus, cs, motorID4)
    time.sleep(time_delay)

    set_motor_angle_zero(spibus, cs, motorID5)
    set_motion_mode(spibus, cs, motorID5)
    set_motion_enable(spibus, cs, motorID5)
    time.sleep(time_delay)

    set_motor_angle_zero(spibus, cs, motorID6)
    set_motion_mode(spibus, cs, motorID6)
    set_motion_enable(spibus, cs, motorID6)
    time.sleep(time_delay)

    #------------以下是控制电机来回旋转的案例-----------------------
    rad1 = 0.0           #最小角度也是起始角度必须为0
    rad2 = 0.0           #最小角度也是起始角度必须为0
    rad3 = 0.0           #最小角度也是起始角度必须为0
    fx1= 0               #方向标志位
    fx2= 0               #方向标志位
    fx3= 0               #方向标志位
    rad_max = 0.5        #最大角度
    rad_min = -0.5
    speed_time_delay = 0.01   #速率延时
    while True:
        try:
            if fx1 == 0 and rad1 < rad_max:
                rad1 = rad1 + 0.01
            elif fx1 == 0 and rad1 >= rad_max:
                fx1 = 1
            elif fx1 == 1 and rad1 > 0:
                rad1 = rad1 - 0.01
            elif fx1 == 1 and rad1 <= 0:
                fx1 = 0
            motor_set_motion_parameter(spibus, cs, motorID1, 0, rad1, 0, 10, 0.5)
            motor_set_motion_parameter(spibus, cs, motorID4, 0, rad1, 0, 10, 0.5)
            time.sleep(speed_time_delay)

            if fx2 == 0 and rad2 < rad_max:
                rad2 = rad2 + 0.01
            elif fx2 == 0 and rad2 >= rad_max:
                fx2 = 1
            elif fx2 == 1 and rad2 > 0:
                rad2 = rad2 - 0.01
            elif fx2 == 1 and rad2 <= 0:
                fx2 = 0
            motor_set_motion_parameter(spibus, cs, motorID2, 0, rad2, 0, 10, 0.5)
            motor_set_motion_parameter(spibus, cs, motorID5, 0, rad2, 0, 10, 0.5)
            time.sleep(speed_time_delay)
            
            if fx3 == 0 and rad3 < rad_max:
                rad3 = rad3 + 0.01
            elif fx3 == 0 and rad3 >= rad_max:
                fx3 = 1
            elif fx3 == 1 and rad3 > 0:
                rad3 = rad3 - 0.01
            elif fx3 == 1 and rad3 <= 0:
                fx3 = 0
            motor_set_motion_parameter(spibus, cs, motorID3, 0, rad3, 0, 10, 0.5)
            motor_set_motion_parameter(spibus, cs, motorID6, 0, rad3, 0, 10, 0.5)
            time.sleep(speed_time_delay)


        # 当用户按下 Ctrl+C 时，会执行这里的代码
        except KeyboardInterrupt:
            set_motion_stop(spibus, cs, motorID1)
            time.sleep(time_delay)
            set_motion_stop(spibus, cs, motorID2)
            time.sleep(time_delay)
            set_motion_stop(spibus, cs, motorID3)
            time.sleep(time_delay)
            set_motion_stop(spibus, cs, motorID4)
            time.sleep(time_delay)
            set_motion_stop(spibus, cs, motorID5)
            time.sleep(time_delay)
            set_motion_stop(spibus, cs, motorID6)
            time.sleep(time_delay)
            print("按下 Ctrl+C,结束----")
            break

def four_leg_motion_test(spibus, cs1, cs2, motorID1, motorID2, motorID3, motorID4, motorID5, motorID6, motorID7, motorID8, motorID9, motorID10, motorID11, motorID12):
    time_delay = 0.01 #------请测试，小米电机enable/stop等指令需适当设置延迟时间，保证指令能够执行
    # 假设以控制12个小米电机为例(ID号分别为 1、2、3、4、5、6、7、8、9、10、11、12)，ID号需要改为你自己电机的实际ID号                     
    #------1：将电机1机械角度归零
    set_motor_angle_zero(spibus, cs1, motorID1)
    #------2：再将电机1设置为运控模式
    set_motion_mode(spibus, cs1, motorID1)
    #------3：再将电机1设置为运控模式 使能
    set_motion_enable(spibus, cs1, motorID1)
    time.sleep(time_delay)

    set_motor_angle_zero(spibus, cs1, motorID2)
    set_motion_mode(spibus, cs1, motorID2)
    set_motion_enable(spibus, cs1, motorID2)
    time.sleep(time_delay)

    set_motor_angle_zero(spibus, cs1, motorID3)
    set_motion_mode(spibus, cs1, motorID3)
    set_motion_enable(spibus, cs1, motorID3)
    time.sleep(time_delay)

    set_motor_angle_zero(spibus, cs1, motorID4)
    set_motion_mode(spibus, cs1, motorID4)
    set_motion_enable(spibus, cs1, motorID4)
    time.sleep(time_delay)

    set_motor_angle_zero(spibus, cs1, motorID5)
    set_motion_mode(spibus, cs1, motorID5)
    set_motion_enable(spibus, cs1, motorID5)
    time.sleep(time_delay)

    set_motor_angle_zero(spibus, cs1, motorID6)
    set_motion_mode(spibus, cs1, motorID6)
    set_motion_enable(spibus, cs1, motorID6)
    time.sleep(time_delay)

    set_motor_angle_zero(spibus, cs2, motorID7)
    set_motion_mode(spibus, cs2, motorID7)
    set_motion_enable(spibus, cs2, motorID7)
    time.sleep(time_delay)

    set_motor_angle_zero(spibus, cs2, motorID8)
    set_motion_mode(spibus, cs2, motorID8)
    set_motion_enable(spibus, cs2, motorID8)
    time.sleep(time_delay)

    set_motor_angle_zero(spibus, cs2, motorID9)
    set_motion_mode(spibus, cs2, motorID9)
    set_motion_enable(spibus, cs2, motorID9)
    time.sleep(time_delay)

    set_motor_angle_zero(spibus, cs2, motorID10)
    set_motion_mode(spibus, cs2, motorID10)
    set_motion_enable(spibus, cs2, motorID10)
    time.sleep(time_delay)

    set_motor_angle_zero(spibus, cs2, motorID11)
    set_motion_mode(spibus, cs2, motorID11)
    set_motion_enable(spibus, cs2, motorID11)
    time.sleep(time_delay)

    set_motor_angle_zero(spibus, cs2, motorID12)
    set_motion_mode(spibus, cs2, motorID12)
    set_motion_enable(spibus, cs2, motorID12)
    time.sleep(time_delay)
    #------------以下是控制电机来回旋转的案例-----------------------
    rad1 = 0.0           #最小角度也是起始角度必须为0
    rad2 = 0.0           #最小角度也是起始角度必须为0
    rad3 = 0.0           #最小角度也是起始角度必须为0
    fx1= 0               #方向标志位
    fx2= 0               #方向标志位
    fx3= 0               #方向标志位
    rad_max = 0.5        #最大角度
    rad_min = -0.5
    speed_time_delay = 0.01   #速率延时
    while True:
        try:
            if fx1 == 0 and rad1 < rad_max:
                rad1 = rad1 + 0.01
            elif fx1 == 0 and rad1 >= rad_max:
                fx1 = 1
            elif fx1 == 1 and rad1 > 0:
                rad1 = rad1 - 0.01
            elif fx1 == 1 and rad1 <= 0:
                fx1 = 0
            motor_set_motion_parameter(spibus, cs1, motorID1, 0, rad1, 0, 10, 0.5)
            motor_set_motion_parameter(spibus, cs1, motorID4, 0, rad1, 0, 10, 0.5)
            motor_set_motion_parameter(spibus, cs2, motorID7, 0, rad1, 0, 10, 0.5)
            motor_set_motion_parameter(spibus, cs2, motorID10, 0, rad1, 0, 10, 0.5)
            time.sleep(speed_time_delay)

            if fx2 == 0 and rad2 < rad_max:
                rad2 = rad2 + 0.01
            elif fx2 == 0 and rad2 >= rad_max:
                fx2 = 1
            elif fx2 == 1 and rad2 > 0:
                rad2 = rad2 - 0.01
            elif fx2 == 1 and rad2 <= 0:
                fx2 = 0
            motor_set_motion_parameter(spibus, cs1, motorID2, 0, rad2, 0, 10, 0.5)
            motor_set_motion_parameter(spibus, cs1, motorID5, 0, rad2, 0, 10, 0.5)
            motor_set_motion_parameter(spibus, cs2, motorID8, 0, rad2, 0, 10, 0.5)
            motor_set_motion_parameter(spibus, cs2, motorID11, 0, rad2, 0, 10, 0.5)            
            time.sleep(speed_time_delay)
            
            if fx3 == 0 and rad3 < rad_max:
                rad3 = rad3 + 0.01
            elif fx3 == 0 and rad3 >= rad_max:
                fx3 = 1
            elif fx3 == 1 and rad3 > 0:
                rad3 = rad3 - 0.01
            elif fx3 == 1 and rad3 <= 0:
                fx3 = 0
            motor_set_motion_parameter(spibus, cs1, motorID3, 0, rad3, 0, 10, 0.5)
            motor_set_motion_parameter(spibus, cs1, motorID6, 0, rad3, 0, 10, 0.5)
            motor_set_motion_parameter(spibus, cs2, motorID9, 0, rad3, 0, 10, 0.5)
            motor_set_motion_parameter(spibus, cs2, motorID12, 0, rad3, 0, 10, 0.5)            
            time.sleep(speed_time_delay)


        # 当用户按下 Ctrl+C 时，会执行这里的代码
        except KeyboardInterrupt:
            set_motion_stop(spibus, cs1, motorID1)
            time.sleep(time_delay)
            set_motion_stop(spibus, cs1, motorID2)
            time.sleep(time_delay)
            set_motion_stop(spibus, cs1, motorID3)
            time.sleep(time_delay)
            set_motion_stop(spibus, cs1, motorID4)
            time.sleep(time_delay)
            set_motion_stop(spibus, cs1, motorID5)
            time.sleep(time_delay)
            set_motion_stop(spibus, cs1, motorID6)
            time.sleep(time_delay)
            set_motion_stop(spibus, cs2, motorID7)
            time.sleep(time_delay)
            set_motion_stop(spibus, cs2, motorID8)
            time.sleep(time_delay)
            set_motion_stop(spibus, cs2, motorID9)
            time.sleep(time_delay)
            set_motion_stop(spibus, cs2, motorID10)
            time.sleep(time_delay)
            set_motion_stop(spibus, cs2, motorID11)
            time.sleep(time_delay)
            set_motion_stop(spibus, cs2, motorID12)
            time.sleep(time_delay)            
            print("按下 Ctrl+C,结束----")
            break

def motor_motion_test(spibus=0, cs=0, motorID=3):
                       
    #------1：将某个电机机械角度归零
    set_motor_angle_zero(spibus, cs, motorID)
    #------2：再将某个电机设置为运控模式
    set_motion_mode(spibus, cs, motorID)
    #------3：再将某个电机设置为运控模式 使能
    set_motion_enable(spibus, cs, motorID)

    #------------以下是控制电机来回旋转的案例-----------------------
    rad = 0.0           #最小角度也是起始角度必须为0
    fx = 0              #方向标志位
    rad_max = 0.5       #最大角度
    time_delay = 0.002  #速率延时
    while True:
        try:  
            if fx == 0 and rad < rad_max:
                rad = rad + 0.01
            elif fx == 0 and rad >= rad_max:
                fx = 1
                time.sleep(1) 
            elif fx == 1 and rad > 0:
                rad = rad - 0.01
            elif fx == 1 and rad <= 0:
                fx = 0
                time.sleep(1)

            motor_set_motion_parameter(spibus, cs, motorID, 0, rad, 0, 10, 0.5)
            time.sleep(time_delay) 

        # 当用户按下 Ctrl+C 时，会执行这里的代码
        except KeyboardInterrupt:  
            set_motion_stop(spibus, cs, motorID)
            print("按下 Ctrl+C,结束----")
            break



import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="这是一个示例脚本。")
    parser.add_argument("--cmd", help="1: 置负载端零角度。 2: 电机停止。", default="2")
    parser.add_argument("--spibus", help="0 or 1", default="0")
    parser.add_argument("--cs", help="0 or 1", default="1")
    parser.add_argument("--motorid", help="motor id", default="9")
    args = parser.parse_args()
    motorID = decimal_to_hexadecimal(int(args.motorid))
    spibus = int(args.spibus)
    cs = int(args.cs)
    
    # 小米CyberGear电机CAN总线要求1000000波特率，设置4个CAN通道的波特率为1000000
    sys_set_can_baudrate(0, 0, 1, 1000000)
    sys_set_can_baudrate(0, 0, 2, 1000000)
    sys_set_can_baudrate(0, 1, 3, 1000000)
    sys_set_can_baudrate(0, 1, 4, 1000000)

    if args.cmd == "1":
        set_motor_angle_zero(spibus, cs, motorID)
    elif args.cmd == "2":
        set_motion_stop(spibus, cs, motorID)
    elif args.cmd == "3":
        #------1：设置电机机械角度归零
        set_motor_angle_zero(spibus, cs, motorID)
        #------2：设置电机设置为运控模式
        set_motion_mode(spibus, cs, motorID)
        #------3：设置电机设置为运控模式使能
        set_motion_enable(spibus, cs, motorID)
        #------4：设置电机运控模式参数，转动一定弧度
        motor_set_motion_parameter(spibus, cs, motorID, 0, 0.2, 0, 10, 0.5)
    elif args.cmd == "4":
        #------设置某一个电机来回转动一定弧度
        motor_motion_test(spibus, cs, motorID)
    elif args.cmd == "5":
        #------设置一条腿3个电机或者两条腿6个电机来回转动一定弧度, 假设Motor ID号为1,2,3,10,11,12
        #one_leg_motion_test(spibus, cs, 1, 2, 3)
        two_leg_motion_test(spibus, cs, 1, 2, 3, 10, 11, 12)
    elif args.cmd == "6":
        #------设置四条腿12个电机来回转动一定弧度, 假设Motor ID号为1,2,3,4,5,6,7,8,9,10,11,12
        cs1 = 0
        cs2 = 1
        four_leg_motion_test(spibus, cs1, cs2, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12)
    else:
        print(f"未知命令: {args.cmd}")
