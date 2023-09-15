from machine import SPI
import sensor
import image
import lcd
import KPU as kpu
import time
from Maix import FPIOA, GPIO
import gc
from fpioa_manager import fm
from board import board_info
import utime
from machine import Timer,PWM
from machine import UART
from machine import WDT
import os
from micropython import const

# ==============================RFID CONFIG==============================



# 定义了一个名为 MFRC522 的Python类，它包含了MFRC522 RFID读写器的各种常量。
class MFRC522:
    NRSTPD = 22

    MAX_LEN = 16

    PCD_IDLE = 0x00
    PCD_AUTHENT = 0x0E
    PCD_RECEIVE = 0x08
    PCD_TRANSMIT = 0x04
    PCD_TRANSCEIVE = 0x0C
    PCD_RESETPHASE = 0x0F
    PCD_CALCCRC = 0x03

    PICC_REQIDL = 0x26
    PICC_REQALL = 0x52
    PICC_ANTICOLL = 0x93
    PICC_SElECTTAG = 0x93
    PICC_AUTHENT1A = 0x60
    PICC_AUTHENT1B = 0x61
    PICC_READ = 0x30
    PICC_WRITE = 0xA0
    PICC_DECREMENT = 0xC0
    PICC_INCREMENT = 0xC1
    PICC_RESTORE = 0xC2
    PICC_TRANSFER = 0xB0
    PICC_HALT = 0x50

    MI_OK = 0
    MI_NOTAGERR = 1
    MI_ERR = 2

    Reserved00 = 0x00
    CommandReg = 0x01
    CommIEnReg = 0x02
    DivlEnReg = 0x03
    CommIrqReg = 0x04
    DivIrqReg = 0x05
    ErrorReg = 0x06
    Status1Reg = 0x07
    Status2Reg = 0x08
    FIFODataReg = 0x09
    FIFOLevelReg = 0x0A
    WaterLevelReg = 0x0B
    ControlReg = 0x0C
    BitFramingReg = 0x0D
    CollReg = 0x0E
    Reserved01 = 0x0F

    Reserved10 = 0x10
    ModeReg = 0x11
    TxModeReg = 0x12
    RxModeReg = 0x13
    TxControlReg = 0x14
    TxAutoReg = 0x15
    TxSelReg = 0x16
    RxSelReg = 0x17
    RxThresholdReg = 0x18
    DemodReg = 0x19
    Reserved11 = 0x1A
    Reserved12 = 0x1B
    MifareReg = 0x1C
    Reserved13 = 0x1D
    Reserved14 = 0x1E
    SerialSpeedReg = 0x1F

    Reserved20 = 0x20
    CRCResultRegM = 0x21
    CRCResultRegL = 0x22
    Reserved21 = 0x23
    ModWidthReg = 0x24
    Reserved22 = 0x25
    RFCfgReg = 0x26
    GsNReg = 0x27
    CWGsPReg = 0x28
    ModGsPReg = 0x29
    TModeReg = 0x2A
    TPrescalerReg = 0x2B
    TReloadRegH = 0x2C
    TReloadRegL = 0x2D
    TCounterValueRegH = 0x2E
    TCounterValueRegL = 0x2F

    Reserved30 = 0x30
    TestSel1Reg = 0x31
    TestSel2Reg = 0x32
    TestPinEnReg = 0x33
    TestPinValueReg = 0x34
    TestBusReg = 0x35
    AutoTestReg = 0x36
    VersionReg = 0x37
    AnalogTestReg = 0x38
    TestDAC1Reg = 0x39
    TestDAC2Reg = 0x3A
    TestADCReg = 0x3B
    Reserved31 = 0x3C
    Reserved32 = 0x3D
    Reserved33 = 0x3E
    Reserved34 = 0x3F

    serNum = []

# ==============================RFID SYSTEM FUNCTIONS==============================
    # __init__(self, spi, cs)：类的构造函数，接收2个参数，分别是SPI总线对象和片选引脚对象。
    # 在构造函数中，将SPI总线和片选引脚保存到类的成员变量中，并调用MFRC522_Init()方法进行初始化。
    def __init__(self, spi, cs):
        # 初始化
        self.spi = spi
        self.cs = cs
        self.MFRC522_Init()
    # Write_MFRC522(self, addr, val)：写入MFRC522寄存器，接收2个参数，分别是寄存器地址和写入的值。
    # 在方法中，先将片选引脚置为低电平，然后通过SPI总线写入寄存器地址和写入的值，最后将片选引脚置为高电平。
    def Write_MFRC522(self, addr, val):
        self.cs.value(0)
        self.spi.write(((addr << 1) & 0x7E))
        self.spi.write(val)
        self.cs.value(1)
    # Read_MFRC522(self, addr)：读取MFRC522寄存器，接收1个参数，即寄存器地址。
    # 在方法中，先将片选引脚置为低电平，然后通过SPI总线写入寄存器地址，并读取该地址的值，最后将片选引脚置为高电平，并返回读取的值。
    def Read_MFRC522(self, addr):
        self.cs.value(0)
        self.spi.write((((addr << 1) & 0x7E) | 0x80))
        val = self.spi.read(1)
        self.cs.value(1)
        return val[0]
    # SetBitMask(self, reg, mask)：设置MFRC522寄存器的位掩码，接收2个参数，即寄存器地址和位掩码。
    # 在方法中，先读取该寄存器的值，然后将位掩码与该寄存器的值进行按位或运算，最后将运算结果写入该寄存器。
    def SetBitMask(self, reg, mask):
        tmp = self.Read_MFRC522(reg)
        self.Write_MFRC522(reg, tmp | mask)
    # ClearBitMask(self, reg, mask)：清除MFRC522寄存器的位掩码，接收2个参数，即寄存器地址和位掩码。
    # 在方法中，先读取该寄存器的值，然后将位掩码按位取反，并与该寄存器的值进行按位与运算，最后将运算结果写入该寄存器。
    def ClearBitMask(self, reg, mask):
        tmp = self.Read_MFRC522(reg)
        self.Write_MFRC522(reg, tmp & (~mask))

    # 通过RC522与卡片通信
    def MFRC522_ToCard(self, command, sendData):
        backData = []
        backLen = 0
        status = self.MI_ERR
        irqEn = 0x00
        waitIRq = 0x00
        lastBits = None
        n = 0
        i = 0

        if command == self.PCD_AUTHENT:     # Mifare认证
            irqEn = 0x12                    # 允许错误中断请求ErrIEn  允许空闲中断IdleIEn
            waitIRq = 0x10                  # 认证寻卡等待时候 查询空闲中断标志位
        if command == self.PCD_TRANSCEIVE:  # 接收发送 发送接收
            irqEn = 0x77                    # 允许TxIEn RxIEn IdleIEn LoAlertIEn ErrIEn TimerIEn
            waitIRq = 0x30                  # 寻卡等待时候 查询接收中断标志位与 空闲中断标志位

        # IRqInv置位管脚IRQ与Status1Reg的IRq位的值相反
        self.Write_MFRC522(self.CommIEnReg, irqEn | 0x80)
        # Set1该位清零时，CommIRqReg的屏蔽位清零
        self.ClearBitMask(self.CommIrqReg, 0x80)
        # 写空闲命令
        self.Write_MFRC522(self.CommandReg, self.PCD_IDLE)

        # 置位FlushBuffer清除内部FIFO的读和写指针以及ErrReg的BufferOvfl标志位被清除
        self.SetBitMask(self.FIFOLevelReg, 0x80)

        while(i < len(sendData)):
            self.Write_MFRC522(self.FIFODataReg, sendData[i])  # 写数据进FIFOdata
            i = i+1

        self.Write_MFRC522(self.CommandReg, command)  # 写命令

        if command == self.PCD_TRANSCEIVE:
            # StartSend置位启动数据发送 该位与收发命令使用时才有效
            self.SetBitMask(self.BitFramingReg, 0x80)

        i = 1000 * 3

        # 认证 与寻卡等待时间
        while True:
            n = self.Read_MFRC522(self.CommIrqReg)  # 查询事件中断
            i = i - 1
            if not ((i != 0) and (not (n & 0x01)) and (not (n & waitIRq))):
                break

        self.ClearBitMask(self.BitFramingReg, 0x80)  # 清理允许StartSend位

        if i != 0:
            # 读错误标志寄存器BufferOfI CollErr ParityErr ProtocolErr
            if not (self.Read_MFRC522(self.ErrorReg) & 0x1B):
                status = self.MI_OK

                if n & irqEn & 0x01:
                    status = self.MI_NOTAGERR

                if command == self.PCD_TRANSCEIVE:
                    n = self.Read_MFRC522(self.FIFOLevelReg)
                    lastBits = self.Read_MFRC522(self.ControlReg) & 0x07
                    # print("n: {}, {}".format(n, lastBits))
                    if lastBits != 0:
                        backLen = (n-1)*8 + lastBits
                    else:
                        backLen = n*8

                    if n == 0:
                        n = 1
                    if n > self.MAX_LEN:
                        n = self.MAX_LEN

                    i = 0
                    while i < n:
                        backData.append(self.Read_MFRC522(self.FIFODataReg))
                        i = i + 1
            else:
                # print("erro: {}".format(hex(self.Read_MFRC522(self.ErrorReg))))
                status = self.MI_ERR
        # print("backlen: {}".format(backLen))
        self.SetBitMask(self.ControlReg, 0x80)
        # stop timer now
        self.Write_MFRC522(self.CommandReg, self.PCD_IDLE)
        return (status, backData, backLen)

    def MFRC522_Request(self, reqMode):
          status = None
          backBits = None
          TagType = []

          # 清理指示MIFARECyptol单元接通以及所有卡的数据通信被加密的情况
          self.ClearBitMask(self.Status2Reg, 0x08)
          # 发送的最后一个字节的 七位
          self.Write_MFRC522(self.BitFramingReg, 0x07)
          # TX1,TX2管脚的输出信号传递经发送调制的13.56的能量载波信号
          self.SetBitMask(self.TxControlReg, 0x03)

          TagType.append(reqMode)
          (status, backData, backBits) = self.MFRC522_ToCard(
              self.PCD_TRANSCEIVE, TagType)
          # print("backBits: {}".format(backBits))
          if ((status != self.MI_OK) | (backBits != 0x10)):
              status = self.MI_ERR

          return (status, backData)

    def MFRC522_Anticoll(self):
           # 定义返回数据列表
           backData = []
           # 定义序列号校验值
           serNumCheck = 0
           # 定义序列号列表
           serNum = []
           # 将BitFramingReg寄存器的值设为0x00
           self.Write_MFRC522(self.BitFramingReg, 0x00)
           # 将PICC_ANTICOLL命令添加到序列号列表中
           serNum.append(self.PICC_ANTICOLL)
           # 将0x20添加到序列号列表中
           serNum.append(0x20)
           # 通过MFRC522_ToCard方法向RC522模块发送序列号列表，并获取响应状态、返回数据和返回数据位数
           (status, backData, backBits) = self.MFRC522_ToCard(
               self.PCD_TRANSCEIVE, serNum)
           # 如果响应状态为MI_OK
           if(status == self.MI_OK):
               i = 0
               if len(backData) == 5:
                   while i < 4:
                       serNumCheck = serNumCheck ^ backData[i]
                       i = i + 1
                   if serNumCheck != backData[i]:
                       status = self.MI_ERR
               else:
                   status = self.MI_ERR

           return (status, backData)

    def CalulateCRC(self, pIndata):
           # 清除DivIrqReg寄存器的4位
           self.ClearBitMask(self.DivIrqReg, 0x04)
           # 设置FIFOLevelReg寄存器的7位
           self.SetBitMask(self.FIFOLevelReg, 0x80)
           i = 0
           while i < len(pIndata):
               # 将要进行CRC校验的数据写入FIFODataReg寄存器中
               self.Write_MFRC522(self.FIFODataReg, pIndata[i])
               i = i + 1
           # 发送PCD_CALCCRC命令，开始进行CRC校验
           self.Write_MFRC522(self.CommandReg, self.PCD_CALCCRC)
           i = 0xFF
           while True:
           # 读取DivIrqReg寄存器的值
               n = self.Read_MFRC522(self.DivIrqReg)
               i = i - 1
                # 如果i为0或者DivIrqReg寄存器的4位为1，则退出循环
               if not ((i != 0) and not (n & 0x04)):
                   break
           # 定义输出列表
           pOutData = []
           # 将CRCResultRegL寄存器的值添加到输出列表中
           pOutData.append(self.Read_MFRC522(self.CRCResultRegL))
           # 将CRCResultRegM寄存器的值添加到输出列表中
           pOutData.append(self.Read_MFRC522(self.CRCResultRegM))
           # 返回输出列表
           return pOutData

    def MFRC522_SelectTag(self, serNum):
        # 定义返回数据列表
        backData = []
        # 定义发送数据列表
        buf = []
        # 将PICC_SElECTTAG命令添加到发送数据列表中
        buf.append(self.PICC_SElECTTAG)
        # 将0x70添加到发送数据列表中
        buf.append(0x70)
        i = 0
        # 将标签序列号的5个字节添加到发送数据列表中
        while i < 5:
            buf.append(serNum[i])
            i = i + 1
        # 计算发送数据列表的CRC校验码
        pOut = self.CalulateCRC(buf)
        # 将CRC校验码的第一个字节添加到发送数据列表中
        buf.append(pOut[0])
        # 将CRC校验码的第二个字节添加到发送数据列表中
        buf.append(pOut[1])
        # 通过MFRC522_ToCard方法向RC522模块发送发送数据列表，并获取响应状态、返回数据和返回数据长度
        (status, backData, backLen) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, buf)
        # 如果响应状态为MI_OK，并且返回数据长度为0x18
        if (status == self.MI_OK) and (backLen == 0x18):
        # 打印返回数据的第一个字节（表示标签容量）
            print("Size: ", str(backData[0]))
            # 返回返回数据的第一个字节
            return backData[0]
        else:
            # 否则返回0
            return 0

    def MFRC522_Auth(self, authMode, BlockAddr, Sectorkey, serNum):
            buff = []

            # First byte should be the authMode (A or B)
            buff.append(authMode)

            # Second byte is the trailerBlock (usually 7)
            buff.append(BlockAddr)

            # Now we need to append the authKey which usually is 6 bytes of 0xFF
            i = 0
            while(i < len(Sectorkey)):
                buff.append(Sectorkey[i])
                i = i + 1
            i = 0

            # Next we append the first 4 bytes of the UID
            while(i < 4):
                buff.append(serNum[i])
                i = i + 1

            # Now we start the authentication itself
            (status, backData, backLen) = self.MFRC522_ToCard(self.PCD_AUTHENT, buff)

            # Check if an error occurred
            if not(status == self.MI_OK):
                print("AUTH ERROR!!")
            if not (self.Read_MFRC522(self.Status2Reg) & 0x08) != 0:
                print("AUTH ERROR(status2reg & 0x08) != 0")

            # Return the status
            return status

    def MFRC522_Read(self, blockAddr):
        # 定义接收数据列表
        recvData = []
        # 将PICC_READ命令添加到接收数据列表中
        recvData.append(self.PICC_READ)
        # 将数据块地址添加到接收数据列表中
        recvData.append(blockAddr)
        # 计算接收数据列表的CRC校验码
        pOut = self.CalulateCRC(recvData)
        # 将CRC校验码的第一个字节添加到接收数据列表中
        recvData.append(pOut[0])
        # 将CRC校验码的第二个字节添加到接收数据列表中
        recvData.append(pOut[1])
        (status, backData, backLen) = self.MFRC522_ToCard(
            self.PCD_TRANSCEIVE, recvData)
        # 通过MFRC522_ToCard方法向RC522模块发送接收数据列表，并获取响应状态、返回数据和返回数据长度
        if not (status == self.MI_OK):
            # 如果响应状态不为MI_OK，则打印错误信息
            print("Error while reading!")
        i = 0
        # 如果返回的数据长度为16，则说明读取到了完整的数据块
        if len(backData) == 16:
        # 打印读取到的数据块信息
            print("Sector "+str(blockAddr)+" "+str(backData))
            # 返回读取到的数据块
            return backData


    def MFRC522_Init(self):
        # 调用M500PcdConfigISOType方法，传入' A'参数，表示使用ISO14443_A协议进行通信
        self.M500PcdConfigISOType('A')
        # 延时2毫秒
        time.sleep_ms(2)

    def M500PcdConfigISOType(self, ucType):
        # 如果传入的参数为' A'，表示使用ISO14443_A协议进行通信
        if ucType == 'A':  # ISO14443_A
            # 清除Status2Reg寄存器的8位
            self.ClearBitMask(self.Status2Reg, 0x08)
            # 将ModeReg寄存器的值设为0x3D（3F）
            self.Write_MFRC522(self.ModeReg, 0x3D)  # 3F
            # 将RxSelReg寄存器的值设为0x86（84）
            self.Write_MFRC522(self.RxSelReg, 0x86)  # 84
            # 将RFCfgReg寄存器的值设为0x7F（4F）
            self.Write_MFRC522(self.RFCfgReg, 0x7F)  # 4F
            # 将TReloadRegL寄存器的值设为30
            self.Write_MFRC522(self.TReloadRegL, 30)
            # 将TReloadRegH寄存器的值设为0
            self.Write_MFRC522(self.TReloadRegH, 0)
            # 将TModeReg寄存器的值设为0x8D
            self.Write_MFRC522(self.TModeReg, 0x8D)
            # 将TPrescalerReg寄存器的值设为0x3E
            self.Write_MFRC522(self.TPrescalerReg, 0x3E)
        else:
            # 如果传入的参数不是' A'，则打印错误信息
            print("unk ISO type\r\n")




#=================================  FACE  =====================================

# 加载模型

#task_fd = kpu.load(0x100000) # 从flash 0x100000 加载人脸检测模型
#task_ld = kpu.load(0x200000) # 从flash 0x200000 加载人脸五点关键点检测模型
#task_fe = kpu.load(0x300000) # 从flash 0x300000 加载人脸196维特征值模型
task_fd = kpu.load("/sd/FaceDetection.smodel")
task_ld = kpu.load("/sd/FaceLandmarkDetection.smodel")
task_fe = kpu.load("/sd/FeatureExtraction.smodel")


#####################配置蓝牙串口#####################
fm.register(34, fm.fpioa.UART1_TX, force=True)
fm.register(35, fm.fpioa.UART1_RX, force=True)

uart = UART(UART.UART1, 9600, 8, 1, 0, timeout=1000, read_buf_len=4096)

#####################配置舵机####################
#PWM通过定时器配置
tim = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PWM)
SS = PWM(tim, freq=50, duty=0, pin=21)
#'''
#说明：舵机控制函数
#功能：180度舵机：angle:-90至90 表示相应的角度
     #360连续旋转度舵机：angle:-90至90 旋转方向和速度值。
    #【duty】占空比值：0-100
#'''
def Servo(servo,angle):
    SS.duty((angle+90)/180*10+2.5)
Servo(SS,0)

g_led = 12
r_led = 13
fm.register(r_led, fm.fpioa.GPIO0)     # 将引脚 r_led 映射为了 GPIO0 的功能
fm.register(g_led, fm.fpioa.GPIO1)
led_r=GPIO(GPIO.GPIO0, GPIO.OUT)
led_g=GPIO(GPIO.GPIO1, GPIO.OUT)
# 一开始两个灯都灭
led_r.value(1)
led_g.value(1)


#####################配置蜂鸣器#####################
beep_pin=22
fpioa = FPIOA()
#PWM 通过定时器配置，接到 IO15 引脚，蜂鸣器
tim2 = Timer(Timer.TIMER0, Timer.CHANNEL1, mode=Timer.MODE_PWM)
beep = PWM(tim2, freq=1, duty=50, pin=beep_pin)
beep.disable()
#beep.enable()
#beep.freq(800)

#####################配置人体红外传感#####################
fpioa.set_function(23,FPIOA.GPIO7)
people_find = GPIO(GPIO.GPIO7,GPIO.IN)

lcd.init()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_hmirror(1)
sensor.set_vflip(1)
sensor.run(1)
anchor = (1.889, 2.5245, 2.9465, 3.94056, 3.99987, 5.3658, 5.155437,
          6.92275, 6.718375, 9.01025)  # anchor for face detect
dst_point = [(44, 59), (84, 59), (64, 82), (47, 105),
             (81, 105)]  # standard face key point position
a = kpu.init_yolo2(task_fd, 0.5, 0.3, 5, anchor)
img_lcd = image.Image()
img_face = image.Image(size=(128, 128))
a = img_face.pix_to_ai()
record_ftr = []
record_ftrs = []
names = []
record_ftrtemp=[]  #临时存储录入时的人脸特征





################## 开机时读取SD卡中的人脸信息 ##################
i = 0
temp_num = ''
with open("/sd/faceinfo.txt", "r") as f:
    while(1):
        lin = f.readline()
        if not lin:
            break
        stu_num = lin[0:lin.index('#')]    #获取学号
        names.append(stu_num)              #追加到学号列表
        faceftr = lin[lin.index('#')+1:]   #截取后半段的人脸特征
        record_ftrs.append(eval(faceftr))  #向人脸特征列表中添加SD卡中的已存特征
        if stu_num == temp_num:
            continue
        temp_num = stu_num
        i = i + 1
        print("%d:%s#%s" % (i,stu_num,faceftr))
    f.close()


#####################配置按键####################
LuRu_mode = False   # 录入模式
Door_mode = False   # 门禁模式
Normal_mode = True  # 正常模式

fpioa = FPIOA()
fm.register(16, fm.fpioa.GPIOHS0)
key_gpio = GPIO(GPIO.GPIOHS0, GPIO.IN)
fm.register(24, fm.fpioa.GPIOHS1)
key_gpio1 = GPIO(GPIO.GPIOHS1, GPIO.IN, GPIO.PULL_UP)
fm.register(25, fm.fpioa.GPIOHS2)
key_gpio2 = GPIO(GPIO.GPIOHS2, GPIO.IN, GPIO.PULL_UP)
#fm.register(20, fm.fpioa.GPIOHS3)
#key_gpio3 = GPIO(GPIO.GPIOHS3, GPIO.IN, GPIO.PULL_UP)

last_key_state=1
key_pressed=0
last_key_state1=1
key_pressed1=0
last_key_state2=1
key_pressed2=0
#last_key_state3=1
#key_pressed3=0

################### RFID config ###################
CS_NUM = const(18)
SPI_FREQ_KHZ = const(600)
SPI_SCK = const(19)
SPI_MOSI = const(8)
SPI_MISO = const(15)

def check_key(): # 按键检测函数，用于在循环中检测按键是否按下，下降沿有效
    global last_key_state
    global key_pressed
    global last_key_state1
    global key_pressed1
    global last_key_state2
    global key_pressed2
    global LuRu_mode
    global Door_mode
    global Normal_mode
    #global last_key_state3
    #global key_pressed3
    #global RFID_mode

    val=key_gpio.value()
    val1=key_gpio1.value()
    val2=key_gpio2.value()
    #val3=key_gpio3.value()
    if last_key_state == 1 and val == 0:
        LuRu_mode = False
        Door_mode = False
        Normal_mode = True
        #RFID_mode = False
        #lcd.clear()
    if last_key_state1 == 1 and val1 == 0:
        LuRu_mode = True
        Door_mode = False
        Normal_mode = False
        #RFID_mode = False
        #lcd.clear()
    if last_key_state2 == 1 and val2 == 0:
        LuRu_mode = False
        Door_mode = True
        Normal_mode = False
        #RFID_mode = False
        #lcd.clear()
    #if last_key_state3 == 1 and val3 == 0:
        #LuRu_mode = False
        #Door_mode = False
        #Normal_mode = False
        #RFID_mode = True

    last_key_state = val
    last_key_state1 = val1
    last_key_state2 = val2
    #last_key_state3 = val3

#####################配置看门狗#####################
def on_wdt(self):
    self.feed()
wdt0 = WDT(id=1, timeout=5000, callback=on_wdt, context={})

###################################################################
##                            主循环                              ##
###################################################################

# 设置各种flag
no_people = 0 # 是否无人
no_flag = 0
delline = []
check_num = 0
shibie_num = 0
uart.read()

# 设置人脸对比置信度
ACCURACY = 75
# 录入模式flag
luru_flag = 0
# 查找模式flag
find_flag = 0

feature = ''
max_score = 0
index = 0
while (1):
    # 程序首先调用了check_key()函数，该函数可能用于检测设备是否已经授权使用相关的模型。
    check_key()
    # 调用了sensor.snapshot()函数获取一张图像。
    img = sensor.snapshot()
    # 使用kpu.run_yolo2()函数对输入的图像进行人脸检测，返回一个包含检测结果的列表code。
    code = kpu.run_yolo2(task_fd, img)
    max_score = 0
    # 如果code不为空，即检测到人脸，程序会遍历code中的每个检测结果，计算人脸框的面积，并选择面积最大的人脸进行处理。
    if code:
        t = 0
        max_face = 0
        totalRes = len(code)
        area = []
        # 如果检测到的人脸数大于1，则只处理面积最大的人脸。
        if totalRes > 1:  #多张人脸时，挑选面积最大的，一般即最前面的
            for i in code: # 迭代坐标框  多张人脸
                area.append(i.w()*i.h())
            for j in range(len(area)):
                if max_face < area[j]:
                    max_face = area[j]
                    t = j    #保存最大脸面积下标
            totalRes = 1
            del area
        if totalRes == 1:  #只有一张脸
            i = code[t]
            # 首先使用img.draw_rectangle()函数在原图像上绘制人脸框
            # Cut face and resize to 128x128
            a = img.draw_rectangle(i.rect())
            # 然后使用img.cut()函数对原图像进行裁剪，得到人脸图像。
            face_cut = img.cut(i.x(), i.y(), i.w(), i.h())
            # 使用img.resize()函数将人脸图像缩放为128x128大小
            face_cut_128 = face_cut.resize(128, 128)
            # 使用img.pix_to_ai()函数将图像转换为KPU能够处理的格式。
            a = face_cut_128.pix_to_ai()
            # a = img.draw_image(face_cut_128, (0,0))
            # Landmark for face 5 points
            # 程序使用kpu.forward()函数对人脸图像进行特征点检测，得到人脸的五个关键点坐标。
            fmap = kpu.forward(task_ld, face_cut_128)
            plist = fmap[:]
            le = (i.x() + int(plist[0] * i.w() - 10), i.y() + int(plist[1] * i.h()))
            re = (i.x() + int(plist[2] * i.w()), i.y() + int(plist[3] * i.h()))
            nose = (i.x() + int(plist[4] * i.w()), i.y() + int(plist[5] * i.h()))
            lm = (i.x() + int(plist[6] * i.w()), i.y() + int(plist[7] * i.h()))
            rm = (i.x() + int(plist[8] * i.w()), i.y() + int(plist[9] * i.h()))
            # 使用img.draw_circle()函数在原图像上绘制出这些关键点的位置。
            a = img.draw_circle(le[0], le[1], 4)
            a = img.draw_circle(re[0], re[1], 4)
            a = img.draw_circle(nose[0], nose[1], 4)
            a = img.draw_circle(lm[0], lm[1], 4)
            a = img.draw_circle(rm[0], rm[1], 4)
            # align face to standard position
            src_point = [le, re, nose, lm, rm]
            # 程序使用这些关键点的坐标计算人脸的仿射变换矩阵，并使用image.get_affine_transform()函数计算变换矩阵。
            T = image.get_affine_transform(src_point, dst_point)
            # 使用image.warp_affine_ai()函数将人脸图像进行对齐，得到标准的人脸图像。
            a = image.warp_affine_ai(img, img_face, T)
            # 然后，程序使用img.ai_to_pix()函数将对齐后的图像转换为原图像能够显示的格式。
            a = img_face.ai_to_pix()
            # a = img.draw_image(img_face, (128,0))
            del (face_cut_128)
            # calculate face feature vector
            # 程序使用kpu.forward()函数对对齐后的人脸图像进行特征提取
            fmap = kpu.forward(task_fe, img_face)
            # 使用kpu.face_encode()函数将特征向量进行编码
            feature = kpu.face_encode(fmap[:])
            reg_flag = False
            scores = []
            # 程序遍历预先录入的人脸特征列表
            for j in range(len(record_ftrs)):
                # 使用kpu.face_compare()函数计算当前人脸特征向量与列表中每个特征向量的相似度得分。
                score = kpu.face_compare(record_ftrs[j], feature)
                scores.append(score)
            max_score = 0
            index = 0
            # 最终，程序选择得分最高的特征向量，并返回其在列表中的索引。这个索引可以用于识别当前人脸是否为已知人脸。
            for k in range(len(scores)):
                if max_score < scores[k]:
                    max_score = scores[k]
                    index = k
    # 执行录入任务
    if luru_flag == 1:
        # 如果检测到当前画面中存在人脸，且识别率超过预设值
        if max_score > ACCURACY:
            # 人脸采集次数清零
            check_num = 0
            # 在屏幕上显示提示信息“Face Exist”
            a = img.draw_string(200,0, b'Face Exist', color=(255,0,0),scale=1.6,mono_space=1)  #提示人脸已存在
            # 在屏幕上显示图像
            a = lcd.display(img)
            # 继续循环，等待下一张图像的检测和处理
            continue
        # 如果检测到人脸特征
        if code:
            # 记录当前采集的人脸数量，每检测到一次人脸就加1
            check_num = check_num + 1     #检测到一次人脸则加 1
            # 在屏幕上显示当前采样进度
            a = img.draw_string(5,40, b'%d'%check_num, color=(0,255,0),scale=1.4,mono_space=1) #显示采样进度
            # 间隔录入，每采集6次人脸特征就将当前特征加入到临时特征值列表中
            if check_num % 6 == 0 and check_num != 0:
                record_ftrtemp.append(feature)   #加入当前人脸特征
            # 如果已采集到18次人脸特征，表示录入结束
            if check_num == 18:
            # 录入成功标志位设为1
                save_success = 1
                try:
                    # 尝试打开SD卡上的faceinfo.txt文件，并以追加模式写入数据
                    with open("/sd/faceinfo.txt", "a") as f:
                        for i in range(len(record_ftrtemp)):  #循环遍历临时特征值列表，写入SD卡
                            f.write(stu_num+'#'+str(record_ftrtemp[i]))
                            # 每个特征值占一行
                            f.write("\n")
                        # 关闭文件
                        f.close()
                except Exception:
                    save_success = 0  #表示保存到SD卡失败了
                    pass
                # 如果保存到SD卡成功，则将当前特征添加到已知特征列表中，并将学号添加到names列表中；同时清空临时特征值列表和采集次数，显示录入成功的提示信息，并进行蜂鸣器声音提示；最后退出录入任务并回到正常模式。
                if save_success == 1:
                    #将当前特征添加到已知特征列表，学号添加到names
                    for i in record_ftrtemp:
                        record_ftrs.append(i)
                        names.append(stu_num)
                    record_ftrtemp.clear()  #清空临时特征值列表
                    # 在屏幕上显示录入成功的提示信息
                    a = img.draw_string(0,5, b'Success!', color=(255,0,0),scale=1.4,mono_space=1)  #录入成功
                    # 在屏幕上显示图像
                    a = lcd.display(img)

                    #蜂鸣器声音提示
                    beep.enable()
                    beep.freq(1000)
                    time.sleep(2)
                    beep.disable()
                    luru_flag = 0  # 退出录入任务
                    # 回到正常模式
                    LuRu_mode = False   # 录入模式
                    Door_mode = False   # 门禁模式
                    Normal_mode = True  # 正常模式
                #如果保存到SD卡失败，则按键次数清零，check_num清零，学号清空
                else:
                    a = img.draw_string(0,5, b'Fail!', color=(255,0,0),scale=1.4,mono_space=1)  #录入失败
                    #蜂鸣器声音提示
                    beep.enable()
                    beep.freq(600)
                    time.sleep(2)
                    beep.disable()
                    luru_flag = 0  # 退出录入任务
                    # 回到正常模式
                    LuRu_mode = False   # 录入模式
                    Door_mode = False   # 门禁模式
                    Normal_mode = True  # 正常模式
        # 删除code变量，释放内存空间。
        del code

    # 有人的时候LCD显示摄像头的拍摄画面
    if people_find.value() == 1:
        no_flag = 1



        # 接收蓝牙数据
        # 垃圾回收
        gc.collect()
        # 读取蓝牙数据
        text = uart.read()
        if text != None and len(text) >= 2: #如果读取到了数据，且大于等于2
            # 临时开门禁
            # 如果蓝牙数据中包含 'open'
            if 'open' in text:
                print("--------蓝牙开门--------")
                # 舵机旋转
                Servo(SS,90)
                time.sleep(2)
                Servo(SS,0)

            # 删除全部用户命令
            # 如果蓝牙数据中包含 'erase'
            if 'erase' in text:
                print("--------删除全部用户--------")
                # 清空名称列表、特征值列表和人脸信息文件
                names.clear()
                record_ftrs.clear()
                # 打开文件
                file_new = open('/sd/faceinfo.txt', 'w')
                # 写入空字符串
                file_new.write(''.join(''))
                # 关闭文件
                file_new.close()
                # 清除图像
                img.clear()
                # 绘制矩形
                img.draw_rectangle((90, 85, 140, 70), fill=True, color=(0, 0, 255))
                # 绘制字符串
                img.draw_string(110, 112, "delete all", color=(255, 255, 255), scale=1.5, mono_space=0)
                # 在LCD屏幕上显示图像
                lcd.display(img)
                #蜂鸣器声音提示
                beep.enable()
                beep.freq(1000)
                time.sleep(2)
                beep.disable()
                a = img.clear()

            # 删除用户命令
            # 如果蓝牙数据中包含 'delete'
            if 'del' in text:
                print("--------删除用户--------")
                delnum = text[3:].decode('utf-8')  #截取出删除的学号
                print('删除的学号: ', delnum)
                print(delnum)
                del text
                k=-1
                # 打开人脸信息文件
                with open('/sd/faceinfo.txt', 'r') as f:
                    while(1):
                    # 逐行读取文件内容
                        thisline=f.readline()
                        if not thisline:
                            # 如果文件已读完
                            break
                        # 计数器加1
                        k=k+1
                        # 判断是否找到要删除的学号
                        result = delnum in thisline
                        # 如果找到了第一个匹配项
                        if result==True and find_flag == 0:
                            # 标记为找到
                            find_flag=1
                            print('ok')
                        # 如果找到了多个匹配项
                        if result==True and find_flag == 1:
                            # 记录要删除的行数
                            delline.append(k)
                        # 如果匹配项已结束
                        if result==False and find_flag == 1:
                            # 标记为未找到
                            find_flag = 0
                            break
                    # 关闭文件
                    f.close()
                # 如果SD卡中有此人，那么就删除，
                if delline:
                    # 按行读入，删除最后一行
                    # 打开文件
                    file_old = open('/sd/faceinfo.txt', 'r')
                    # 逐行读取文件内容
                    lines = [i for i in file_old]
                    # 删除指定行数
                    del lines[int(delline[0]):int(delline[-1])+1]
                    # 关闭文件
                    file_old.close()
                    del file_old
                    # 清空要删除的行数列表
                    delline.clear()
                    # 再覆盖写入
                    # 打开文件
                    file_new = open('/sd/faceinfo.txt', 'w')
                    # 将修改后的内容写入文件
                    file_new.write(''.join(lines))
                    # 关闭文件
                    file_new.close()
                    del lines
                    ##删除姓名列表和特征值列表中的数据
                    #print('之前',names)
                    i = 0
                    # 清空名称列表
                    temp_num = ''
                    names = []
                    # 清空特征值列表
                    record_ftrs = []
                    # 垃圾回收
                    gc.collect()
                    try:
                        # 打开人脸信息文件
                        with open("/sd/faceinfo.txt", "r") as f:
                            while(1):
                                # 垃圾回收
                                gc.collect()
                                # 逐行读取文件内容
                                lin = f.readline()
                                # 如果文件已读完
                                if not lin:
                                    break
                                stunum = lin[0:lin.index('#')]    #获取学号
                                lin = lin[lin.index('#')+1:]      #截取除了学号以外的字符串
                                stu_name = lin[0:lin.index('#')]  #获取姓名
                                names.append(stunum+'#'+stu_name) #追加到姓名列表
                                lin = lin[lin.index('#')+1:]      #截取人脸特征
                                record_ftrs.append(eval(lin))     #向人脸特征列表中添加已存特征
                    except:
                        pass
                    # 垃圾回收
                    gc.collect()
                    # 清除图像
                    img.clear()
                    # 绘制矩形
                    img.draw_rectangle((90, 85, 140, 70), fill=True, color=(0, 0, 255))
                    # 绘制字符串
                    img.draw_string(110, 112, "delete %s"%delnum, color=(255, 255, 255), scale=1.5, mono_space=0)
                    # 在LCD屏幕上显示图像
                    lcd.display(img)
                    #蜂鸣器声音提示
                    beep.enable()
                    beep.freq(1000)
                    time.sleep(2)
                    beep.disable()
                    a = img.clear()

                    # 回到正常模式
                    LuRu_mode = False   # 录入模式
                    Door_mode = False   # 门禁模式
                    Normal_mode = True  # 正常模式
                else:
                    a = img.clear()
                    # 绘制矩形
                    a = img.draw_rectangle((90, 85, 140, 70), fill=True, color=(0, 0, 255))
                    # 绘制字符串
                    a = img.draw_string(118, 112, "No people!", color=(255, 255, 255), scale=1.5, mono_space=0)
                    # 在LCD屏幕上显示图像
                    a = lcd.display(img)
                    #蜂鸣器声音提示
                    beep.enable()
                    beep.freq(600)
                    time.sleep(2)
                    beep.disable()
                    a = img.clear()
                    # 回到正常模式
                    LuRu_mode = False   # 录入模式
                    Door_mode = False   # 门禁模式
                    Normal_mode = True  # 正常模式

        # 检测按键
        # 录入模式按键
        if LuRu_mode:
            try:
                # 在屏幕上显示“录入模式”字样
                a = img.draw_string(0,0, b'Register Mode', color=(0,255,0),scale=1.6,mono_space=1)
            except:
                pass
            # 判断蓝牙数据是否读取到并且长度大于等于2
            if text != None and len(text) >= 2: #如果读取到了数据，且大于等于2
                # 判断是否收到“register”命令，如果是则进行人脸注册
                if 'register' in text:
                    print("--------人脸注册--------")
                    stu_num = text[2:].decode('utf-8')  # 截取出学号
                    print('学号: ', stu_num)

                    # 判断该学号是否已被录入
                    # 如果该学号已经被录入，显示“ID Exist!”字样，并在屏幕上显示红色矩形区域
                    if stu_num in names:
                        a = img.clear()
                        a = img.draw_rectangle((90, 85, 140, 70), fill=True, color=(0, 0, 255))
                        a = img.draw_string(118, 112, "ID Exist!", color=(255, 255, 255), scale=1.5, mono_space=0)
                        a = lcd.display(img)
                        #蜂鸣器声音提示
                        beep.enable()
                        beep.freq(600)
                        time.sleep(2)
                        beep.disable()
                        # 回到正常模式
                        LuRu_mode = False   # 录入模式
                        Door_mode = False   # 门禁模式
                        Normal_mode = True  # 正常模式
                    # 如果学号没有被录入，进行下一步录入
                    else:
                        # 设置录入标志位为1
                        luru_flag = 1

        # 门禁模式按键
        if Door_mode:
        # 录入标志位设置为0
            luru_flag = 0
            # 校验次数设置为0
            check_num = 0
            # 在屏幕上显示“门禁模式”字样
            a = img.draw_string(0,0, b'Entrance Mode', color=(0,255,0),scale=1.6,mono_space=1)
            # ================================== RFID ================================

            # time.sleep(2)
            # from micropython import const


            #############################################

            #continue_reading = True

            # 20: CS_NUM;
            fm.register(CS_NUM, fm.fpioa.GPIOHS20, force=True)

            # set gpiohs work mode to output mode
            cs = GPIO(GPIO.GPIOHS20, GPIO.OUT)

            spi1 = SPI(SPI.SPI_SOFT, mode=SPI.MODE_MASTER, baudrate=SPI_FREQ_KHZ * 1000,
                    polarity=0, phase=0, bits=8, firstbit=SPI.MSB, sck=SPI_SCK, mosi=SPI_MOSI, miso=SPI_MISO)

            # Create an object of the class MFRC522
            MIFAREReader = MFRC522(spi1, cs)

            # time.sleep_ms(300)
            # Scan for cards
            (status, ataq) = MIFAREReader.MFRC522_Request(MIFAREReader.PICC_REQALL)

            # If a card is found
            if status == MIFAREReader.MI_OK:
                # 如果卡片被找到，打印卡片类型和UID
                print("Card detected type: ",hex(ataq[0]<<8|ataq[1]))
                # Get the UID of the card
                (status, uid) = MIFAREReader.MFRC522_Anticoll()

                # If we have the UID, continue
                if status == MIFAREReader.MI_OK:

                    # Print UID
                    print("Card read UID: " +
                        str(uid[0])+","+str(uid[1])+","+str(uid[2])+","+str(uid[3]))

                    # This is the default key of M1(S50) for authentication
                    # M1卡片的默认密钥
                    key = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]

                    # Select the scanned tag
                    # 选择扫描到的标签
                    MIFAREReader.MFRC522_SelectTag(uid)

                    # Authenticate
                    # 验证卡片密钥
                    status = MIFAREReader.MFRC522_Auth(
                        MIFAREReader.PICC_AUTHENT1A, 0x12, key, uid)

                    # Check if authenticated
                    # 检查是否验证成功
                    #if status == MIFAREReader.MI_OK:
                        # 示例：定义一个包含16个元素的列表，并将第一个元素设为'Y'
                        ##data = ['Y',0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                        ##data = ['N',0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                        ## Fill the data with 0~16
                        #for x in range(0, 16):
                            #data.append(x)

                        ## Write the data
                        # 向扇区0x12写入数据
                        #print("Sector 11 will now be filled with 1~16:")
                        # 使用MFRC522_Write方法将data列表写入扇区0x12
                        #status = MIFAREReader.MFRC522_Write(0x12, data)
                    # 如果验证成功，读取RFID模块中的数据
                    try:
                        if status == MIFAREReader.MI_OK:
                            print("start to read")
                            # read the data
                            # MIFAREReader.MFRC522_Read(0x12)
                            datas = MIFAREReader.MFRC522_Read(0x12)
                            #读出卡中的id号
                            strnum = ' '
                            strnum = chr(datas[0])+str(datas[1])+str(datas[2])+str(datas[3])+str(datas[4])
                            #如果第一个字符不是B则说明卡错误
                            if chr(datas[0]) == 'Y':
                                led_g.value(0)  # 绿灯亮
                                Servo(SS,90)
                                time.sleep(2)
                                Servo(SS,0)
                                led_g.value(1)  # 绿灯灭

                            if chr(datas[0]) == 'N':
                                #蜂鸣器声音提示
                                beep.enable()
                                beep.freq(300)
                                time.sleep(2)
                                beep.disable()
                                ##读出卡中的姓名
                                #if datas[11]==0: #名字两个字
                                    #strname = "b'"+hex(datas[5])+hex(datas[6])+hex(datas[7])+hex(datas[8])+hex(datas[9])+hex(datas[10])+"'"
                                #else :          #名字三个字
                                    #strname = "b'"+hex(datas[5])+hex(datas[6])+hex(datas[7])+hex(datas[8])+hex(datas[9])+hex(datas[10])+hex(datas[11])+hex(datas[12])+hex(datas[13])+"'"
                                #strname = eval(strname.replace('0','\\')).decode('utf8')

                            # Stop
                            MIFAREReader.MFRC522_StopCrypto1()
                        #else:
                            #print("Authentication error")
                    except Exception as e:
                         beep.enable()
                         beep.freq(1000)
                         time.sleep(2)
                         beep.disable()


            try:
                # 如果人脸识别分数高于设定的阈值，显示学号与分数，并开启绿灯
                if max_score > ACCURACY:
                    a = img.draw_string(i.x()+4,i.y()-20, ("%s: %2.1f" % (names[index], max_score)), color=(0,255,0),scale=2) # 显示学号与分数
                    a = lcd.display(img)
                    print("--------人脸识别成功--------")
                    led_g.value(0)  # 绿灯亮
                    Servo(SS,90)
                    time.sleep(2)
                    Servo(SS,0)
                    led_g.value(1)  # 绿灯灭
                    shibie_num = 0
                elif code:
                    # 如果人脸识别分数低于设定的阈值，显示分数，并计数，如果计数超过3次则表示人脸识别失败，开启红灯并发出蜂鸣器声音
                    img.draw_string(i.x()+4,i.y()-20, ("%s: %2.1f" % ("No",max_score)), color=(0,255,0),scale=2) # 显示分数
                    a = lcd.display(img)
                    shibie_num = shibie_num + 1
                    if shibie_num > 3:
                        shibie_num = 0
                        print("--------人脸识别失败--------")
                        led_r.value(0)  # 红灯亮
                        beep.enable()
                        beep.freq(600)
                        time.sleep(2)
                        beep.disable()
                        led_r.value(1)  # 红灯灭
            except:
                pass
                # 删除变量code，释放内存空间
            del code

        # 正常模式按键
        # 设置录入标志和检查数字为0，使用LCD显示屏显示“等待……”
        if Normal_mode:
            luru_flag = 0
            check_num = 0
            a = img.draw_string(0,0, b'Waiting......', color=(0,255,0),scale=1.6,mono_space=1)
            ## ================================== RFID ================================

            ## time.sleep(2)
            ## from micropython import const
            #################### config ###################
            #CS_NUM = const(18)
            #SPI_FREQ_KHZ = const(600)
            #SPI_SCK = const(19)
            #SPI_MOSI = const(8)
            #SPI_MISO = const(15)

            ##############################################

            ##continue_reading = True

            ## 20: CS_NUM;
            #fm.register(CS_NUM, fm.fpioa.GPIOHS20, force=True)

            ## set gpiohs work mode to output mode
            #cs = GPIO(GPIO.GPIOHS20, GPIO.OUT)

            #spi1 = SPI(SPI.SPI_SOFT, mode=SPI.MODE_MASTER, baudrate=SPI_FREQ_KHZ * 1000,
                    #polarity=0, phase=0, bits=8, firstbit=SPI.MSB, sck=SPI_SCK, mosi=SPI_MOSI, miso=SPI_MISO)

            ## Create an object of the class MFRC522
            #MIFAREReader = MFRC522(spi1, cs)

            ## time.sleep_ms(300)
            ## Scan for cards
            #(status, ataq) = MIFAREReader.MFRC522_Request(MIFAREReader.PICC_REQALL)

            ## If a card is found
            #if status == MIFAREReader.MI_OK:
                #print("Card detected type: ",hex(ataq[0]<<8|ataq[1]))
                ## Get the UID of the card
                #(status, uid) = MIFAREReader.MFRC522_Anticoll()

                ## If we have the UID, continue
                #if status == MIFAREReader.MI_OK:

                    ## Print UID
                    #print("Card read UID: " +
                        #str(uid[0])+","+str(uid[1])+","+str(uid[2])+","+str(uid[3]))

                    ## This is the default key of M1(S50) for authentication
                    #key = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]

                    ## Select the scanned tag
                    #MIFAREReader.MFRC522_SelectTag(uid)

                    ## Authenticate
                    #status = MIFAREReader.MFRC522_Auth(
                        #MIFAREReader.PICC_AUTHENT1A, 0x12, key, uid)

                    ## Check if authenticated
                    ##if status == MIFAREReader.MI_OK:
                        ###data = ['Y',0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                        ###data = ['N',0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                        ### Fill the data with 0~16
                        ##for x in range(0, 16):
                            ##data.append(x)

                        ### Write the data
                        ##print("Sector 11 will now be filled with 1~16:")
                        ##status = MIFAREReader.MFRC522_Write(0x12, data)
                    #try:
                        #if status == MIFAREReader.MI_OK:
                            #print("start to read")
                            ## read the data
                            ## MIFAREReader.MFRC522_Read(0x12)
                            #datas = MIFAREReader.MFRC522_Read(0x12)
                            ##读出卡中的id号
                            #strnum = ' '
                            #strnum = chr(datas[0])+str(datas[1])+str(datas[2])+str(datas[3])+str(datas[4])
                            ##如果第一个字符不是B则说明卡错误
                            #if chr(datas[0]) == 'Y':
                                #led_g.value(0)  # 绿灯亮
                                #Servo(SS,90)
                                #time.sleep(2)
                                #Servo(SS,0)
                                #led_g.value(1)  # 绿灯灭

                            #if chr(datas[0]) == 'N':
                                ##蜂鸣器声音提示
                                #beep.enable()
                                #beep.freq(300)
                                #time.sleep(2)
                                #beep.disable()
                                ###读出卡中的姓名
                                ##if datas[11]==0: #名字两个字
                                    ##strname = "b'"+hex(datas[5])+hex(datas[6])+hex(datas[7])+hex(datas[8])+hex(datas[9])+hex(datas[10])+"'"
                                ##else :          #名字三个字
                                    ##strname = "b'"+hex(datas[5])+hex(datas[6])+hex(datas[7])+hex(datas[8])+hex(datas[9])+hex(datas[10])+hex(datas[11])+hex(datas[12])+hex(datas[13])+"'"
                                ##strname = eval(strname.replace('0','\\')).decode('utf8')

                            ## Stop
                            #MIFAREReader.MFRC522_StopCrypto1()
                        ##else:
                            ##print("Authentication error")
                    #except Exception as e:
                         #beep.enable()
                         #beep.freq(1000)
                         #time.sleep(2)
                         #beep.disable()

        a = lcd.display(img)

    # 无人的时候LCD黑屏，休眠状态（录入的时候不能进入休眠状态）
    # 如果没有人在操作，将LCD清空并显示“休眠中……”，并将标志位no_people设置为1。
    elif luru_flag == 0:
        no_people = 1
    # 如果有人在操作，则将标志位no_people和no_flag都置为0，表示有人正在操作，不进行休眠。
    if no_people == 1 and no_flag == 1:
        no_people = 0
        no_flag = 0
        a = img.clear()   # 清空一次LCD
        # 最后，将img对象在LCD上显示出来，并回收内存。
        a = img.draw_string(40,110, b'Suspending......', color=(0,255,0),scale=2,mono_space=1)
        a = lcd.display(img)
    # del img用于删除img对象占用的内存
    del img
    # gc.collect()用于回收未被使用的内存。
    gc.collect()

#########################RFID###############################################



# a = kpu.deinit(task_fe)
# a = kpu.deinit(task_ld)
# a = kpu.deinit(task_fd)


