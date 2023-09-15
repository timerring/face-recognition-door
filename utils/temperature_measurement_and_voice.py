import ustruct,time
from machine import I2C
from micropython import const
import sensor
import image,lcd
from fpioa_manager import fm
from machine import UART

##################### 配置lu90614测温模块 #####################
fm.register(31, fm.fpioa.UART3_TX, force=True)
fm.register(32, fm.fpioa.UART3_RX, force=True)

uart3 = UART(UART.UART3, 9600, 8, 1, 0, timeout=1000, read_buf_len=4096)

data = uart3.read()
read_flag = 0
count = 0

code1=[0xfa,0xc6,0xc0]  # 物体温度指令
code2=[0xfa,0xc5,0xbf]  # 人体温度指令
code3=[0xfa,0xca,0xc4]  # 返回温度指令

def read_temp():
    uart3.write(bytes(code2))  # 发送读取人体温度指令

    uart3.write(bytes(code3))  # 发送返回温度指令

    data = uart3.read()
    if data:
        # 异常处理
        try:
            temp = data[2] + data[3] / 100
            if temp < 100:
                print(temp)
                return temp
            else:
                return 255
        except:
            return 255
    return 255


##################### 语音播报模块 #####################
fm.register(21, fm.fpioa.UART2_TX, force=True)

uart = UART(UART.UART2, 9600, 8, 1, 0, timeout=1000, read_buf_len=4096)

# 初始化词条
nihao = [0xC4, 0xE3, 0xBA, 0xC3]   # 你好
huanying = [0xBB, 0xB6, 0xD3, 0xAD, 0xB9, 0xE2, 0xC1, 0xD9]   # 欢迎光临
shibieshibai = [0xCA , 0xB6 , 0xB1 , 0xF0 , 0xCA , 0xA7 , 0xB0 , 0xDC ]   # 识别失败
shibiechenggong = [0xCA , 0xB6 , 0xB1 , 0xF0 , 0xB3 , 0xC9 , 0xB9 , 0xA6 ]   #识别成功
luruchenggon = [0xC2 , 0xBC , 0xC8 , 0xEB , 0xB3 , 0xC9 , 0xB9 , 0xA6 ]   #录入成功
lurushibai = [0xC2 , 0xBC , 0xC8 , 0xEB , 0xCA , 0xA7 , 0xB0 , 0xDC ]   #录入失败
wenduzhengchang = [0xCE , 0xC2 , 0xB6 , 0xC8 , 0xD5 , 0xFD , 0xB3 , 0xA3]  #温度正常
wenduyichang = [0xCE , 0xC2 , 0xB6 , 0xC8 , 0xD2 , 0xEC , 0xB3 , 0xA3 ]   #温度异常


uart.write(bytes(huanying))   # 后面播报其他内容时，直接将huanying改成相应列表的名字即可

wendu_count = 0
wendu_array = []


##################### 与STM32通信的串口初始化 #####################
#####################配置蓝牙串口#####################
fm.register(9, fm.fpioa.UART1_TX, force=True)
fm.register(10, fm.fpioa.UART1_RX, force=True)

uart_stm32 = UART(UART.UART1, 9600, 8, 1, 0, timeout=1000, read_buf_len=4096)


while True:
    temp = read_temp()
    if temp != 255:
        wendu_array.append(temp)
        wendu_count = wendu_count + 1
        if wendu_count == 5:
            max_temp = max(wendu_array)
            print(wendu_array)
            wendu_array.clear()  # 清空测温列表，便于下次测量
            wendu_count = 0
            if max_temp < 37.5:
                uart.write(bytes(wenduzhengchang))
                uart_stm32.write('{cewen_ok}')
                time.sleep(2)
