# 1.通信系统制作方案概述

## 1.1系统设计的立意

此处略

## 1.2系统的主要组成

设计基于 YOLOv 2 的人脸识别门禁系统，主要由成品模块组成。具体包含：K210 Maix Bit、配套24PIN DVP 摄像头及 LCD 屏、SG90舵机、HC-SR501人体红外感应模块、MFRC-522射频模块、HC-05蓝牙模块、有源蜂鸣器、32G SD 内存卡及读卡器、4位独立按键。
## 1.3系统的制作方案

### 1.3.1制作方案框图

![](https://raw.githubusercontent.com/timerring/scratchpad2023/main/2023/20230914200614.png)
### 1.3.2制作方案原理描述

初步设立门禁系统三大模式，等待、门禁及录入模式。

等待模式无人时显示屏全黑只显示 waiting......；门禁模式打开摄像头锁定人脸显示比对精度（百分制）与识别框，此时 RFID 数据通道打开，并在左上角标注所处模式；

录入模式打开摄像头锁定人脸显示采样进度与识别框，此时蓝牙通道打开，并在左上角标注所处模式，默认等待模式。使用 K210 MAIXBIT 做主控，当红外感知模块检测到人时，屏幕亮起，可识别人脸但无反应（红外感知模块可调节灵敏度与延时，但过于灵敏会导致画面卡顿，反之则检测效果不理想）。使用独立按键切换到录入或门禁模式，30s 未检测到人脸自动进入等待黑屏模式。

录入模式时，使用移动端 app 调试全能王连接蓝牙模块并发送指令：
+ 输入“`register****`”（`****` 代表录入人脸编号）后按照 yolo v 2 算法采集 196 维人脸数据（3 轮 18 次采样），并储存在 SD 卡文件中，且屏幕显示采样进度，录入完成屏幕显示“successful”，蜂鸣器发低声。
+ 输入“`delete****`”则删除对应用户储存在 SD 卡的特征值，蜂鸣器发高声。
+ 输入“`erase`”则删除 SD 卡内所有用户人脸特征值，蜂鸣器发高声。
+ 输入“`open`”则无条件控制舵机旋转。

门禁模式时，设定比对阈值，高于则蜂鸣器发低声且舵机旋转，低于则蜂鸣器发高声舵机无反应。此外 RFID 数据通道开通，我们已给两张 S50标准卡与异形卡相同扇区注入不同信息，分别会有舵机与蜂鸣器的不同响应情形，当其他 RFID 卡靠近则默认发生系统错误并发出独有蜂鸣器声音（可以修改）。
## 1.4系统方案的可行性论证

1. 技术可行性分析:
深度学习算法和神经网络理论已有较成熟应用,图像识别、人脸识别等领域有较多成功案例,技术上实现人脸识别门禁系统是可行的。传感器技术也比较成熟,可以采集人脸图像和相关生物特征,作为识别的输入。系统集成方面,人脸识别算法、传感器技术和门禁系统的结合也是现有技术可以实现的。

2. 经济可行性分析:
相关硬件成本在可接受范围内,如高清摄像头等。软件实现可以采用开源算法和框架,开发成本不高。人脸识别门禁系统可以降低人工成本,尤其适用于人流较大场景,具有较好的经济效益。

3. 操作可行性分析:
人脸识别门禁系统操作简单,易于推广使用,无需复杂训练,符合大多数人的接受习惯。该系统也便于维护和管理,可靠性较高。

综上,从技术、经济和操作等多方面考虑,基于深度学习的人脸识别门禁系统是一种可行的智能门禁方案。但是,数据安全和隐私保护也需要重点考虑,总体而言该方案是值得研究和探索的。
# 2.具体模块分布图及集成原理图

![](https://raw.githubusercontent.com/timerring/scratchpad2023/main/2023/20230914201142.png)
图2-1 人脸识别系统实操图

![](https://raw.githubusercontent.com/timerring/scratchpad2023/main/2023/20230914201208.png)
图2-2 通信系统原理图

# 3.各个模块硬件调试中遇见难题及解决方案

## 3.1蓝牙

HC-05蓝牙模块是一种基于蓝牙协议的简单无线通信设备。该模块基于 BC417单芯片蓝牙 IC，符合蓝牙 v2.0标准，支持 UART 和 USB 接口。

具有两种工作模式：命令响应工作模式和自动连接工作模式。

当模块处于命令响应工作模式（或者AT模式）时能才能执行 AT 命令，用户可向模块发送各种 AT指令，为模块设定控制参数或发布控制命令。（AT指令就是我们PC与一些终端设备（例如蓝牙，WiFi模块）之间进行通信的，配置这些终端设备参数的一套指令。）在自动连接工作模式下模块又可分为主（Master）、从（Slave）和回环（Loopback）三种工作角色。当模块处于自动连接工作模式时，将自动根据事先设定的方式连接的数据传输。主模式：该模块可以主动搜索并连接其它蓝牙模块并接收发送数据。从模式：只能被搜索被其它蓝牙模块连接进行接收发送数据。回环：蓝牙模块就是将接收的数据原样返回给远程的主设备。

初次使用模块由于不知道蓝牙模块密码，故需要再使用 TTL 转串口模块，使用 AT 指令查询和修改蓝牙名称与密码。

![](https://raw.githubusercontent.com/timerring/scratchpad2023/main/2023/20230914201259.png)
图3-1正点原子XCOM串口调试助手图

## 3.2蜂鸣器

蜂鸣器主要用于功能实现的提示音，一开始使用了低电平的蜂鸣器，导致一直响以为是代码不当导致的功能卡顿，多次检查后发现和代码无关，后来选取了高电平触发的蜂鸣器得以成功得到提示音。

## 3.3红外感知模块

红外模块用来感知是否有人在使用，从而开启或者关闭识别元件，红外检测器件的使用难点在于灵敏性调整，需要仔细修改，否则就会导致过快或者不进入休眠模式，从而影响使用。除此之外，不准确的红外检测还会导致器件反应过慢等等的问题，所以在红外器件的修改上下了很多时间才得到比较准确的识别数据。

## 3.4矩阵键盘

用于模式转换，录入人脸模式和正常使用模式，安装需要注意连接线。

## 3.5 RFID

RFID模块：标签进入磁场后，接收解读器发出的射频信号，凭借感应电流所获得的能量发送出存储在芯片中的产品信息（Passive Tag，无源标签或被动标签），或者由标签主动发送某一频率的信号（Active Tag，有源标签或主动标签），解读器读取信息并解码后，再进行有关数据处理。在RFID模块的设计中，为了方便地将标签的信息进行区分，首先设计了不同权限卡之间的对应规则，分别设计了存储信息中Y开头的万能通用卡，N开头的带有序号的普通用户权限卡，以及未规定的其他RFID卡，然后按照设定写了读卡器的写入程序，将对应的规则写入测试卡片，并且对卡片进行了测试。具体要将RFID验证模块加入到哪个环节之中，我们也对应地做了测试，所有流程都会回到普通模式，因此最开始是将RFID是加入在普通模式下，但是经过实际地测试与检验后，发现普通模式并不适合加入RFID模块，应该在检测到有人存在时再启动RFID。 其一，当门禁系统不被使用时，启动RFID模块会浪费大量的能源。通过检测到有人存在再启动RFID模块，可以最大程度地节省能源，降低系统的运行成本。其二，在门禁系统中，启动RFID模块是为了实现对进出人员的身份识别和控制。通过检测到有人存在后再启动RFID模块，可以确保门禁系统只对授权人员开放，提高门禁系统的安全性。最后综合考虑，将RFID模块放置在门禁模式之下。

## 3.6人脸识别模块

由k210芯片，摄像头和LED显示屏组成，用于识别用户脸部数据决定是否开启舵机。原理是YOLO 人脸识别，是一种基于深度学习的目标检测算法，它可以在一张图像中同时检测出多个目标，并且实时性能非常好。YOLO 模型的识别原理是通过将图像分成名个网格，然后对每个网格进行预测，最终将所有网格的预测结果合并起来得到最终的检迎结果。YOLO 模型的输入是一张图像，输出是每个目标的类别、位置和置信度。在训练阶段，YOLO 模型会学习如何将图像分成多个网格，并且对每个网格进行预测。每个网格的大小可以根据图像的大小 或者目标的大小进行调整。对于每个网格，YOLO 模型会预测出多个边界框，每个边界框包含了一个日标的位置和大小信息。同时，YOLO模型还会预测出每个边界框对应的目标的类别和置信度。除了写入训练的数学模型外，还需要设置识别的准确数据分数，超过标准才可以运行下一步功能。还需协调模型的数据存储和循环量，以免占用过多的数据空间，造成大量数据冗余，导致死机。

## 3.7舵机

SG 90舵机需要使用正确的通信协议与门禁系统的其他部件进行通信。在门禁系统中使用SG 90舵机时，应该选择适当的通信协议，例如PWM或者串口通信，并确保通信协议的准确性和稳定性。

# 4.开发平台介绍及代码思路简介

## 4.1 yolo v2算法介绍

![](https://raw.githubusercontent.com/timerring/scratchpad2023/main/2023/20230914201356.png)
图4-1 YOLOv2原理图

1. 输入图片被分成 S x S 个网格(cell)，每个网格负责检测该网格内是否存在目标物体。
2. 对于每个网格，预测出 B 个边界框（bounding box），每个边界框包括(x, y, w, h, confidence)五个元素，其中(x, y)是边界框中心的坐标，w 和 h 是边界框的宽和高，confidence 表示该边界框中包含目标物体的置信度。
3. 对于每个边界框，计算它与 ground truth（真实标注的物体位置）的 IoU（交并比），并选取 IoU 最大的边界框作为该物体的预测框。
4. 使用非极大值抑制（NMS）方法来剔除重叠的预测框，最终输出预测结果。

**关于IoU**：

IoU 是交并比（Intersection over Union）的缩写，是一种用于衡量目标检测算法检测准确性的指标。它是通过计算预测框和 ground truth（真实标注的物体位置）之间的重叠程度来得出的。具体来说，IoU 是预测框和 ground truth 的交集面积除以它们的并集面积，即：

IoU = Intersection over Union = Intersection / Union

其中，Intersection表示预测框和ground truth的交集面积，Union表示它们的并集面积。IoU的取值范围在0到1之间，取值越大表示检测准确性越高。一般来说，当IoU大于某个阈值时，我们就认为预测框和ground truth匹配成功，可以将其作为检测结果；反之则认为匹配失败。

在目标检测算法中，通常会使用IoU作为评价指标来衡量算法的准确性。比如，在YOLOv2算法中，会使用IoU来选取每个边界框中与ground truth匹配的框。在训练过程中，如果预测框与ground truth之间的IoU大于某个阈值，我们就认为这个预测框是正确的，并计算它的损失函数；反之则认为它是错误的，不参与损失函数的计算。通过不断地调整模型参数，最终可以得到一个准确性较高的目标检测算法。

**关于NMS**：

非极大值抑制（Non-Maximum Suppression，NMS）是一种用于目标检测算法中的后处理方法，主要用于剔除重叠的预测框，保留最准确的预测结果。在目标检测算法中，一些目标可能会被多个预测框所检测到，这些预测框之间可能存在重叠。在这种情况下，我们需要对这些预测框进行筛选，保留最准确的预测结果。NMS方法就是用来完成这个任务的。NMS的基本原理如下：

1. 对所有预测框按照其置信度（confidence）进行排序，从置信度最高的预测框开始遍历。
2. 对于当前遍历到的预测框，计算它与之前已经选中的预测框的 IoU（交并比），如果 IoU 大于某个阈值（如0.5），则将该预测框剔除，否则保留该预测框。
3. 重复步骤2和3，直到遍历完所有的预测框。

通过这个过程，NMS方法可以将多个重叠的预测框剔除，保留最准确的预测结果。NMS方法可以有效地解决目标检测算法中的多框问题，提高检测准确性。需要注意的是，NMS方法的效果受到阈值的影响。如果阈值较高，那么将会保留更少的预测框，可能会漏检一些目标；如果阈值较低，那么将会保留更多的预测框，可能会增加误检的概率。

同时，为了提高准确性和速度，YOLOv2采用了一些技巧：

1. Darknet-19网络：YOLOv2使用了一个19层的卷积神经网络 Darknet-19来提取特征。该网络比 VGG16等经典网络更轻量，同时具有更好的准确性。
2. Anchor Boxes：YOLOv2在每个网格上预测 B 个边界框，而不是像 YOLOv1一样只预测一个。此外，YOLOv2使用了 Anchor Boxes，即预定义的边界框，来提高检测准确性。在训练过程中，YOLOv2通过调整 Anchor Boxes 和预测框的位置和大小来进行目标检测。
3. Batch Normalization：在 Darknet-19网络中使用 Batch Normalization 来加速收敛和提高准确性。
4. High Resolution Classifier：YOLOv2使用了一个高分辨率的分类器来提高检测准确性。具体来说，在训练时，YOLOv2将输入图片的分辨率提高到608x608，而在测试时，将其缩小到416x416，以加快处理速度。
5. Convolutional With Anchor Boxes (CWA)：YOLOv2引入了一种新的卷积层，称为 Convolutional With Anchor Boxes (CWA)。该层同时计算多个 Anchor Boxes 的位置和置信度，以提高检测准确性和速度。

总的来说，YOLOv2是一个快速且准确的目标检测算法，它的核心思想是将目标检测问题转化为一个回归问题，并使用神经网络来解决。通过使用Anchor Boxes、Batch Normalization、High Resolution Classifier和Convolutional With Anchor Boxes等技巧，YOLOv2在准确性和速度方面都有所提高。
## 4.2 开发平台

**开发环境：MaixPy IDE**
![](https://raw.githubusercontent.com/timerring/scratchpad2023/main/2023/20230914203339.png)

MaixPy IDE 是一款基于 Python 语言的集成开发环境（IDE），主要用于开发和调试 MaixPy（开源的嵌入式人工智能框架）项目。MaixPy IDE 提供了一系列功能，包括代码编辑、调试、编译、下载、串口调试、固件升级等，方便用户对 MaixPy 进行开发和调试。

**烧录环境：kflash_gui**

![](https://raw.githubusercontent.com/timerring/scratchpad2023/main/2023/20230914203417.png)

kflash_gui是一款开源的图形化固件烧录工具，用于将固件烧录到Kendryte K210芯片上。K210是一款基于RISC-V架构的高性能、低功耗的嵌入式处理器，广泛应用于物联网、人工智能等领域。kflash_gui是Kendryte K210的官方固件烧录工具，提供了一系列功能，包括固件下载、擦除、烧录、调试等。kflash_gui基于Python语言和PyQt库开发，支持命令行和GUI两种模式。用户可以通过命令行模式进行批量烧录等高级操作，也可以通过GUI模式进行简单易用的固件烧录。kflash_gui还提供了一些高级功能，如自动检测串口、自动切换烧录模式、支持多种烧录设备等，方便用户进行高效的固件烧录。

**串口调试环境：XCOM**

![](https://raw.githubusercontent.com/timerring/scratchpad2023/main/2023/20230914203432.png)

1.     支持多个常用波特率，支持自定义波特率
2.     支持5/6/7/8位数据，支持1/1.5/2个停止位
3.     支持奇/偶/无校验
4.     支持16禁止发送/接收显示，支持DTR/RTS控制
5.     支持窗口保存，并可以设置编码格式
6.     支持延时设置，支持时间戳功能
7.     支持定时发送，支持文件发送，支持发送新行
8.     支持多条发送，并关联数字键盘，支持循环发送
9.     支持无限制扩展条数，可自行增删
10.  支持发送条目导出/导入（excel格式）
11.  支持协议传输（类modbus）
12.  支持发送/接收区字体大小、颜色和背景色设置
13.  支持简体中文、繁体中文、英文三种语言
14.  支持原子软件仓库

## 4.3代码解析

![](https://raw.githubusercontent.com/timerring/scratchpad2023/main/2023/20230914205415.png)
图4-3 代码流程图

从左到右依次将代码模块化为三个模块，分别是**人脸数据采集模块，人脸信息录入模块，人脸蓝牙RFID综合验证模块**。下面依次对三个模块的思路进行解析：

首先是**人脸数据采集模块**，这段代码是一个人脸识别的应用程序，主要分为以下几个步骤：首先设置各种标志位和参数，包括是否检测到人、是否录入、查找模式、人脸对比置信度等。

然后执行以下操作：
a. 调用 check_key()函数，可能用于检测设备是否已经授权使用相关的模型。
b. 调用sensor.snapshot()函数获取一张图像。
c. 使用kpu.run_yolo2()函数对输入的图像进行人脸检测，返回一个包含检测结果的列表code。
d. 如果检测到人脸，程序会遍历code中的每个检测结果，计算人脸框的面积，并选择面积最大的人脸进行处理。
e. 使用 img.draw_rectangle()函数在原图像上绘制人脸框，然后使用 img.cut()函数对原图像进行裁剪，得到人脸图像。
f. 对人脸图像进行特征点检测，得到人脸的五个关键点坐标，使用img.draw_circle()函数在原图像上绘制出这些关键点的位置。
g. 使用这些关键点的坐标计算人脸的仿射变换矩阵，并使用image.get_affine_transform()函数计算变换矩阵，将人脸图像进行对齐，得到标准的人脸图像。
h. 使用kpu.forward()函数对对齐后的人脸图像进行特征提取，再使用kpu.face_encode()函数将特征向量进行编码。
i. 遍历预先录入的人脸特征列表，使用kpu.face_compare()函数计算当前人脸特征向量与列表中每个特征向量的相似度得分，并选择得分最高的特征向量，返回其在列表中的索引，用于识别当前人脸是否为已知人脸。
j. 根据人脸识别结果，可以执行不同的操作，如录入新人脸特征、查找已知人脸特征，或者进行相应的提示和处理。

代码详细注释如下：
```python
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
```

其次是**人脸信息录入模块**，主要分为以下几个步骤：

1. 如果检测到当前画面中存在人脸，且识别率超过预设值，程序会在屏幕上显示提示信息“Face Exist”，然后继续循环，等待下一张图像的检测和处理。
2. 如果检测到人脸特征，程序会记录当前采集的人脸数量，每检测到一次人脸就加 1，并在屏幕上显示当前采样进度。程序会间隔录入，每采集 6 次人脸特征就将当前特征加入到临时特征值列表中。
3. 如果已采集到 18 次人脸特征，表示录入结束，程序会将当前特征添加到已知特征列表中，并将编号添加到 names 列表中。然后程序会尝试打开 SD 卡上的 faceinfo. Txt 文件，并以追加模式写入数据。写入完成后，程序会将当前特征添加到已知特征列表中，并将编号添加到 names 列表中。同时清空临时特征值列表和采集次数，显示录入成功或失败的提示信息，并进行蜂鸣器声音提示。最后退出录入任务并回到正常模式。
4. 如果保存到 SD 卡失败，则按键次数清零，check_num 清零，编号清空，并显示录入失败的提示信息。
5. 最后，程序会删除code变量，释放内存空间。

代码详细注释如下：

```python
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
```

最后是**人脸蓝牙RFID综合验证模块**，这个模块相较于前面两个模块，逻辑设计较为复杂，首先判断是否检测到人：程序首先会检测人是否出现在门禁的监控区域，如果有人，则将 no_flag 置为1。同时进行垃圾回收，程序会定期进行垃圾回收，以释放不再使用的内存空间。再接收蓝牙数据：程序会读取蓝牙模块发送的数据，如果读取到了数据且长度大于等于2，则进行后续的处理。
a. 如果读取到的蓝牙数据中包含 'open'，则程序会将舵机旋转以打开门禁，以实现临时门禁的功能。
b. 如果读取到的蓝牙数据中包含 'erase'，执行删除全部用户的操作。具体来说，代码实现了以下功能：
1. 如果读取到的蓝牙数据中包含 'erase'，则执行内部代码块。
2. 清空名称列表、特征值列表和人脸信息文件。
3. 打开人脸信息文件，并将其内容清空。清空图像缓存。
4. 绘制矩形和字符串，并在 LCD 屏幕上显示图像。
5. 发出蜂鸣器声音提示。
c. 如果读取到的蓝牙数据中包含 'delete' ，执行删除指定用户的操作。具体来说，代码实现了以下功能：
1. 如果读取到的蓝牙数据中包含 'delete'，则执行内部代码块。截取出要删除的编号，并打印输出。
2. 逐行读取人脸信息文件，查找要删除的编号，并记录要删除的行数。
3. 如果找到了要删除的行，则打开人脸信息文件，删除指定行，并重新写入文件。
4. 从名称列表和特征值列表中删除指定的用户。
5. 绘制矩形和字符串，并在 LCD 屏幕上显示图像。发出蜂鸣器声音提示。
6. 将程序回到正常模式。
d. 如果当前处于录入模式，则在 LCD 屏幕上显示“Register Mode”字样。如果读取到的蓝牙数据中包含 'register'，则执行人脸注册操作。
1. 截取出要注册的编号，并打印输出。判断该编号是否已经被录入，如果已经被录入，则在 LCD 屏幕上显示“ID Exist!”字样，并在屏幕上显示红色矩形区域，发出蜂鸣器声音提示，并回到正常模式；否则，设置录入标志位为 1。
2. 如果录入标志位为 1，则进行人脸录入操作。
3. 检测到人脸后，提取人脸特征，并将其添加到特征值列表 record_ftrs 中。将编号和姓名组合成一个字符串，并添加到名称列表 names 中。发出蜂鸣器声音提示。将录入标志位设置为 0。
4. 将程序回到正常模式。
e. 如果当前处于门禁模式，脚本初始化 RFID 模块并尝试从中读取数据。如果检测到有效的 RFID 卡片，代码将读取卡片数据并根据不同卡片内容执行相应的操作。如果卡片包含预期数据，则触发一个绿色 LED 和一个舵机来打开门锁。如果卡片包含无效数据，则触发一个红色 LED 和一个蜂鸣器来警告用户。此外，为了更加丰富使用场景，这里还预留了一个万能卡的对应信息，保证管理员可以在特殊情况下进行无限制开门。

除了 RFID 部分，“门禁模式”还同时运行人脸识别算法。如果检测到人脸，比较当前人脸与存储人脸相似的置信度，如果识别的置信度高于设定的阈值，则触发与有效 RFID 卡片相同的操作，控制舵机打开门禁并且显示绿灯。如果人脸识别分数低于设定的阈值，显示分数，并计数，如果计数超过3次则表示人脸识别失败，开启红灯并发出蜂鸣器声音

f. 如果当前处于正常模式，设置录入标志和检查数字为 0，使用 LCD 显示屏显示“等待......”,并将标志位 no_people 设置为 0。如果当前没有人在操作，程序会将 LCD 清空并显示“休眠中......”,并将标志位 no_people 设置为 1。程序会在适当的时候调用 del img 和 gc.collect() 来回收内存。其中，del img 用于删除 img 对象占用的内存，而 gc.collect() 用于回收未被使用的内存。

代码详细注释如下：

```python
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
```
# 5.各模块实操结果展示和实现指标

![](https://raw.githubusercontent.com/timerring/scratchpad2023/main/2023/20230914210221.png)
图5-1人脸录入成功

输入“`register****`”（`****`代表录入人脸编号）后按照yolo v2算法采集196维人脸数据（3轮18次采样），并储存在SD卡文件中，且屏幕显示采样进度，录入完成屏幕显示“successful”，蜂鸣器发低声。

![](https://raw.githubusercontent.com/timerring/scratchpad2023/main/2023/20230914210251.png)
图5-2人脸识别成功

![](https://raw.githubusercontent.com/timerring/scratchpad2023/main/2023/20230914210303.png)
图5-3人脸识别比对失败

门禁模式时，人脸识别框上会显示序列号和相似度，设定比对阈值，此处设置75，高于则蜂鸣器发低声且舵机旋转，低于则蜂鸣器发高声舵机无反应。

![](https://raw.githubusercontent.com/timerring/scratchpad2023/main/2023/20230914210317.png)
图5-4 人脸数据删除成功

输入“`delete****`”则删除对应用户储存在SD卡的特征值，蜂鸣器发高声。且屏幕显示ok，输入“erase”则删除SD卡内所有用户人脸特征值，蜂鸣器发高声且屏幕显示delete all。输入“open”则无条件控制舵机旋转，用作临时开关门禁。

![](https://raw.githubusercontent.com/timerring/scratchpad2023/main/2023/20230914210347.png)

图5-5蓝牙app操作

![](https://raw.githubusercontent.com/timerring/scratchpad2023/main/2023/20230914220923.png)

图5-6删除所有人脸数据
# 6.可改进之处及拓展方向

## 6.1 不足及改进之处

1、深度学习算法改进:可以采用更加先进的人脸检测方法和特征提取方法,如RetinaNet检测器和ResNet特征提取网络等,提高人脸识别的准确率。还可以采用多模型结合的方法提高系统性能。

2、采集更丰富的人脸数据:随着更多人员的使用,可以不断采集用户的人脸数据进行模型训练,提高对不同人脸的识别能力。还可以采集不同光照、表情等条件下的人脸,使人脸识别系统具有更好的适应性。

3、增加更多生物特征识别:人脸识别系统可以与其他生物特征识别技术结合,如指纹识别、虹膜识别等,采用多生物特征融合识别可以大大提高识别准确性和安全性。


## 6.2 系统方案可拓展方向

1、面部表情识别：门禁系统可以引入面部表情识别技术，实现对人员情绪和健康状况的识别和监测，提高门禁系统的应用价值和服务质量。需要考虑面部表情识别技术的稳定性和精度，以及对个人隐私和数据安全的保护等问题。

2、智能硬件应用：可以结合智能硬件技术，例如智能门锁、智能摄像头等，实现门禁系统的智能化升级和拓展。智能硬件应用需要对门禁系统的硬件设备进行升级和改进，例如引入智能门锁、智能摄像头等设备，以增强门禁系统的功能和性能。同时，还需要考虑硬件的兼容性、可靠性和安全性等问题。

3、云计算应用：可以将门禁系统与云计算技术相结合，实现数据存储、计算、分析和管理的云端化，从而实现门禁系统的远程监控和管理。云计算应用需要对门禁系统的数据进行云端存储和管理，利用云计算平台进行数据分析和处理，从而实现门禁系统的远程监控和管理。同时，还需要考虑数据隐私、信息安全和云计算平台的可靠性和稳定性等问题。

# 7.参考资料

1、RFID 学习资料 [https://blog.csdn.net/HuangChen666/article/details/114024767?spm=1001.2014.3001.5506](https://blog.csdn.net/HuangChen666/article/details/114024767?spm=1001.2014.3001.5506)

2、K210学习系列

[https://blog.csdn.net/Thousand_drive/article/details/123796878?spm=1001.2014.3001.5506](https://blog.csdn.net/Thousand_drive/article/details/123796878?spm=1001.2014.3001.5506)

3、人脸识别学习资料

[https://blog.csdn.net/Aaron357/article/details/93485279?spm=1001.2101.3001.6650.5&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7ERate-5-93485279-blog-107153393.235%5Ev36%5Epc_relevant_default_base3&depth_1-utm_source=](https://blog.csdn.net/Aaron357/article/details/93485279?spm=1001.2101.3001.6650.5&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7ERate-5-93485279-blog-107153393.235%5Ev36%5Epc_relevant_default_base3&depth_1-utm_source=)

4、固件烧录、信息储存学习资料

[https://blog.csdn.net/HuangChen666/article/details/113995079?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522168536128716800222862427%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=168536128716800222862427&biz_id=0&utm_me](https://blog.csdn.net/HuangChen666/article/details/113995079?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522168536128716800222862427%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=168536128716800222862427&biz_id=0&utm_me)

5、蓝牙模块学习资料

[https://blog.csdn.net/qq_44125275/article/details/128266379?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522168536196416800192217378%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=168536196416800192217378&biz_id](https://blog.csdn.net/qq_44125275/article/details/128266379?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522168536196416800192217378%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=168536196416800192217378&biz_id)

[1] Zhu, X., Lei, Z., Liu, X., Shi, H., & Li, S. Z. (2018). Face recognition: From traditional to deep learning methods. Advances in Computers, 113, 1-69.

[2] AlBdairi, A.J.A.; Xiao, Z.; Alkhayyat, A.; Humaidi, A.J.; Fadhel, M.A.; Taher, B.H.; Alzubaidi, L.; Santamaría, J.; Al-Shamma, O. Face Recognition Based on Deep Learning and FPGA for Ethnicity Identification. Appl. Sci. 2022, 12, 2605. https://doi.org/10.3390/app12052605

[3] Chen, J., Liao, S., & Liu, Y. (2021). Intelligent access control system based on deep learning and IoT. Journal of Physics: Conference Series, 1821(1), 012074.

[4] Derbel, A., Vivet, D. and Emile, B. (2015), Access control based on gait analysis and face recognition. Electron. Lett., 51: 751-752. [https://doi.org/10.1049/el.2015.0767](https://doi.org/10.1049/el.2015.0767)

[5] A. Nag, J. N. Nikhilendra and M. Kalmath, "IOT Based Door Access Control Using Face Recognition," 2018 3rd International Conference for Convergence in Technology (I2CT), Pune, India, 2018, pp. 1-3, doi: 10.1109/I2CT.2018.8529749.

[6] Radzi S A, Alif M K M F, Athirah Y N, et al. IoT based facial recognition door access control home security system using raspberry pi[J]. International Journal of Power Electronics and Drive Systems, 2020, 11(1): 417.
