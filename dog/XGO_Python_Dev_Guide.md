# XGO Python开发手册

来源: https://www.yuque.com/luwudynamics/cn/mxkaodwpo2h5zmvw

---

XGO使用树莓派做为机器狗的大脑，推荐使用VScode对XGO进行编程，可以通过一下步骤进行开发：

## 准备工作
- **安装 VScode**： 确保你已经在你的电脑上安装了 VScode。你可以从[VScode官网](https://code.visualstudio.com/)下载并安装。
- **连接网络：**根据快速入门让XGO连上网络，操作机器狗按键让机器狗显示IP地址如下图所示

## 在 VScode 中设置远程连接
- **安装 Remote Development 插件**：
- 打开 VScode，点击左侧扩展（Extensions）图标。
- 搜索并安装**Remote - SSH**扩展。
- **配置 SSH 连接**：
- 在VScode中按 **Ctrl+Shift+P** 打开命令面板，输入**Remote-SSH: Connect to Host...**，然后选择**Add New SSH Host...**。
- 输入连接字符串，例如：
其中**pi**是默认用户名，**192.168.1.2**是树莓派的IP地址。
- 接下来会提示输入SSH密码，输入你设置的密码，默认密码为pi
- **连接到树莓派**：
- 在VScode中再次按 **Ctrl+Shift+P**，选择**Remote-SSH: Connect to Host...**，然后选择刚刚添加的主机。
- 成功连接后，VScode会重新启动并连接到树莓派的远程环境。

### 在树莓派上编写和调试代码
- **打开文件夹**：
- 在VScode中，通过文件菜单选择**Open Folder...**，选择你想在树莓派上操作的目录。
- **安装必要的扩展**：
- 根据你要编写的代码语言（例如Python、C++等），在VScode中安装相应的扩展（如Python、C++等）。
- **编写代码**：
- 你现在可以像在本地一样，在VScode中编写代码，并通过终端运行和调试。
- **使用终端**：
- VScode的终端会直接连接到树莓派的终端。你可以在VScode中打开终端（使用**Ctrl+**或**View -> Terminal**），并在其中执行树莓派上的命令。

### 示例：在树莓派上运行 Python 代码
- **创建 Python 文件**：
- 在VScode中，创建一个新的Python文件，例如**xgo.py**。
- **编写代码**：
- **运行代码**：
- 在终端中运行以下命令：

通过这些步骤，你就可以使用VScode对树莓派进行编程了。这种方法使你可以在舒适的桌面环境中编写代码，同时利用树莓派的硬件进行实际运行和测试。

# Python库详解
XGO2内置了运动控制库文件xgolib.py，教育库xgoedu.py,开发者可以直接调用相关接口函数来控制机器狗。
通过命令安装xgo的python库:
CM4:已内置安装下面的 pip 包。
CM5: 已内置，需要激活虚拟环境，
cd /home/pi/RaspberryPi-CM5/
source xgovenv/bin/activate
然后这这个环境下执行即可。

使用前确保\home\pi\model文件夹中包含所有的模型文件
全部模型文件可在[https://github.com/Xgorobot/XGO-PythonLib](https://github.com/Xgorobot/XGO-PythonLib)的model文件夹中获取。

## 初始化
以下为初始化代码：

## 判断狗的类型
由于xgolite与xgomni在运动性能和参数上有差异，为了准确控制狗的运动，可通过`dog.read_firmware()`函数检测机器狗的类型，示例代码：
通过此段代码可以判断狗的类型，若变量dog_type为'M'则狗的类型为XGOMINI，若为'L'则类型为XGOLITE。

## 运动控制库介绍

### 移动相关方法

#### 前后左右平移
**move(direction, step)**
参数名
格式
输入范围
说明
direction
字符
'x'、'X'、'y'、'Y'
'x'或'X'使机器狗前进或后退，'y'或'Y'使机器狗左移或者右移
step
数字
x:[-25,25],y:[-18,18]
该参数代表平移步长，根据方向，正值代表前进或左移，负值代表后退或右移。输入值超过范围时，按照极限值移动。

#### 旋转
**turn(step)**
参数名
格式
输入范围
说明
step
数字
[-150,150]
该参数代表旋转速度，单位为°/s，正值为左转，负值为右转。

#### 原地踏步，只适用于mini
**mark_time(data)**
参数名
格式
输入范围
说明
data
数字
[10,35]
该参数代表原地踏步抬腿高度，单位为mm，输入为0时停止原地踏步

#### 改变迈步频率
**pace(mode)** 速度 = 步频 x 步幅
参数名
格式
输入范围
说明
mode
字符串
['normal','slow','high']
该参数代表迈步频率，normal为默认步频，low为慢速步频，high为高速步频

#### 停止移动
**stop()**
**移动相关方法示例**
库中基于这些方法封装了一系列方法以便于使用。
方法名
说明
move_x(step)
前后移动，相当于move('x', step)
move_y(step)
左右移动，相当于move('y', step)
forward(step)
前进，相当于move('x', abs(step))
back(step)
后退，相当于move('x', -abs(step))
left(step)
左移，相当于move('y', abs(step))
right(step)
右移，相当于move('y', -abs(step))
turnleft(step)
左转，相当于turn(abs(step))
turnright(step)
右转，相当于turn(-abs(step))

### 位姿相关方法
调节位姿时，机器狗四条腿足端位置不发生改变，机身的位置或角度发生变化。

#### 机身位置平移
**translation(direction, data)**
参数名
格式
输入范围
说明
direction
单字符或字符列表
'x'、'y'、'z'或包含以上值的列表
'x'代表前后平移，'y'代表左右平移，'z'代表身高
data
数字
x:[-35,35],y:[-18,18],z:[75,115]
该参数代表机身位置平移距离，单位为mm

#### 机身姿态调整
**attitude(direction, data)**
参数名
格式
输入范围
说明
direction
单字符或字符列表
'r'、'p'、'y'或包含以上值的列表
'r'代表滚转角，'p'代表俯仰角，'y'代表偏航角
data
数字
r:[-20,20],p:[-15,15],y:[11,11]
该参数代表机身姿态调节幅度，单位为°

#### 机身周期平移
**periodic_tran(direction, period)**机器狗机身将以指定周期和方向进行往复平移，幅度为位置平移极限值的一半，可以同时进行多个方向的周期运动。机身周期运动和整机运动不可同时进行。
参数名
格式
输入范围
说明
direction
单字符或字符列表
'x'、'y'、'z'或包含以上值的列表
'x'代表前后平移，'y'代表左右平移，'z'代表高低移动
period
数字
[1.5,8]
该参数代表运动周期，单位为s;输入0时代表停止运动

#### 机身周期旋转
**periodic_rot(direction, period)**
参数名
格式
输入范围
说明
direction
单字符或字符列表
'r'、'p'、'y'或包含以上值的列表
'r'代表滚转角，'p'代表俯仰角，'y'代表偏航角
period
数字
[1.5,8]
该参数代表运动周期，单位为s;输入0时代表停止运动
**位姿相关方法示例**

### 机械臂相关方法

#### 设置机械臂末端位置
**arm( arm_x, arm_z)**
参数名
格式
输入范围
说明
arm_x
float
[-80, 155]
单位为mm
arm_z
float
[-95, 155]
单位为mm
此处的x和z是相对于机械臂的基座的坐标，单位为毫米。
设定超过机械臂工作空间的值时，机械臂会保持最后一个有效值对应的姿态，比如(155,0)对应的姿态是向前伸到最大，(0,155)对应向上伸到最大，(155,155)是斜向上最大，但是机械臂达不到这个位置，就会保持上一次发送的有效位置。

#### 设置机械臂夹爪开合
**claw(pos)**
参数名
格式
输入范围
说明
pos
uint_8
0-255
0对应完全张开，255对应完全闭合

#### 设置机械臂是否开启稳定模式
**arm_mode(mode)**
参数名
格式
输入范围
说明
mode
int
0\1
0不开启，1开启
开启之后机械臂末端会不随着身体的平移而平移（平移指四脚站定躯干运动，而非前后左右迈步平移）。

### 其余方法

#### 恢复初始状态
**reset()**停止所有运动，所有状态全部恢复到初始状态

#### 设置自稳状态
**imu(mode)**自稳状态下，机器狗将自动调节姿态角以保持背部处于水平位置，不可在开启时手动设定姿态角。
参数名
格式
输入范围
说明
mode
整数
0、1
0代表关闭、1代表开启
**perform(mode)**表演模式，机器狗将循环执行预设的动作。
参数名
格式
输入范围
说明
mode
整数
0、1
0代表关闭、1代表开启

#### 单腿控制
**leg(leg_id, data)**控制指定腿的足端位置
参数名
格式
输入范围
说明
leg_id
整数
1、2、3、4
分别代表左前腿、右前腿、右后腿、左后腿
data
长度为3的数字列表
x:[-35,35],y:[-18,18],z:[75,115]
该参数代表足端位置，单位为mm
**其余方法示例**
四条腿以各自**肩部中间点**为原点，前为X轴正方向，左为Y轴正方向，下为Z轴的方向

#### 舵机控制
**motor(motor_id, data)**控制舵机旋转角度
参数名
格式
输入范围
说明
motor_id
整数或整数列表
[11,12,13,21,22,23,31,32,33,41,42,43,51,52,53]
第一位数字代表舵机所在的腿，第二位数字代表在该腿上的位置，从下到上依次是1，2，3
51、52、53分别是夹爪、小臂、大臂舵机
51推荐使用claw命令来控制比较直观
data
数字或数字列表
Mini
下:[-73, 57],
中:[-66, 93],
上:[-31, 31]
51：[-65, 65]
52：[-85, 50]
53：[-75, 90]
---------------
Lite
下:[-70, 50],
中:[-66, 93],
上:[-31, 31]
51：[-65, 65]
52：[-115, 70]
53：[-85, 100]
该参数代表舵机角度位置，单位为°
**舵机示例**

#### 单腿舵机卸载
**unload_motor(leg_id)**使一条腿上的三个舵机卸载，不输出力矩，之后可以随意用手转动，一般用于编写动作
参数名
格式
输入范围
说明
leg_id
整数
1,2,3,4
分别代表左前腿、右前腿、右后腿、左后腿

#### 所有舵机卸载
**unload_allmotor()**使所有舵机卸载，不输出力矩，可以随意用手转动

#### 单腿舵机加载
**load_motor(leg_id)**使一条腿上的三个舵机**保持当前位置**加载，输出力矩，之后不可以用手转动，一般用于编写动作
参数名
格式
输入范围
说明
leg_id
整数
1,2,3,4
分别代表左前腿、右前腿、右后腿、左后腿

#### 所有舵机加载
**load_allmotor()**使所有舵机加载，输出力矩，**机器狗回到默认站姿**，之后不可以用手转动

#### 设置舵机转动速度
**motor_speed(speed)**调节舵机转动速度，适用于单独控制舵机的情况
参数名
格式
输入范围
说明
speed
整数
[0,255]
0为最低速，255为最高速

#### 修改蓝牙名称
**bt_rename(name)**重新修改蓝牙名称，调用该函数后蓝牙会断开链接
参数名
格式
输入范围
说明
name
字符串
长度不大于10
机器狗的蓝牙名称格式为**XGO_xxx**，xxx为可修改部分，仅支持ascii码中的字符。

#### 执行预设动作
**action(action_id)**
参数名
格式
输入范围
说明
action_id
整数
[1,255]
ID与动作对应关系见下表
ID
动作
持续时间/s
ID
动作
持续时间/s
ID
动作
持续时间/s
1
趴下
3
2
站起
3
3
匍匐前进
5
4
转圈
5
5
mini为踏步
4
6
蹲起
4
7
转动Roll
4
8
转动Pitch
4
9
转动Yaw
4
10
三轴转动
7
11
撒尿
7
12
坐下
5
13
招手
7
14
伸懒腰
10
15
波浪
6
16
摇摆
6
17
乞讨
6
18
找食物
6
19
握手
10
20
鸡头
9
21
俯卧撑
8
22
张望
8
23
跳舞
6
24
调皮
7
128
上抓
10
129
中抓
10
130
下抓
10
144
上楼梯
12

备注：单机模式循环执行以上所有动作组，群控模式去除匍匐前进，转圈，踏步，乞讨，找食物。
备注：microblocks中去掉踏步

#### 标定舵机位置
**calibration(state)**如果开机后，某些关节出现了明显的位置偏差，可以调用该功能进行标定。其他情况请**谨慎使用**
参数名
格式
输入范围
说明
state
整数
["start","end"]
"start" 进入标定状态，此时舵机卸力，然后将机器狗摆至标定状态，小腿与地面平行，大腿与躯干呈90°，躯干与地面平行；"end" 完成标定

## 读取相关方法

### 读取舵机角度
**read_motor()**读取15个舵机的角度， 读取成功则返回长度为15的列表，对应编号[11,12,13,21,22,23,31,32,33,41,42,43,51,52,53]的舵机角度， 读取失败则返回空列表

### 读取电池电量
**read_battery()**读取当前电池电量， 读取成功则返回1-100的整数，代表电池剩余电量百分比， 读取失败则返回0。

### 读取姿态角度
**read_roll()****read_pitch()****read_yaw()**读取当前姿态角度，读取成功则浮点数，读取失败则返回0
​

# XGO双轮足产品系列 

## 初始化
以下为初始化代码：

## 运动控制库介绍

### 移动相关方法

#### 前后平移
**rider_move_x(speed, runtime=0)**
参数名
格式
输入范围
说明
**speed**
float
[-1.5,1.5]
单位为m/s，正值为前进，负值为后退
runtime
float
≥0
**单位为s，如果runtime为0，则轮足会一直以该速度运行。**
**如果runtime不为0，则运行指定时间后会停止。**

#### 旋转
**rider_turn(speed, runtime=0)**
**参数名**
**格式**
**输入范围**
**说明**
**speed**
**float**
**[-360,360]**
**单位为°/s，正值为逆时针，负值为顺时针**
**runtime**
**float**
**≥0**
**单位为s，如果runtime为0，则轮足会一直以该速度运行。**
**如果runtime不为0，则运行指定时间后会停止。**

### 位姿相关方法

#### 调节身高
**rider_height(data)**
参数名
格式
输入范围
说明
data
float
z:[60,120]
该参数代表身高，单位为mm

#### 机身姿态调整
**rider_roll(data)**
参数名
格式
输入范围
说明
data
float
r:[-17,17]
该参数代表机身姿态调节幅度，单位为°

#### 机身周期蹲起
**rider_periodic_z(period)**​
参数名
格式
输入范围
说明
period
数字
[2,4]
该参数代表运动周期，单位为秒（s）;输入0时代表停止运动

#### 机身周期左右晃动
**rider_periodic_roll(period)**
参数名
格式
输入范围
说明
period
数字
[2,4]
该参数代表运动周期，单位为秒（s）;输入0时代表停止运动

#### 平衡模式
**rider_balance_roll(mode)**自稳状态下，轮足将自动调节Roll以保持背部处于水平位置，不可在开启时手动设定姿态角，用于单边桥等左右两边高度不同的地形
参数名
格式
输入范围
说明
mode
整数
0、1
0代表关闭、1代表开启

## 读取相关方法

#### 恢复初始状态
**rider_reset()**停止所有运动，所有状态全部恢复到初始状态，如果是倒地状态，调用该方法后会站起。

### 读取版本号
**rider_read_firmware()**
读取下位机固件的版本号，返回长度最大为10的字符串，如"R-1.2.3"

### 电池电量
**rider_read_battery()**读取当前电池电量， 读取成功则返回1-100的整数，代表电池剩余电量百分比， 读取失败则返回0。

### 读取姿态角度
**rider_read_roll()****rider_read_pitch()****rider_read_yaw()**读取当前姿态角度，读取成功则浮点数，读取失败则返回0
**rider_read_imu_int16(direction)**
**读取姿态角度参数范围为["roll","pitch","yaw"]，读取成功则int16类型整数角度，读取失败则返回0**
​

## 其他方法

#### 执行预设动作
**action(action_id，wait=False)**
参数名
格式
输入范围
说明
action_id
整数
[1,255]
ID与动作对应关系见下表
wait
布尔值
TRUE/FALSE
是否延时等待动作做完，默认会等待
ID
动作
持续时间/s
ID
动作
持续时间/s
ID
动作
持续时间/s
1
左右摇摆
3
2
高低起伏
4
3
前进后退
3
4
四方蛇形
4
5
升降旋转
6
6
圆周晃动
5
英文命名：
1.Rocking  2.Altitude vary 3.Shifting 4.Zigzag 5.Lift&rotate 6.Trembling

**rider_perform(mode)**表演模式，机器狗将循环执行预设的动作。
参数名
格式
输入范围
说明
mode
整数
0、1
0代表关闭、1代表开启

#### 标定舵机位置
**rider_calibration(state)**如果开机后，某些关节出现了明显的位置偏差，可以调用该功能进行标定。其他情况请**谨慎使用**
参数名
格式
输入范围
说明
state
整数
[start,end]
start 进入标定状态，此时舵机卸力，将轮足扶正，将腿降低至最低，然后向后倚靠在地面上，然后发送end完成标定
​

#### 修改蓝牙名称
**rider_bt_rename(name)**重新修改蓝牙名称，调用该函数后蓝牙会断开链接
参数名
格式
输入范围
说明
name
字符串
长度不大于10
蓝牙名称格式为**XGORider_xxx**，xxx为可修改部分，仅支持ascii码中的字符。

#### 设定静止时背部LED灯色彩
**rider_led(index, color)**双轮足**静止时刻**背部LED灯的颜色，默认为四个灯全灭
参数名
格式
输入范围
说明
index
uint8
1-4
左上，左下，右下，右上分别为1、2、3、4号LED
color
[uint8, uint8, uint8]
0-255
写入三个字节数据，数值范围为0-255，代表RGB的亮度，[0,0,0]代表灭，[255,255,255]代表最亮的白光

# 教育库介绍
XGO的教育库主要是给出了集成在AI模组中的摄像头，屏幕，按键，麦克风和喇叭等硬件的Python接口。以及部分AI模型调用等功能。

## 初始化
以下为初始化代码：

## 屏幕绘图

### 需要先杀掉自启动main.py进程，否则会屏幕刷新冲突
sudo ps -ef | grep main.py

### 画直线
lcd_line(x1,y1,x2,y2,color=(r,g,b),width=width)
参数名
格式
输入范围
说明
x1,y1,x2,y2
数字
x1 x2:[0,320]
y1 y2:[0,240]
x1,y1为初始点标
x2,y2为终止点坐标
color(可缺省)
默认为白色
rgb元组
r g b:[0,255]
color为线颜色
width（可缺省）
默认为2
数字

width为线宽

### 画圆形
lcd_round(center_x, center_y, radius, color=(255, 255, 255), width=2)
参数名
格式
输入范围
说明
center_x
center_y
数字
center_x:[0,320]
center_x:[0,240]
center_x,center_y为圆心坐标
raduius
数字
​
raduius为半径
color(可缺省)
默认为白色
rgb元组
r g b:[0,255]
color为圆弧颜色
width（可缺省）
默认为2
数字

width为圆弧宽

### 画圆弧
lcd_arc(x1,y1,x2,y2,angle0,angle1,color=(255,255,255),width=2)
参数名
格式
输入范围
说明
x1,y1,x2,y2
数字
x1 x2:[0,320]
y1 y2:[0,240]
x1,y1,x2,y2为定义边界框的两个点
angle0,angle1
数字
angle0 angle1:[0,360]
angle0为初始角度，三点钟方向为起始点，顺时针增加。
angle1为终止角度
color(可缺省)
默认为白色
rgb元组
r g b:[0,255]
color为圆弧颜色
width（可缺省）
默认为2
数字

width为圆弧宽

### 画矩形
lcd_rectangle(x1,y1,x2,y2,fill=None,outline=(255,255,255),width=2)
参数名
格式
输入范围
说明
x1,y1,x2,y2
数字
x1 x2:[0,320]
y1 y2:[0,240]
x1,y1为初始点标
x2,y2为终止点坐标
fill(可缺省)
默认为None
rgb元组
r g b:[0,255]
fill为填充颜色
None则为不填充
outline(可缺省)
默认为白色
rgb元组
r g b:[0,255]
outline为线颜色
width（可缺省）
默认为2
数字

width为线宽

### 显示文字
可显示中文与英文使用微软雅黑字体，字体大小可调节
lcd_text(x,y,content,color=(255,255,255),fontsize=15)
参数名
格式
输入范围
说明
x,y
数字
x y:[0,320]
x,y为初始点标
content
字符串
content为显示内容
color(可缺省)
默认为白色
rgb元组
r g b:[0,255]
color为文字颜色
fontsize（可缺省）
默认为15
数字

fontsize为字体大小

### 显示图片
lcd_picture(filename)   
参数名
格式
说明
filename
字符串
图片文件名需要加jpg扩展名
图片文件显示路径为/home/pi/xgoPictures，图片大小为320*240

### 清除屏幕
lcd_clear()

## 按键检测
xgoButton(button)
参数名
格式
输入范围
返回值
button
指定字符串
["a","b"]
False未按下 True按下

## 音视频功能

### 播放音频
xgoSpeaker(filename) 
参数名
格式
输入范围
说明
filename
字符串
​
音频文件扩展名wav，路径为/home/pi/xgoMusic

### 播放视频
xgoVideo(filename)  
参数名
格式
输入范围
说明
filename
字符串
​
视频文件扩展名mp4
路径为/home/pi/xgoVideos

### 录制音频
xgoAudioRecord(filename="record",seconds=5)   
参数名
格式
输入范围
说明
filename(可缺省)
默认"record"
字符串
​
录制音频的文件名
会自动添加扩展名wav，录制的文件路径为/home/pi/xgoMusic
seconds(可缺省)
默认为5
数字

录制文件的长度(秒)

## 摄像头功能

### 摄像头
xgoCamera(status)   
参数名
格式
输入范围
说明
status
布尔值
True,False
打开和关闭摄像头，屏幕会实时显示视频流

### 录制视频 
xgoVideoRecord(filename="record",seconds=5)   
参数名
格式
输入范围
说明
filename(可缺省)
默认"record"
字符串
​
录制视频的文件名
会自动添加扩展名mp4,录制的文件路径为/home/pi/xgoVideos
seconds(可缺省)
默认为5
数字

录制文件的长度(秒)

### 拍摄照片
xgoTakePhoto(filename="photo")  
参数名
格式
输入范围
说明
filename(可缺省)
默认"photo."
字符串
​
拍摄照片的文件名
会自动添加扩展名jpg，图片保存路径为/home/pi/xgoPictures
注意：使用此函数会自动运行xgoCamera(True)，如不需要实时显示摄像头画面，请在此函数后面加上xgoCamera(False)

## AI功能
此系列的api核心功能是调取一帧图像进行分析并返回结果，可在参数传入图片的路径，实现对单张图片的检测。如需实时分析摄像头画面，请配合while使用，下面为示例代码：
手势识别单张图片：
实时通过摄像头进行手势识别：
获取手势识别结果的具体内容：

### 骨骼识别
posenetRecognition(target="camera")
参数名
格式
说明
返回值
target(可缺省)
默认"camera"
即使用摄像头捕捉图像
字符串
target为图像文件的路径
[angle1,angle2,angle3,angle4]
angle1、angle2 俩大臂和小臂之间的夹角
angle3、angle4 俩大臂和身体之间的夹角
​

### 手势识别手势识别
gestureRecognition(target="camera")
参数名
格式
说明
返回值
target(可缺省)
默认"camera"
即使用摄像头捕捉图像
字符串
target为图像文件的路径
(ges,(x,y))
ges为手势识别结果
目前包括的手势有：
["1","2","3","4","5","Good","Ok","Rock","Stone"]
坐标值xy

### YOLO识别
yoloFast(target="camera")
参数名
格式
说明
返回值
target(可缺省)
默认"camera"
即使用摄像头捕捉图像
字符串
target为图像文件的路径
(object,(x,y))
object为YOLO识别结果
目前包括的物体有：
['person','bicycle','car','motorbike','aeroplane','bus','train','truck','boat','trafficlight','firehydrant','stopsign','parkingmeter','bench','bird','cat','dog','horse','sheep','cow','elephant','bear','zebra','giraffe','backpack','umbrella','handbag','tie','suitcase','frisbee','skis','snowboard','sportsball','kite','baseballbat','baseballglove','skateboard','surfboard','tennisracket','bottle','wineglass','cup','fork','knife','spoon','bowl','banana','apple','sandwich','orange','broccoli','carrot','hotdog','pizza','donut','cake','chair','sofa','pottedplant','bed','diningtable','toilet','tvmonitor','laptop','mouse','remote','keyboard','cell phone','microwave','oven','toaster','sink','refrigerator','book','clock','vase','scissors','teddy bear','hair drier','toothbrush']
坐标值xy

### 人脸检测
face_detect(target="camera")
参数名
格式
说明
返回值
target(可缺省)
默认"camera"
即使用摄像头捕捉图像
字符串
target为图像文件的路径
[x,y,w,h] 人脸识别框的
x坐标，y坐标，宽度，高度

### 情绪识别
emotion(target="camera")
参数名
格式
说明
返回值
target(可缺省)
默认"camera"
即使用摄像头捕捉图像
字符串
target为图像文件的路径
(emotion,(x,y))
emotion包括:
['Angry','Happy','Neutral','Sad','Surprise']
坐标值xy

### 年龄性别识别
agesex(target="camera")
参数名
格式
说明
返回值
target(可缺省)
默认"camera"
即使用摄像头捕捉图像
字符串
target为图像文件的路径
(gender,age,(x,y))
gender包括['Male', 'Female']
age包括['(0-2)', '(4-6)', '(8-12)', '(15-20)', '(25-32)', '(38-43)', '(48-53)', '(60-100)']
坐标值xy

### 语音识别（需要联网，离线不可用）
SpeechRecognition(seconds=3)
参数名
格式
说明
返回值
seconds(可缺省)
默认为3
数字
录制文件的长度(秒)
语句执行后，稍作停顿再讲话
识别结果字符串

### 语音合成（需要联网，离线不可用）
SpeechSynthesis(texts)
参数名
格式
说明
返回值
texts
字符串
支持中文、英语及混用
无 会自动播放合成后的语音

### 二维码识别 ​
QRRecognition(target="camera")
参数名
格式
说明
返回值
target(可缺省)
默认"camera"
即使用摄像头捕捉图像
字符串
target为图像文件的路径
二维码识别结果 result
可以识别多个结果，返回值为列表

### 小球识别​
BallRecognition(color_mask,target="camera")
参数名
格式
说明
返回值
target(可缺省)
默认"camera"
即使用摄像头捕捉图像
color为预设颜色
字符串
target为图像文件的路径
小球识别结果 ((x,y))
返回圆心坐标
color_mask
获取颜色遮罩
需要函数cap_color_mask()获得
hsv颜色范围
实时获取特定颜色的小球轮廓：

### 颜色识别​
ColorRecognition(target="camera",mode='R')
参数名
格式
说明
返回值
target(可缺省)
默认"camera"
即使用摄像头捕捉图像
字符串
target为图像文件的路径
颜色识别结果((x,y),r)
可以识别多个结果，返回值为列表
mode为预设颜色
固定值（字符串）
RGBY对应红绿蓝黄
如选黄色则填入mode='Y'