# STM32F407 现场调试命令速查

串口：`COM11`，波特率：`115200`。

## 基本状态

```text
STATUS        # 查看当前状态
RESET         # 停止运动，恢复默认 PID/acc/舵机
SERVO         # 查看 CH2/CH3/CH4 当前 PWM
ENC           # 读取编码器
IMU           # 读取 IMU 角度
```

## 云台 CH4

角度命令：

```text
YT 0          # 上电/安全起始位，置物架/放置方向
YT 120        # 正左方向，正对中心圆环
YT 150        # 左圆环
YT 90         # 右圆环
```

当前默认值：

```text
YT 0 -> CH4 PWM 约 500
```

## 置物台 CH3

优先用角度命令：

```text
PT 0          # 置物盘1，PWM 约 500
PT 135        # 置物盘2，PWM 约 1500
PT 270        # 置物盘3，PWM 约 2500
```

直接 PWM 微调：

```text
SERVO 3 500
SERVO 3 1500
SERVO 3 2500
```

## 抓手 CH2

```text
上电默认开爪: ZH 50，PWM 约 862
ZH 25         # 合爪，PWM 约 678
ZH 50         # 开爪，PWM 约 862
SERVO 2 678   # 直接合爪 PWM
SERVO 2 862   # 直接开爪 PWM
```

## 升降机 ZDT 5

位置命令：

```text
SJ            # 查看升降机内部估计位置和默认参数
SJ?           # 同上
SJ EN         # 使能 ZDT 地址5
SJ DIS        # 失能 ZDT 地址5
SJ READ       # 读取地址5当前位置/速度/状态回包
SJ UP 200 30 20      # 向上(dir=1)轻顶200ms后停止，不清零
SJ UPZERO 500 30 20  # 向上轻顶500ms后停止，并将驱动器/软件位置清零
SJ HOMEZERO           # 多圈无限位碰撞回零：向上直到碰撞/堵转检测成立，再清零
SJ HOMEZERO 1 600 40 10000  # 指定方向/碰撞电流mA/持续时间ms/超时ms
SJ HCFG 0     # 配置地址5碰撞回零方向CW，不保存，默认600mA/40ms，auto home关闭
SJ HCFG 1     # 配置地址5碰撞回零方向CCW，不保存，默认600mA/40ms，auto home关闭
SJ HCFG 1 400 20  # 临时试更敏感阈值：400mA/20ms，不保存
SJ HSTAT      # 读取地址5回零参数(0x22)和回零状态(0x3B)
SJ HOME       # 触发地址5多圈无限位碰撞回零(mode=2)，先确认方向/参数再用
SJ HOME3      # 触发地址5多圈有限位开关回零(mode=3)，仅限已接限位开关
SJ HSTOP      # 中断地址5回零，并连续发送3次立即停止
SJ CLEAR      # 将地址5驱动器当前位置/误差/脉冲清零，同时软件位置置0；确认在上方0位后再用
SJ UNSTALL    # 解除地址5堵转保护
SJ ZERO       # 把当前物理位置标记为软件0位，不驱动电机
SJ SET 132    # 把当前物理位置标记为软件132位，不驱动电机；用于撞边界/命令失败后重新对齐
SJ 10 600 80  # 小幅保守测试：目标10，速度600，加速度80
SJ 33 1000 120
SJ 0 600 80   # 回到底部估计位置
SJ STOP       # 急停升降机电机5
```

调参建议：

```text
RESET
SJ
SJ 5 400 60
SJ 10 600 80
SJ 20 800 100
SJ 33 1000 120
```

说明：升降机当前是 ZDT 步进位置模式，不是 PID 闭环。AI 调的是 `目标位置 / speed / acc` 组合；若卡顿、撞限位或方向反了，立即发 `SJ STOP` 或 `SJ HSTOP`。

升降归零优先使用碰撞回零：

```text
SJ HOMEZERO           # 默认 dir=1,current=600mA,time=40ms,timeout=10000ms
SJ READ              # 确认 pos_pul=0, vel_rpm=0
```

`SJ UPZERO` 只是限时轻顶，只有顶部附近才适合：

```text
SJ UPZERO 500 30 20  # 顶部附近归零：最多向上顶500ms，然后停止并清零
SJ READ              # 确认 pos_pul=0, vel_rpm=0
```

上电初始化会执行一次 `SJ HOMEZERO`，然后把升降驱动器/软件位置清零。

## 平推 / 推动 ZDT 6

位置命令：

```text
PUSH             # 查看平推内部估计位置和默认参数
PUSH?            # 同上
PUSH EN          # 使能 ZDT 地址6
PUSH DIS         # 失能 ZDT 地址6
PUSH READ        # 读取地址6当前位置/速度/状态回包
PUSH HCFG 0      # 配置地址6碰撞回零方向CW，不保存，800mA/60ms，auto home关闭
PUSH HCFG 1      # 配置地址6碰撞回零方向CCW，不保存，800mA/60ms，auto home关闭
PUSH HCFG 1 600 40  # 临时试更敏感阈值：600mA/40ms，不保存
PUSH HSTAT       # 读取地址6回零参数(0x22)和回零状态(0x3B)
PUSH HOME        # 触发地址6多圈无限位碰撞回零(mode=2)，先确认方向/参数再用
PUSH HOME3       # 触发地址6多圈有限位开关回零(mode=3)，仅限已接限位开关
PUSH HSTOP       # 中断地址6回零，并连续发送3次立即停止
PUSH CLEAR       # 将地址6驱动器当前位置/误差/脉冲清零，同时软件位置置0；确认在推手零点后再用
PUSH UNSTALL     # 解除地址6堵转保护
PUSH ZERO        # 把当前物理位置标记为软件0位，不驱动电机
PUSH SET 50      # 把当前物理位置标记为软件50位，不驱动电机
PUSH 5 400 60    # 小幅保守测试：目标5，速度400，加速度60
PUSH -5 400 60   # 反方向小幅测试
PUSH 50 200 20   # 夹取/伸出取物位置
PUSH 10 200 20   # 实际放置/释放位置
PUSH 0 200 20    # 回推手零点，接近放置位置
PUSH STOP        # 急停平推电机6，连续发送3次立即停止
```

调参建议：

```text
PUSH ZERO
PUSH 5 400 60
PUSH 10 600 80
PUSH 50 200 20
PUSH 10 200 20
PUSH 0 200 20
PUSH -5 400 60
PUSH 0 600 80
```

说明：`PUSH` 是平推动作，ZDT 地址 `6`。当前现场标定范围是 `-20..65`；`0` 是推手零点，接近放置位置；`10` 是实际放置/释放平推位置；`50` 是夹取/伸出取物位置；`65` 是软件正向上限，`-20` 已接近后退限位。`PUSH 80` 已验证会卡死，不再允许。烧录/复位后软件位置会回到 `0`，若物理位置没在零点附近，先用 `PUSH SET <当前位置>` 对齐软件位置；它只改内部位置，不驱动电机。`PUSH CLEAR` 会清驱动器内部位置，只能在确认物理位置就是0位后用。它和置物台 `PT` 不是一个东西。

## 参考版动作回归 ACT

基础：

```text
ACT?                    # 查看 ACT 命令
ACT STOP               # 急停：底盘停、平推/升降先中断回零再各连续发送3次立即停止、加热关；不改软件零点
ACT ZERO               # 平推软件当前位置标为0；升降执行一次 HOMEZERO 后清零
ACT OPEN               # 开爪
ACT CLOSE              # 合爪
ACT SAFE               # 安全姿态：开爪、平台1、平推0、升降0、云台0、加热关
ACT RESET              # 同 ACT SAFE
ACT JIXIE              # 同 ACT SAFE，参考版 jixie_reset 的可重复调试版本
ACT RACK               # 回置物架抓取准备位
ACT ROUND3             # 先执行升降 HOMEZERO，再平台1/2/3各抓一次，按右/中/左环放置一轮；合爪后升降抬到10，再切到目标释放位，最后开爪；结束后平推/升降重置
ACT PICK 1             # 平台1切到置物架抓取位并合爪
ACT PICK 2             # 平台2切到置物架抓取位并合爪
ACT PICK 3             # 平台3切到置物架抓取位并合爪
ACT PLATFORM 1         # 置物盘1，等价 PT 0
ACT PLATFORM 2         # 置物盘2，等价 PT 135
ACT PLATFORM 3         # 置物盘3，等价 PT 270
ACT HEAT ON            # 开加热，TIM3_CH1 compare=300
ACT HEAT OFF           # 关加热
ACT HEAT 300           # 直接设置加热 PWM compare，范围 0..1999
ACT POSE 50 60 0       # 直接测试：平推夹取位，升降60，云台置物架方向
ACT POSE 10 60 90      # 直接测试：右环放置/释放姿态
```

参考动作：

```text
ACT NA R       # 色环区取红色并带回置物架
ACT NA G       # 色环区取绿色并带回置物架
ACT NA B       # 色环区取蓝色并带回置物架
ACT YTNA R     # 圆台红色完整取回动作
ACT YTNA G     # 圆台绿色完整取回动作
ACT YTNA B     # 圆台蓝色完整取回动作
ACT NA1 R      # 圆台红色识别位，只开爪不夹取
ACT NA1 G
ACT NA1 B
ACT NA2 R      # 圆台红色夹取并带回置物架
ACT NA2 G
ACT NA2 B
ACT FANG R     # 放到红色位置参考动作
ACT FANG G
ACT FANG B
ACT FANGMD R   # 码垛放置参考动作
ACT FANGMD G
ACT FANGMD B
```

说明：`ACT SAFE/RESET/JIXIE/RACK/PICK/ROUND3/FANG/FANGMD/NA/NA1/NA2/YTNA`
都会先执行一次升降 `HOMEZERO`，再进入动作链。`ACT POSE` 是手动点位测试，不会自动置零。

已知固定：

```text
云台：0=置物架/放置/初始，90=右圆环，120=中心圆环，150=左圆环
夹爪：OPEN=50°/约862，CLOSE=25°/约678
置物盘：PT 0 / 135 / 270
升降机：上电会先执行 HOMEZERO，再把顶点作为软件 0
```

未知待你现场替换：

```text
平推位置：0=推手零点/接近放置，10=实际放置/释放，50=夹取/伸出取物，65=软件正向上限，-20=后退限位附近
升降位置：0=上电/动作链前自动HOMEZERO后的顶点零位，60=放置东西高度；高位110/130/132已顶边界，废弃
```

这些值先按参考版占位。你每测出一个真实值，就替换 `Core/Src/motor_ack_test.c` 顶部 `ACT_*` 或对应 `act_*` 动作里的数值。

推荐整套低风险回归顺序：

```text
ACT STOP
ACT SAFE
ACT OPEN
ACT CLOSE
ACT OPEN
ACT PLATFORM 1
ACT PLATFORM 2
ACT PLATFORM 3
ACT PLATFORM 1
PUSH ZERO
PUSH 5 400 60
PUSH -5 400 60
PUSH 0 400 60
PUSH 10 400 60
SJ ZERO
SJ 5 400 60
SJ 0 400 60
ACT RACK
ACT PICK 1
ACT OPEN
ACT YTNA R
ACT FANG R
ACT FANGMD R
ACT SAFE
ACT ROUND3
ACT SAFE
```

## 底盘 / 姿态调试

```text
ACC 160       # 当前推荐速度环加速度
SET P:1.1 I:0 D:0
LLM ON 0 90   # 原地 90 度姿态调参/测试
LLM OFF       # 停止 LLM 调参运动
```

## 构建和烧录

WSL 构建：

```bash
./build.sh
```

Windows OpenOCD 烧录已封装：

```bash
./flash.sh
```

烧录成功关键输出：

```text
Programming Finished
Verified OK
Resetting Target
```
