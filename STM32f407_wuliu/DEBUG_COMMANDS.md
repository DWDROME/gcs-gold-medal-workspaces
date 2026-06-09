# STM32F407 现场调试命令速查

注意：本文的 `STATUS` / `ACT` / `PUSH` / `SJ` 等串口命令只适用于
`motor_ack_test` 诊断固件。当前比赛主固件 `STM32f407_wuliu` 的 race target
不再编译 `Core/Src/motor_ack_test.c`，烧录主固件后不会响应这些诊断命令。

串口：`COM11`，波特率：`115200`。

## 基本状态

```text
STATUS        # 查看当前状态，不运动
RESET         # 停止测试运动，恢复 PID/acc 状态
SERVO         # 查看 CH2/CH3/CH4 当前 PWM 记录值
ENC           # 读取编码器
IMU           # 读取 IMU 角度
ACT ?         # 查看动作组命令
```

上电安全契约：

```text
# BOOT race_chassis_init
```

看到这行表示当前烧录的是比赛主固件启动路径。主固件随后会执行
`race_chassis_init()`：上位机接收初始化、关加热、开爪、置物盘 1 位、
云台回 `0`、平推清零、升降 HOMEZERO，然后等待上位机握手和开关。

## 云台 CH4 / YT

当前标定：

```text
YT 0          # 置物架/放置方向，初始方向
YT 90         # 右环
YT 120        # 正左/中心圆环
YT 150        # 左环
```

## 置物盘 CH3 / PT

当前标定：

```text
PT 0          # 置物盘 1
PT 135        # 置物盘 2
PT 270        # 置物盘 3
```

## 夹爪 CH2 / ZH

当前标定：

```text
ZH 25         # 合爪
ZH 50         # 开爪
```

## 平推 PUSH

当前标定：

```text
PUSH 0        # 最小安全/回零位
PUSH 10       # 放置伸出位
PUSH 50       # 抓取位
PUSH 65       # 最大安全范围，别再加大
PUSH STOP     # 停止平推
PUSH HSTOP    # 停止平推归零流程
```

物理无感归零会运动，必须单独测试方向：

```text
PUSH HOME     # 平推无感归零，观察应朝 0 位回收；方向反就立刻断电
```

## 升降 SJ

当前标定：

```text
SJ 0          # 最上方/安全上位
SJ 10         # 抓取后上升/转移高度
SJ 60         # 放置/抓取高度
SJ STOP       # 停止升降
SJ HSTOP      # 停止升降归零流程
```

## 动作组 ACT

安全测试顺序：

```text
STATUS        # 先确认无异常
ACT ?         # 只读帮助
PUSH HOME     # 先单独确认平推归零方向
ACT ZERO      # 再测动作组归零
ACT ROUND3    # 最后再跑三件物品动作链
```

动作组当前核心值：

```text
YT: 0 / 90 / 120 / 150
PT: 0 / 135 / 270
ZH: 25 合爪, 50 开爪
PUSH: 0 安全/回零, 10 放置, 50 抓取, 65 最大
SJ: 0 上位, 10 转移, 60 放置/抓取
```

## 底盘 / 姿态调试

```text
ACC 160       # 当前推荐速度环加速度
SET P:1.1 I:0 D:0
LLM ON 0 90   # 原地 90 度姿态调参/测试
LLM OFF       # 停止 LLM 调参运动
```

## 比赛主固件硬件验证顺序

连接 Horco CMSIS-DAP 后先确认 Windows 能识别到调试器：

```powershell
Get-PnpDevice -PresentOnly |
  Where-Object {
    $_.InstanceId -like "*VID_FAED*" -or
    $_.FriendlyName -like "*CMSIS-DAP*" -or
    $_.FriendlyName -like "*Horco*"
  } |
  Select-Object Status,Class,FriendlyName,InstanceId,Manufacturer |
  Format-List
```

确认串口：

```powershell
Get-CimInstance Win32_SerialPort |
  Where-Object {
    $_.PNPDeviceID -like "*VID_FAED*" -or
    $_.Name -like "*CMSIS-DAP*" -or
    $_.Name -like "*Horco*"
  } |
  Select-Object DeviceID,Name,PNPDeviceID,Description,Manufacturer |
  Format-List
```

主固件烧录后观察顺序：

```text
1. 串口应输出 # BOOT race_chassis_init。
2. 上电初始化应关加热、开爪、置物盘到 1 位、云台回 0。
3. 平推应清零到 0；升降应执行 SJ HOMEZERO，到顶后清驱动位置和软件位置。
4. 日志应出现 # SJ HOMEZERO START ... current=750 time=30 timeout=3000，成功时出现 # SJ HOMEZERO CLEAR driver_pos=0 software_pos=0.000。
5. 进入比赛流程前会等待上位机握手，再等待开关。
```

如果升降 HOMEZERO 方向或堵转判定异常，先断电，不继续跑路线。

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

如果连续出现：

```text
CMSIS-DAP command CMD_INFO failed
```

先拔插 Horco CMSIS-DAP 或给调试器/板子重新上电，再重试 `./flash.sh`。
