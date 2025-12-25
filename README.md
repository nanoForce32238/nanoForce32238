## Hi there 👋

<!--
**nanoForce32238/nanoForce32238** is a ✨ _special_ ✨ repository because its `README.md` (this file) appears on your GitHub profile.

Here are some ideas to get you started:

自动驾驶模块中文指南
一、AutoSpinFire - 自动发射模块
功能
控制发射机构的旋转（spin）、盖子（cover）和支撑器（supporter），实现自动三连发。

主要方法
fireInTurn()
按默认角度（10°, 130°, 250°）执行三次发射。

fireInTurn(double angle1, double angle2, double angle3)
按自定义角度发射三次。

fireInTurn_Manual()
智能发射：从当前位置最近的默认角度开始顺时针发射。

fireInTurn_Manual(double angle1, double angle2, double angle3)
智能发射：从最近的自定义角度开始。

关键参数（通过 @Config 可动态调整）
public static long spinDuration = 500;          // 每个角度停留时间（ms）
public static long beforeSupporterDuration = 350; // 支撑器抬升前等待时间
public static long supportDuration = 150;       // 支撑器抬升→下降间隔
public static long spinTimeout = 2500;         // 旋转超时保护
public static double spinAngleTolerance = 4;   // 角度到位误差（°）
// PIDF 参数
public static double Kp = 0.01, Ki = 0.0, Kd = 0.0001, Kf = 0.5;
使用示例
// 初始化
AutoSpinFire fire = new AutoSpinFire(opMode, spinServo, spinPot, coverServo, supporterServo);

// 发射三次
fire.fireInTurn();

// 智能发射（从最近角度开始）
fire.fireInTurn_Manual();
二、AutoShooterController - 发射器控制器
功能
控制发射器电机（左右轮）和机械臂角度，支持PID调节。

主要方法
start(int targetAngle, double speed)
启动发射器并移动机械臂到目标角度。

stop()
停止所有电机。

moveArmOnly(int targetAngle)
仅移动机械臂。

moveArmToPosition(int targetAngle, int timeoutMs)
阻塞式移动机械臂，带超时返回。

机械臂参数
private static final double ARM_Kp = 0.015;
private static final double ARM_Ki = 0.000;
private static final double ARM_Kd = 0.0001;
// 电位器校准
private static final double POT_MIN_VOLTAGE = 1.78; // 0°
private static final double POT_MAX_VOLTAGE = 2.01; // 100°
使用示例
// 初始化
AutoShooterController shooter = new AutoShooterController(opMode, leftMotor, rightMotor, armMotor, armPot);

// 启动发射器，机械臂到45°，速度1500 RPM
shooter.start(45, 1500);

// 仅移动机械臂
shooter.moveArmOnly(60);
三、AprilTagDetector - AprilTag视觉检测器
功能
通过摄像头检测AprilTag（ID 21,22,23），返回对应编号。

主要方法
update()
在循环中调用，更新检测结果。

getResult()
返回检测到的标签ID（1,2,3 对应 21,22,23）。

hasResult()
是否已检测到有效标签。
// 在VisionPortal初始化后使用
AprilTagDetector detector = new AprilTagDetector(opMode, visionPortal, aprilTagProcessor);

// 在循环中更新
detector.update();

if (detector.hasResult()) {
    int tag = detector.getResult(); // 1, 2 或 3
}
四、AutoSpinCollector - 自动旋转收集器
功能
控制收集机构的旋转和进料，通过距离/颜色传感器检测像素块。

主要方法
start(double p1, double p2, double p3)
开始收集流程，依次移动到三个预设角度。

start(double p1, double p2, double p3, long maxCollectTimeMs)
带超时的收集流程。

update()
在循环中调用，驱动状态机运行。

stop()
停止收集。

isIdle()
是否处于空闲状态。

状态机流程
IDLE → MOVE_TO_P1 → DETECT_P1 → MOVE_TO_P2 → DETECT_P2 → MOVE_TO_P3 → DETECT_P3 → RETURN_HOME → IDLE
每个检测点若超时或检测到物体，则进入下一位置。
关键参数
public static double Kp = 0.016, Ki = 0, Kd = 0.0001, Kf = 0.5;
private static final double DIST_TH = 5.0;      // 距离阈值（cm）
private static final int COLOR_TH = 200;        // 颜色Alpha阈值
private static final double ANGLE_TOL = 4.0;    // 角度容差（°）
private static final double HOME = 10.0;        // 归位角度
使用示例
// 初始化
AutoSpinCollector collector = new AutoSpinCollector(intakeMotor, spinServo, pot, colorSensor, distanceSensor);

// 开始收集（三个预设角度）
collector.start(30.0, 150.0, 270.0);

// 在循环中更新
while (opMode.opModeIsActive()) {
    collector.update();
    if (collector.isIdle()) {
        break;
    }
}
通用建议
参数调优
所有 public static 参数可通过Dashboard实时调整。
PID参数需根据实际机械特性微调。
非阻塞设计
update() 方法需在循环中频繁调用（通常每20ms）。
避免在状态机运行时长时间阻塞。
安全保护
各模块均内置超时保护，防止死循环。
建议在 opMode.opModeIsActive() 条件下运行。
调试信息
可通过 telemetry 输出各模块状态（如角度、检测结果）。
利用 getCurrentState()（如Collector）监控流程进展。
模块依赖
MapUtils：角度映射工具（需确保 Libraries.MapUtils 存在）。
FTCDashboard：用于参数实时调整（可选）。
VisionPortal：AprilTag检测所需。

-->
