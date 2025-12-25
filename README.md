

<!--
**nanoForce32238/nanoForce32238** is a ✨ _special_ ✨ repository because its `README.md` (this file) appears on your GitHub profile.

Here are some ideas to get you started:

模块关系图
AutoSpinCollector (收集)
      ↓
AutoShooterController (发射器准备)
      ↓
AutoSpinFire (执行发射)
      ↓
AprilTagDetector (视觉导航)
环境要求
Android Studio + FTC SDK
FTC Dashboard (用于参数实时调整)

支持以下硬件：
REV Expansion Hub / Control Hub
伺服电机 (Servo)
直流电机 (DcMotor)
电位器 (AnalogInput)
颜色传感器 (ColorSensor)
距离传感器 (DistanceSensor)
摄像头 (Webcam)

AutoSpinFire 自动发射模块
硬件连接
// 硬件映射名称示例
Servo spin = hardwareMap.servo.get("spin_servo");
AnalogInput spinPot = hardwareMap.analogInput.get("spin_pot");
Servo cover = hardwareMap.servo.get("cover_servo");
Servo supporter = hardwareMap.servo.get("supporter_servo");
详细工作原理
1. 旋转机构控制 (Spin)
传感器: 360° 电位器 (0-3.26V 对应 0-360°)
控制算法: PIDF (比例-积分-微分-前馈)
防卡死机制:
超时保护 (spinTimeout = 2500ms)
稳定计数 (连续3次误差<4°才认为到位)

2. 发射序列
1. 打开盖子 (cover.setPosition(0.55))
2. 旋转到角度1 → 触发支撑器 → 停留500ms
3. 旋转到角度2 → 触发支撑器 → 停留500ms
4. 旋转到角度3 → 触发支撑器 → 停留500ms
5. 可选关闭盖子 (cover.setPosition(0.86))
3. 支撑器状态机
IDLE → WAIT_BEFORE_UP (350ms) → 
支撑器抬升 → WAIT_BEFORE_DOWN (150ms) → 
支撑器下降 → IDLE
完整使用示例
public class AutoOpMode extends LinearOpMode {
    
    private AutoSpinFire autoSpinFire;
    
    @Override
    public void runOpMode() {
        // 初始化硬件
        Servo spin = hardwareMap.servo.get("spin");
        AnalogInput spinPot = hardwareMap.analogInput.get("spin_pot");
        Servo cover = hardwareMap.servo.get("cover");
        Servo supporter = hardwareMap.servo.get("supporter");
        
        // 初始化模块
        autoSpinFire = new AutoSpinFire(this, spin, spinPot, cover, supporter);
        
        waitForStart();
        
        // 方法1: 固定角度发射
        autoSpinFire.fireInTurn(); // 默认: 10°, 130°, 250°
        
        // 方法2: 自定义角度
        autoSpinFire.fireInTurn(15, 135, 255);
        
        // 方法3: 智能发射（从当前位置最近角度开始）
        autoSpinFire.fireInTurn_Manual();
        
        // 方法4: 智能发射+自定义角度
        autoSpinFire.fireInTurn_Manual(20, 140, 260);
    }
}
PIDF 参数调试指南
参数	作用	推荐调整范围	调整效果
Kp	比例项	0.005-0.03	增大: 响应更快, 可能振荡
Ki	积分项	0-0.001	消除稳态误差, 太大会超调
Kd	微分项	0-0.0005	抑制振荡, 提高稳定性
Kf	前馈项	0.4-0.6	舵机中点位置, 补偿重力
调试技巧
快速测试: 设置 spinAngleTolerance = 10 放宽要求

观察日志: 打印当前角度和目标角度差

手动模式: 先测试单个角度移动 moveToAngle(angle)

电压检查: 确认电位器电压范围 0-3.26V

AutoShooterController 发射器控制器
硬件配置

// 双飞轮 + 机械臂配置
DcMotorEx leftFlywheel = hardwareMap.get(DcMotorEx.class, "left_fly");
DcMotorEx rightFlywheel = hardwareMap.get(DcMotorEx.class, "right_fly");
DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");
AnalogInput armPot = hardwareMap.analogInput.get("arm_pot");
机械臂角度映射

电位器电压: 1.78V → 0° (最低位置)
          2.01V → 100° (最高位置)
实际角度 = 100 - ((电压 - 1.78) / (2.01 - 1.78) * 100)
使用模式
模式1: 启动发射器 + 自动调整角度
java
AutoShooterController shooter = new AutoShooterController(
    this, leftFlywheel, rightFlywheel, armMotor, armPot
);

// 启动飞轮 (1500 RPM) 同时移动机械臂到45°
shooter.start(45, 1500);

// 发射完成后停止
shooter.stop();
模式2: 仅控制机械臂

// 移动机械臂到特定角度
shooter.moveArmOnly(60);

// 阻塞式移动 (等待完成或超时)
boolean success = shooter.moveArmToPosition(70, 1500); // 1500ms超时
if (!success) {
    telemetry.addData("警告", "机械臂移动超时");
}
模式3: 获取当前状态

int currentAngle = shooter.getCurrentArmAngle();
telemetry.addData("当前角度", "%d°", currentAngle);
PID 参数说明

// 机械臂专用PID参数 (已优化)
private static final double ARM_Kp = 0.015;    // 比例: 决定响应速度
private static final double ARM_Ki = 0.000;    // 积分: 消除静态误差 (设为0)
private static final double ARM_Kd = 0.0001;   // 微分: 抑制振荡
安全特性
超时保护: 2秒自动停止

积分限幅: ±100 防止积分饱和

到位检测: ±2° 误差内停止

BRAKE模式: 断电后保持位置

AprilTagDetector 视觉检测器
前置配置
// 在OpMode的init方法中配置VisionPortal
AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
    .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
    .setDrawTagID(true)
    .setDrawTagOutline(true)
    .build();

VisionPortal visionPortal = new VisionPortal.Builder()
    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
    .addProcessor(aprilTagProcessor)
    .build();

AprilTagDetector detector = new AprilTagDetector(this, visionPortal, aprilTagProcessor);
检测逻辑

// 对应关系
AprilTag ID 21 → 返回结果 1
AprilTag ID 22 → 返回结果 2
AprilTag ID 23 → 返回结果 3
未检测到或非目标 → 返回 0
完整工作流程

public class VisionOpMode extends LinearOpMode {
    
    private AprilTagDetector detector;
    private int detectedTag = 0;
    
    @Override
    public void runOpMode() {
        // 初始化视觉系统 (见上面代码)
        
        waitForStart();
        
        ElapsedTime detectionTimer = new ElapsedTime();
        detectionTimer.reset();
        
        // 检测5秒或直到检测到目标
        while (opModeIsActive() && 
               detectionTimer.seconds() < 5.0 && 
               !detector.hasResult()) {
            
            detector.update();
            
            if (detector.hasResult()) {
                detectedTag = detector.getResult();
                break;
            }
            
            telemetry.addData("状态", "检测中...");
            telemetry.update();
            sleep(50);
        }
        
        if (detectedTag != 0) {
            telemetry.addData("检测结果", "Tag ID: %d", detectedTag);
            
            // 根据检测结果执行不同策略
            switch (detectedTag) {
                case 1:
                    // 执行策略1
                    break;
                case 2:
                    // 执行策略2
                    break;
                case 3:
                    // 执行策略3
                    break;
            }
        } else {
            telemetry.addData("结果", "未检测到目标标签");
        }
        telemetry.update();
    }
}
优化建议
摄像头位置: 俯视角度约30°, 距离AprilTag 0.5-1.5米

光照条件: 避免强光直射和阴影

检测范围: 限制只检测需要的ID (21,22,23)

结果缓存: 检测到结果后停止检测，节省资源

AutoSpinCollector 自动旋转收集器
硬件连接

DcMotor intake = hardwareMap.dcMotor.get("intake_motor");
Servo spinServo = hardwareMap.servo.get("collector_spin");
AnalogInput pot = hardwareMap.analogInput.get("collector_pot");
ColorSensor color = hardwareMap.colorSensor.get("color_sensor");
DistanceSensor distance = hardwareMap.get(DistanceSensor.class, "distance_sensor");
状态机详细流程

初始化 → 设置目标角度 p1 → 旋转到 p1 → 
检测物体 (距离<5cm 或 alpha>200) → 
检测到: 旋转到 p2 → 检测 → 旋转到 p3 → 检测 → 返回HOME
超时: 跳过当前位置 → 继续下一位置
检测参数说明

// 距离检测: < 5cm 认为有物体
private static final double DIST_TH = 5.0; // 单位: cm

// 颜色检测: Alpha值 > 200 认为有物体
private static final int COLOR_TH = 200; // Alpha通道 (0-255)

// 双条件满足任一即认为检测到
boolean detected = distance < DIST_TH || alpha > COLOR_TH;
使用示例
示例1: 基本收集流程
java
AutoSpinCollector collector = new AutoSpinCollector(
    intake, spinServo, pot, color, distance
);

// 设置三个收集位置 (角度)
double pos1 = 45.0;  // 位置1
double pos2 = 165.0; // 位置2  
double pos3 = 285.0; // 位置3

// 开始收集 (无超时限制)
collector.start(pos1, pos2, pos3);

// 在主循环中更新
while (opModeIsActive()) {
    collector.update();
    
    // 显示当前状态
    telemetry.addData("收集器状态", collector.getCurrentState());
    telemetry.addData("是否空闲", collector.isIdle());
    telemetry.update();
    
    if (collector.isIdle()) {
        telemetry.addData("完成", "收集完成!");
        break;
    }
    
    sleep(20); // 20ms更新周期
}
示例2: 带超时的收集
// 设置5秒超时 (每个位置最多检测5秒)
long timeoutMs = 5000;
collector.start(pos1, pos2, pos3, timeoutMs);

while (opModeIsActive() && !collector.isIdle()) {
    collector.update();
    
    // 实时显示传感器数据
    telemetry.addData("距离", "%.1f cm", 
        distance.getDistance(DistanceUnit.CM));
    telemetry.addData("颜色Alpha", color.alpha());
    telemetry.update();
    
    sleep(20);
}
PIDF 调参指南
参数	默认值	调整建议
Kp	0.016	增大: 更快响应, 可能过冲
Ki	0.0	0.0001-0.001 消除稳态误差
Kd	0.0001	0.00005-0.0005 抑制振荡
Kf	0.5	舵机中点, 根据安装调整
故障排除表
问题现象	可能原因	解决方案
旋转不准确	电位器松动/电压不准	重新校准, 紧固电位器
误检测	传感器阈值不当	调整 DIST_TH 或 COLOR_TH
不检测	传感器故障/距离太远	检查连接, 减小检测距离
超时跳过	物体不存在/位置不对	调整预设角度
集成使用示例
完整自动程序示例
@Autonomous(name="完整自动程序", group="Competition")
public class FullAutoOpMode extends LinearOpMode {
    
    // 硬件声明
    private DcMotorEx leftFlywheel, rightFlywheel, armMotor, intakeMotor;
    private Servo spinServo, coverServo, supporterServo, collectorSpin;
    private AnalogInput spinPot, armPot, collectorPot;
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    
    // 模块声明
    private AutoSpinCollector collector;
    private AutoShooterController shooter;
    private AutoSpinFire spinner;
    private AprilTagDetector tagDetector;
    private VisionPortal visionPortal;
    
    @Override
    public void runOpMode() {
        // ========== 1. 硬件初始化 ==========
        initHardware();
        
        // ========== 2. 模块初始化 ==========
        initModules();
        
        // ========== 3. 视觉系统预热 ==========
        telemetry.addData("状态", "视觉系统初始化...");
        telemetry.update();
        
        // 等待摄像头启动
        sleep(1000);
        
        // 检测AprilTag决定策略
        int strategy = detectAprilTagStrategy();
        
        telemetry.addData("策略", "策略%d", strategy);
        telemetry.update();
        
        waitForStart();
        
        // ========== 4. 自动程序开始 ==========
        
        // 阶段1: 收集像素块
        telemetry.addData("阶段", "1 - 收集");
        telemetry.update();
        performCollection();
        
        // 阶段2: 移动到发射位置
        telemetry.addData("阶段", "2 - 移动");
        telemetry.update();
        moveToShootingPosition(strategy);
        
        // 阶段3: 发射
        telemetry.addData("阶段", "3 - 发射");
        telemetry.update();
        performShooting();
        
        // 阶段4: 停靠
        telemetry.addData("阶段", "4 - 停靠");
        telemetry.update();
        parkRobot(strategy);
        
        telemetry.addData("状态", "自动程序完成!");
        telemetry.update();
    }
    
    private void initHardware() {
        // 初始化所有硬件设备
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "left_fly");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "right_fly");
        armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        intakeMotor = hardwareMap.dcMotor.get("intake");
        
        spinServo = hardwareMap.servo.get("spin_servo");
        coverServo = hardwareMap.servo.get("cover_servo");
        supporterServo = hardwareMap.servo.get("supporter_servo");
        collectorSpin = hardwareMap.servo.get("collector_spin");
        
        spinPot = hardwareMap.analogInput.get("spin_pot");
        armPot = hardwareMap.analogInput.get("arm_pot");
        collectorPot = hardwareMap.analogInput.get("collector_pot");
        
        colorSensor = hardwareMap.colorSensor.get("color");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
    }
    
    private void initModules() {
        // 初始化收集器
        collector = new AutoSpinCollector(
            intakeMotor, collectorSpin, collectorPot, 
            colorSensor, distanceSensor
        );
        
        // 初始化发射器
        shooter = new AutoShooterController(
            this, leftFlywheel, rightFlywheel, 
            armMotor, armPot
        );
        
        // 初始化旋转发射模块
        spinner = new AutoSpinFire(
            this, spinServo, spinPot, 
            coverServo, supporterServo
        );
        
        // 初始化视觉检测 (简化示例)
        // tagDetector = ... (需要VisionPortal配置)
    }
    
    private int detectAprilTagStrategy() {
        // 简化: 返回固定策略
        return 1; // 实际应从摄像头检测
    }
    
    private void performCollection() {
        // 设置收集角度
        double pos1 = 30.0, pos2 = 150.0, pos3 = 270.0;
        
        // 开始收集 (带5秒超时)
        collector.start(pos1, pos2, pos3, 5000);
        
        // 等待收集完成
        while (opModeIsActive() && !collector.isIdle()) {
            collector.update();
            
            telemetry.addData("收集进度", collector.getCurrentState());
            telemetry.update();
            
            sleep(20);
        }
    }
    
    private void moveToShootingPosition(int strategy) {
        // 根据策略移动到不同位置
        switch (strategy) {
            case 1:
                // 移动到位置1
                // driveToPosition(x1, y1, angle1);
                break;
            case 2:
                // 移动到位置2
                // driveToPosition(x2, y2, angle2);
                break;
            case 3:
                // 移动到位置3
                // driveToPosition(x3, y3, angle3);
                break;
        }
        
        // 调整机械臂到发射角度
        shooter.moveArmOnly(45); // 45度发射角度
    }
    
    private void performShooting() {
        // 启动飞轮
        shooter.start(45, 1500); // 45度角度, 1500 RPM
        
        // 等待飞轮加速
        sleep(1000);
        
        // 执行三次发射
        spinner.fireInTurn();
        
        // 停止飞轮
        shooter.stop();
    }
    
    private void parkRobot(int strategy) {
        // 根据策略停靠到不同位置
        switch (strategy) {
            case 1:
                // 停靠到区域1
                break;
            case 2:
                // 停靠到区域2
                break;
            case 3:
                // 停靠到区域3
                break;
        }
    }
}
调试与故障排除
通用调试步骤
硬件检查


// 检查所有硬件连接
telemetry.addData("电位器电压", "%.2fV", spinPot.getVoltage());
telemetry.addData("舵机位置", "%.2f", spinServo.getPosition());
telemetry.addData("电机速度", "%d", leftFlywheel.getVelocity());
模块单独测试


// 创建测试模式
@TeleOp(name="模块测试", group="Test")
public class ModuleTestOpMode extends LinearOpMode {
    // 单独测试每个模块的功能
}
参数实时调整 (使用FTC Dashboard)
常见问题解决方案
问题1: 旋转不到位
症状: 旋转机构在目标角度附近振荡或无法到达
解决:

调整 Kp (增大使响应更快)

调整 Kd (增大抑制振荡)

检查电位器电压范围是否正确

问题2: 发射不稳定
症状: 像素块发射力度不一致
解决:

确保飞轮速度稳定后再发射

调整 spinDuration (增加停留时间)

检查支撑器时序 (beforeSupporterDuration)

问题3: 检测不准确
症状: AprilTag或物体检测不稳定
解决:

优化摄像头位置和角度

调整光照条件

增加检测稳定时间

性能优化建议
减少延迟


// 使用 while 循环代替多个 sleep
while (System.currentTimeMillis() - startTime < duration) {
    updateAllModules(); // 集中更新所有模块
    sleep(5); // 短间隔
}
并行处理


// 在主循环中并行更新多个模块
@Override
public void runOpMode() {
    while (opModeIsActive()) {
        collector.update();
        tagDetector.update();
        // ... 其他模块
        sleep(10);
    }
}
内存管理

及时释放VisionPortal资源

避免在循环中创建新对象

使用局部变量代替成员变量

附录
参数推荐值
模块	参数	推荐值	说明
AutoSpinFire	spinDuration	400-600ms	根据发射力度调整
AutoSpinFire	spinAngleTolerance	3-5°	精度要求高则减小
AutoShooterController	ARM_Kp	0.01-0.02	机械臂刚性决定
AutoSpinCollector	DIST_TH	4-6cm	根据传感器性能
AutoSpinCollector	COLOR_TH	180-220	环境光照影响


-->
