package org.firstinspires.ftc.teamcode.AutoLib;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoShooterController {

    /* ================== 硬件 ================== */
    private final DcMotorEx left;
    private final DcMotorEx right;
    private final DcMotorEx armMotor;
    private final AnalogInput armPot;
    private final LinearOpMode opMode;

    /* ================== Arm PID 参数 ================== */
    private static final double ARM_Kp = 0.015;
    private static final double ARM_Ki = 0.000;
    private static final double ARM_Kd = 0.0001;

    /* ================== 电位器校准参数 ================== */
    private static final double POT_MIN_VOLTAGE = 1.78;  // 对应0度
    private static final double POT_MAX_VOLTAGE = 2.01;  // 对应100度

    /* ================== 构造 ================== */
    public AutoShooterController(
            LinearOpMode opMode,
            DcMotorEx left,
            DcMotorEx right,
            DcMotorEx armMotor,
            AnalogInput armPot
    ) {
        this.opMode = opMode;
        this.left = left;
        this.right = right;
        this.armMotor = armMotor;
        this.armPot = armPot;

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // 初始化机械臂电机
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /* ================== 对外接口 ================== */

    /** 开启 shooter（会自动移动 arm） */
    public void start(int targetAngle, double speed) {
        moveArm(targetAngle);
        left.setVelocity(-speed);
        right.setVelocity(speed);
    }

    /** 停止 shooter */
    public void stop() {
        left.setVelocity(0);
        right.setVelocity(0);
        armMotor.setPower(0);
    }

    /** 只移动机械臂，不启动发射器 */
    public void moveArmOnly(int targetAngle) {
        moveArm(targetAngle);
    }

    /* ================== 改进的 Arm PID 控制 ================== */

    /**
     * 使用改进的 PID 控制 arm 旋转到目标角度
     * 最大运行时间：2000ms（2秒）
     */
    private void moveArm(int targetAngle) {
        ElapsedTime pidTimer = new ElapsedTime();
        double integral = 0;
        double lastError = 0;
        double lastTime = 0;

        // 最大运行时间设为2秒
        ElapsedTime timeoutTimer = new ElapsedTime();
        timeoutTimer.reset();

        final double TIMEOUT_MS = 2000;

        while (opMode.opModeIsActive() && timeoutTimer.milliseconds() < TIMEOUT_MS) {
            int currentAngle = getArmAngle();
            double error = targetAngle - currentAngle;

            // 到位判定（±2°）
            if (Math.abs(error) <= 2) {
                armMotor.setPower(0);
                integral = 0; // 重置积分项
                return;
            }

            // 获取时间增量
            double currentTime = pidTimer.seconds();
            double dt = currentTime - lastTime;
            if (lastTime == 0 || dt <= 0) {
                dt = 0.02; // 默认20ms
            }

            // 计算积分项（限制积分饱和）
            integral += error * dt;
            integral = Math.max(-100, Math.min(100, integral));

            // 计算微分项
            double derivative = (error - lastError) / dt;

            // 计算输出
            double output = ARM_Kp * error + ARM_Ki * integral + ARM_Kd * derivative;

            // 限制输出范围
            output = Math.max(-1.0, Math.min(1.0, output));

            armMotor.setPower(output);

            lastError = error;
            lastTime = currentTime;

            // 短暂暂停，让控制循环稳定
            opMode.sleep(20);
        }

        // 超时兜底
        armMotor.setPower(0);
        integral = 0;
    }

    /**
     * 改进的角度读取方法，使用map函数将电压映射到角度
     */
    private int getArmAngle() {
        double voltage = armPot.getVoltage();

        // 使用线性映射将电压转换为角度
        // 电压范围[POT_MIN_VOLTAGE, POT_MAX_VOLTAGE] 映射到角度范围[0, 100]
        double angle = 100 - ((voltage - POT_MIN_VOLTAGE) / (POT_MAX_VOLTAGE - POT_MIN_VOLTAGE) * 100);

        // 限制角度在0-100范围内
        angle = Math.max(0, Math.min(100, angle));

        return (int) angle;
    }

    /**
     * 获取当前机械臂角度（用于调试）
     */
    public int getCurrentArmAngle() {
        return getArmAngle();
    }

    /**
     * 阻塞式移动机械臂并等待完成（带超时）
     * @param targetAngle 目标角度(0-100)
     * @param timeoutMs 超时时间(毫秒)
     * @return 是否成功到达目标
     */
    public boolean moveArmToPosition(int targetAngle, int timeoutMs) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opMode.opModeIsActive() && timer.milliseconds() < timeoutMs) {
            moveArm(targetAngle);

            if (Math.abs(getArmAngle() - targetAngle) <= 2) {
                return true; // 成功到达
            }

            opMode.sleep(10);
        }

        return false; // 超时或中断
    }
}