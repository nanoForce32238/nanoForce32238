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
    private static final double POT_MIN_VOLTAGE = 1.78;  // 对应 0 度
    private static final double POT_MAX_VOLTAGE = 2.01;  // 对应 100 度

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
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // 初始化机械臂电机
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /* ================== 对外接口 ================== */

    /** 开启 shooter（会自动移动 arm） */
    public void start(int targetAngle, double speed) {
        left.setVelocity(speed);
        right.setVelocity(speed);

        moveArm(targetAngle);

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

        // 最大运行时间设为 2 秒
        ElapsedTime timeoutTimer = new ElapsedTime();
        timeoutTimer.reset();
        final double TIMEOUT_MS = 2000;

        while (opMode.opModeIsActive()
                && timeoutTimer.milliseconds() < TIMEOUT_MS) {

            int currentAngle = getArmAngle();
            double error = targetAngle - currentAngle;

            // 到位判定（±2°）
            if (Math.abs(error) <= 2) {
                armMotor.setPower(0);
                integral = 0;
                return;
            }

            double currentTime = pidTimer.seconds();
            double dt = currentTime - lastTime;
            if (lastTime == 0 || dt <= 0) {
                dt = 0.02;
            }

            integral += error * dt;
            integral = Math.max(-100, Math.min(100, integral));

            double derivative = (error - lastError) / dt;

            double output =
                    ARM_Kp * error +
                            ARM_Ki * integral +
                            ARM_Kd * derivative;

            output = Math.max(-1.0, Math.min(1.0, output));
            armMotor.setPower(output);

            lastError = error;
            lastTime = currentTime;

            opMode.sleep(20);
        }

        // 超时兜底
        armMotor.setPower(0);
        integral = 0;
    }

    /**
     * 角度读取（⚠️这是你原来正确的映射）
     */
    private int getArmAngle() {
        double voltage = armPot.getVoltage();

        double angle =
                100 - ((voltage - POT_MIN_VOLTAGE)
                        / (POT_MAX_VOLTAGE - POT_MIN_VOLTAGE) * 100);

        angle = Math.max(0, Math.min(100, angle));
        return (int) angle;
    }

    /** 获取当前机械臂角度（用于调试） */
    public int getCurrentArmAngle() {
        return getArmAngle();
    }

    /**
     * 阻塞式移动机械臂并等待完成（带超时）
     */
    public boolean moveArmToPosition(int targetAngle, int timeoutMs) {

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opMode.opModeIsActive()
                && timer.milliseconds() < timeoutMs) {

            moveArm(targetAngle);

            if (Math.abs(getArmAngle() - targetAngle) <= 2) {
                return true;
            }

            opMode.sleep(10);
        }

        return false;
    }
}
