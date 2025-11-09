package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "匀加减速麦轮全向底盘", group = "比赛")
public class SmoothDriveTrain extends LinearOpMode {

    // 定义四个麦克纳姆轮电机
    private DcMotor leftFrontMotor;   // 左前轮
    private DcMotor rightFrontMotor;  // 右前轮
    private DcMotor leftBackMotor;    // 左后轮
    private DcMotor rightBackMotor;   // 右后轮

    // 当前电机功率（用于平滑控制）
    private double currentLeftFrontPower = 0;
    private double currentRightFrontPower = 0;
    private double currentLeftBackPower = 0;
    private double currentRightBackPower = 0;

    // 加速度控制参数
    private final double ACCELERATION_RATE = 0.0001;  // 加速度率（每个循环增加的最大功率）
    private final double DECELERATION_RATE = 0.10;  // 减速度率（每个循环减少的最大功率）

    // 计时器
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // 初始化硬件
        initHardware();

        telemetry.addData("状态", "就绪");
        telemetry.addData("控制说明", "左摇杆:任意方向移动 右摇杆左右:旋转");
        telemetry.addData("平滑控制", "已启用匀加速/匀减速");
        telemetry.update();

        // 等待比赛开始
        waitForStart();
        runtime.reset();

        // 主循环
        while (opModeIsActive()) {
            // 计算目标电机功率
            double[] targetPowers = calculateTargetPowers();

            // 应用平滑加速度控制
            applySmoothAcceleration(targetPowers);

            // 设置电机功率
            setMotorPowers();

            // 更新遥测数据
            updateTelemetry();
        }
    }

    private void initHardware() {
        // 映射四个电机
        leftFrontMotor = hardwareMap.get(DcMotor.class, "lf");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rf");
        leftBackMotor = hardwareMap.get(DcMotor.class, "lb");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rb");

        // 设置电机方向（根据实际安装调整）
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        // 设置电机模式
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 初始时停止所有电机
        stopAllMotors();
    }

    private double[] calculateTargetPowers() {
        // 获取左摇杆输入（移动控制）
        double driveX = gamepad1.left_stick_x;   // 左右平移
        double driveY = -gamepad1.left_stick_y;  // 前后移动（反转Y轴）

        // 获取右摇杆X轴输入（旋转控制）
        double rotate = gamepad1.right_stick_x;  // 左右旋转

        // 计算移动向量的角度和大小
        double moveAngle = Math.atan2(driveY, driveX);
        double moveMagnitude = Math.sqrt(driveX*driveX + driveY*driveY);

        // 限制移动速度在0-1之间
        moveMagnitude = Range.clip(moveMagnitude, 0, 1);

        // 麦克纳姆轮全向运动算法
        return calculateMecanumPowers(moveAngle, moveMagnitude, rotate);
    }

    private double[] calculateMecanumPowers(double angle, double magnitude, double rotation) {
        // 计算三角函数值
        double cosA = Math.cos(angle);  // X方向分量
        double sinA = Math.sin(angle);  // Y方向分量

        // 麦克纳姆轮功率计算公式
        double leftFrontPower = magnitude * (sinA + cosA) + rotation;
        double rightFrontPower = magnitude * (sinA - cosA) - rotation;
        double leftBackPower = magnitude * (sinA - cosA) + rotation;
        double rightBackPower = magnitude * (sinA + cosA) - rotation;

        // 标准化功率值，确保不超过±1.0
        return normalizePowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    private double[] normalizePowers(double lf, double rf, double lb, double rb) {
        // 找到最大功率绝对值
        double maxPower = Math.max(Math.abs(lf), Math.abs(rf));
        maxPower = Math.max(maxPower, Math.abs(lb));
        maxPower = Math.max(maxPower, Math.abs(rb));

        // 如果任何功率超过1.0，则按比例缩放所有功率
        if (maxPower > 1.0) {
            lf /= maxPower;
            rf /= maxPower;
            lb /= maxPower;
            rb /= maxPower;
        }

        return new double[]{lf, rf, lb, rb};
    }

    private void applySmoothAcceleration(double[] targetPowers) {
        double leftFrontTarget = targetPowers[0];
        double rightFrontTarget = targetPowers[1];
        double leftBackTarget = targetPowers[2];
        double rightBackTarget = targetPowers[3];

        // 对每个电机应用平滑加速度控制
        currentLeftFrontPower = smoothPowerChange(currentLeftFrontPower, leftFrontTarget);
        currentRightFrontPower = smoothPowerChange(currentRightFrontPower, rightFrontTarget);
        currentLeftBackPower = smoothPowerChange(currentLeftBackPower, leftBackTarget);
        currentRightBackPower = smoothPowerChange(currentRightBackPower, rightBackTarget);
    }

    private double smoothPowerChange(double current, double target) {
        // 如果目标功率接近0，使用更快的减速度
        if (Math.abs(target) < 0.05) {
            return approachZero(current);
        }

        // 计算功率差异
        double difference = target - current;

        // 根据差异方向选择加速度或减速度率
        double rate = (Math.abs(difference) > Math.abs(current)) ? ACCELERATION_RATE : DECELERATION_RATE;

        // 应用平滑变化
        if (difference > rate) {
            return current + rate;
        } else if (difference < -rate) {
            return current - rate;
        } else {
            return target;
        }
    }

    private double approachZero(double current) {
        // 当目标功率为0时，快速减速
        if (Math.abs(current) < DECELERATION_RATE) {
            return 0;
        } else if (current > 0) {
            return current - DECELERATION_RATE;
        } else {
            return current + DECELERATION_RATE;
        }
    }

    private void setMotorPowers() {
        leftFrontMotor.setPower(currentLeftFrontPower);
        rightFrontMotor.setPower(currentRightFrontPower);
        leftBackMotor.setPower(currentLeftBackPower);
        rightBackMotor.setPower(currentRightBackPower);
    }

    private void stopAllMotors() {
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);

        // 重置当前功率状态
        currentLeftFrontPower = 0;
        currentRightFrontPower = 0;
        currentLeftBackPower = 0;
        currentRightBackPower = 0;
    }

    private void updateTelemetry() {
        // 获取左摇杆输入
        double driveX = gamepad1.left_stick_x;
        double driveY = -gamepad1.left_stick_y;

        // 计算移动角度和速度
        double moveAngle = Math.atan2(driveY, driveX);
        double moveMagnitude = Math.sqrt(driveX*driveX + driveY*driveY);

        // 转换为角度（0-360度）
        double moveAngleDeg = Math.toDegrees(moveAngle);
        if (moveAngleDeg < 0) moveAngleDeg += 360;

        // 计算目标功率
        double[] targetPowers = calculateTargetPowers();

        // 显示基本信息
        telemetry.addData("运行时间", "%.1f 秒", runtime.seconds());
        telemetry.addData("移动方向", "%.1f°", moveAngleDeg);
        telemetry.addData("移动速度", "%.2f", moveMagnitude);
        telemetry.addData("旋转输入", "%.2f", gamepad1.right_stick_x);

        telemetry.addLine();
        telemetry.addData("左前电机", "目标: %.2f, 实际: %.2f",
                targetPowers[0], currentLeftFrontPower);
        telemetry.addData("右前电机", "目标: %.2f, 实际: %.2f",
                targetPowers[1], currentRightFrontPower);
        telemetry.addData("左后电机", "目标: %.2f, 实际: %.2f",
                targetPowers[2], currentLeftBackPower);
        telemetry.addData("右后电机", "目标: %.2f, 实际: %.2f",
                targetPowers[3], currentRightBackPower);

        telemetry.addLine();
        telemetry.addData("当前方向", getDirectionDescription(moveAngleDeg, moveMagnitude));
        telemetry.addData("控制模式", "全向移动 + 独立旋转 + 平滑控制");

        telemetry.update();
    }

    private String getDirectionDescription(double angle, double magnitude) {
        if (magnitude < 0.1) {
            return "停止";
        }

        // 将360度分为8个主要方向（仅用于显示）
        if (angle >= 337.5 || angle < 22.5) return "正右方 →";
        if (angle < 67.5) return "右前方 ↗";
        if (angle < 112.5) return "正前方 ↑";
        if (angle < 157.5) return "左前方 ↖";
        if (angle < 202.5) return "正左方 ←";
        if (angle < 247.5) return "左后方 ↙";
        if (angle < 292.5) return "正后方 ↓";
        return "右后方 ↘";
    }
}