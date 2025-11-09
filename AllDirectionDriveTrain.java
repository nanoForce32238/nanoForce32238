package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "麦轮360度全向控制", group = "比赛")
public class AllDirectionDriveTrain extends LinearOpMode {

    // 定义四个麦克纳姆轮电机
    private DcMotor leftFrontMotor;   // 左前轮
    private DcMotor rightFrontMotor;  // 右前轮
    private DcMotor leftBackMotor;    // 左后轮
    private DcMotor rightBackMotor;   // 右后轮

    @Override
    public void runOpMode() {
        // 初始化硬件
        initHardware();

        telemetry.addData("状态", "就绪");
        telemetry.addData("控制说明", "左摇杆:任意方向移动 右摇杆左右:旋转");
        telemetry.addData("全向功能", "支持360度任意方向移动");
        telemetry.update();

        // 等待比赛开始
        waitForStart();

        // 主循环
        while (opModeIsActive()) {
            // 计算并设置电机功率
            calculateAndSetPowers();

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
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        // 设置电机模式
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 初始时停止所有电机
        stopAllMotors();
    }

    private void calculateAndSetPowers() {
        // 获取左摇杆输入（移动控制）
        double driveX = gamepad1.left_stick_x;   // 左右平移
        double driveY = -gamepad1.left_stick_y;  // 前后移动（反转Y轴）

        // 获取右摇杆X轴输入（旋转控制）
        double rotate = gamepad1.right_stick_x;  // 左右旋转

        // 计算移动向量的角度和大小
        double moveAngle = Math.atan2(driveY, driveX);        // 移动方向角度（弧度）
        double moveMagnitude = Math.sqrt(driveX*driveX + driveY*driveY); // 移动速度大小

        // 限制移动速度在0-1之间
        moveMagnitude = Range.clip(moveMagnitude, 0, 1);

        // 麦克纳姆轮全向运动算法
        double[] motorPowers = calculateMecanumPowers(moveAngle, moveMagnitude, rotate);

        // 设置电机功率
        setMotorPowers(motorPowers);
    }

    private double[] calculateMecanumPowers(double angle, double magnitude, double rotation) {
        /**
         * 麦克纳姆轮全向运动核心算法
         * 参数：
         * angle - 移动方向（弧度），0弧度=正右方，π/2=正前方
         * magnitude - 移动速度（0到1）
         * rotation - 旋转分量（-1到1）
         */

        // 计算三角函数值
        double cosA = Math.cos(angle);  // X方向分量
        double sinA = Math.sin(angle);  // Y方向分量

        // 麦克纳姆轮功率计算公式：
        // 每个轮子功率 = 前后移动分量 + 左右移动分量 + 旋转分量

        // 左前轮：前后移动(sinA) + 左右移动(cosA) + 旋转
        double leftFrontPower = magnitude * (sinA + cosA) + rotation;

        // 右前轮：前后移动(sinA) - 左右移动(cosA) - 旋转
        double rightFrontPower = magnitude * (sinA - cosA) - rotation;

        // 左后轮：前后移动(sinA) - 左右移动(cosA) + 旋转
        double leftBackPower = magnitude * (sinA - cosA) + rotation;

        // 右后轮：前后移动(sinA) + 左右移动(cosA) - 旋转
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

    private void setMotorPowers(double[] powers) {
        leftFrontMotor.setPower(powers[0]);
        rightFrontMotor.setPower(powers[1]);
        leftBackMotor.setPower(powers[2]);
        rightBackMotor.setPower(powers[3]);
    }

    private void stopAllMotors() {
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
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

        // 显示基本信息
        telemetry.addData("左摇杆x", "%.1f°", driveX);
        telemetry.addData("左摇杆y", "%.1f°", driveY);
        telemetry.addData("移动方向", "%.1f°", moveAngleDeg);
        telemetry.addData("移动速度", "%.2f", moveMagnitude);
        telemetry.addData("旋转输入", "%.2f", gamepad1.right_stick_x);

        // 显示各电机功率
        telemetry.addData("左前轮功率", "%.2f", leftFrontMotor.getPower());
        telemetry.addData("右前轮功率", "%.2f", rightFrontMotor.getPower());
        telemetry.addData("左后轮功率", "%.2f", leftBackMotor.getPower());
        telemetry.addData("右后轮功率", "%.2f", rightBackMotor.getPower());

        // 显示当前移动方向
        telemetry.addData("当前方向", getDirectionDescription(moveAngleDeg, moveMagnitude));

        telemetry.update();
    }

    private String getDirectionDescription(double angle, double magnitude) {
        if (magnitude < 0.1) {
            return "停止";
        }

        // 将360度分为8个主要方向（仅用于显示，实际控制是连续的）
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