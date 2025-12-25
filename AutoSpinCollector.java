package org.firstinspires.ftc.teamcode.AutoLib;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Libraries.MapUtils;

public class AutoSpinCollector {

    private enum State {
        IDLE,
        MOVE_TO_P1, DETECT_P1,
        MOVE_TO_P2, DETECT_P2,
        MOVE_TO_P3, DETECT_P3,
        RETURN_HOME
    }

    private State state = State.IDLE;

    private final DcMotor intake;
    private final Servo spin;
    private final AnalogInput potentiometer;
    private final ColorSensor color;
    private final DistanceSensor distance;

    private double pos1, pos2, pos3;
    private static final double HOME = 10.0;

    private static final double DIST_TH = 5.0;
    private static final int COLOR_TH = 200;
    private static final double ANGLE_TOL = 4.0;

    // PIDF 参数
    public static double Kp = 0.016, Ki = 0, Kd = 0.0001;
    public static double Kf = 0.5;  // ⭐ 前馈，默认舵机中点

    private double targetAngle = HOME;
    private double integral = 0, lastError = 0;
    private long lastTime = 0;

    // 超时相关变量
    private ElapsedTime detectTimer = new ElapsedTime();
    private long maxCollectTime = 0; // 0表示无时间限制
    private boolean timeoutEnabled = false;

    public boolean isIdle() {
        return state == State.IDLE;
    }

    // 获取当前状态（用于调试）
    public State getCurrentState() {
        return state;
    }

    public AutoSpinCollector(
            DcMotor intake,
            Servo spin,
            AnalogInput potentiometer,
            ColorSensor color,
            DistanceSensor distance
    ) {
        this.intake = intake;
        this.spin = spin;
        this.potentiometer = potentiometer;
        this.color = color;
        this.distance = distance;
    }

    // 原有的start方法，保持向后兼容
    public void start(double p1, double p2, double p3) {
        start(p1, p2, p3, 0); // 0表示无时间限制
    }

    // 新的start方法，支持超时参数
    public void start(double p1, double p2, double p3, long maxCollectTimeMs) {
        if (state != State.IDLE) return;

        pos1 = p1;
        pos2 = p2;
        pos3 = p3;

        // 设置超时参数
        this.maxCollectTime = maxCollectTimeMs;
        this.timeoutEnabled = (maxCollectTimeMs > 0);

        targetAngle = pos1;
        intake.setPower(0);
        resetPID();
        state = State.MOVE_TO_P1;

        // 重置计时器
        detectTimer.reset();
    }

    public void stop() {
        intake.setPower(0);
        state = State.IDLE;
    }

    public void update() {
        if (state == State.IDLE) return;

        double current = getCurrentAngle();
        spin.setPosition(runPIDF(current));

        boolean detected =
                distance.getDistance(DistanceUnit.CM) < DIST_TH
                        || color.alpha() > COLOR_TH;

        // 检查超时（只在检测状态下检查）
        boolean timeoutOccurred = timeoutEnabled &&
                detectTimer.milliseconds() > maxCollectTime &&
                (state == State.DETECT_P1 || state == State.DETECT_P2 || state == State.DETECT_P3);

        switch (state) {
            case MOVE_TO_P1:
                if (reached(current)) {
                    intake.setPower(1);
                    detectTimer.reset(); // 开始检测时重置计时器
                    state = State.DETECT_P1;
                }
                break;

            case DETECT_P1:
                if (detected) {
                    targetAngle = pos2;
                    resetPID();
                    state = State.MOVE_TO_P2;
                } else if (timeoutOccurred) {
                    // 超时，跳过当前位置
                    targetAngle = pos2;
                    resetPID();
                    state = State.MOVE_TO_P2;
                }
                break;

            case MOVE_TO_P2:
                if (reached(current)) {
                    detectTimer.reset(); // 开始检测时重置计时器
                    state = State.DETECT_P2;
                }
                break;

            case DETECT_P2:
                if (detected) {
                    targetAngle = pos3;
                    resetPID();
                    state = State.MOVE_TO_P3;
                } else if (timeoutOccurred) {
                    // 超时，跳过当前位置
                    targetAngle = pos3;
                    resetPID();
                    state = State.MOVE_TO_P3;
                }
                break;

            case MOVE_TO_P3:
                if (reached(current)) {
                    detectTimer.reset(); // 开始检测时重置计时器
                    state = State.DETECT_P3;
                }
                break;

            case DETECT_P3:
                if (detected) {
                    // 检测到物体，返回HOME
                    targetAngle = HOME;
                    resetPID();
                    intake.setPower(0);
                    state = State.RETURN_HOME;
                } else if (timeoutOccurred) {
                    // 超时，也返回HOME
                    targetAngle = HOME;
                    resetPID();
                    intake.setPower(0);
                    state = State.RETURN_HOME;
                }
                break;

            case RETURN_HOME:
                if (reached(current)) {
                    state = State.IDLE;
                }
                break;
        }
    }

    // ===== PIDF 控制 =====
    private double runPIDF(double current) {
        double error = targetAngle - current;
        if (error > 180) error -= 360;
        else if (error < -180) error += 360;

        long now = System.currentTimeMillis();
        double dt = (now - lastTime) / 1000.0;
        if (dt <= 0) dt = 0.02;
        lastTime = now;

        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double pid = Kp * error + Ki * integral + Kd * derivative;

        // PIDF 输出
        double output = Kf - pid;

        // 限幅 0~1，适合伺服 setPosition
        output = Math.max(0.0, Math.min(1.0, output));

        return output;
    }

    private void resetPID() {
        integral = 0;
        lastError = 0;
        lastTime = System.currentTimeMillis();
    }

    private double getCurrentAngle() {
        return MapUtils.mapClamp(
                potentiometer.getVoltage(),
                0, 3.26,
                0, 360
        );
    }

    private boolean reached(double current) {
        double e = Math.abs(targetAngle - current);
        if (e > 180) e = 360 - e;
        return e < ANGLE_TOL;
    }
}