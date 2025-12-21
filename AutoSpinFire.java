package org.firstinspires.ftc.teamcode.AutoLib;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Libraries.MapUtils;

/**
 * 自动发射模块：Spin + Cover + Supporter
 * 比赛级非阻塞实现（带 spin 防卡死机制）
 */
@Config
public class AutoSpinFire {

    /* ================== 硬件 ================== */
    private final LinearOpMode opMode;
    private final Servo spin;
    private final AnalogInput spinPot;
    private final Servo cover;
    private final Servo supporter;

    /* ================== PIDF 参数 ================== */
    public static double Kp = 0.0125;
    public static double Ki = 0.0;
    public static double Kd = 0.0002;
    public static double Kf = 0.5;

    /* ================== 时间参数（可调） ================== */
    /** spin 在每个角度停留时间 */
    public static long spinDuration = 300;

    /** supporter 触发前等待时间 */
    public static long beforeSupporterDuration = 350;

    /** supporter 抬升 → 下降 间隔 */
    public static long supportDuration = 250;

    /** 单次 spin 旋转最大允许时间（防死循环） */
    public static long spinTimeout = 1200;

    /** spin 到位角度允许误差（度） */
    public static double spinAngleTolerance = 3.0;

    /* ================== PIDF 状态 ================== */
    private double integral = 0;
    private double lastError = 0;
    private long lastTime = 0;

    /* ================== Supporter 状态机 ================== */
    private enum SupporterState {
        IDLE,
        WAIT_BEFORE_UP,
        WAIT_BEFORE_DOWN
    }

    private SupporterState supporterState = SupporterState.IDLE;
    private long supporterTimer = 0;

    /* ================== 构造 ================== */
    public AutoSpinFire(
            LinearOpMode opMode,
            Servo spin,
            AnalogInput spinPot,
            Servo cover,
            Servo supporter
    ) {
        this.opMode = opMode;
        this.spin = spin;
        this.spinPot = spinPot;
        this.cover = cover;
        this.supporter = supporter;
        lastTime = System.currentTimeMillis();
    }

    /* ======================================================
                        对外主接口
       ====================================================== */

    /**
     * 执行三次发射动作（10 → 130 → 250）
     * ❗不再回到 finalPosition
     */
    public void fireInTurn() {

        double[] positions = {10, 130, 250};
        boolean coverOpened = false;

        for (int i = 0; i < positions.length; i++) {

            moveToAngle(positions[i]);

            // 第一次到 10° 打开 cover
            if (!coverOpened && i == 0) {
                cover.setPosition(0.55);
                coverOpened = true;
                opMode.sleep(250);
            }

            // 请求一次 supporter 触发
            requestSupporterTrigger();

            // 在该位置停留 spinDuration，同时维护 supporter 状态
            long stayStart = System.currentTimeMillis();
            while (opMode.opModeIsActive()
                    && System.currentTimeMillis() - stayStart < spinDuration) {

                updateSupporter();
                opMode.sleep(10);
            }
        }

        // ❌ 不再 moveToAngle(finalPosition)

        // 关闭 cover
//        if (coverOpened) {
//            cover.setPosition(0.86);
//        }
    }

    /* ======================================================
                        Supporter 状态机
       ====================================================== */

    /** 请求一次 supporter 触发（非阻塞） */
    private void requestSupporterTrigger() {
        if (supporterState == SupporterState.IDLE) {
            supporterTimer = System.currentTimeMillis();
            supporterState = SupporterState.WAIT_BEFORE_UP;
        }
    }

    /** 在循环中持续调用 */
    private void updateSupporter() {

        long now = System.currentTimeMillis();

        switch (supporterState) {

            case WAIT_BEFORE_UP:
                if (now - supporterTimer >= beforeSupporterDuration) {
                    supporter.setPosition(0.55);
                    supporterTimer = now;
                    supporterState = SupporterState.WAIT_BEFORE_DOWN;
                }
                break;

            case WAIT_BEFORE_DOWN:
                if (now - supporterTimer >= supportDuration) {
                    supporter.setPosition(0.86);
                    supporterState = SupporterState.IDLE;
                }
                break;

            case IDLE:
            default:
                break;
        }
    }

    /* ======================================================
                        Spin PIDF 控制
       ====================================================== */

    public void moveToAngle(double targetAngle) {

        integral = 0;
        lastError = 0;
        lastTime = System.currentTimeMillis();

        int stableCount = 0;
        long startTime = System.currentTimeMillis();

        while (opMode.opModeIsActive()) {

            // 超时保护
            if (System.currentTimeMillis() - startTime > spinTimeout) {
                break;
            }

            double currentAngle = getSpinAngle();
            double output = calculatePIDF(currentAngle, targetAngle);
            spin.setPosition(output);

            double error = targetAngle - currentAngle;
            if (error > 180) error -= 360;
            else if (error < -180) error += 360;

            // 到位判定
            if (Math.abs(error) <= spinAngleTolerance) {
                stableCount++;
            } else {
                stableCount = 0;
            }

            if (stableCount >= 3) break;

            updateSupporter();
            opMode.sleep(20);
        }
    }

    private double getSpinAngle() {
        double voltage = spinPot.getVoltage();
        return MapUtils.mapClamp(voltage, 0.0, 3.26, 0.0, 360.0);
    }

    private double calculatePIDF(double currentAngle, double targetAngle) {

        double error = targetAngle - currentAngle;
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
        double output = Kf - pid;

        return Math.max(0.0, Math.min(1.0, output));
    }
}
