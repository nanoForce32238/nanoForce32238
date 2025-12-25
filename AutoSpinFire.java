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
    public static double Kp = 0.01;
    public static double Ki = 0.0;
    public static double Kd = 0.0001;
    public static double Kf = 0.5;

    /* ================== 时间参数（可调） ================== */
    /** spin 在每个角度停留时间 */
    public static long spinDuration = 500; //500

    /** supporter 触发前等待时间 */
    public static long beforeSupporterDuration = 350; //300

    /** supporter 抬升 → 下降 间隔 */
    public static long supportDuration = 150; //150

    /** 单次 spin 旋转最大允许时间（防死循环） */
    public static long spinTimeout = 2500;

    /** spin 到位角度允许误差（度） */
    public static double spinAngleTolerance = 4;

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
     * 执行三次发射动作（使用默认角度 11 → 131 → 251）
     * ❗不再回到 finalPosition
     */
    public void fireInTurn() {
        // 调用重载方法，使用默认角度
        fireInTurn(10, 130, 250);
    }

    /**
     * 执行三次发射动作（自定义角度）
     * ❗不再回到 finalPosition
     * @param angle1 第一个发射角度
     * @param angle2 第二个发射角度
     * @param angle3 第三个发射角度
     */
    public void fireInTurn(double angle1, double angle2, double angle3) {
        double[] positions = {angle1, angle2, angle3};
        boolean coverOpened = false;

        for (int i = 0; i < positions.length; i++) {

            // 第一次发射时打开 cover
            if (!coverOpened && i == 0) {
                cover.setPosition(0.55);
                coverOpened = true;
                opMode.sleep(0);
            }

            moveToAngle(positions[i]);



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

        // 可选：发射完成后关闭 cover
        // if (coverOpened) {
        //     cover.setPosition(0.86);
        // }
    }

    /**
     * 智能发射：从离当前位置最近的角度开始，顺时针发射三个位置
     * 使用默认角度：11, 131, 251
     */
    public void fireInTurn_Manual() {
        fireInTurn_Manual(11, 131, 251);
    }

    /**
     * 智能发射：从离当前位置最近的角度开始，顺时针发射三个位置
     * @param angle1 第一个角度
     * @param angle2 第二个角度
     * @param angle3 第三个角度
     */
    public void fireInTurn_Manual(double angle1, double angle2, double angle3) {
        // 获取当前角度
        double currentAngle = getSpinAngle();

        // 将三个角度放入数组
        double[] angles = {angle1, angle2, angle3};

        // 找到离当前角度最近的角度
        int nearestIndex = findNearestAngleIndex(currentAngle, angles);

        // 构建发射顺序：从最近的角度开始，顺时针发射
        double[] firingOrder = new double[3];
        for (int i = 0; i < 3; i++) {
            firingOrder[i] = angles[(nearestIndex + i) % 3];
        }

        // 执行发射
        boolean coverOpened = false;

        for (int i = 0; i < firingOrder.length; i++) {

            // 第一次发射时打开 cover
            if (!coverOpened && i == 0) {
                cover.setPosition(0.55);
                coverOpened = true;
                opMode.sleep(0);
            }

            moveToAngle(firingOrder[i]);



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

        // 可选：发射完成后关闭 cover
        // if (coverOpened) {
        //     cover.setPosition(0.86);
        // }
    }

    /**
     * 辅助方法：找到离当前角度最近的角度的索引
     * @param currentAngle 当前角度 (0-360度)
     * @param angles 角度数组
     * @return 最近角度的索引
     */
    private int findNearestAngleIndex(double currentAngle, double[] angles) {
        int nearestIndex = 0;
        double minDistance = Double.MAX_VALUE;

        for (int i = 0; i < angles.length; i++) {
            // 计算两个角度之间的最短距离（考虑360度循环）
            double diff = Math.abs(angles[i] - currentAngle);
            double distance = Math.min(diff, 360 - diff);

            if (distance < minDistance) {
                minDistance = distance;
                nearestIndex = i;
            }
        }

        return nearestIndex;
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