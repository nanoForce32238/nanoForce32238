package org.firstinspires.ftc.teamcode.AutoLib;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.*;

import java.util.List;

public class AprilTagDetector {

    private final AprilTagProcessor processor;
    private final LinearOpMode opMode;

    private int result = 0;

    public AprilTagDetector(
            LinearOpMode opMode,
            VisionPortal portal,
            AprilTagProcessor processor
    ) {
        this.opMode = opMode;
        this.processor = processor;
    }

    /** 在 Action / loop 中反复调用 */
    public void update() {
        if (result != 0) return;

        List<AprilTagDetection> detections = processor.getDetections();
        for (AprilTagDetection d : detections) {
            if (d.id == 21) result = 1;
            else if (d.id == 22) result = 2;
            else if (d.id == 23) result = 3;
        }
    }

    public int getResult() {
        return result;
    }

    public boolean hasResult() {
        return result != 0;
    }
}
