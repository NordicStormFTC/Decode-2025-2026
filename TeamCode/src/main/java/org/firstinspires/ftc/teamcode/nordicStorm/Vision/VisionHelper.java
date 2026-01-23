package org.firstinspires.ftc.teamcode.nordicStorm.Vision;

import static org.firstinspires.ftc.teamcode.nordicStorm.NordicConstants.CAMERA_HEIGHT;
import static org.firstinspires.ftc.teamcode.nordicStorm.NordicConstants.CAMERA_PITCH_DEG;
import static org.firstinspires.ftc.teamcode.nordicStorm.NordicConstants.TARGET_HEIGHT;

import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.Deque;
import java.util.LinkedList;
import java.util.List;

public class VisionHelper {

    private final Deque<List<LLResultTypes.DetectorResult>> frameQueue;
    private final int maxSize;
    private SmoothedTarget smoothedTarget = null;
    private static final double ALPHA = 0.35;

    public VisionHelper(int size) {
        frameQueue = new LinkedList<List<LLResultTypes.DetectorResult>>();
        this.maxSize = size;
    }

    public void update(List<LLResultTypes.DetectorResult> newBlock) {
        if (frameQueue.size() >= maxSize) {
            frameQueue.pollLast();
        }
        frameQueue.addFirst(newBlock);
    }

    public boolean seesBall() {
        for (List<LLResultTypes.DetectorResult> detections : frameQueue) {

            if (detections == null || detections.isEmpty()) {
                continue;
            }
            return true;
        }
        return false;
    }

    public LLResultTypes.DetectorResult getClosest() {
        for (List<LLResultTypes.DetectorResult> frame : frameQueue) {
            if (frame == null || frame.isEmpty()) continue;

            LLResultTypes.DetectorResult best = null;
            double maxArea = -1.0;

            for (LLResultTypes.DetectorResult d : frame) {

                double area = d.getTargetArea(); // Limelight-provided area
                if (area > maxArea) {
                    maxArea = area;
                    best = d;
                }
            }

            if (best != null) {
                return best; // newest frame wins
            }
        }
        return null;
    }

    /**
     * Smoothed closest target (EMA filtered)
     */
    public SmoothedTarget getSmoothedClosest() {
        LLResultTypes.DetectorResult raw = getClosest();

        if (raw == null) {
            return null;
        }

        double x = raw.getTargetXDegrees();
        double y = raw.getTargetYDegrees();
        double area = raw.getTargetArea();

        if (smoothedTarget == null) {
            smoothedTarget = new SmoothedTarget(x, y, area);
        } else {
            smoothedTarget.xDeg =
                    ALPHA * x + (1.0 - ALPHA) * smoothedTarget.xDeg;
            smoothedTarget.yDeg =
                    ALPHA * y + (1.0 - ALPHA) * smoothedTarget.yDeg;
            smoothedTarget.area =
                    ALPHA * area + (1.0 - ALPHA) * smoothedTarget.area;
        }

        return smoothedTarget;
    }

    public CoordinateConverter getTargetCoordinates(SmoothedTarget t) {
        if (t == null) return null;

        double totalPitchRad =
                Math.toRadians(CAMERA_PITCH_DEG + t.yDeg);

        double z =
                (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(totalPitchRad);

        double x =
                z * Math.tan(Math.toRadians(t.xDeg));

        return new CoordinateConverter(x * 39.37, z * 39.37);
    }
}
/*List<DetectorResult> detections = result.getDetectorResults();
for (DetectorResult detection : detections) {
    String className = detection.getClassName(); // What was detected
    double x = detection.getTargetXDegrees(); // Where it is (left-right)
    double y = detection.getTargetYDegrees(); // Where it is (up-down)
    telemetry.addData(className, "at (" + x + ", " + y + ") degrees");
}*/



