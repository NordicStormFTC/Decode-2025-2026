package org.firstinspires.ftc.teamcode.nordicStorm.Vision;

import static org.firstinspires.ftc.teamcode.nordicStorm.NordicConstants.CAMERA_HEIGHT;
import static org.firstinspires.ftc.teamcode.nordicStorm.NordicConstants.CAMERA_PITCH_DEG;
import static org.firstinspires.ftc.teamcode.nordicStorm.NordicConstants.TARGET_HEIGHT;

import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.Deque;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;

public class VisionHelper {

    private final Deque<List<LLResultTypes.DetectorResult>> frameQueue;
    private final int maxSize; // Size of the queue
    private SmoothedTarget smoothedTarget = null;
    private static final double ALPHA = 0.35; // Constant for frame smoothing

    public VisionHelper(int size) {
        frameQueue = new LinkedList<>();
        this.maxSize = size;
    }

    /**
     * Add newBlock to the front of the queue, remove the back element from the queue.
     */
    public void update(List<LLResultTypes.DetectorResult> newBlock) {
        if (frameQueue.size() >= maxSize) {
            frameQueue.pollLast();
        }
        frameQueue.addFirst(newBlock);
    }

    /**
     * Iterate through all held frames to check if we have seen an Artifact.
     */
    public boolean seesBall() {
        for (List<LLResultTypes.DetectorResult> detections : frameQueue) {

            if (detections == null || detections.isEmpty()) {
                continue;
            }
            return true;
        }
        return false;
    }

    public double getClosestColor() {
        return Objects.equals(getClosest().getClassName(), "purple") ? .722 : .5;
    }

    /**
     * If we have seen an artifact in the last few frames,
     * return the LLResult data on that object.
     */
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
                return best; // newest frame is the best for tracking
            }
        }
        return null;
    }

    /**
     * Smoothed closest target (EMA filtered)
     * Takes the most recently seen Artifact and
     * smooths it with the 2nd most recently seen Artifact
     * EMA = a * x + (1 - a) * previous x
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

    /**
     * Convert the xDgree, yDegree, area data
     * from the parameter into x, y coordinate distances
     * from the limelight. (inches)
     */
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