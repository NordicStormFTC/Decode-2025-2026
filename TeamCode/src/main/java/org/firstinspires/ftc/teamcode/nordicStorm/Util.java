package org.firstinspires.ftc.teamcode.nordicStorm;

public class Util {
    public static double clamp(double x, double min, double max) {
        if (x < min) {
            return min;
        } else if (x > max) {
            return max;
        } else {
            return x;
        }
    }

    /**
     * @param angle1 Source angle
     * @param angle2 Target angle
     */
    public static double angleDiff(double angle1, double angle2) {
        return Math.atan2(Math.sin(angle1 - angle2), Math.cos(angle1 - angle2));
    }
}
