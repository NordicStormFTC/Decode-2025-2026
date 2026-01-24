package org.firstinspires.ftc.teamcode.nordicStorm.Vision;

public class CoordinateConverter {
    public double x; // right inches
    public double y; // forward inches

    public CoordinateConverter(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public static double xConvertToField(double headingRadians, double yRobotOffset, double xRobotOffset) {
        return yRobotOffset * Math.cos(headingRadians) + xRobotOffset * Math.sin(headingRadians);
    }

    public static double yConvertToField(double headingRadians, double yRobotOffset, double xRobotOffset) {
        return yRobotOffset * Math.sin(headingRadians) - xRobotOffset * Math.cos(headingRadians);
    }
}

