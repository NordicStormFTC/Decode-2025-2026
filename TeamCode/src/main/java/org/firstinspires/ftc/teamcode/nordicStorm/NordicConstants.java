package org.firstinspires.ftc.teamcode.nordicStorm;

import com.pedropathing.geometry.Pose;

public class NordicConstants {
    // All names for the non-drivetrain hardware.
    public static final String shootingMotorName = "shiitee";
    public static final String intakeMotorName = "intake";
    public static final String rightElevatorName = "right lift";
    public static final String leftElevatorName = "left lift";
    public static final String signalLightName = "light";
    public static final String limelightName = "limelight";

    public static final double CAMERA_PITCH_DEG = -25.5;
    public static final double TARGET_HEIGHT = 0.0635;
    public static final double CAMERA_HEIGHT = .267;

    public static final double limelightForwardOffset = 7;


    // Physical location of each goal Position
    public static final Pose redGoalPose = new Pose(137, 140, 0);
    public static final Pose blueGoalPose = new Pose(7, 140, 0);

    public enum AllianceColor {
        RED,
        BLUE
    }
}
