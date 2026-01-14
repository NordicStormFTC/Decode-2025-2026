package org.firstinspires.ftc.teamcode.nordicStorm.subsystems;

import com.pedropathing.geometry.Pose;

public class NordicConstants {
    // All names for the non-drivetrain hardware.
    public static final String shootingMotorName = "shiitee";
    public static final String intakeMotorName = "intake";
    public static final String rightElevatorName = "right lift";
    public static final String leftElevatorName = "left lift";
    public static final String signalLightName = "light";

    // Physical location of each goal Position
    public static final Pose redGoalPose = new Pose(137, 137, 0);
    public static final Pose blueGoalPose = new Pose(7, 137, 0);

    public enum AllianceColor {
        RED,
        BLUE
    }
}
