package org.firstinspires.ftc.teamcode.nordicStorm.subsystems;

import com.pedropathing.geometry.Pose;

public class NordicConstants {

    public static final double metersToInches = 39.3701;

    // All names for the non-drivetrain hardware.
    public static final String intakeMotorName = "intake";
    public static final String shootingMotorName = "shiitee";
    public static final String rightElevatorName = "right lift";
    public static final String leftElevatorName = "left lift";
    public static final String signalLightName = "light";
    public static final int pixyCenterXPixel = 125; //158

    // Physical location of each goal Position
    public static final Pose redGoalPose = new Pose(139, 139, 0);
    public static final Pose blueGoalPose = new Pose(5, 139, 0);

    public enum AllianceColor {
        RED,
        BLUE
    }
}
