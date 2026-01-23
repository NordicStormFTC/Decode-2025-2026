package org.firstinspires.ftc.teamcode.nordicStorm.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.nordicStorm.NordicConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
// 32 47 140 32 137 21 334 178

/**
 * Drivetrain Subsystem.
 * Most drivetrain functions are handled naturally by PP
 */
public class DriveTrain extends SubsystemBase {

    public final Follower follower;
    private final Pose shootingPose;

    public DriveTrain(final HardwareMap hardwareMap, NordicConstants.AllianceColor allianceColor) {
        follower = Constants.createFollower(hardwareMap);
        shootingPose = allianceColor == NordicConstants.AllianceColor.RED ? NordicConstants.redGoalPose : NordicConstants.blueGoalPose;
    }

    /**
     * Find the angle the robot needs to face the goal.
     */
    public double getAngleToGoal() {
        Pose position = follower.getPose();
        double angle = Math.atan((shootingPose.getX() - position.getX()) / (shootingPose.getY() - position.getY()));
        return (Math.PI / 2 - angle);
    }
}
