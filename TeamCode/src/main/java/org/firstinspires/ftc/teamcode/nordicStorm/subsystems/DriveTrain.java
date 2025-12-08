package org.firstinspires.ftc.teamcode.nordicStorm.subsystems;



import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Drivetrain Subsystem.
 * Most drivetrain functions are handled naturally by PP
 * This subsystem initializes the PP follower and the limelight.
 * It also contains helper methods related to the drivetrain.
 */
public class DriveTrain extends SubsystemBase {


    public final Follower follower;

    public final Limelight3A limelight;

    private final GoBildaPinpointDriver pinpoint;

    private final Pose shootingPose;



    public DriveTrain(final HardwareMap hardwareMap, NordicConstants.AllianceColor allianceColor) {
        follower = Constants.createFollower(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        limelight.start();
        shootingPose = allianceColor == NordicConstants.AllianceColor.RED ? NordicConstants.redGoalPose : NordicConstants.blueGoalPose;
    }

    public void updatePosition() {
        limelight.updateRobotOrientation(pinpoint.getHeading(AngleUnit.DEGREES));
    }

    /**
     * Find the angle the robot needs to face the goal.
     */
    public double getAngleToGoal() {
        Pose position = follower.getPose();
        double angle = Math.atan((shootingPose.getX() - position.getX())/(shootingPose.getY() - position.getY()));
        return (Math.PI/2 - angle);
    }


    private double metersToInches(double meters) {
        return meters * NordicConstants.metersToInches;
    }

    public LLResult getLLResult() {
        return limelight.getLatestResult();
    }

    public boolean llResultsAreGood() {
        return getLLResult() != null && getLLResult().isValid() && getLLResult().getStaleness() < 100;
    }
}
