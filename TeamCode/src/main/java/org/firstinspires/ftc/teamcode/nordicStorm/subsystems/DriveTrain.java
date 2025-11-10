package org.firstinspires.ftc.teamcode.nordicStorm.subsystems;



import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

public class DriveTrain extends SubsystemBase {


    public final Follower follower;

    public final Limelight3A limelight;

    private LLResult llResult;
    private final GoBildaPinpointDriver pinpoint;

    private Pose shootingPose;

    private TelemetryManager telemetryM;

    private NordicConstants.AllianceColor allianceColor;

    public DriveTrain(final HardwareMap hardwareMap, NordicConstants.AllianceColor allianceColor) {
        follower = Constants.createFollower(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        this.allianceColor = allianceColor;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        limelight.start();
        shootingPose = allianceColor == NordicConstants.AllianceColor.RED ? NordicConstants.redGoalPose : NordicConstants.blueGoalPose;

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }


    public void updatePosition(final Telemetry telemetry) {
        limelight.updateRobotOrientation(pinpoint.getHeading(AngleUnit.DEGREES));
    }

    public double getAngleToGoal(Pose startPose) {
        Pose position = follower.getPose();
        double angle = Math.atan((shootingPose.getX() - position.getX())/(shootingPose.getY() - position.getY()));
        return (Math.PI/2 - angle) + Math.PI;
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
