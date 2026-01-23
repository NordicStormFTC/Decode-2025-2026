package org.firstinspires.ftc.teamcode.nordicStorm.subsystems;

import static org.firstinspires.ftc.teamcode.nordicStorm.NordicConstants.signalLightName;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.nordicStorm.NordicConstants;
import org.firstinspires.ftc.teamcode.nordicStorm.Vision.CoordinateConverter;
import org.firstinspires.ftc.teamcode.nordicStorm.Vision.SmoothedTarget;
import org.firstinspires.ftc.teamcode.nordicStorm.Vision.VisionHelper;

import java.util.List;


/**
 * The goal here is to have only one robot info class instantiated
 * in our OpModes. Langskip means 'boat' in old norse, and publicly
 * contains all of our subsystems. Users should not instantiate
 * individual subsystems in OpModes and access them through langskip.
 */
public class Langskip {

    public final DriveTrain driveTrain;
    public final InnerSubsystem innerSubsystem;
    public final Intake intake;


    public final Limelight3A limelight;
    private final VisionHelper visionHelper;

    public final Servo signalLight;

    private final Pose afterHPIntake, beforeHPIntake;

    private final NordicConstants.AllianceColor allianceColor;

    private final Follower follower;

    public enum State {
        HPINTAKE,
        AUTO_INTAKE,
        AIMING,
        SHOOTING,
        AUTO_PARKING,
        IDLE
    }

    public State currentState = State.IDLE;
    private final Pose shootingPose;
    private final boolean isTeleop;


    /**
     * @param hardwareMap the hardware map for our subsystems to use. This provides the same instance of the hardware map to all subsystems
     */
    public Langskip(final HardwareMap hardwareMap, NordicConstants.AllianceColor allianceColor, boolean isTeleop) {
        driveTrain = new DriveTrain(hardwareMap, allianceColor);
        innerSubsystem = new InnerSubsystem(hardwareMap);
        intake = new Intake(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, NordicConstants.limelightName);
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(1);
        visionHelper = new VisionHelper(5);


        this.isTeleop = isTeleop;

        signalLight = hardwareMap.get(Servo.class, signalLightName);
        follower = driveTrain.follower;

        this.allianceColor = allianceColor;
        shootingPose = allianceColor == NordicConstants.AllianceColor.RED ? NordicConstants.redGoalPose : NordicConstants.blueGoalPose;

        beforeHPIntake = allianceColor == NordicConstants.AllianceColor.BLUE ? new Pose(106, 12, Math.toRadians(0)) : new Pose(38, 12, Math.toRadians(180));
        afterHPIntake = allianceColor == NordicConstants.AllianceColor.BLUE ? new Pose(132, 12, Math.toRadians(180)) : new Pose(12, 12, Math.toRadians(0));
    }

    public Follower getFollower() {
        return follower;
    }

    public void changeState(State newState) {
        currentState = newState;
    }

    public void setSignalColor(double color) {
        signalLight.setPosition(color);
    }

    private boolean llResultsAreGood() {
        LLResult recent = limelight.getLatestResult();
        return recent != null && recent.isValid() && recent.getStaleness() < 100;
    }

    public void periodic(Telemetry telemetry) {
        double shootDistance = Math.sqrt(Math.pow(follower.getPose().getX() - shootingPose.getX(), 2) + Math.pow(follower.getPose().getY() - shootingPose.getY(), 2));
        innerSubsystem.periodic(telemetry, shootDistance);
        if (llResultsAreGood()) {
            visionHelper.update(limelight.getLatestResult().getDetectorResults());
        }

        //if (innerSubsystem.getDistance() < 25) {
        if (visionHelper.seesBall()) {
            setSignalColor(.72);
        } else {
            if (allianceColor == NordicConstants.AllianceColor.BLUE) {
                setSignalColor(.615);
            } else {
                setSignalColor(.283);
            }
        }

        telemetry.addData("Shooting distance: ", shootDistance);
        telemetry.addData("Follower Pose: ", follower.getPose());

        switch (currentState) {
            case HPINTAKE:
                if (!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(beforeHPIntake.getX(), beforeHPIntake.getY()), beforeHPIntake.getHeading());
                    /*PathChain HPIntake = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), beforeHPIntake))
                            .setLinearHeadingInterpolation(follower.getHeading(), beforeHPIntake.getHeading(), .75)
                            .addParametricCallback(.9, () -> follower.setMaxPower(.29))
                            .addPath(new BezierLine(beforeHPIntake, afterHPIntake))
                            .addParametricCallback(.01, () -> intake.runIntake(true))
                            .addParametricCallback(.99, () -> intake.runIntake(false))
                            .addParametricCallback(.99, () -> follower.setMaxPower(1))
                            .setLinearHeadingInterpolation(beforeHPIntake.getHeading(), afterHPIntake.getHeading(), .75)
                            .build();
                    follower.followPath(HPIntake); */
                }
                break;
            case AUTO_INTAKE:
                if (visionHelper.seesBall()) {
                    CoordinateConverter target = visionHelper.getTargetCoordinates(visionHelper.getSmoothedClosest());
                    double xRobotOffset = target.x;
                    double yRobotOffset = target.y + NordicConstants.limelightForwardOffset;
                    telemetry.addData("Limelight left/right: ", target.x);
                    telemetry.addData("Limelight forward/backwards: ", target.y);
                }
                break;
            case AIMING:
                double angleToGoal = driveTrain.getAngleToGoal();
                telemetry.addData("Angle to goal degrees: ", Math.toDegrees(angleToGoal));
                telemetry.addData("Heading: ", Math.toDegrees(follower.getHeading()));

                follower.turnTo(angleToGoal);
                if (this.isTeleop) {
                    innerSubsystem.setShooting(true);
                }

                if (Math.abs(follower.getHeading() - angleToGoal) < .035 || (Math.abs(follower.getHeading()) + Math.abs(angleToGoal) - 2 * Math.PI < .035) && Math.abs(follower.getHeading()) + Math.abs(angleToGoal) - 2 * Math.PI > 0) {
                    currentState = State.SHOOTING;
                    if (this.isTeleop) {
                        innerSubsystem.setAtPoint(true);
                        follower.holdPoint(follower.getPose());
                    }
                }

                break;
            case SHOOTING:
                this.innerSubsystem.setShooting(true);
                break;
            case AUTO_PARKING:
                if (allianceColor == NordicConstants.AllianceColor.BLUE) {
                    follower.holdPoint(new Pose(105.25, 33.1, Math.toRadians(90)));
                } else {
                    follower.holdPoint(new Pose(38.75, 33.1, Math.toRadians(90)));
                }
                break;
            case IDLE:
        }
    }
}