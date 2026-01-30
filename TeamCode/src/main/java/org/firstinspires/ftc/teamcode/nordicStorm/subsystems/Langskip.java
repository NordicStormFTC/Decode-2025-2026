package org.firstinspires.ftc.teamcode.nordicStorm.subsystems;

import static org.firstinspires.ftc.teamcode.nordicStorm.NordicConstants.signalLightName;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.nordicStorm.Globals;
import org.firstinspires.ftc.teamcode.nordicStorm.NordicConstants;
import org.firstinspires.ftc.teamcode.nordicStorm.Util;
import org.firstinspires.ftc.teamcode.nordicStorm.Vision.CoordinateConverter;
import org.firstinspires.ftc.teamcode.nordicStorm.Vision.SmoothedTarget;
import org.firstinspires.ftc.teamcode.nordicStorm.Vision.VisionHelper;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

import java.util.Arrays;
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
    private Timer chargeTimer = new Timer();

    private final Pose afterHPIntake, beforeHPIntake, gatePose;

    private final NordicConstants.AllianceColor allianceColor;

    private final Follower follower;

    private final PIDFController rotationPIDController = new PIDFController(new CustomPIDFCoefficients(Globals.rotationalP, 0, Globals.rotationalD, 0));

    private Path IntakePath;

    public enum State {
        HPINTAKE,
        AUTO_INTAKE,
        CHARGING,
        AIMING,
        SHOOTING,
        AUTO_PARKING,
        OPEN_GATE,
        IDLE
    }

    public State currentState = State.IDLE;
    private final Pose shootingPose;
    private final boolean isTeleop;

    public boolean a = false;


    /**
     * @param hardwareMap the hardware map for our subsystems to use. This provides the same instance of the hardware map to all subsystems
     */
    public Langskip(final HardwareMap hardwareMap, NordicConstants.AllianceColor allianceColor, boolean isTeleop) {
        driveTrain = new DriveTrain(hardwareMap, allianceColor);
        innerSubsystem = new InnerSubsystem(hardwareMap);
        intake = new Intake(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, NordicConstants.limelightName);
        limelight.setPollRateHz(90);
        limelight.pipelineSwitch(0);
        limelight.start();

        visionHelper = new VisionHelper(5);

        this.isTeleop = isTeleop;

        signalLight = hardwareMap.get(Servo.class, signalLightName);
        setSignalColor(.388);
        follower = driveTrain.follower;

        this.allianceColor = allianceColor;
        shootingPose = allianceColor == NordicConstants.AllianceColor.RED ? NordicConstants.redGoalPose : NordicConstants.blueGoalPose;

        gatePose = allianceColor == NordicConstants.AllianceColor.RED ? new Pose(126.4, 69, Math.toRadians(180)) : new Pose(18.3, 69, Math.toRadians(0));

        beforeHPIntake = allianceColor == NordicConstants.AllianceColor.BLUE ? new Pose(106, 12, Math.toRadians(0)) : new Pose(38, 12, Math.toRadians(180));
        afterHPIntake = allianceColor == NordicConstants.AllianceColor.BLUE ? new Pose(132, 12, Math.toRadians(0)) : new Pose(12, 12, Math.toRadians(180));
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

    public void periodic(Telemetry telemetry) {
        double shootDistance = Math.sqrt(Math.pow(follower.getPose().getX() - shootingPose.getX(), 2) + Math.pow(follower.getPose().getY() - shootingPose.getY(), 2));
        innerSubsystem.periodic(telemetry, shootDistance);
        visionHelper.update(limelight.getLatestResult().getDetectorResults(), follower);

        if (currentState == State.AIMING || currentState == State.SHOOTING) {
            if (innerSubsystem.getDistance() < 22) {
                setSignalColor(.388);
            }
        } else if (visionHelper.seesBall()) {
            setSignalColor(.722);
        } else {
            if (allianceColor == NordicConstants.AllianceColor.BLUE) {
                setSignalColor(.615);
            } else {
                setSignalColor(.283);
            }
        }

        telemetry.addData("Langskip state:", this.currentState);
        telemetry.addData("Follower Pose: ", follower.getPose());

        switch (currentState) {
            case HPINTAKE:
                intake.runIntake(true);
                if (!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(afterHPIntake.getX(), afterHPIntake.getY()), afterHPIntake.getHeading());
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
                    CoordinateConverter ballFromLimelight = visionHelper.getTargetCoordinates(visionHelper.getSmoothedClosest());
                    double xError = Util.clamp(ballFromLimelight.x / 23, -1, 1);
                    double forward = Util.clamp((1 - Math.abs(xError) * Globals.forwardScale) * .5, 0, .3);
                    double rotation = Util.clamp(-xError * Globals.rotationP, -.25, .25);
                    double strafe = -xError * Globals.strafeP;
                    follower.setTeleOpDrive(forward, strafe, rotation, true);
                    intake.runIntake(true);
                    if (ballFromLimelight.y < 8.5 && false) {
                        currentState = State.CHARGING;
                        chargeTimer.resetTimer();
                        intake.turnOffLight();
                    }
                } else {
                    follower.setTeleOpDrive(0, 0, 0, true);
                }
                break;
            case CHARGING:
                follower.setTeleOpDrive(.3, 0, 0, true);
                if (chargeTimer.getElapsedTimeSeconds() > .3) {
                    currentState = State.AUTO_INTAKE;
                }
                break;
            case AIMING:
                intake.runIntakeSlow();
                double angleToGoal = driveTrain.getAngleToGoal();
                telemetry.addData("Angle to goal degrees: ", Math.toDegrees(angleToGoal));
                telemetry.addData("Heading: ", Math.toDegrees(follower.getHeading()));
                //follower.turnTo(angleToGoal);
                double rotationalError = Util.angleDiff(follower.getHeading(), angleToGoal);
                if (this.isTeleop || true) {
                    innerSubsystem.setShooting(true);
                }
                rotationPIDController.updateError(rotationalError);
                double rotation = rotationPIDController.runPIDF();
                if (Math.abs(rotation) < .1 && Math.abs(rotationalError) > .005) {
                    rotation = .1 * Math.signum(rotation);
                }

                follower.setTeleOpDrive(0, 0, -rotation, true);
                telemetry.addData("Angle diff: ", Util.angleDiff(follower.getHeading(), angleToGoal));
                if (Math.abs(Util.angleDiff(follower.getHeading(), angleToGoal)) < .035) {
                    innerSubsystem.setAtPoint(true);

                    currentState = State.SHOOTING;
                    if (this.isTeleop) {
                        follower.holdPoint(follower.getPose());
                    } else {
                        follower.setTeleOpDrive(0, 0, 0, true);
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
            case OPEN_GATE:
                if (!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(gatePose.getX(), gatePose.getY()), gatePose.getHeading());
                }

            case IDLE:
        }
    }
}