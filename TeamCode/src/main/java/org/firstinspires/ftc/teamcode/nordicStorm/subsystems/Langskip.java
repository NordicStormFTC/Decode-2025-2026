package org.firstinspires.ftc.teamcode.nordicStorm.subsystems;

import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.NordicConstants.pixyCenterXPixel;
import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.NordicConstants.pixyName;
import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.NordicConstants.signalLightName;

import static java.lang.System.currentTimeMillis;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.nordicStorm.pixy.Pixy;
import org.firstinspires.ftc.teamcode.nordicStorm.pixy.PixyBlock;
import org.firstinspires.ftc.teamcode.nordicStorm.pixy.PixyHelper;


/**
 * The goal here is to have only one robot info class instantiated
 * in our OpModes. Langskip means 'boat' in old norse, and publicly
 * contains all of our subsystems. Users should not instantiate
 * individual subsystems in OpModes and access them through langskip.
 */
public class Langskip {

    public final Intake intake;
    public final DriveTrain driveTrain;
    public final InnerSubsystem innerSubsystem;


    private final Pixy pixy;
    private final Limelight3A limelight;
    private final Servo signalLight;
    private final PixyHelper pixyHelper = new PixyHelper(5);

    private final Pose afterHPIntake, beforeHPIntake;

    private final NordicConstants.AllianceColor allianceColor;

    private final Follower follower;

    public enum State {
        BALL_SEARCHING,
        BALL_SEEKING,
        BALL_CHARGING,
        HPINTAKE,
        AIMING,
        SHOOTING,
        AUTO_PARKING,
        IDLE
    }

    public State currentState = State.IDLE;

    private double chargeTime = Integer.MAX_VALUE;


    /**
     * @param hardwareMap the hardware map for our subsystems to use. This provides the same instance of the hardware map to all subsystems
     */
    public Langskip(@NonNull final HardwareMap hardwareMap, NordicConstants.AllianceColor allianceColor) {

        intake = new Intake(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap, allianceColor);
        innerSubsystem = new InnerSubsystem(hardwareMap);

        signalLight = hardwareMap.get(Servo.class, signalLightName);
        limelight = driveTrain.limelight;
        pixy = hardwareMap.get(Pixy.class, pixyName);
        pixy.turnOnLamps();
        follower = driveTrain.follower;

        this.allianceColor = allianceColor;

        beforeHPIntake = allianceColor == NordicConstants.AllianceColor.BLUE ? new Pose(106, 10, Math.toRadians(180)) : new Pose(38, 10, Math.toRadians(0));
        afterHPIntake = allianceColor == NordicConstants.AllianceColor.BLUE ? new Pose(132, 10, Math.toRadians(180)) : new Pose(12, 10, Math.toRadians(0));
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

        // ----- Retrieve and update Pixy information -----
        PixyBlock ball = pixy.getBlock();
        pixyHelper.update(ball);
        telemetry.addData("Pixy Reading", pixyHelper.getData());
        telemetry.addData("Light State: ", signalLight.getPosition());
        telemetry.addData("Follower Pose: ", follower.getPose());
        telemetry.addData("Is busy: ", follower.isBusy());

        if (limelight.getLatestResult() != null && limelight.getLatestResult().isValid()) {
            for (LLResultTypes.FiducialResult detection : limelight.getLatestResult().getFiducialResults()) {
                if (detection.getFiducialId() == 20 || detection.getFiducialId() == 24) {

                    LLResult llResult = limelight.getLatestResult();
                    Pose3D botPose = llResult.getBotpose();

                    Position position = botPose.getPosition();
                    double pedroY = position.x * NordicConstants.metersToInches * -1 + 72;
                    double pedroX = position.y * NordicConstants.metersToInches + 72;

                    Pose pedroPose = new Pose(pedroX, pedroY, follower.getHeading());

                    telemetry.addData("Limelight Pose Estimation: ", pedroPose);
                    continue;
                }
            }
        }


        //
        driveTrain.updatePosition(telemetry);
        /*if (pixyHelper.seesBall() && signalLight.getPosition() != .5) {
            signalLight.setPosition(.5);
        } else if (currentState == State.BALL_SEARCHING && signalLight.getPosition() != .625) {
            signalLight.setPosition(.625);
        } else if (signalLight.getPosition() != .25) {
            signalLight.setPosition(.25);
        } */
        if (driveTrain.llResultsAreGood() && signalLight.getPosition() != .5) {
            signalLight.setPosition(.5);
        } else if (signalLight.getPosition() != .25) {
            signalLight.setPosition(.25);
        }

        switch (currentState) {
            case BALL_SEARCHING:
                if (pixyHelper.seesBall()) {
                    currentState = State.BALL_SEEKING;
                } else {
                    Pose currentPose = follower.getPose();
                    Pose targetPose = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading() - Math.PI);
                    follower.holdPoint(new BezierPoint(targetPose), targetPose.getHeading());
                }
                break;
            case BALL_SEEKING:
                if (!pixyHelper.seesBall()) {
                    currentState = State.BALL_SEARCHING;
                } else {
                    intake.runIntake(true);
                    double xPixelOffset = pixyHelper.getWeightedX() - pixyCenterXPixel;
                    Pose currentPose = follower.getPose();
                    double distance = 1265 / Math.sqrt(pixyHelper.getWeightedArea());
                    telemetry.addData("X Offset: ", xPixelOffset);
                    telemetry.addData("Calculated minimum offset: ", 40 / Math.sqrt(distance));

                    if (xPixelOffset < -20 / Math.sqrt(distance)) {
                        follower.holdPoint(new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading() + .5));

                    } else if (xPixelOffset > 20 / Math.sqrt(distance)) {
                        follower.holdPoint(new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading() - .5));

                    } else {
                        currentState = State.BALL_CHARGING;
                        signalLight.setPosition(0);
                        chargeTime = currentTimeMillis();
                    }
                }
                break;
            case BALL_CHARGING:
                if (currentTimeMillis() - chargeTime > 300) {
                    chargeTime = Integer.MAX_VALUE;
                    currentState = State.BALL_SEARCHING;
                } else {
                    if (pixyHelper.seesBall()) {
                        chargeTime = currentTimeMillis();
                    }
                    Pose currentPose = follower.getPose();
                    double ballAngle = currentPose.getHeading() + Math.PI;
                    double distance = 400 / Math.sqrt(pixyHelper.getMaxArea());


                    BezierPoint finalPose = new BezierPoint(new Pose(currentPose.getX() - Math.cos(ballAngle) * (-distance), currentPose.getY() - Math.sin(ballAngle) * (-distance), currentPose.getHeading()));
                    follower.holdPoint(finalPose, currentPose.getHeading(), false);
                }

                break;
            case HPINTAKE:
                if (!follower.isBusy()) {
                    PathChain HPIntake = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), beforeHPIntake))
                            .setLinearHeadingInterpolation(follower.getHeading(), beforeHPIntake.getHeading())
                            .addParametricCallback(.9, () -> follower.setMaxPower(.29))
                            .addPath(new BezierLine(beforeHPIntake, afterHPIntake))
                            .setLinearHeadingInterpolation(beforeHPIntake.getHeading(), afterHPIntake.getHeading())
                            .addParametricCallback(.01, () -> intake.runIntake(true))
                            .addParametricCallback(.99, () -> intake.runIntake(false))
                            .addParametricCallback(.99, () -> follower.setMaxPower(1))
                            .build();
                    follower.followPath(HPIntake);
                }
                break;
            case AIMING:
                double angleToGoal = driveTrain.getAngleToGoal(follower.getPose());
                telemetry.addData("Angle to goal: ", angleToGoal);
                telemetry.addData("Angle to goal degrees: ", Math.toDegrees(angleToGoal));
                follower.turnTo(angleToGoal);
                if (Math.abs(follower.getHeading()-angleToGoal) < .1 || (Math.abs(follower.getHeading()) + Math.abs(angleToGoal) - 2*Math.PI < .05) && Math.abs(follower.getHeading()) + Math.abs(angleToGoal) - 2 * Math.PI > 0) {
                    currentState = State.SHOOTING;
                }
                break;
            case SHOOTING:
                intake.runIntake(true);
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