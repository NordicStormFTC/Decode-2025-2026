package org.firstinspires.ftc.teamcode.nordicStorm.subsystems;

import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.NordicConstants.pixyCenterXPixel;
import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.NordicConstants.signalLightName;

import static java.lang.System.currentTimeMillis;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.nordicStorm.pixy.PixyHelper;


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


    public final Servo signalLight;
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

    private final Pose shootingPose;


    /**
     * @param hardwareMap the hardware map for our subsystems to use. This provides the same instance of the hardware map to all subsystems
     */
    public Langskip(@NonNull final HardwareMap hardwareMap, NordicConstants.AllianceColor allianceColor) {
        driveTrain = new DriveTrain(hardwareMap, allianceColor);
        innerSubsystem = new InnerSubsystem(hardwareMap);
        intake = new Intake(hardwareMap);

        signalLight = hardwareMap.get(Servo.class, signalLightName);
        setSignalColor(.333);
        follower = driveTrain.follower;

        this.allianceColor = allianceColor;
        shootingPose = allianceColor == NordicConstants.AllianceColor.RED ? NordicConstants.redGoalPose : NordicConstants.blueGoalPose;


        beforeHPIntake = allianceColor == NordicConstants.AllianceColor.BLUE ? new Pose(106, 12, Math.toRadians(180)) : new Pose(38, 12, Math.toRadians(0));
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

    public void periodic(Telemetry telemetry) {
        double shootDistance = Math.sqrt(Math.pow(follower.getPose().getX() - shootingPose.getX(), 2) + Math.pow(follower.getPose().getY() - shootingPose.getY(), 2));
        telemetry.addData("Shooting distance: ", shootDistance);

        innerSubsystem.periodic(telemetry, shootDistance);
        //innerSubsystem.periodic(telemetry, sRPM);


        // ----- Retrieve and update Robot information -----

        telemetry.addData("Is busy: ", follower.isBusy());
        telemetry.addData("Follower Pose: ", follower.getPose());
        telemetry.addData("Signal Light: ", signalLight.getPosition());

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
            case AIMING:
                double angleToGoal = driveTrain.getAngleToGoal();
                telemetry.addData("Angle to goal degrees: ", Math.toDegrees(angleToGoal));
                telemetry.addData("Heading: ", Math.toDegrees(follower.getHeading()));

                follower.turnTo(angleToGoal);

                if (Math.abs(follower.getHeading() - angleToGoal) < .1 || (Math.abs(follower.getHeading()) + Math.abs(angleToGoal) - 2 * Math.PI < .05) && Math.abs(follower.getHeading()) + Math.abs(angleToGoal) - 2 * Math.PI > 0) {
                    currentState = State.SHOOTING;
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