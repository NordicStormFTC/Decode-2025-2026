package org.firstinspires.ftc.teamcode.nordicStorm.subsystems;

import static org.firstinspires.ftc.teamcode.nordicStorm.Globals.shooterP;
import static org.firstinspires.ftc.teamcode.nordicStorm.Globals.shooterI;
import static org.firstinspires.ftc.teamcode.nordicStorm.Globals.shooterD;
import static org.firstinspires.ftc.teamcode.nordicStorm.Globals.shooterFeedForwards;
import static org.firstinspires.ftc.teamcode.nordicStorm.NordicConstants.leftElevatorName;
import static org.firstinspires.ftc.teamcode.nordicStorm.NordicConstants.rightElevatorName;
import static org.firstinspires.ftc.teamcode.nordicStorm.NordicConstants.shootingMotorName;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class InnerSubsystem {
    private final DcMotorEx shootingMotor;
    public final Servo leftElevator, rightElevator;
    private final RevColorSensorV3 proximity1;
    private final RevColorSensorV3 proximity2;
    private boolean shooting = false;
    private boolean override = false;
    private boolean atPoint = true;
    private final Timer atRPMsince = new Timer();
    private final Timer timeSinceShot = new Timer();
    private final Timer shootingSince = new Timer();

    private boolean flipperIsDown = true;
    private int intakeSpeed = 2500;


    public InnerSubsystem(final HardwareMap hardwareMap) {
        shootingMotor = hardwareMap.get(DcMotorEx.class, shootingMotorName);
        shootingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootingMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(shooterP, shooterI, shooterD, shooterFeedForwards));

        proximity1 = hardwareMap.get(RevColorSensorV3.class, "proximity1");
        proximity2 = hardwareMap.get(RevColorSensorV3.class, "proximity2");


        rightElevator = hardwareMap.get(Servo.class, rightElevatorName);
        leftElevator = hardwareMap.get(Servo.class, leftElevatorName);
        rightElevator.setPosition(.02);
        leftElevator.setPosition(.66);
    }

    public void periodic(Telemetry telemetry, double distance) {
        double targetRPM;
        if (shooting) {
            targetRPM = findRPMFromDistance(distance);
            //targetRPM = intakeSpeed;
        } else {
            shootingSince.resetTimer();
            targetRPM = 0;
            shootingMotor.setPower(0);
        }

        setRPM(targetRPM);
        telemetry.addData("RPM: ", getRPM());
        telemetry.addData("Target RPM: ", targetRPM);


        if (shooting && atPoint && Math.abs(targetRPM - getRPM()) < 80) {
            if (flipperIsDown && waitedSinceLastShot() && getDistance() < 22 && nonOscillatingRPM() && rampingTimeIsGood()) {
                moveFlipperUp();
                timeSinceShot.resetTimer();
            }
        } else {
            atRPMsince.resetTimer();
        }

        if (!flipperIsDown && !override) {
            if (timeSinceShot.getElapsedTimeSeconds() > .5) {
                rightElevator.setPosition(.02);
            }
            if (timeSinceShot.getElapsedTimeSeconds() > .65) {
                moveFlipperDown();
            }
        }
    }

    private boolean rampingTimeIsGood() {
        return shootingSince.getElapsedTimeSeconds() > 1;
    }

    private boolean nonOscillatingRPM() {
        return atRPMsince.getElapsedTimeSeconds() > .25;
    }

    private boolean waitedSinceLastShot() {
        return timeSinceShot.getElapsedTimeSeconds() > .6;
    }

    public void setRPM(double RPM) {
        shootingMotor.setVelocity(RPM * 28 / 60);
    }

    public double getRPM() {
        return shootingMotor.getVelocity() / 28 * 60;
    }

    public void setOverride(boolean b) {
        override = b;
    }

    public void setIntakeSpped(int speed) {
        intakeSpeed = speed;
    }

    public void setAtPoint(boolean set) {
        this.atPoint = set;
    }

    public void moveFlipperDown() {
        flipperIsDown = true;
        rightElevator.setPosition(.02);
        leftElevator.setPosition(.66);
    }

    public void moveFlipperUp() {
        flipperIsDown = false;
        rightElevator.setPosition(.48); //.48
        leftElevator.setPosition(.1);
    }

    public double findRPMFromDistance(double distance) {
        return 10.4 * distance + 1924;
    }

    public void setShooting(boolean doShoot) {
        shooting = doShoot;
    }

    public double getDistance() {
        double dist1 = proximity1.getDistance(DistanceUnit.MM);
        double dist2 = proximity2.getDistance(DistanceUnit.MM);
        if (Double.isNaN(dist1)) {
            dist1 = 10000;
        } else if (Double.isNaN(dist2)) {
            dist2 = 10000;
        }
        return Math.min(dist1, dist2);
    }
}
