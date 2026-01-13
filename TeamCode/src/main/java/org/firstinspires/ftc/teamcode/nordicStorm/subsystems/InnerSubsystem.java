package org.firstinspires.ftc.teamcode.nordicStorm.subsystems;

import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Globals.shooterP;
import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Globals.shooterI;
import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Globals.shooterD;
import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Globals.shooterFeedForwards;
import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.NordicConstants.leftElevatorName;
import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.NordicConstants.rightElevatorName;
import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.NordicConstants.shootingMotorName;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class InnerSubsystem {
    private final DcMotorEx shootingMotor;
    public final Servo leftElevator, rightElevator;
    private final DistanceSensor proximity1;
    private final DistanceSensor proximity2;
    private boolean shooting = false;
    private boolean override = false;
    private final Timer atRPMsince = new Timer();
    private final Timer timeSinceShot = new Timer();
    private final Timer shootingSince = new Timer();

    private boolean flipperIsDown = true;
    private int intakeSpeed = 2500;


    public InnerSubsystem(final HardwareMap hardwareMap) {
        shootingMotor = hardwareMap.get(DcMotorEx.class, shootingMotorName);
        shootingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootingMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(shooterP, shooterI, shooterD, shooterFeedForwards));

        proximity1 = hardwareMap.get(DistanceSensor.class, "proximity1");
        proximity2 = hardwareMap.get(DistanceSensor.class, "proximity2");


        rightElevator = hardwareMap.get(Servo.class, rightElevatorName);
        leftElevator = hardwareMap.get(Servo.class, leftElevatorName);
        rightElevator.setPosition(.02);
        leftElevator.setPosition(.66);
    }

    public void periodic(Telemetry telemetry, double distance) {
        double targetRPM;
        if (shooting) {
            //targetRPM = findRPMFromDistance(distance);
            targetRPM = intakeSpeed;
        } else {
            shootingSince.resetTimer();
            targetRPM = 0;
        }

        setRPM(targetRPM);
        telemetry.addData("RPM: ", getRPM());
        telemetry.addData("Ball Distance: ", getDistance());


        if (shooting && Math.abs(targetRPM - getRPM()) < 100) {
            if (flipperIsDown && waitedSinceLastShot() && getDistance() < 70 && nonOscillatingRPM() && rampingTimeIsGood()) {
                moveFlipperUp();
                timeSinceShot.resetTimer();
            }
        } else {
            atRPMsince.resetTimer();
        }

        if (timeSinceShot.getElapsedTimeSeconds() > .65 && !flipperIsDown) {
            if (!override) {
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

    public void moveFlipperDown() {
        flipperIsDown = true;
        rightElevator.setPosition(.02);
        leftElevator.setPosition(.66);
    }

    public void moveFlipperUp() {
        flipperIsDown = false;
        rightElevator.setPosition(.52);
        leftElevator.setPosition(.16);
    }

    public double findRPMFromDistance(double distance) {
        return (-0.002267 * distance * distance * distance + 0.6989 * distance * distance - 61.34 * distance + 6341.27) - 100;
    }

    public void setShooting(boolean doShoot) {
        shooting = doShoot;
    }

    public double getDistance() {
        double dist1 = Double.isNaN(proximity1.getDistance(DistanceUnit.MM)) ? 10000 : proximity1.getDistance(DistanceUnit.MM);
        double dist2 = Double.isNaN(proximity2.getDistance(DistanceUnit.MM)) ? 10000 : proximity2.getDistance(DistanceUnit.MM);
        return Math.min(dist1, dist2);
    }
}
