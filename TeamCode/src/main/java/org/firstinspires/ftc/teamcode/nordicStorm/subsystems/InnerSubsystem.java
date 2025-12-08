package org.firstinspires.ftc.teamcode.nordicStorm.subsystems;

import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Globals.shooterD;
import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Globals.shooterFeedForwards;
import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Globals.shooterI;
import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.NordicConstants.flipperServoName;
import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.NordicConstants.shootingMotorName;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class InnerSubsystem {
    @IgnoreConfigurable
    private final DcMotorEx shootingMotor;
    private final Servo flipperServo;
    private final Rev2mDistanceSensor distance1, distance2;
    private boolean shooting = false;
    private final PIDFController shooterPID;
    private boolean override = false;
    private final Timer atRPMsince = new Timer();
    private final Timer timeSinceShot = new Timer();
    private final Timer shootingSince = new Timer();


    public InnerSubsystem(final HardwareMap hardwareMap) {
        shootingMotor = hardwareMap.get(DcMotorEx.class, shootingMotorName);
        shootingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        distance1 = hardwareMap.get(Rev2mDistanceSensor.class, "d1");
        distance2 = hardwareMap.get(Rev2mDistanceSensor.class, "d2");


        flipperServo = hardwareMap.get(Servo.class, flipperServoName);
        flipperServo.setPosition(.35);
        shooterPID = new PIDFController(new PIDFCoefficients(Globals.shooterP, shooterI, shooterD, shooterFeedForwards));
    }

    public void periodic(Telemetry telemetry, double distance) {
        double ballDistance = getDistance();
        double targetRPM;
        if (shooting) {
            targetRPM = findRPMFromDistance(distance);
        } else {
            shootingSince.resetTimer();
            targetRPM = 0;
        }

        shooterPID.updateError((targetRPM - getRPM()));
        setRPM(shooterPID.run());
        telemetry.addData("RPM: ", getRPM());
        telemetry.addData("Target RPM: ", targetRPM);
        telemetry.addData("Ball Distance: ", ballDistance);


        if (Math.abs(targetRPM - getRPM()) < 100 && shooting) {
            if (flipperIsDown() && waitedSinceLastShot() && ballDistance < 60 && nonOscillatingRPM() && rampingTimeIsGood()) {
                moveFlipperUp();
                timeSinceShot.resetTimer();
            }
        } else {
            atRPMsince.resetTimer();
        }
        if (timeSinceShot.getElapsedTimeSeconds() > .35 && flipperServo.getPosition() != .35) {
            if (!override) {
                moveFlipperDown();
            }
        }
    }

    private boolean rampingTimeIsGood() {
        return shootingSince.getElapsedTimeSeconds() > 1.5;
    }

    private boolean flipperIsDown() {
        return flipperServo.getPosition() != .75;
    }

    private boolean nonOscillatingRPM() {
        return atRPMsince.getElapsedTimeSeconds() > .15;
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

    public double getFlipperPose() {
        return flipperServo.getPosition();
    }

    public void setOverride(boolean b) {
        override = b;
    }

    public void moveFlipperDown() {
        flipperServo.setPosition(.35);
    }

    public void moveFlipperUp() {
        flipperServo.setPosition(.75);
    }

    public void moveFlipperToPoint(double point) {
        flipperServo.setPosition(point);
    }

    public double findRPMFromDistance(double distance) {
        return -(-0.002267*distance*distance*distance+0.6989*distance*distance-61.34*distance+6341.27)+100;
        //return -(0.1270 * distance * distance - 14.4090 * distance + 5092.8328);
        //return -1.31E+09 + 1.68E+08*distance  -9.63E+06*Math.pow(distance, 2) + 325631*Math.pow(distance, 3) - -7185*Math.pow(distance, 4) + 108*Math.pow(distance, 5) -1.12*Math.pow(distance, 6) + 7.95E-03*Math.pow(distance, 7) -3.67E-05*Math.pow(distance, 8) + 9.99E-08*Math.pow(distance, 9) -1.22E-10*Math.pow(distance, 10);
    }

    public void setShooting(boolean doShoot) {
        shooting = doShoot;
    }

    private double getDistance() {
        double d1 = distance1.getDistance(DistanceUnit.MM);
        double d2 = distance2.getDistance(DistanceUnit.MM);

        return Math.min(d1, d2);
    }
}
