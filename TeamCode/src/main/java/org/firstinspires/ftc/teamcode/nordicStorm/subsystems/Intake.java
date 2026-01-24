package org.firstinspires.ftc.teamcode.nordicStorm.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.nordicStorm.NordicConstants.intakeMotorName;

public class Intake extends SubsystemBase {

    private final DcMotorEx intakeMotor;
    private boolean isRunning = false;
    private final DcMotor light;

    public Intake(final HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, intakeMotorName);
        light = hardwareMap.get(DcMotor.class, "limelightlight");
    }

    public boolean isRunning() {
        return this.isRunning;
    }

    public void runIntake(boolean doRun) {
        double power = doRun ? -1 : 0;
        intakeMotor.setPower(power);
        isRunning = doRun;
    }

    public void runIntakeReverse(boolean doRun) {
        int power = doRun ? 1 : 0;
        intakeMotor.setPower(power);
        isRunning = doRun;
    }

    public void runIntakeSlow() {
        isRunning = true;
        intakeMotor.setPower(-.5);
    }

    public void turnOnLight() {
        light.setPower(1);
    }

    public void turnOffLight() {
        light.setPower(0);
    }
}
