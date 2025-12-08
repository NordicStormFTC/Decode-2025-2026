package org.firstinspires.ftc.teamcode.nordicStorm.subsystems;


import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.NordicConstants.intakeMotorName;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Intake subsystem.
 * Contains one motor that runs the intake wheel.
 * Public methods allow for running the intake inwards and outwards,
 * as well as checking if the intake is currently running.
 */
public class Intake extends SubsystemBase {


    private final DcMotorEx intakeMotor;

    private boolean isRunning = false;

    public Intake(final HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, intakeMotorName);
    }

    public boolean isRunning() {
        return this.isRunning;
    }

    public void runIntake(boolean doRun){
        if (doRun) {
            intakeMotor.setPower(-1);
        } else {
            intakeMotor.setPower(0);
        }
        isRunning = doRun;
    }

    public void runIntakeSlow() {
        isRunning = true;
        intakeMotor.setPower(-.75);
    }

    public void runIntakeReverse(boolean doRun){
        if (doRun) {
            intakeMotor.setPower(1);
        } else {
            intakeMotor.setPower(0);
        }
        isRunning = doRun;
    }
}
