package org.firstinspires.ftc.teamcode.nordicStorm.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.NordicConstants.intakeMotorName;

public class Intake extends SubsystemBase {

    private final DcMotorEx intakeMotor;
    private boolean isRunning = false;

    public Intake(final HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, intakeMotorName);
    }

    public boolean isRunning() {
        return this.isRunning;
    }

    public void runIntake(boolean doRun) {
        if (doRun) {
            intakeMotor.setPower(-1);
        } else {
            intakeMotor.setPower(0);
        }
        isRunning = doRun;
    }

    public void runIntakeSlow() {
        isRunning = true;
        intakeMotor.setPower(.75);
    }
}
