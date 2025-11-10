package org.firstinspires.ftc.teamcode.nordicStorm.subsystems;


import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.NordicConstants.intakeMotorName;
import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.NordicConstants.pixyName;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.nordicStorm.pixy.Pixy;

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
        isRunning = !isRunning;
    }

    public void runIntakeReverse(boolean doRun){
        if (doRun) {
            intakeMotor.setPower(1);
        } else {
            intakeMotor.setPower(0);
        }
        isRunning = !isRunning;
    }
}
