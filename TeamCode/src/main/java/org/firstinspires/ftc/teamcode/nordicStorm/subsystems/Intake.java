package org.firstinspires.ftc.teamcode.nordicStorm.subsystems;

import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Constants.intakeMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
public class Intake {


    private DcMotorEx intakeMotor;

    public Intake(final HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, intakeMotorName);

    }

    public void example(){

    }
}
