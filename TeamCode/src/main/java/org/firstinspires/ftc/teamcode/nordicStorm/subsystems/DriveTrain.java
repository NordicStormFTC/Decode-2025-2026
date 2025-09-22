package org.firstinspires.ftc.teamcode.nordicStorm.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

public class DriveTrain {


    public final Follower follower;

    private final Limelight3A limelight;

    private LLResult llResult;

    //private final PIDFController limelightDriveController;
    //private final PIDFController limelightRotationController;

    public DriveTrain(final HardwareMap hardwareMap) {
        follower = new Follower(hardwareMap);

        limelight = null; // TODO

        limelight.start();


    }

    public void setTeleopMovementVectors(double forwardDrive, double lateralDrive, double heading, boolean robotCentric) {
        this.follower.setTeleOpMovementVectors(forwardDrive, lateralDrive, heading, robotCentric);
    }
}
