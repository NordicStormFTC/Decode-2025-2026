package org.firstinspires.ftc.teamcode.nordicStorm.subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;
import org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Constants;

import java.util.List;

public class DriveTrain {


    public final Follower follower;

    private final DcMotorEx leftFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightFront;
    private final DcMotorEx rightRear;

    private final Limelight3A limelight;

    private LLResult llResult;

    private Pose position;

    //private final PIDFController limelightDriveController;
    //private final PIDFController limelightRotationController;

    public DriveTrain(final HardwareMap hardwareMap) {
        follower = new Follower(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);

        follower.setStartingPose(new Pose());

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(Constants.aprilTagPipeline);

        limelight.start();

        //limelightDriveController = new PIDFController(new CustomPIDFCoefficients(0.03, 0, 0.03, 0.001));
        //limelightRotationController = new PIDFController(new CustomPIDFCoefficients(0.03, 0, 0.035, 0.001));

        //limelightDriveController.setTargetPosition(-15);
        //limelightRotationController.setTargetPosition(0);


        position = new Pose();
    }

    public Pose getPosition(final Telemetry telemetry) {
        updatePosition(telemetry);
        return follower.getPose();
    }

    public void updatePosition(final Telemetry telemetry) {

        if (llResultsAreGood()) {
            telemetry.addLine("Using April Tags");

            Pose3D botPose = getLLResult().getBotpose();

            position.setX(metersToInches(botPose.getPosition().x));
            position.setY(metersToInches(botPose.getPosition().y));
            position.setHeading(follower.getTotalHeading());

            follower.setPose(position);
            follower.update();
        } else {
            telemetry.addLine("Not using April Tags, ll results are not good.");
            follower.update();
            position = follower.getPose();
        }
    }


    public void seekBall(final Telemetry telemetry) {
        if (llResultsAreGood()) {
            double rotationError = llResult.getTx();
            double driveError = llResult.getTy();

            //limelightRotationController.updatePosition(rotationError);
            //limelightDriveController.updatePosition(driveError);

            telemetry.addLine("Seeking");
            telemetry.addData("Tx", rotationError);
            telemetry.addData("Ty", driveError);

            double rotationPower = 0;
            double drivePower = 0;

            /*
             * if the rotation error is within [-1, 1] we intentionally don't apply power
             */
            //if (Math.abs(limelightRotationController.getError()) > 1) {
             //   rotationPower = limelightRotationController.runPIDF();
            //}


            //if (Math.abs(limelightDriveController.getError()) > 0.5) {
            //    drivePower = limelightDriveController.runPIDF();
            //}

            //telemetry.addData("Drive PID", limelightDriveController.getError());

            follower.setTeleOpMovementVectors(-drivePower, 0, rotationPower, false);
        } else {
            telemetry.addLine("No valid results!");
        }
        telemetry.update();
    }

    public void aim(final Telemetry telemetry) {
        if (llResultsAreGood()) {
            Pose3D botPose = llResult.getBotpose();
            List<LLResultTypes.FiducialResult> tags = limelight.getLatestResult().getFiducialResults();

        }
    }
    private double metersToInches(double meters) {
        return meters * Constants.metersToInches;
    }

    private LLResult getLLResult() {
        llResult =  limelight.getLatestResult();
        return llResult;
    }

    private boolean llResultsAreGood() {
        return getLLResult() != null && getLLResult().isValid() && getLLResult().getStaleness() < 100;
    }
}
