package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Langskip;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;


@TeleOp
public class Teleop extends OpMode{

    public Follower follower;

    private Langskip langskip;


    @Override
    public void init() {
        langskip = new Langskip(hardwareMap);
        follower = langskip.driveTrain.follower;
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        telemetry.update();
        langskip.driveTrain.updatePosition(telemetry);

        follower.setTeleOpMovementVectors(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false);
        follower.update();
// Robot Centric);

        while (gamepad1.a) {
            langskip.driveTrain.aim(telemetry);
        }
    }
}
