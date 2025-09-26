package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Intake;
import org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Langskip;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;


@Configurable
@TeleOp
public class Teleop2 extends OpMode{

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
        follower.update();
        telemetry.update();
        langskip.driveTrain.updatePosition(telemetry);

        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true); // Robot Centric);

        if (gamepad1.a) {
            langskip.intake.example();
        }
    }
}
