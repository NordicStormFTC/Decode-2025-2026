package org.firstinspires.ftc.teamcode.opmode.teleop;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Globals;
import org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Langskip;
import org.firstinspires.ftc.teamcode.nordicStorm.subsystems.NordicConstants;


@TeleOp(name = "Tournament Teleop")
public class TournamentTeleop extends OpMode {
    private Follower follower;

    private Langskip langskip;
    public static Pose startingPose;
    private TelemetryManager telemetryM;
    private boolean slowMode = true;
    private final double slowModeMultiplier = .65;

    public int a = 3000;

    private NordicConstants.AllianceColor allianceColor;


    @Override
    public void init() {
        langskip = new Langskip(hardwareMap, Globals.ALLIANCE_COLOR);
        startingPose = Globals.END_OF_AUTO_POSE;
        follower = langskip.getFollower();
        follower.setStartingPose(startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        allianceColor = Globals.ALLIANCE_COLOR;
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        langskip.signalLight.setPosition(.277);
        /*if (allianceColor == NordicConstants.AllianceColor.BLUE) {
            langskip.setSignalColor(.611);
        } else {
            langskip.setSignalColor(.277);
        } */
    }


    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();
        langskip.periodic(telemetry);
        telemetry.addData("Langskip state:", langskip.currentState);
        telemetry.addData("Heading Error: ", follower.getHeadingError());
        telemetry.addData("Intake: ", a);

        if (langskip.currentState == Langskip.State.IDLE) {
            if (allianceColor == NordicConstants.AllianceColor.RED) {
                if (!slowMode) follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        false
                );

                    //This is how it looks with slowMode on
                else follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * slowModeMultiplier,
                        -gamepad1.left_stick_x * slowModeMultiplier,
                        -gamepad1.right_stick_x * slowModeMultiplier,
                        false
                );
            } else {
                if (!slowMode) follower.setTeleOpDrive(
                        gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        false
                );

                    //This is how it looks with slowMode on
                else follower.setTeleOpDrive(
                        gamepad1.left_stick_y * slowModeMultiplier,
                        gamepad1.left_stick_x * slowModeMultiplier,
                        -gamepad1.right_stick_x * slowModeMultiplier,
                        false
                );
            }
        }

        // Right Trigger = Intake
        // Right DPad = Move flipper up
        // Left DPad = Switch slow mode on/off
        // Y = Shooting
        // Left Bumper = Auto Park
        // Right Bumper = HP Intake


        // Right Trigger = Intake
        if (gamepad1.right_trigger > .5 && !langskip.intake.isRunning()) {
            langskip.intake.runIntake(true);
        }

        if (gamepad1.right_trigger < .1 && langskip.currentState == Langskip.State.IDLE && langskip.intake.isRunning()) {
            langskip.intake.runIntake(false);
        }

        // Right DPad = Move flipper up
        if (gamepad1.dpadRightWasPressed()) {
            langskip.innerSubsystem.setOverride(true);
            langskip.innerSubsystem.moveFlipperUp();
        }

        if (gamepad1.dpadRightWasReleased()) {
            langskip.innerSubsystem.setOverride(false);
            langskip.innerSubsystem.moveFlipperDown();
        }


        //Slow Mode
        /*if (gamepad1.dpadLeftWasPressed()) {
            slowMode = !slowMode;
        }

        if (gamepad1.dpadDownWasPressed()) {
            follower.setPose(new Pose(72, 72, Math.toRadians(90)));
        } */

        if (gamepad1.dpadDownWasPressed()) {
            langskip.setSignalColor(.277);
            a -= 50;
        }

        if (gamepad1.dpadUpWasPressed()) {
            a += 50;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            langskip.innerSubsystem.setIntakeSpped(a);
        }

        // Y = Shooting
        if (gamepad1.yWasPressed()) {
            langskip.changeState(Langskip.State.SHOOTING);
            langskip.intake.runIntakeSlow();
        }
        if (gamepad1.yWasReleased()) {
            langskip.changeState(Langskip.State.IDLE);
            langskip.innerSubsystem.setShooting(false);
            langskip.intake.runIntake(false);
            follower.startTeleopDrive();
        }

        // Left Bumper = Auto Park
        if (gamepad1.leftBumperWasPressed()) {
            langskip.changeState(Langskip.State.AUTO_PARKING);
        }

        if (gamepad1.leftBumperWasReleased()) {
            langskip.changeState(Langskip.State.IDLE);
            slowMode = true;
            follower.startTeleopDrive();
        }

        // Right Bumper = Human Player Intake
        if (gamepad1.rightBumperWasPressed()) {
            langskip.changeState(Langskip.State.HPINTAKE);
        }

        if (gamepad1.rightBumperWasReleased()) {
            follower.setMaxPower(1);
            langskip.intake.runIntake(false);
            langskip.changeState(Langskip.State.IDLE);
            follower.startTeleopDrive();
        }
    }
}