package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nordicStorm.Globals;
import org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Langskip;
import org.firstinspires.ftc.teamcode.nordicStorm.NordicConstants;


@TeleOp(name = "Tournament Teleop")
public class TournamentTeleop extends OpMode {
    private Follower follower;

    private Langskip langskip;
    public static Pose startingPose;
    private boolean slowMode = true;
    private final double slowModeMultiplier = .35;

    private int a = 2500;

    private NordicConstants.AllianceColor allianceColor;


    @Override
    public void init() {
        langskip = new Langskip(hardwareMap, Globals.ALLIANCE_COLOR, true);
        langskip.innerSubsystem.setAtPoint(false);
        startingPose = Globals.END_OF_AUTO_POSE;
        follower = langskip.getFollower();
        follower.setStartingPose(startingPose);
        follower.update();

        allianceColor = Globals.ALLIANCE_COLOR;
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        langskip.intake.turnOnLight();
    }
// 94.4, 30

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetry.update();
        langskip.periodic(telemetry);
        telemetry.addData("Langskip state:", langskip.currentState);
        telemetry.addData("Heading Error: ", follower.getHeadingError());
        telemetry.update();
        //telemetry.addData("Intake: ", a);

        if (langskip.currentState == Langskip.State.IDLE) {
            if (allianceColor == NordicConstants.AllianceColor.RED) {
                if (!slowMode) follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x * slowModeMultiplier,
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
                        -gamepad1.right_stick_x * slowModeMultiplier,
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
        // Right Bumper = Auto Intake

        // Left Trigger = Run Intake Reverse
        // Left Bumper = Human Player Intake

        // Right DPad = Move flipper up
        // Left DPad = Switch slow mode on/off
        // Down Dpad = reset position

        // Y = Shooting
        // A = Auto Park
        // B = Open Gate


        // Right Trigger = Intake
        if (gamepad1.right_trigger > .5 && !langskip.intake.isRunning()) {
            langskip.intake.runIntake(true);
        }

        if (gamepad1.right_trigger < .1 && gamepad1.left_trigger < .1 && langskip.currentState == Langskip.State.IDLE && langskip.intake.isRunning()) {
            langskip.intake.runIntake(false);
        }

        if (gamepad1.rightBumperWasPressed()) {
            langskip.changeState(Langskip.State.AUTO_INTAKE);
        }

        if (gamepad1.rightBumperWasReleased()) {
            langskip.intake.turnOnLight();
            langskip.changeState(Langskip.State.IDLE);
            follower.breakFollowing();
            follower.startTeleopDrive();
            langskip.a = !langskip.a;
            langskip.intake.runIntake(false);
        }

        if (gamepad1.left_trigger > .5 && !langskip.intake.isRunning()) {
            langskip.intake.runIntakeReverse(true);
        }

        if (gamepad1.leftBumperWasPressed()) {
            langskip.changeState(Langskip.State.HPINTAKE);
            follower.breakFollowing();
        }

        if (gamepad1.leftBumperWasReleased()) {
            follower.setMaxPower(1);
            langskip.intake.runIntake(false);
            langskip.changeState(Langskip.State.IDLE);
            follower.breakFollowing();
            follower.startTeleopDrive();
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
        if (gamepad1.dpadLeftWasPressed()) {
            slowMode = !slowMode;
        }

        if (gamepad1.dpadDownWasPressed()) {
            follower.setPose(new Pose(72, 72, Math.toRadians(90)));
        }

        // Y = Shooting
        if (gamepad1.yWasPressed()) {
            langskip.changeState(Langskip.State.AIMING);
            langskip.intake.runIntakeSlow();
        }

        if (gamepad1.yWasReleased()) {
            langskip.changeState(Langskip.State.IDLE);
            langskip.innerSubsystem.setShooting(false);
            langskip.intake.runIntake(false);
            langskip.innerSubsystem.setAtPoint(false);
            follower.startTeleopDrive();
        }

        // A = Auto Park
        if (gamepad1.aWasPressed()) {
            langskip.changeState(Langskip.State.AUTO_PARKING);
        }

        if (gamepad1.aWasReleased()) {
            langskip.changeState(Langskip.State.IDLE);
            slowMode = true;
            follower.startTeleopDrive();
        }

        // B = Open the Gate
        if (gamepad1.bWasPressed()) {
            langskip.changeState(Langskip.State.OPEN_GATE);
            follower.breakFollowing();
        }

        if (gamepad1.bWasReleased()) {
            follower.setMaxPower(1);
            langskip.intake.runIntake(false);
            langskip.changeState(Langskip.State.IDLE);
            follower.breakFollowing();
            follower.startTeleopDrive();
        }
    }
}