package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Globals.END_OF_AUTO_POSE;
import static java.lang.System.currentTimeMillis;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Globals;
import org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Langskip;
import org.firstinspires.ftc.teamcode.nordicStorm.subsystems.NordicConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@TeleOp(name = "Tournament Teleop")
public class TournamentTeleop extends OpMode {
    private Follower follower;

    private Langskip langskip;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

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
    }


    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();


        //Make the last parameter false for field-centric
        //In case the drivers want to use a "slowMode" you can scale the vectors

        //This is the normal version to use in the TeleOp
        if (langskip.currentState == Langskip.State.IDLE) {
            if (allianceColor == NordicConstants.AllianceColor.RED) {
                if (!slowMode) follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        false // Robot Centric
                );

                    //This is how it looks with slowMode on
                else follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * slowModeMultiplier,
                        -gamepad1.left_stick_x * slowModeMultiplier,
                        -gamepad1.right_stick_x * slowModeMultiplier,
                        false // Robot Centric
                );
            } else {
                if (!slowMode) follower.setTeleOpDrive(
                        gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        false // Robot Centric
                );

                    //This is how it looks with slowMode on
                else follower.setTeleOpDrive(
                        gamepad1.left_stick_x * slowModeMultiplier,
                        gamepad1.left_stick_y * slowModeMultiplier,
                        -gamepad1.right_stick_x * slowModeMultiplier,
                        false // Robot Centric
                );
            }
        }


        langskip.periodic(telemetry);
        telemetry.addData("Langskip state:", langskip.currentState);


        if (gamepad1.aWasPressed()) {
            langskip.intake.runIntake(true);
        }

        if (gamepad1.aWasReleased()) {
            langskip.intake.runIntake(false);
        }

        if (gamepad1.bWasPressed()) {
            langskip.intake.runIntakeReverse(true);
        }

        if (gamepad1.bWasReleased()) {
            langskip.intake.runIntakeReverse(false);
        }

        if (gamepad1.xWasPressed()) {
            langskip.changeState(Langskip.State.BALL_SEARCHING);
        }
        if (gamepad1.xWasReleased()) {
            follower.breakFollowing();
            langskip.changeState(Langskip.State.IDLE);
            langskip.intake.runIntake(false);
            follower.startTeleopDrive();
        }

        if (gamepad1.dpadLeftWasPressed()) {
            follower.breakFollowing();
            langskip.changeState(Langskip.State.AIMING);
        }
        if (gamepad1.dpadLeftWasReleased()) {
            follower.breakFollowing();
            langskip.changeState(Langskip.State.IDLE);
            langskip.intake.runIntake(false);
            follower.startTeleopDrive();
        }

        if (gamepad1.yWasPressed()) {
            langskip.changeState(Langskip.State.AIMING);
        }
        if (gamepad1.yWasReleased()) {
            langskip.changeState(Langskip.State.IDLE);
        }

        if (gamepad1.bWasReleased()) {
            langskip.intake.runIntakeReverse(false);
        }

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        //Optional way to change slow mode strength
        // if (gamepad1.xWasPressed()) {
        //   slowModeMultiplier += 0.1;
        //}

        //Optional way to change slow mode strength
        if (gamepad1.yWasPressed()) {
            slowModeMultiplier -= 0.1;
        }

        if (gamepad2.leftBumperWasPressed()) {
            langskip.changeState(Langskip.State.AUTO_PARKING);
        }
        if (gamepad2.leftBumperWasReleased()) {
            follower.breakFollowing();
            langskip.changeState(Langskip.State.IDLE);
            follower.startTeleopDrive();
        }

        if (gamepad2.rightBumperWasPressed()) {
            follower.breakFollowing();
            langskip.changeState(Langskip.State.HPINTAKE);
        }

        if (gamepad2.rightBumperWasReleased()) {
            follower.breakFollowing();
            follower.setMaxPower(1);
            langskip.intake.runIntake(false);
            langskip.changeState(Langskip.State.IDLE);
            follower.startTeleopDrive();
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}