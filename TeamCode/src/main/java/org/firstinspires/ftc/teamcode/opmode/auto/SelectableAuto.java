package org.firstinspires.ftc.teamcode.opmode.auto;


import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Globals.GPPPriority;
import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Globals.HP1Priority;
import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Globals.HP2Priority;
import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Globals.HP3Priority;
import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Globals.PGPPriority;
import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Globals.PPGPriority;
import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Globals.findMotifTag;
import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Globals.openGateAfterPickup;
import static org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Globals.pickupOrder;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Globals;
import org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Langskip;
import org.firstinspires.ftc.teamcode.nordicStorm.subsystems.NordicConstants;

@Autonomous(name = "Selectable Auto", group = "Tournament")
public class SelectableAuto extends OpMode {
    // Declare essential variables for path following, timers, and hardware.
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Langskip langskip;


    private int pathState; // Tracks which path the robot is currently following

    // Define the key poses
    private Pose startPose = Globals.startingObeliskSide ? new Pose(24.5, 123, Math.toRadians(142)) : new Pose(48, 9.5, Math.toRadians(90)); // Start Pose of our robot.
    private Pose readMotifPose = Globals.shootingClose ? new Pose(60, 84, Math.toRadians(260)) : new Pose(53, 20, Math.toRadians(90));
    private Pose scorePose = Globals.shootingClose ? new Pose(60, 84, Math.toRadians(135)) : new Pose(53, 15, Math.toRadians(270 - 338.84)); // Scoring Pose of the robot. // TODO (56, 119, 309)
    private Pose beforeReleaseGatePose = new Pose(40, 75, Math.toRadians(90));
    private Pose releaseGatePose = new Pose(20, 75, Math.toRadians(90)); // TODO
    private Pose endPose = Globals.endingObeliskSide ? new Pose(44, 72, Math.toRadians(90)) : new Pose(48, 35, Math.toRadians(90));

    private final Pose[] ARTIFACT_POSES = {
            new Pose(49, 83.5, Math.toRadians(180)), // PPG 83.5
            new Pose(49, 59.5, Math.toRadians(180)), // PGP 59.5
            new Pose(49, 35.5, Math.toRadians(180)), // GPP 35.5
            new Pose(13.5, 25, Math.toRadians(180)), // Before Human Player Balls

            new Pose(16, 83.5, Math.toRadians(180)), // After PPG
            new Pose(14, 59.5, Math.toRadians(180)), // After PGP
            new Pose(10, 35.5, Math.toRadians(180)), // After GPP
            new Pose(13.5, 11, Math.toRadians(30)) // After Human Player Balls
    };

    private PathChain scorePreload, readMotif, moveToEnd;

    private final PathChain[] paths = new PathChain[4]; // Array to store the pickup paths.

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();

        // If the alliance color is red, flip all poses to match the red field layout.
        if (Globals.ALLIANCE_COLOR == NordicConstants.AllianceColor.RED) {
            flipAllPoses();
        }

        // Initialize an instance of the robot (all subsystems)
        langskip = new Langskip(hardwareMap, Globals.ALLIANCE_COLOR);
        follower = langskip.getFollower();
        follower.setStartingPose(startPose);

        // Initialize the pickup order for the different artifact collection priorities.
        // This is based on how the priority was set from Globals, which can be done through Panels.
        pickupOrder.put(PPGPriority, new Pose[]{ARTIFACT_POSES[0], ARTIFACT_POSES[4]});
        pickupOrder.put(PGPPriority, new Pose[]{ARTIFACT_POSES[1], ARTIFACT_POSES[5]});
        pickupOrder.put(GPPPriority, new Pose[]{ARTIFACT_POSES[2], ARTIFACT_POSES[6]});
        pickupOrder.put(HP1Priority, new Pose[]{ARTIFACT_POSES[3], ARTIFACT_POSES[7]}); // The three Human Player ball pickups are the same.
        pickupOrder.put(HP2Priority, new Pose[]{ARTIFACT_POSES[3], ARTIFACT_POSES[7]});
        pickupOrder.put(HP3Priority, new Pose[]{ARTIFACT_POSES[3], ARTIFACT_POSES[7]});

        buildConstantPaths(); // Paths that are always used
        buildPathsFromPanels(); // Pickup paths based on the priority order.
    }

    private void buildConstantPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose)) // Path from start to scoring position.
                .addParametricCallback(.01, () -> follower.setMaxPower(.7))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        readMotif = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, readMotifPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), readMotifPose.getHeading())
                .addParametricCallback(.95, () -> follower.setMaxPower(.5))
                .addPath(new BezierLine(readMotifPose, scorePose))
                .setLinearHeadingInterpolation(readMotifPose.getHeading(), scorePose.getHeading())
                .addParametricCallback(.05, () -> follower.setMaxPower(1))
                .build();

        moveToEnd = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose)) // Path from scoring position to end position.
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();
    }

    private void buildPathsFromPanels() {
        // Use the 3 highest priority ball pickups from panels to create the auto.
        int openGateNumber = Globals.openGateAfterPickup - 1;
        for (int i = 0; i < 3; i++) {
            Pose[] pickupPoses = pickupOrder.get(i);
            assert pickupPoses != null;
            paths[i] = grabBallPath(pickupPoses[0], pickupPoses[1], i == openGateNumber, (!Globals.shootFourTimes) && i == 2);
        }
    }

    private PathChain grabBallPath(Pose beforeBall, Pose afterBall, boolean openGate, boolean endAfter) {
        if (!openGate && !endAfter) {
            return follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, beforeBall)) // Drive from the scoring position to the first spike mark
                    .addParametricCallback(.01, () -> langskip.intake.runIntake(false))
                    .addParametricCallback(.8, () -> follower.setMaxPower(.6)) // When 80& of the way complete, slow speed to 60%
                    .setLinearHeadingInterpolation(scorePose.getHeading(), beforeBall.getHeading(), .75)
                    .addPath(new BezierLine(beforeBall, afterBall)) // We are now inline with the balls, drive straight forwards.
                    .addParametricCallback(.01, () -> langskip.intake.runIntake(true)) // Turn on the intake
                    .addParametricCallback(.01, () -> follower.setMaxPower(.3)) // Move at 20% speed
                    .setLinearHeadingInterpolation(beforeBall.getHeading(), afterBall.getHeading())
                    //.addPath(new BezierLine(afterBall, beforeBall))
                    //.setLinearHeadingInterpolation(afterBall.getHeading(), beforeBall.getHeading())
                    //.addParametricCallback(.5, () -> langskip.intake.runIntake(false)) // Turn off the intake after the path is 50% complete
                    //.addParametricCallback(.01, () -> follower.setMaxPower(1)) // Drive at full speed.
                    .addPath(new BezierLine(afterBall, scorePose)) // After picking up the balls, move back to the scoring position
                    .addParametricCallback(.01, () -> follower.setMaxPower(1)) // Move at 20% speed
                    .addParametricCallback(.25, () -> langskip.intake.runIntake(false))
                    .setLinearHeadingInterpolation(afterBall.getHeading(), scorePose.getHeading(), .75)
                    .build();
        } else if (!endAfter) {
            return follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, beforeBall)) // Drive from the scoring position to the first spike mark
                    .addParametricCallback(.01, () -> langskip.intake.runIntake(false))
                    .addParametricCallback(.8, () -> follower.setMaxPower(.6)) // When 80% of the way complete, slow speed to 60%
                    .setLinearHeadingInterpolation(scorePose.getHeading(), beforeBall.getHeading(), .75)
                    .addPath(new BezierLine(beforeBall, afterBall)) // We are now inline with the balls, drive straight forwards.
                    .addParametricCallback(.01, () -> langskip.intake.runIntake(true)) // Turn on the intake
                    .addParametricCallback(.01, () -> follower.setMaxPower(.35)) // Move at 20% speed
                    .setLinearHeadingInterpolation(beforeBall.getHeading(), afterBall.getHeading())
                    .addPath(new BezierLine(afterBall, beforeReleaseGatePose))
                    .addParametricCallback(.5, () -> langskip.intake.runIntake(false)) // Turn off the intake after the path is 20% complete
                    .addParametricCallback(.01, () -> follower.setMaxPower(.75)) // Drive at medium speed.
                    .addPath(new BezierLine(beforeReleaseGatePose, releaseGatePose))
                    .addParametricCallback(.5, () -> follower.setMaxPower(.5))
                    .setLinearHeadingInterpolation(beforeReleaseGatePose.getHeading(), releaseGatePose.getHeading())
                    .addPath(new BezierLine(releaseGatePose, beforeReleaseGatePose))
                    .setLinearHeadingInterpolation(releaseGatePose.getHeading(), beforeReleaseGatePose.getHeading())
                    .addPath(new BezierLine(beforeReleaseGatePose, scorePose)) // After picking up the balls, move back to the scoring position
                    .setLinearHeadingInterpolation(beforeBall.getHeading(), scorePose.getHeading(), .75)
                    .addParametricCallback(.01, () -> follower.setMaxPower(1)) // Drive at full speed.
                    .build();
        } else {
            return follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, beforeBall)) // Drive from the scoring position to the first spike mark
                    .addParametricCallback(.01, () -> langskip.intake.runIntake(false))
                    .addParametricCallback(.8, () -> follower.setMaxPower(.6)) // When 80& of the way complete, slow speed to 60%
                    .setLinearHeadingInterpolation(scorePose.getHeading(), beforeBall.getHeading())
                    .addPath(new BezierLine(beforeBall, afterBall)) // We are now inline with the balls, drive straight forwards.
                    .addParametricCallback(.01, () -> langskip.intake.runIntake(true)) // Turn on the intake
                    .addParametricCallback(.01, () -> follower.setMaxPower(.35)) // Move at 20% speed
                    .setLinearHeadingInterpolation(beforeBall.getHeading(), afterBall.getHeading())
                    .addPath(new BezierLine(afterBall, beforeBall))
                    .setLinearHeadingInterpolation(afterBall.getHeading(), beforeBall.getHeading())
                    .addParametricCallback(.5, () -> langskip.intake.runIntake(false)) // Turn off the intake after the path is 50% complete
                    .addParametricCallback(.01, () -> follower.setMaxPower(1)) // Drive at full speed.
                    .addPath(new BezierLine(beforeBall, endPose)) // After picking up the balls, move back to the scoring position
                    .setLinearHeadingInterpolation(beforeBall.getHeading(), endPose.getHeading())
                    .build();
        }
    }

    private void flipAllPoses() {
        // Flip all the poses if the alliance color is red, adjusting the pose values.
        Pose[] allPoses = new Pose[]{startPose, scorePose, beforeReleaseGatePose, releaseGatePose, readMotifPose, endPose, ARTIFACT_POSES[0], ARTIFACT_POSES[1],
                ARTIFACT_POSES[2], ARTIFACT_POSES[3], ARTIFACT_POSES[4], ARTIFACT_POSES[5], ARTIFACT_POSES[6], ARTIFACT_POSES[7]};
        for (int i = 0; i < allPoses.length; i++) {
            Pose pose = allPoses[i];
            double heading = Math.PI / 2 - (pose.getHeading() - (Math.PI / 2));
            if (pose.getHeading() == Math.PI) {
                heading = 0;
            }

            allPoses[i] = new Pose(144 - pose.getX(), pose.getY(), heading);
        }
        startPose = allPoses[0];
        scorePose = allPoses[1];
        beforeReleaseGatePose = allPoses[2];
        releaseGatePose = allPoses[3];
        readMotifPose = allPoses[4];
        endPose = allPoses[5];
        ARTIFACT_POSES[0] = allPoses[6];
        ARTIFACT_POSES[1] = allPoses[7];
        ARTIFACT_POSES[2] = allPoses[8];
        ARTIFACT_POSES[3] = allPoses[9];
        ARTIFACT_POSES[4] = allPoses[10];
        ARTIFACT_POSES[5] = allPoses[11];
        ARTIFACT_POSES[6] = allPoses[12];
        ARTIFACT_POSES[7] = allPoses[13];
    }

    /**
     * This method updates the robot's autonomous path based on the current path state.
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (findMotifTag) {
                    follower.followPath(readMotif, true);
                } else {
                    follower.followPath(scorePreload, true); // Always start by scoring preload
                }
                setPathState(1);
                actionTimer.resetTimer();
                break;
            case 1:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > .2) { // TODO if we fix auto shooting, the extra states are not needed.
                    /* TODO Score Preload */
                    follower.setMaxPower(.85);
                    langskip.innerSubsystem.setShooting(true);
                    actionTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 4) {
                    langskip.innerSubsystem.setShooting(false);
                    langskip.intake.runIntake(false);
                    follower.followPath(paths[0], true);
                    actionTimer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > .2) {
                    actionTimer.resetTimer();
                    /* TODO Score Sample */
                    langskip.innerSubsystem.setShooting(true);
                    actionTimer.resetTimer();
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 4) {
                    langskip.innerSubsystem.setShooting(false);
                    follower.followPath(paths[1], true);
                    langskip.intake.runIntake(false);
                    actionTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > .2) {
                    actionTimer.resetTimer();
                    /* TODO Score Sample */
                    langskip.innerSubsystem.setShooting(true);
                    actionTimer.resetTimer();
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 4) {
                    langskip.innerSubsystem.setShooting(false);
                    langskip.intake.runIntake(false);
                    follower.followPath(paths[2], true);
                    actionTimer.resetTimer();
                    if (!Globals.shootFourTimes) {
                        setPathState(-1);
                    } else {
                        setPathState(7);
                    }
                }
                break;
            case 7:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > .2) {
                    actionTimer.resetTimer();
                    /* TODO Score Sample */
                    langskip.innerSubsystem.setShooting(true);
                    actionTimer.resetTimer();
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 4) {
                    langskip.innerSubsystem.setShooting(false);
                    langskip.intake.runIntake(false);
                    follower.followPath(moveToEnd, true);
                    setPathState(-1); // End
                }
                break;
            case -1:
                if (!follower.isBusy()) {
                    follower.followPath(follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), endPose))
                            .addParametricCallback(.65, () -> follower.setMaxPower(.4))
                            .addParametricCallback(.1, () -> langskip.intake.runIntake(false))
                            .setLinearHeadingInterpolation(follower.getHeading(), endPose.getHeading())
                            .build());
                }
        }
    }

    @Override
    public void loop() {
        // Update follower and path states. Required to move the robot.
        follower.update();
        langskip.periodic(telemetry);

        telemetry.addData("Path state", pathState);
        telemetry.addData("Position: ", follower.getPose());
        telemetry.addData("Action Timer: ", actionTimer.getElapsedTimeSeconds());
        telemetry.update();

        // If we are not outside of the starting zone at the end of auto, force it to leave.
        if (opmodeTimer.getElapsedTimeSeconds() > 28) {
            follower.breakFollowing();
            setPathState(-1);
            opmodeTimer.resetTimer();
            return;
        }
        autonomousPathUpdate();
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
        buildConstantPaths(); // Paths that are always used
        buildPathsFromPanels(); // Pickup paths based on the priority order.
        follower.update();
        telemetry.addData("Starting Pose: ", follower.getPose());
        telemetry.addData("Shooting Pose: ", scorePose);
        telemetry.addData("Ending Pose: ", endPose);
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * Remember the robot state for Teleop
     **/
    @Override
    public void stop() {
        Globals.END_OF_AUTO_POSE = follower.getPose();
    }
}
