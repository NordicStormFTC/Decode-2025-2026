package org.firstinspires.ftc.teamcode.opmode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Globals;
import org.firstinspires.ftc.teamcode.nordicStorm.subsystems.Langskip;
import org.firstinspires.ftc.teamcode.nordicStorm.subsystems.NordicConstants;

@Autonomous(name = "Auto", group = "Examples")
public class Auto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Limelight3A limelight;
    private Langskip langskip;

    private int pathState;
    private int sawTagNumber = -1;

    private Pose startPose = Globals.ALLIANCE_COLOR == NordicConstants.AllianceColor.RED ? new Pose(96, 9.5, Math.toRadians(90)) : new Pose(48, 9.5, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(72 + 16, 72 + 47, Math.toRadians(205)); // Scoring Pose of the robot.
    private final Pose endPose = Globals.ALLIANCE_COLOR == NordicConstants.AllianceColor.RED ? new Pose(96, 24, Math.toRadians(90)) : new Pose(48, 24, Math.toRadians(90)); // Start Pose of our robot.

    private static final Pose[] ARTIFACT_POSES = {
            new Pose(95, 83, Math.toRadians(180)), // PPG 83.5
            new Pose(95, 59.5, Math.toRadians(180)), // PGP 59.5
            new Pose(95, 35.5, Math.toRadians(180)), // GPP 35.5
            new Pose(122, 83.5, Math.toRadians(180)), // After PPG
            new Pose(120, 59.5, Math.toRadians(180)), // After PGP
            new Pose(130, 35.5, Math.toRadians(180)), // After GPP
    };

    private PathChain scorePreload, moveToEnd;


    private static final int PPG_TAG_ID = 23;
    private static final int PGP_TAG_ID = 22;
    private static final int GPP_TAG_ID = 21;


    private final PathChain[] paths = new PathChain[4];


    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        langskip = new Langskip(hardwareMap, Globals.ALLIANCE_COLOR);
        limelight = langskip.driveTrain.limelight;
        follower = langskip.getFollower();


        buildConstantPaths();
        buildPathsFromTag();

        follower.setStartingPose(startPose);
        //follower.setMaxPower();
    }

    private void buildConstantPaths() {
      /*  scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build(); */

        moveToEnd = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
    }

    /**
     * Instantiate all Paths
     */
    private void buildPathsFromTag() {
        // Update the paths based on which tag we saw
        switch (sawTagNumber) {
            case PPG_TAG_ID:
            case -1:
                // Set paths for PPG
                paths[1] = grabBallPath(ARTIFACT_POSES[0], ARTIFACT_POSES[3]); // Grab PPG starting at ScorePose
                paths[2] = grabBallPath(ARTIFACT_POSES[1], ARTIFACT_POSES[4]); // Grab PGP starting at ScorePose
                paths[3] = grabBallPath(ARTIFACT_POSES[2], ARTIFACT_POSES[5]); // Grab GPP starting at ScorePose
               break;
            case PGP_TAG_ID:
                // Set paths for PGP
                paths[1] = grabBallPath(ARTIFACT_POSES[1], ARTIFACT_POSES[2]); // Grab PGP starting at ScorePose
                paths[2] = grabBallPath(ARTIFACT_POSES[0], ARTIFACT_POSES[3]); // Grab PPG starting at ScorePose
                paths[3] = grabBallPath(ARTIFACT_POSES[2], ARTIFACT_POSES[5]); // Grab GPP starting at ScorePose
                break;
            case GPP_TAG_ID:
                // Set paths for GPP
                paths[1] = grabBallPath(ARTIFACT_POSES[2], ARTIFACT_POSES[5]); // Grab GPP starting at ScorePose
                paths[2] = grabBallPath(ARTIFACT_POSES[0], ARTIFACT_POSES[3]); // Grab PPG starting at ScorePose
                paths[3] = grabBallPath(ARTIFACT_POSES[1], ARTIFACT_POSES[4]); // Grab PGP starting at ScorePose
        }
    }


    private PathChain grabBallPath(Pose beforeBall, Pose afterBall) {
        return follower.pathBuilder()
                .addPath(new BezierLine(scorePose, beforeBall))
                .addParametricCallback(.8, () -> follower.setMaxPower(.6))
                .setLinearHeadingInterpolation(scorePose.getHeading(), beforeBall.getHeading())
                .addPath(new BezierLine(beforeBall, afterBall))
                .addParametricCallback(.01, () -> langskip.intake.runIntake(true))
                .addParametricCallback(.01, () -> follower.setMaxPower(.2))
                .setLinearHeadingInterpolation(beforeBall.getHeading(), afterBall.getHeading())
                .addPath(new BezierLine(afterBall, scorePose))
                .addParametricCallback(.2, () -> langskip.intake.runIntake(false))
                .addParametricCallback(.01, () -> follower.setMaxPower(1))
                .setLinearHeadingInterpolation(afterBall.getHeading(), scorePose.getHeading())
                .build();
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);

                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.holdPoint(follower.getPose());
                }
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {
        // Override our current localization position if we see an AprilTag
        if (limelight.getLatestResult() != null && limelight.getLatestResult().isValid()) {
            for (LLResultTypes.FiducialResult detection : limelight.getLatestResult().getFiducialResults()) {
                if (detection.getFiducialId() == 20 || detection.getFiducialId() == 24) {


                   /* llResult = limelight.getLatestResult();
                    Pose3D botPose = llResult.getBotpose();

                    Position position = botPose.getPosition();
                    double pedroY = position.x * NordicConstants.metersToInches * -1 + 72;
                    double pedroX = position.y * NordicConstants.metersToInches + 72;

                    Pose pedroPose = new Pose(pedroX, pedroY, follower.getHeading());
                    follower.setPose(pedroPose); */

                    continue;
                } else if ((detection.getFiducialId() == 21 || detection.getFiducialId() == 22 || detection.getFiducialId() == 23) && sawTagNumber == -1) {
                    sawTagNumber = detection.getFiducialId();
                    buildPathsFromTag();
                }
            }
        }

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Follower is busy: ", follower.isBusy());
        telemetry.addData("Limelight: ", limelight.getLatestResult().getBotpose());
        telemetry.addData("Limelight Pipeline: ", limelight.getLatestResult().getPipelineIndex());
        telemetry.addData("Tag ID: ", sawTagNumber);
        telemetry.update();
    }


    /**
     * This method is called continuously after Init while waiting for "play".
     * The limelight will continuously search for a motif tag, then set the pathing pattern
     * based on which tag it sees.
     **/
    @Override
    public void init_loop() {
        follower.update();
        telemetry.addData("Starting Pose: ", follower.getPose());
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        limelight.pipelineSwitch(0);
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
        Globals.END_OF_AUTO_POSE = follower.getPose();
    }
}