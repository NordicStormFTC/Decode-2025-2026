package org.firstinspires.ftc.teamcode.nordicStorm;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.pedropathing.geometry.Pose;

import java.util.HashMap;
import java.util.Map;

@Configurable
public class Globals {

    // This holds the robot position at the end of auto so Teleop knows where we start.
    @IgnoreConfigurable
    public static Pose END_OF_AUTO_POSE = new Pose(96, 9.5, Math.toRadians(90));

    @IgnoreConfigurable
    public static Map<Integer, Pose[]> pickupOrder = new HashMap<>(); // Stores the order of ball Pickup and the before/after pickup poses.

    // These are all configurable parameters for our autos.
    public static boolean endingObeliskSide = true;
    public static boolean shootingClose = true;
    public static boolean startingObeliskSide = true;

    public static int PPGPriority = 0;
    public static int PGPPriority = 1;
    public static int GPPPriority = 2;
    public static int HP1Priority = 3;
    public static int HP2Priority = 4;
    public static int HP3Priority = 5;

    public static int openGateAfterPickup = 0; // If one, will empty gate after grabbing the first artifact set.

    public static boolean forceLeave = true;

    // IMPORTANT: This needs to be set at the beginning of every match
    public static NordicConstants.AllianceColor ALLIANCE_COLOR = NordicConstants.AllianceColor.RED;

    // Shooter PID values
    @IgnoreConfigurable
    public static double shooterP = 250;
    @IgnoreConfigurable
    public static double shooterD = 0;
    @IgnoreConfigurable
    public static double shooterI = 0;
    @IgnoreConfigurable
    public static double shooterFeedForwards = 17.5;

    @IgnoreConfigurable
    public static double strafeP = 0;
    @IgnoreConfigurable
    public static double rotationP = .5;
    @IgnoreConfigurable
    public static double forwardScale = 5;

    @IgnoreConfigurable
    public static double rotationalP = 1;
    @IgnoreConfigurable
    public static double rotationalD = .1;


}
