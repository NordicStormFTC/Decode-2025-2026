package org.firstinspires.ftc.teamcode.nordicStorm.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.pedropathing.geometry.Pose;

import java.util.HashMap;
import java.util.Map;

@Configurable
public class Globals {

    @IgnoreConfigurable
    public enum ObeliskPattern {
        PPG,
        PGP,
        GPP,
        UNKNOWN
    }

    @IgnoreConfigurable
    public static Pose END_OF_AUTO_POSE = new Pose(96, 9.5, Math.toRadians(90));

    public static boolean endingObeliskSide = true;
    public static boolean shootingClose = true;
    public static boolean startingObeliskSide = true;



    @IgnoreConfigurable
    public static Map<Integer, Pose[]> pickupOrder = new HashMap<>(); // Stores the order of ball Pickup and the before/after pickup poses.

    public static int PPGPriority = 0;
    public static int PGPPriority = 1;
    public static int GPPPriority = 2;
    public static int HP1Priority = 3;
    public static int HP2Priority = 4;
    public static int HP3Priority = 5;

    public static int openGateAfterPickup = 0; // If one, will empty gate after grabbing the first artifact set.
    public static boolean findMotifTag = false;
    public static boolean shootFourTimes = false;

    public static NordicConstants.AllianceColor ALLIANCE_COLOR = NordicConstants.AllianceColor.RED;

}
