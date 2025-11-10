package org.firstinspires.ftc.teamcode.nordicStorm.subsystems;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.pedropathing.geometry.Pose;

public class NordicConstants {

        public static final int blueAprilTagPipeline = 0;
        public static final int redAprilTagPipeline = 8;
        public static final int obeliskAprilTagPipeline = 1;

        public static final double metersToInches = 39.3701;

        public  static final String intakeMotorName = "intake";
        public static final String pixyName = "pixy";
        public static final String shootingMotorName = "shooter";
        public static final String signalLightName = "light";
        public static final int pixyCenterXPixel = 125; //158
        public static final double degreesPerPixel = .25;
        public static final double pixyHeightInches = 9.375;
        public static final double pixyAngleWithVertical = 44.5;

        public static final Pose redGoalPose = new Pose(139, 139, 0);
        public static final Pose blueGoalPose = new Pose(5, 139, 0);

        public enum AllianceColor {
                RED,
                BLUE
        }

}
