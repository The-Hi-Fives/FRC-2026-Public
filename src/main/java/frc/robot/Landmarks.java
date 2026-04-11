package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.util.HashMap;
import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Landmarks {
    public static double inchesToMeters(double inches) {
        return inches / 39.37;
    }
    public static HashMap hubPosition() {
        HashMap<String, Translation2d> fieldCords = new HashMap<>();
        /*
        public static final double BLUE_HUB_X = inchesToMeters(182.1);
        public static final double RED_HUB_X = Constants.Physical.FIELD_LENGTH - BLUE_HUB_X;
        public static final double HUB_Y = Constants.Physical.FIELD_WIDTH / 2;
        public static final double HUB_Z = 1.83;
        public static final Translation3d HUB_POSE_BLUE = new Translation3d(BLUE_HUB_X, HUB_Y, HUB_Z);
        public static final Translation3d HUB_POSE_RED = new Translation3d(RED_HUB_X, HUB_Y, HUB_Z);
         */
        //TODO: Change BLUE HUB X back to 182.1 at comp
        final double BLUE_HUB_X = inchesToMeters(182.1); //182.1

        // Is this correct???
        // final double RED_HUB_X = inchesToMeters(316.64) - BLUE_HUB_X;
        final double RED_HUB_X = inchesToMeters(469.11);
        final double HUB_Y = inchesToMeters(317.69) / 2;
        final double HUB_Z = 1.83;
        final Translation2d HUB_POSE_BLUE = new Translation2d(BLUE_HUB_X, HUB_Y);
        final Translation2d HUB_POSE_RED = new Translation2d(RED_HUB_X, HUB_Y);
        final Optional<Alliance> alliance = DriverStation.getAlliance();

        final double BLUE_PASS_X = inchesToMeters(91.05);
        final double RED_PASS_X = inchesToMeters(560.17);
        final double PASS_LEFT_Y = inchesToMeters(238.26);
        final double PASS_RIGHT_Y = inchesToMeters(79.42);

        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            fieldCords.put("HUB_POSE", HUB_POSE_BLUE);
            fieldCords.put("PASS_LEFT", new Translation2d(BLUE_PASS_X, PASS_LEFT_Y));
            fieldCords.put("PASS_RIGHT", new Translation2d(BLUE_PASS_X, PASS_RIGHT_Y));
            return fieldCords;
        //    return new Translation2d(Inches.of(36), Inches.of(0)); //182.105, 158.845
        } else {
        fieldCords.put("HUB_POSE", HUB_POSE_RED);
        fieldCords.put("PASS_LEFT", new Translation2d(RED_PASS_X, PASS_LEFT_Y));
        fieldCords.put("PASS_RIGHT", new Translation2d(RED_PASS_X, PASS_RIGHT_Y));
        return fieldCords;
        // return HUB_POSE_RED;
    //    return new Translation2d(Inches.of(36), Inches.of(0)); //469.115, 158.854
    }
}

}
