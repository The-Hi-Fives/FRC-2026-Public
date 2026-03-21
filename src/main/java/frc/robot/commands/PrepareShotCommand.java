package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;
import java.util.stream.IntStream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Landmarks;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class PrepareShotCommand extends Command {
    private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap = new InterpolatingTreeMap<>(
        (startValue, endValue, q) -> 
            InverseInterpolator.forDouble()
                .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
        (startValue, endValue, t) ->
            new Shot(
                Interpolator.forDouble()
                    .interpolate(startValue.shooterRPM, endValue.shooterRPM, t),
                Interpolator.forDouble()
                    .interpolate(startValue.hoodPosition, endValue.hoodPosition, t)
            )
    );

    static {
        distanceToShotMap.put(Inches.of(52.0), new Shot(2800, 0.19)); //2800, 0.19
        distanceToShotMap.put(Inches.of(114.4), new Shot(3275, 0.40)); //3275, 0.40
        distanceToShotMap.put(Inches.of(165.5), new Shot(3650, 0.48)); //3650, 0.48
    }

    private final Shooter shooter;
    private final Hood hood;
    private final Supplier<Pose2d> robotPoseSupplier;
    private double prevDistToTag = 9999999.0;

    public PrepareShotCommand(Shooter shooter, Hood hood, Supplier<Pose2d> robotPoseSupplier) {
        this.shooter = shooter;
        this.hood = hood;
        this.robotPoseSupplier = robotPoseSupplier;
        addRequirements(shooter, hood);
    }

    public boolean isReadyToShoot() {
        return shooter.isVelocityWithinTolerance() && hood.isWithinTolerance();
    }

    private Distance getDistanceToHub() {
        final Translation2d robotPosition = robotPoseSupplier.get().getTranslation();
        final Translation2d hubPosition = Landmarks.hubPosition();
        return Meters.of(robotPosition.getDistance(hubPosition));
    }

    @Override
    public void execute() {
//        //TODO: add in seperate arrays for alliance tags, and braching code for alliances
//        double distanceFromTag = 0;
//        int[] validTags = {25, 26, 27, 20, 24, 21, 19, 20, 18, 17, 31, 8, 5, 4, 3, 2, 11, 10, 9, 1, 22, 1, 6};
////        final Distance distanceToHub = getDistanceToHub();
////        final Shot shot = distanceToShotMap.get(distanceToHub);
////        Shot shot;
//        LimelightHelpers.RawFiducial[] detectedTagsRight = LimelightHelpers.getRawFiducials("limelight-right");
//        LimelightHelpers.RawFiducial[] detectedTagsLeft = LimelightHelpers.getRawFiducials("limelight-left");
//
//        for (LimelightHelpers.RawFiducial detectedTag : detectedTagsRight) {
//            if(IntStream.of(validTags).anyMatch(x -> x == detectedTag.id))
//            {
//                //Distance from tag in inches
//                distanceFromTag = detectedTag.distToCamera * 39.37;
//                SmartDashboard.putNumber("Distance to Hub (inches)", distanceFromTag);
//                if(distanceFromTag < prevDistToTag) prevDistToTag = distanceFromTag;
//            }
//        }
//        for (LimelightHelpers.RawFiducial detectedTag : detectedTagsLeft) {
//            if(IntStream.of(validTags).anyMatch(x -> x == detectedTag.id))
//            {
//                //Distance from tag in inches
//                distanceFromTag = detectedTag.distToCamera * 39.37;
//                SmartDashboard.putNumber("Distance to Hub (inches)", distanceFromTag);
//                if(distanceFromTag < prevDistToTag) prevDistToTag = distanceFromTag;
//            }
//        }
//
//        // if(prevDistToTag <= 52)
//        // {
//        //     //2800, 0.19
//        //     shooter.setRPM(3400);
//        //     hood.setPosition(0.1);
//        //     SmartDashboard.putNumber("RPM: ", 2800);
//        //     SmartDashboard.putNumber("Position: ", 0.1);
//        // }
//        // else if (prevDistToTag <= 114)
//        // {
//        //     //3275, 0.40
//        //     shooter.setRPM(3800);
//        //     hood.setPosition(0.4);
//        //     SmartDashboard.putNumber("RPM: ", 3275);
//        //     SmartDashboard.putNumber("Position: ", 0.4);
//        // }
//        // else if (prevDistToTag <= 165)
//        // {
//        //     //3650, 0.48
//        //     shooter.setRPM(4100);
//        //     hood.setPosition(0.48);
//        //     SmartDashboard.putNumber("RPM: ", 3650);
//        //     SmartDashboard.putNumber("Position: ", 0.48);
//        // }
//        // else
//        // {
//        //     //4000 0.5
//        //     shooter.setRPM(4600);
//        //     hood.setPosition(0.5);
//        // }

        prevDistToTag = getDistanceToHub().in(Inches) - 13;
        SmartDashboard.putNumber("Distance to Hub", prevDistToTag);

        double hoodAngle = -0.00143472 * Math.pow(prevDistToTag, 2) + 0.624497 * prevDistToTag + 4.32421;
        // double shooterRPM = 0.0985926 * Math.pow(prevDistToTag, 2) + -5.42072 * prevDistToTag + 2865.86315;
        double shooterRPM = 0.0985926 * Math.pow(prevDistToTag, 2) - 6.60383 * prevDistToTag + 2902;
        SmartDashboard.putNumber("shooterRPM", shooterRPM);
        SmartDashboard.putNumber("hoodAngle", hoodAngle*.01);
        shooter.setRPM(shooterRPM);
        hood.setPosition(hoodAngle*.01);

//        shooter.setRPM(shot.shooterRPM);
//        hood.setPosition(shot.hoodPosition);
//        SmartDashboard.putNumber("Distance to Hub (inches)", distanceToHub.in(Inches));

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    public static class Shot {
        public final double shooterRPM;
        public final double hoodPosition;

        public Shot(double shooterRPM, double hoodPosition) {
            this.shooterRPM = shooterRPM;
            this.hoodPosition = hoodPosition;
        }
    }
}
