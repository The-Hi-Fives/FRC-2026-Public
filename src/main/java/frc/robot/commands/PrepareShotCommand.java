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
        double distanceFromTag = 0;
        int[] validTags = {26, 27, 20, 24};
//        final Distance distanceToHub = getDistanceToHub();
//        final Shot shot = distanceToShotMap.get(distanceToHub);
//        Shot shot;
        LimelightHelpers.RawFiducial[] detectedTags = LimelightHelpers.getRawFiducials("limelight");
        for (LimelightHelpers.RawFiducial detectedTag : detectedTags) {
            if(IntStream.of(validTags).anyMatch(x -> x == detectedTag.id))
            {
                //Distance from tag in inches
                distanceFromTag = detectedTag.distToCamera * 39.37;
                SmartDashboard.putNumber("Distance to Hub (inches)", distanceFromTag);
            }
        }

        if(distanceFromTag <= 52)
        {
            //2800, 0.19
            shooter.setRPM(2800);
            hood.setPosition(0.19);
        }
        else if (distanceFromTag <= 114)
        {
            //3275, 0.40
            shooter.setRPM(3275);
            hood.setPosition(0.4);
        }
        else if (distanceFromTag <= 165)
        {
            //3650, 0.48
            shooter.setRPM(3650);
            hood.setPosition(0.48);
        }
        else
        {
            //4000 0.5
            shooter.setRPM(4000);
            hood.setPosition(0.5);
        }

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
