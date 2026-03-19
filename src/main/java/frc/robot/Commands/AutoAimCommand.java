package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;


public class AutoAimCommand extends Command {
    private final Drive drive;
    private final Supplier<Double> xSupplier;
    private final Supplier<Double> ySupplier;

    private final PIDController thetaController;

    private final Translation2d target;

    public AutoAimCommand(
            Drive drive,
            Supplier<Double> xSupplier,
            Supplier<Double> ySupplier,
            Translation2d target
    ) {
        this.drive = drive;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.target = target;

        thetaController = new PIDController(4.0, 0.0, 0.1);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    @Override
    public void execute() {
        Pose2d robotPose = drive.getPose();

        // Vector from robot to target
        Translation2d delta = target.minus(robotPose.getTranslation());

        // Desired angle
        Rotation2d targetAngle = new Rotation2d(
                delta.getX(),
                delta.getY()
        );

        // Current heading
        Rotation2d currentHeading = robotPose.getRotation();

        // PID output
        double omega = thetaController.calculate(
                currentHeading.getRadians(),
                targetAngle.getRadians()
        );

        // Driver translation input
        double xSpeed = xSupplier.get();
        double ySpeed = ySupplier.get();

        drive.drive(xSpeed, ySpeed, omega);
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return false; // runs until interrupted
    }
}