package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Landmarks;
import frc.robot.subsystems.drive.Drive;
import frc.util.GeometryUtil;

/** Drives field-relative while aiming at the hub. */
public class AimAndDriveCommand extends Command {
  private static final Angle kAimTolerance = Degrees.of(5);

  private final Drive drive;
  private final Command driveAtAngle;

  public AimAndDriveCommand(Drive drive, DoubleSupplier forwardInput, DoubleSupplier leftInput) {
    this.drive = drive;
    this.driveAtAngle =
        DriveCommands.joystickDriveAtAngle(
            drive,
            forwardInput,
            leftInput,
            this::getDirectionToHubInField);
    addRequirements(drive);
  }

  public AimAndDriveCommand(Drive drive) {
    this(drive, () -> 0.0, () -> 0.0);
  }

  /** Returns true when the robot is within the aim tolerance. */
  public boolean isAimed() {
    Rotation2d target = getDirectionToHubInField();
    Rotation2d current = drive.getRotation();
    return GeometryUtil.isNear(target, current, kAimTolerance);
  }

  private Rotation2d getDirectionToHubInField() {
    Translation2d hubPosition = Landmarks.hubPosition();
    Translation2d robotPosition = drive.getPose().getTranslation();
    return hubPosition.minus(robotPosition).getAngle();
  }

  @Override
  public void initialize() {
    driveAtAngle.initialize();
  }

  @Override
  public void execute() {
    driveAtAngle.execute();
  }

  @Override
  public void end(boolean interrupted) {
    driveAtAngle.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
