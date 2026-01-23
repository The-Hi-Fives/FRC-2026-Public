package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** RobotContainer using AdvantageKit Drive subsystem. */
public class RobotContainer {
  // Drive
  private final Drive drive;

  // Other subsystems (kept from original project)
  private final Intake intake = new Intake();
  private final Floor floor = new Floor();
  private final Feeder feeder = new Feeder();
  private final Shooter shooter = new Shooter();
  private final Hood hood = new Hood();
  private final Hanger hanger = new Hanger();
  private final Limelight limelight = new Limelight("limelight");

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);

  // Commands
  private final SubsystemCommands subsystemCommands;

  // Auto chooser (PathPlanner)
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    // Instantiate Drive based on mode
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;
      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;
      default:
        drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
        break;
    }

    subsystemCommands =
        new SubsystemCommands(
            drive,
            intake,
            floor,
            feeder,
            shooter,
            hood,
            hanger,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX());

    configureBindings();

    // PathPlanner auto chooser (Drive constructor configures AutoBuilder)
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", com.pathplanner.lib.auto.AutoBuilder.buildAutoChooser());
  }

  private void configureBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    // Lock to headings with buttons (optional)
    driver.a().whileTrue(DriveCommands.joystickDriveAtAngle(drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> Rotation2d.kZero));

    // X pattern
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro heading
    driver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Original subsystem bindings
    RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop())
        .onTrue(intake.homingCommand())
        .onTrue(hanger.homingCommand());

    driver.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());
    driver.rightBumper().whileTrue(subsystemCommands.shootManually());
    driver.leftTrigger().whileTrue(intake.intakeCommand());
    driver.leftBumper().onTrue(intake.runOnce(() -> intake.set(Intake.Position.STOWED)));

    driver.povUp().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
    driver.povDown().onTrue(hanger.positionCommand(Hanger.Position.HUNG));

    // Vision update (same logic as original)
    limelight.setDefaultCommand(updateVisionCommand());
  }

  private Command updateVisionCommand() {
    return limelight
        .run(
            () ->
                limelight
                    .getMeasurement(drive.getPose())
                    .ifPresent(
                        (m) ->
                            drive.addVisionMeasurement(
                                m.poseEstimate.pose,
                                m.poseEstimate.timestampSeconds,
                                m.standardDeviations)))
        .ignoringDisable(true);
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
