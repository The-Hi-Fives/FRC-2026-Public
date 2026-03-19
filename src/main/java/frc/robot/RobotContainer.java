// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Objects;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Driving;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.util.SwerveTelemetry;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final Floor floor = new Floor();
    private final Feeder feeder = new Feeder();
    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final Hanger hanger = new Hanger();
     private final Limelight limelightright = new Limelight("limelight-right");
     private final Limelight limelightleft = new Limelight("limelight-left");


    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(Driving.kMaxSpeed.in(MetersPerSecond));
    
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);


    private final AutoRoutines autoRoutines = new AutoRoutines(
        swerve,
        intake,
        floor,
        feeder,
        shooter,
        hood,
        hanger,
        limelightright,
        limelightleft
    );
    private final SubsystemCommands subsystemCommands = new SubsystemCommands(
        swerve,
        intake,
        floor,
        feeder,
        shooter,
        hood,
        hanger,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX()
    );
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        configureBindings();
        autoRoutines.configure();
        swerve.registerTelemetry(swerveTelemetry::telemeterize);
    }
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {

        if (DriverStation.isTeleopEnabled()) {

        }
        configureManualDriveBindings();
        // limelight.setDefaultCommand(updateVisionCommand());

        // RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop())
        (RobotModeTriggers.teleop())
            .onTrue(intake.homingCommand())
            .onTrue(hood.homingCommand())
            .onTrue(hanger.hangerUp());        

        (RobotModeTriggers.autonomous())
            .onTrue(intake.homingCommandAuto())
            .onTrue(hanger.homingCommandMore());

        
        
    }

    private void configureManualDriveBindings() {
        final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
            swerve, 
            () -> -driver.getLeftY(), 
            () -> -driver.getLeftX(), 
            () -> -driver.getRightX()
        );
        swerve.setDefaultCommand(manualDriveCommand);

        //Driver Controls\\  

        driver.start().onTrue(Commands.runOnce(() -> manualDriveCommand.seedFieldCentric())); //Zero Robot Heading

        driver.a().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.k180deg)));
        driver.b().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCW_90deg)));
        driver.x().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCCW_90deg)));
        driver.y().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kZero)));

        driver.leftTrigger().whileTrue(intake.intakePosition());  
        driver.leftTrigger().toggleOnTrue(intake.intakeRollers());                             //Rollers
        driver.back().onTrue(hood.homingCommand());                                           //Zero Hood
        driver.leftBumper().onTrue(intake.runOnce(() -> intake.set(Intake.Position.STOWED))); //Stow

        driver.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());                     //Aim/Shoot
        driver.rightTrigger().whileFalse(Commands.run(() -> subsystemCommands.setRPM("1500"))); //Idle for Shooter AFTER Shooting
        driver.rightBumper().whileFalse(Commands.run(() -> subsystemCommands.setRPM("1500"))); //Idle for Shooter AFTER Feeding
        // driver.rightBumper().whileTrue(subsystemCommands.shootManually());                 //Manual Shoot
        driver.rightBumper().whileTrue(Commands.run(() -> subsystemCommands.setIdleRPM("F")));
        driver.rightBumper().whileTrue(Commands.sequence(
            shooter.runOnce(() -> shooter.setRPM(3700)),
            hood.runOnce(() -> hood.setPosition(1)),
            subsystemCommands.feed())); //Feeding
        
        //Operator Controls\\  
        
        operator.x().whileTrue(Commands.runOnce(() -> subsystemCommands.setRPM("U")));    //Shooter Speed Up
        operator.a().whileTrue(Commands.runOnce(() -> subsystemCommands.setRPM("D")));    //Shooter Speed Down

        operator.leftStick().whileTrue(Commands.run(() -> subsystemCommands.setFeedSpeed("FM")));
        operator.leftStick().whileTrue(Commands.sequence(
            shooter.runOnce(() -> shooter.setRPM(5000)),
            hood.runOnce(() -> hood.setPosition(1)),
            subsystemCommands.feed())); //Feeding

        operator.leftStick().whileFalse(Commands.run(() -> subsystemCommands.setRPM("1500"))); //Idle for Shooter AFTER Hail Mary

        operator.rightTrigger().and(operator.start()).whileTrue((Commands.runOnce(() -> shooter.setRPM(-6000)))); //Reverse Shooter

        operator.leftBumper().onTrue(intake.runOnce(() -> intake.set(Intake.Position.STOWED)));   //Stow
        operator.back().onTrue(intake.homingCommand());                                           //Zero Intake

        operator.y().whileTrue(Commands.runOnce(() -> subsystemCommands.setHoodPercent("U"))); //Hood Angle Up
        operator.b().whileTrue(Commands.runOnce(() -> subsystemCommands.setHoodPercent("D"))); //Hood Angle Down

        operator.povUp().onTrue(hanger.positionCommand(Hanger.Position.HANGING));                 //Climb Hanging
        operator.povDown().onTrue(hanger.positionCommand(Hanger.Position.HUNG));                  //Climb Hung
        operator.start().onTrue(hanger.homingCommand());                                        //Zero Climb

        operator.leftTrigger().whileTrue(intake.reverseIntakeCommand());                          //Outtake

        operator.povRight().whileTrue(subsystemCommands.feed());                                  //Manual Feed
        operator.rightBumper().whileTrue(subsystemCommands.reverseFeed());                        //Manual Feed Reverse
        

    }

    // private Command updateVisionCommand() {
    //     return limelight.run(() -> {
    //         final Pose2d currentRobotPose = swerve.getState().Pose;
    //         final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentRobotPose);
    //         measurement.ifPresent(m -> {
    //             swerve.addVisionMeasurement(
    //                 m.poseEstimate.pose, 
    //                 m.poseEstimate.timestampSeconds,
    //                 m.standardDeviations
    //             );
    //         });
    //     })
    //     .ignoringDisable(true);
    // }
    /**
     * Updates all subsystem simulations. This should be called from the robot's
     * simulationPeriodic method.
     */
    public void simulationPeriodic() {
        swerve.simulationPeriodic();
        shooter.simulationPeriodic();
        // Add other subsystems here as you implement their simulation logic
    }
}
