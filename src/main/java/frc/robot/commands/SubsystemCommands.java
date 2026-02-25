package frc.robot.commands;

import java.util.Objects;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public final class SubsystemCommands {
    private final Swerve swerve;
    private final Intake intake;
    private final Floor floor;
    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final Hanger hanger;

    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;

    public SubsystemCommands(
        Swerve swerve,
        Intake intake,
        Floor floor,
        Feeder feeder,
        Shooter shooter,
        Hood hood,
        Hanger hanger,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.floor = floor;
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.hanger = hanger;

        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
    }

    public SubsystemCommands(
        Swerve swerve,
        Intake intake,
        Floor floor,
        Feeder feeder,
        Shooter shooter,
        Hood hood,
        Hanger hanger
    ) {
        this(
            swerve,
            intake,
            floor,
            feeder,
            shooter,
            hood,
            hanger,
            () -> 0,
            () -> 0
        );
    }

    public Command aimAndShoot() {
        final AimAndDriveCommand aimAndDriveCommand = new AimAndDriveCommand(swerve, forwardInput, leftInput);
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood, () -> swerve.getState().Pose);
        // return Commands.parallel(
        //     aimAndDriveCommand,
        //     Commands.waitSeconds(0.25)
        //         .andThen(prepareShotCommand),
        //     Commands.waitUntil(() -> prepareShotCommand.isReadyToShoot())
        //         .andThen(feed())
        // );

        return Commands.parallel(
            prepareShotCommand, Commands.waitUntil(() -> prepareShotCommand.isReadyToShoot()).andThen(feed())
        );
    }

    public Command shootManually() {
        return shooter.dashboardSpinUpCommand()
            .handleInterrupt(() -> shooter.stop());
        // return shooter.dashboardSpinUpCommand()
        //     .andThen(feed())
        //     .handleInterrupt(() -> shooter.stop());
    }

    public Command feed() {
        return Commands.sequence(
            Commands.waitSeconds(0.25),
            Commands.parallel(
                feeder.feedCommand(),
                Commands.waitSeconds(0.125)
                    .andThen(floor.feedCommand().alongWith(intake.agitateCommand()))
            )
        );
    }

    public void setHoodPercent(String state) {

        double currentHoodPercent = hood.getCurrentPercent();
        SmartDashboard.putNumber("Current Hood Percent: ", currentHoodPercent);

        if(Objects.equals(state, "U")){
            hood.setPercent(currentHoodPercent + .01);
        } else {
            hood.setPercent(currentHoodPercent - .01);
        }
    }

    public void setRPM(String state) {
        double currentRPM = shooter.getMotorVelocity() * 60;
        SmartDashboard.putNumber("Current RPM: ", currentRPM); 
        if (Objects.equals(state, "U")) {

            shooter.setRPM(currentRPM + 100);
        } else {
            shooter.setRPM(currentRPM - 100);
        }
    }
    
}
