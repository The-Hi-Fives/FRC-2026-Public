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
import frc.robot.subsystems.LEDs;
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
    private String lastUser = "";

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
    public static boolean isAimAndShooting = false;

    public Command aimAndShoot(String user) {
        boolean runShoot = false;
        if(Objects.equals(lastUser, ""))
        {
            lastUser = user;
        }
        else if(Objects.equals(user, ""))
        {
            lastUser = "";
        }

        if(Objects.equals(user, "driver"))
        {
            runShoot = true;
        }
        else if (Objects.equals(user, "operator") && Objects.equals(lastUser, "operator"))
        {
            runShoot = true;
        }

        if(!runShoot)
        {
            return Commands.none();
        }
        final AimAndDriveCommand aimAndDriveCommand = new AimAndDriveCommand(swerve, forwardInput, leftInput);
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood, () -> swerve.getPose());
        return Commands.deadline(
            aimAndDriveCommand,
            Commands.waitSeconds(0)
                .andThen(prepareShotCommand),
            // Commands.waitUntil(() -> aimAndDriveCommand.isAimed() && prepareShotCommand.isReadyToShoot())
            Commands.waitUntil(() -> prepareShotCommand.isReadyToShoot())
                .andThen(feed())
                .beforeStarting(() -> isAimAndShooting = true)
                .finallyDo(() -> isAimAndShooting = false)
                
            );
                
        }

    public Command shootManually() {
        final ShootManually manualShoot = new ShootManually(shooter);
        return Commands.parallel(
            manualShoot
        );

        // return shooter.dashboardSpinUpCommand()
        //     .andThen(feed())
        //     .handleInterrupt(() -> shooter.stop());
    }

    public Command feed() {
        return Commands.sequence(
            Commands.parallel(
                feeder.feedCommand(),
                Commands.waitSeconds(0.125)
                    .andThen(floor.feedCommand().alongWith(intake.agitateCommand()))
            )
        );

    }

        public Command reverseFeed() {
        return Commands.sequence(
            Commands.waitSeconds(0.25),
            Commands.parallel(
                feeder.reverseFeedCommand(),
                Commands.waitSeconds(0.125)
                    .andThen(floor.reverseFeedCommand())
            )
        );
    }

    // public Command feedToAlliance() {
    //     return Commands.sequence(
    //         Commands.waitSeconds(0),
    //         Commands.sequence(
    //             shooter.setRPM(4000),
    //             Commands.waitSeconds(0),
    //             hood.setPosition(.70))
    //         );
    // }

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
        } else if (Objects.equals(state, "D")){
            shooter.setRPM(currentRPM - 100);
        } else if (Objects.equals(state, "1500")) {

            shooter.setRPM(1500);
        }
    }

    public void setIdleRPM(String state) {
        if (Objects.equals(state, "F")) {
            shooter.setRPM(3000);
        }
    }

    public void setFeedSpeed(String state) {
        if (Objects.equals(state, "FM")) {
            shooter.setRPM(5000);
        }
    }

    // public void setIdleRPM(String state) {
    //     // if (Objects.equals(state, "1500")) {

    //     //     shooter.setRPM(1500);
    //     } else shooter.setRPM(0);
           
    //     }
          

    public void setRPMSpeed(double speed) {
        shooter.setRPM(speed);
    }

    // public static void staticSetRPM(String state)
    // {
    //     setRPM(state);
    // }
    
}
