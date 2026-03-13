// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.generated.ChoreoTraj.*;


// import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$0;
// import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$1;
// import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$2;
// import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$3;

// import static frc.robot.generated.AZToNZ.ChoreoTraj.AZToNZ;
// import static frc.robot.generated.AZToNZ.ChoreoTraj.AZToNZ$0;
// import static frc.robot.generated.AZToNZ.ChoreoTraj.AZToNZ$1;
// import static frc.robot.generated.AZToNZ.ChoreoTraj.AZToNZ$2;
// import static frc.robot.generated.AZToNZ.ChoreoTraj.AZToNZ$3;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Hanger.Position;

public final class AutoRoutines {
    private final Swerve swerve;
    private final Intake intake;
    private final Floor floor;
    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final Hanger hanger;
    private final Limelight limelight;

    private final SubsystemCommands subsystemCommands;

    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;

    public AutoRoutines(
        Swerve swerve,
        Intake intake,
        Floor floor,
        Feeder feeder,
        Shooter shooter,
        Hood hood,
        Hanger hanger,
        Limelight limelight
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.floor = floor;
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.hanger = hanger;
        this.limelight = limelight;

        this.subsystemCommands = new SubsystemCommands(swerve, intake, floor, feeder, shooter, hood, hanger);

        this.autoFactory = swerve.createAutoFactory();
        this.autoChooser = new AutoChooser();
    }

    public void configure() {
        autoChooser.addRoutine("Outpost and Depot", this::outpostAndDepotRoutine);
        autoChooser.addRoutine("AZ -> NZ", this::allianceZoneToNeutralZoneRoutine);
        autoChooser.addRoutine("Outpost and Hub", this::outpostToHubRoutine);
        autoChooser.addRoutine("Outpost and Depot from Bump", this::outpostAndDepotRoutineFromBump);
        autoChooser.addRoutine("Center to Shoot to Climb", this::centerToHubToClimb);
        autoChooser.addRoutine("Depot to Hub", this::depotToHubTrajectory);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    private AutoRoutine outpostAndDepotRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("Outpost and Depot");
        final AutoTrajectory startToOutpost = OutpostAndDepotTrajectory$0.asAutoTraj(routine);
        final AutoTrajectory outpostToDepot = OutpostAndDepotTrajectory$1.asAutoTraj(routine);
        final AutoTrajectory depotToShootingPose = OutpostAndDepotTrajectory$2.asAutoTraj(routine);
        final AutoTrajectory shootingPoseToTower = OutpostAndDepotTrajectory$3.asAutoTraj(routine);


        
        // routine.active().onTrue(intake.runOnce(() -> intake.set(Intake.Position.INTAKE)));

        // routine.observe(hanger::isHomed).onTrue(
        // routine.active().onTrue(
        //     Commands.sequence(
        //         hanger.runOnce(() -> hanger.set(Position.EXTEND_HOPPER)),
        //         Commands.waitSeconds(0.25),
        //         intake.runOnce(() -> intake.set(Intake.Position.INTAKE))
        //     )
        // );


        // routine.active().onTrue(
        //     Commands.sequence(
        //         // hanger.runOnce(() -> hanger.set(Position.EXTEND_HOPPER)),
        //         Commands.waitSeconds(.25),
        //         intake.runOnce(() -> intake.set(Intake.Position.INTAKE))
        //     )
        // );

        routine.active().onTrue(
            Commands.sequence(
                startToOutpost.resetOdometry(),
                startToOutpost.cmd()
            )
        );

         routine.active().onTrue(
            Commands.sequence(
                // hanger.runOnce(() -> hanger.set(Position.HANGING)),
                // Commands.waitSeconds(1.5),
                intake.runOnce(() -> intake.set(Intake.Position.INTAKE))
            )
        );

        startToOutpost.doneDelayed(1).onTrue(outpostToDepot.cmd());

        outpostToDepot.atTimeBeforeEnd(1).onTrue(intake.intakeCommand());
        outpostToDepot.doneDelayed(0.1).onTrue(depotToShootingPose.cmd());

        depotToShootingPose.active().whileTrue(limelight.idle());
        depotToShootingPose.atTime(0.5).onTrue(
            Commands.parallel(
                shooter.spinUpCommand(3000),
                hood.positionCommand(0.35)
            )
        );
        depotToShootingPose.done().onTrue(
            Commands.sequence(
                subsystemCommands.aimAndShoot()
                    .withTimeout(5),
                shootingPoseToTower.cmd()
            )
        );

        shootingPoseToTower.active().whileTrue(limelight.idle());
        shootingPoseToTower.active().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
        shootingPoseToTower.done().onTrue(hanger.positionCommand(Hanger.Position.HUNG));

        return routine;
    }

     private AutoRoutine outpostAndDepotRoutineFromBump() {
        final AutoRoutine routine = autoFactory.newRoutine("Outpost and Depot from Bump");
        final AutoTrajectory startToOutpost = OutpostAndDepotTrajectoryFromBump$0.asAutoTraj(routine);
        final AutoTrajectory outpostToDepot = OutpostAndDepotTrajectoryFromBump$1.asAutoTraj(routine);
        final AutoTrajectory depotToShootingPose = OutpostAndDepotTrajectoryFromBump$2.asAutoTraj(routine);
        final AutoTrajectory shootingPoseToTower = OutpostAndDepotTrajectoryFromBump$3.asAutoTraj(routine);


        
        // routine.active().onTrue(intake.runOnce(() -> intake.set(Intake.Position.INTAKE)));

        // routine.observe(hanger::isHomed).onTrue(
       

        routine.active().onTrue(
            Commands.sequence(
                startToOutpost.resetOdometry(),
                startToOutpost.cmd()
            )
        );

         routine.active().onTrue(
            Commands.sequence(
                hanger.runOnce(() -> hanger.set(Position.HANGING)),
                Commands.waitSeconds(1.5),
                intake.runOnce(() -> intake.set(Intake.Position.INTAKE))
            )
        );

        startToOutpost.doneDelayed(1).onTrue(outpostToDepot.cmd());

        outpostToDepot.atTimeBeforeEnd(1).onTrue(intake.intakeCommand());
        outpostToDepot.doneDelayed(0.1).onTrue(depotToShootingPose.cmd());

        depotToShootingPose.active().whileTrue(limelight.idle());
        depotToShootingPose.atTime(0.5).onTrue(
            Commands.parallel(
                shooter.spinUpCommand(3000),
                hood.positionCommand(0.35)
            )
        );
        depotToShootingPose.done().onTrue(
            Commands.sequence(
                subsystemCommands.aimAndShoot()
                    .withTimeout(5),
                shootingPoseToTower.cmd()
            )
        );

        shootingPoseToTower.active().whileTrue(limelight.idle());
        shootingPoseToTower.active().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
        shootingPoseToTower.done().onTrue(hanger.positionCommand(Hanger.Position.HUNG));

        return routine;
    }

    private AutoRoutine allianceZoneToNeutralZoneRoutine() {
         final AutoRoutine routine = autoFactory.newRoutine("AZ -> NZ");
         final AutoTrajectory nZToStartIntake = AZToNZ$0.asAutoTraj(routine);
         final AutoTrajectory startIntakeToShoot = AZToNZ$1.asAutoTraj(routine);
         final AutoTrajectory shootToStartIntake = AZToNZ$2.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                nZToStartIntake.resetOdometry(),
                nZToStartIntake.cmd()
            )
        );

        routine.active().onTrue(
            Commands.sequence(
                hanger.runOnce(() -> hanger.set(Position.HANGING)),
                Commands.waitSeconds(1.5),
                intake.runOnce(() -> intake.set(Intake.Position.INTAKE))
            )
        );

        nZToStartIntake.atTimeBeforeEnd(1).onTrue(intake.intakeCommand());
        nZToStartIntake.doneDelayed(0.1).onTrue(startIntakeToShoot.cmd());

        startIntakeToShoot.atTime(0.5).onTrue(
            Commands.parallel(
                shooter.spinUpCommand(2600),
                hood.positionCommand(0.32)
            )
        );

        startIntakeToShoot.done().onTrue(
            Commands.sequence(
                subsystemCommands.aimAndShoot()
                    .withTimeout(5),
                shootToStartIntake.cmd()
            )
        );

        nZToStartIntake.atTimeBeforeEnd(1).onTrue(intake.intakeCommand());
        nZToStartIntake.doneDelayed(0.1).onTrue(
            Commands.sequence(
                subsystemCommands.aimAndShoot()
                    .withTimeout(5)
            )
        );


        return routine;
    }

    private AutoRoutine outpostToHubRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("Outpost and Hub");
        final AutoTrajectory startToOutpost = OutpostToHubTrajectory$0.asAutoTraj(routine);
        final AutoTrajectory outpostToShootingPoseToTower = OutpostToHubTrajectory$1.asAutoTraj(routine);
        final AutoTrajectory shootingPoseToTowerToClimb = OutpostAndDepotTrajectory$2.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                startToOutpost.resetOdometry(),
                startToOutpost.cmd()
            )
        );

         routine.active().onTrue(
            Commands.sequence(
                hanger.runOnce(() -> hanger.set(Position.HANGING)),
                Commands.waitSeconds(1.5),
                intake.runOnce(() -> intake.set(Intake.Position.INTAKE))
            )
        );

        startToOutpost.doneDelayed(1).onTrue(outpostToShootingPoseToTower.cmd());

        outpostToShootingPoseToTower.active().whileTrue(limelight.idle());
        outpostToShootingPoseToTower.atTime(0.5).onTrue(
            Commands.parallel(
                shooter.spinUpCommand(2600),
                hood.positionCommand(0.32)
            )
        );
        outpostToShootingPoseToTower.done().onTrue(
            Commands.sequence(
                subsystemCommands.aimAndShoot()
                    .withTimeout(5),
                shootingPoseToTowerToClimb.cmd()
            )
        );

        shootingPoseToTowerToClimb.active().whileTrue(limelight.idle());
        shootingPoseToTowerToClimb.active().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
        shootingPoseToTowerToClimb.done().onTrue(hanger.positionCommand(Hanger.Position.HUNG));

        return routine;
    }

    private AutoRoutine centerToHubToClimb() {
        final AutoRoutine routine = autoFactory.newRoutine("Center to Shoot to Climb");
        final AutoTrajectory backUpToShoot = CenterToHubToClimb$0.asAutoTraj(routine);
        final AutoTrajectory flipToClimb = CenterToHubToClimb$1.asAutoTraj(routine);
        final AutoTrajectory climb = CenterToHubToClimb$2.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                backUpToShoot.resetOdometry(),
                backUpToShoot.cmd()
            )
        );

         backUpToShoot.done().onTrue(
            Commands.sequence(
                hanger.runOnce(() -> hanger.set(Position.HANGING)),
                Commands.waitSeconds(1.5),
                intake.runOnce(() -> intake.set(Intake.Position.INTAKE))
            )
        );

        backUpToShoot.atTime(5).onTrue(
            Commands.parallel(
                Commands.waitSeconds(2),
                shooter.spinUpCommand(3000),
                hood.positionCommand(0.35),
                 subsystemCommands.feed()
            )
        );

        backUpToShoot.doneDelayed(5).onTrue(
            Commands.sequence(
                flipToClimb.cmd()
            )
        );

        climb.active().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
        climb.done().onTrue(hanger.positionCommand(Hanger.Position.HUNG));

        return routine;


    }

    private AutoRoutine depotToHubTrajectory() {
        final AutoRoutine routine = autoFactory.newRoutine("Depot to Hub");
        final AutoTrajectory startToDepot = DepotToHubTrajectory$0.asAutoTraj(routine);
        final AutoTrajectory depotToHub = DepotToHubTrajectory$1.asAutoTraj(routine);
        final AutoTrajectory hubToClimb = DepotToHubTrajectory$2.asAutoTraj(routine);

         routine.active().onTrue(
            Commands.sequence(
                startToDepot.resetOdometry(),
                startToDepot.cmd()));

        routine.active().onTrue(
            Commands.sequence(
                Commands.waitSeconds(5),
                intake.runOnce(() -> intake.set(Intake.Position.INTAKE)))
                
            );



        startToDepot.atTimeBeforeEnd(3).onTrue(intake.intakeCommand());
        startToDepot.doneDelayed(0.1).onTrue(depotToHub.cmd());

        depotToHub.atTime(0.5).onTrue(
            Commands.parallel(
                shooter.spinUpCommand(3000),
                hood.positionCommand(0.35)
            )
        );

       

        depotToHub.done().onTrue(
            Commands.sequence(
                Commands.waitSeconds(2),
                hubToClimb.cmd()
            )
        );

        //  hubToClimb.atTime(1).onTrue(
        //     Commands.sequence(
        //         intake.runOnce(() -> intake.set(Intake.Position.STOWED))
        //     )  
        //  );

        // hubToClimb.active().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
        // Commands.waitSeconds(1);
        // hubToClimb.done().onTrue(hanger.positionCommand(Hanger.Position.HUNG));
        return routine;

    }

}


