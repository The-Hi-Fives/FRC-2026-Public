// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.generated.ChoreoTraj.*;

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
import frc.robot.subsystems.LEDs;
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
    private final Limelight limelightright;
    private final Limelight limelightleft;
    private final Limelight limelightbottom;

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
        Limelight limelightright,
        Limelight limelightleft,
        Limelight limelightbottom
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.floor = floor;
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.hanger = hanger;
        this.limelightright = limelightright;
        this.limelightleft = limelightleft;
        this.limelightbottom = limelightbottom;

        this.subsystemCommands = new SubsystemCommands(swerve, intake, floor, feeder, shooter, hood, hanger);

        this.autoFactory = swerve.createAutoFactory();
        this.autoChooser = new AutoChooser();
    }

    public void configure() {
        autoChooser.addRoutine("AZ -> NZ Right", this::allianceZoneToNeutralZoneRightSideRoutine);
        autoChooser.addRoutine("AZ -> NZ Left", this::allianceZoneToNeutralZoneLeftSideRoutine);
        autoChooser.addRoutine("Depot to Tower", this::depotToHubToTowerTrajectory);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    private AutoRoutine outpostAndDepotRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("Outpost and Depot");
        final AutoTrajectory startToOutpost = OutpostAndDepotTrajectory$0.asAutoTraj(routine);
        final AutoTrajectory outpostToDepot = OutpostAndDepotTrajectory$1.asAutoTraj(routine);
        final AutoTrajectory depotToShootingPose = OutpostAndDepotTrajectory$2.asAutoTraj(routine);
        final AutoTrajectory shootingPoseToTower = OutpostAndDepotTrajectory$3.asAutoTraj(routine);


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

        depotToShootingPose.active().whileTrue(limelightright.idle());
        depotToShootingPose.atTime(0.5).onTrue(
            Commands.parallel(
                shooter.spinUpCommand(3000),
                hood.positionCommand(0.35)
            )
        );
        depotToShootingPose.done().onTrue(
            Commands.sequence(
                subsystemCommands.aimAndShoot("driver")
                    .withTimeout(5),
                shootingPoseToTower.cmd()
            )
        );

        shootingPoseToTower.active().whileTrue(limelightright.idle());
        shootingPoseToTower.active().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
        shootingPoseToTower.done().onTrue(hanger.positionCommand(Hanger.Position.HUNG));

        return routine;
    }


    private AutoRoutine allianceZoneToNeutralZoneRightSideRoutine() {
         final AutoRoutine routine = autoFactory.newRoutine("AZ -> NZ Right");
         final AutoTrajectory nZToStartIntake = AZToNZRightSideTrajectory$0.asAutoTraj(routine);
         final AutoTrajectory startIntakeToFuel = AZToNZRightSideTrajectory$1.asAutoTraj(routine);
         final AutoTrajectory shoot = AZToNZRightSideTrajectory$2.asAutoTraj(routine);
         final AutoTrajectory backToNZ = AZToNZRightSideTrajectory$3.asAutoTraj(routine);

         routine.active().onTrue(
            Commands.sequence(
                Commands.runOnce(() -> nZToStartIntake.getInitialPose().ifPresent(pose -> swerve.initializeForAuto(pose))),
                nZToStartIntake.cmd()
            )
         );

        //  nZToStartIntake.doneDelayed(1).onTrue(startIntakeToFuel.cmd());
        nZToStartIntake.atTimeBeforeEnd(1).onTrue(intake.intakeCommand());
        nZToStartIntake.done().onTrue(startIntakeToFuel.cmd());
        //  startIntakeToFuel.atTimeBeforeEnd(.5).onTrue(intake.intakeCommand());
        startIntakeToFuel.done().onTrue(shoot.cmd());

        //  toFuelToShoot.done().onTrue(shoot.cmd());

        shoot.atTimeBeforeEnd(0.2).onTrue(Commands.sequence(
            subsystemCommands.aimAndShoot("driver").withTimeout(4),
            Commands.deadline(
                intake.intakeCommand(),
                backToNZ.cmd())
            ));

        // shoot.done().onTrue(backToNZ.cmd());

         backToNZ.atTimeBeforeEnd(0.2).onTrue(subsystemCommands.aimAndShoot("driver"));




        
        return routine;
    }

    private AutoRoutine allianceZoneToNeutralZoneLeftSideRoutine() {
         final AutoRoutine routine = autoFactory.newRoutine("AZ -> NZ Left");
         final AutoTrajectory nZToStartIntake = AZToNZLeftSideTrajectory$0.asAutoTraj(routine);
         final AutoTrajectory startIntakeToFuel = AZToNZLeftSideTrajectory$1.asAutoTraj(routine);
         final AutoTrajectory shoot = AZToNZLeftSideTrajectory$2.asAutoTraj(routine);
         final AutoTrajectory backToNZ = AZToNZLeftSideTrajectory$3.asAutoTraj(routine);

         routine.active().onTrue(
            Commands.sequence(
                Commands.runOnce(() -> nZToStartIntake.getInitialPose().ifPresent(pose -> swerve.initializeForAuto(pose))),
                nZToStartIntake.cmd()
            )
         );

        //  nZToStartIntake.doneDelayed(1).onTrue(startIntakeToFuel.cmd());
        nZToStartIntake.atTimeBeforeEnd(1).onTrue(intake.intakeCommand());
        nZToStartIntake.done().onTrue(startIntakeToFuel.cmd());
        //  startIntakeToFuel.atTimeBeforeEnd(.5).onTrue(intake.intakeCommand());
        startIntakeToFuel.done().onTrue(shoot.cmd());

        //  toFuelToShoot.done().onTrue(shoot.cmd());

        shoot.atTimeBeforeEnd(0.2).onTrue(Commands.sequence(
            subsystemCommands.aimAndShoot("driver").withTimeout(4),
            Commands.deadline(
                intake.intakeCommand(),
                backToNZ.cmd())
            ));

        // shoot.done().onTrue(backToNZ.cmd());

         backToNZ.atTimeBeforeEnd(0.2).onTrue(subsystemCommands.aimAndShoot("driver"));



        
        return routine;
    }

    private AutoRoutine depotToHubToTowerTrajectory() {
        final AutoRoutine routine = autoFactory.newRoutine("Depot and Tower");
        final AutoTrajectory toDepot = DepotAndTowerTrajectory$0.asAutoTraj(routine);
        final AutoTrajectory shoot = DepotAndTowerTrajectory$1.asAutoTraj(routine);
        final AutoTrajectory toHang = DepotAndTowerTrajectory$2.asAutoTraj(routine);
        final AutoTrajectory hang = DepotAndTowerTrajectory$3.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                Commands.runOnce(() -> toDepot.getInitialPose().ifPresent(pose -> swerve.initializeForAuto(pose))),
                toDepot.cmd()
            )
         );

        toDepot.atTimeBeforeEnd(0.2).onTrue(intake.intakeCommand());
        toDepot.doneDelayed(1).onTrue(shoot.cmd());
        // toDepot.done().onTrue(Commands.sequence(
        //         intake.intakeCommand().withTimeout(1),
        //         shoot.cmd())
        //     );

        shoot.atTimeBeforeEnd(0.2).onTrue(Commands.sequence(subsystemCommands.aimAndShoot("driver").withTimeout(5),
        toHang.cmd()));

            toHang.done().onTrue(hang.cmd());
            hang.done().onTrue(hanger.positionCommand(Position.HUNG));


         
        
        return routine;
    }

}


