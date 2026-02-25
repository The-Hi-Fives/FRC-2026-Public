// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.stream.IntStream;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.SolenoidSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Limelight;


/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    private final RobotContainer m_robotContainer;
    
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {
        Hood hood = new Hood();
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        SmartDashboard.putData(CommandScheduler.getInstance());
        RobotController.setBrownoutVoltage(Volts.of(6.1));
        SmartDashboard.putNumber("Current Hood Percent: ", hood.getCurrentPercent());
    }
    
    private final CANdle shootingColoLight = new CANdle(22, "rio");

    private final RGBWColor statusGreen = new RGBWColor(0, 255, 0, 0);
    private final RGBWColor statusRed = new RGBWColor(255, 0, 0, 0);

    private final SolidColor ledStatusColorGreen = new SolidColor(0, 399).withColor(statusGreen);
    private final SolidColor ledStatusColorRed = new SolidColor(0,399).withColor(statusRed);

//    private LimelightHelpers.RawFiducial[] detectedTags;

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        // isHubActive();
        shootingColoLight.setControl(ledStatusColorRed);

        double distanceFromTag = 0.0;
        int[] validTags = {25, 26, 27, 20, 24};
        LimelightHelpers.RawFiducial[] detectedTags = LimelightHelpers.getRawFiducials("limelight");
        for (LimelightHelpers.RawFiducial detectedTag : detectedTags) {
            if(IntStream.of(validTags).anyMatch(x -> x == detectedTag.id))
            {
                //Distance from tag in inches
                distanceFromTag = detectedTag.distToCamera * 39.37;
                SmartDashboard.putNumber("Distance to Hub (inches)", distanceFromTag);
            }
        }


        


//        detectedTags = LimelightHelpers.getRawFiducials("limelight");
//
//        SmartDashboard.putString("detectedTags Size: ", String.valueOf(detectedTags.length));
//        //            SmartDashboard.putNumberArray("Detected IDs: ", )
//        for(int i = 0; i < detectedTags.length; i++)
//        {
//        for (LimelightHelpers.RawFiducial detectedTag : detectedTags) {
//            SmartDashboard.putNumber("Tag" + i + "# ", detectedTags[i].id);
//        }
    }

    public void isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
        if (gameData.isEmpty()) {
            return;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
            // If we have invalid game data, assume hub is active.
            return;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        // boolean shift1Active = switch (alliance.get()) {
        //     case Red -> !redInactiveFirst;
        //     case Blue -> redInactiveFirst;
        // };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return;
        } else if (matchTime > 105) {
            // Shift 1
            if(redInactiveFirst) {
                shootingColoLight.setControl(ledStatusColorGreen);
            } else {
                shootingColoLight.setControl(ledStatusColorRed);
            }
            // return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            if(!redInactiveFirst) {
                shootingColoLight.setControl(ledStatusColorGreen);
            } else {
                shootingColoLight.setControl(ledStatusColorRed);
            }
            // return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            if(redInactiveFirst) {
                shootingColoLight.setControl(ledStatusColorGreen);
            } else {
                shootingColoLight.setControl(ledStatusColorRed);
            }
            // return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            if(!redInactiveFirst) {
                shootingColoLight.setControl(ledStatusColorGreen);
            } else {
                shootingColoLight.setControl(ledStatusColorRed);
            }
            // return !shift1Active;
        } else {
            // End game, hub always active.
            return;
        }
    }
}
