// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import java.util.stream.IntStream;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Intake;

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

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        SmartDashboard.putData(CommandScheduler.getInstance());
        RobotController.setBrownoutVoltage(Volts.of(6.1));
    }


    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
    SmartDashboard.putNumber("Match Time: ", DriverStation.getMatchTime());
       
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        double distanceFromTag = 0.0;
        int[] validTags = {25, 26, 27, 20, 24, 21, 19, 20, 18, 17, 31, 8, 5, 4, 3, 2, 11, 10, 9, 1, 22, 1, 6};
        LimelightHelpers.RawFiducial[] detectedTags = LimelightHelpers.getRawFiducials("limelight");
        for (LimelightHelpers.RawFiducial detectedTag : detectedTags) {
            if(IntStream.of(validTags).anyMatch(x -> x == detectedTag.id))
            {
                //Distance from tag in inches
                distanceFromTag = detectedTag.distToCamera * 39.37;
                SmartDashboard.putNumber("Distance to Hub (inches)", distanceFromTag);
            }
        }


        double time = DriverStation.getMatchTime();
        String shiftName = "Unknown";
        double shiftTimeRemaning = 0;

        if(DriverStation.isAutonomous()) {
            shiftName = "AUTONOMOUS";
            shiftTimeRemaning = time;
        } else if (DriverStation.isTeleop()) {
            if (time > 130) {
                shiftName = "TRANSITION";
                shiftTimeRemaning = time - 130;
            } else if (time > 105) {
                shiftName = "SHIFT 1";
                shiftTimeRemaning = time - 105;
            } else if (time > 80) {
                shiftName = "SHIFT 2";
                shiftTimeRemaning = time - 80;
            } else if (time > 55) {
                shiftName = "SHIFT 3";
                shiftTimeRemaning = time - 55;
            } else if (time > 30) {
                shiftName = "SHIFT 4";
                shiftTimeRemaning = time - 30;
            } else {
                shiftName = "END GAME";
                shiftTimeRemaning = time;
            }
        }

        SmartDashboard.putString("Active Shift", shiftName);
        SmartDashboard.putNumber("Shift Countdown", Math.round(shiftTimeRemaning));
        SmartDashboard.putNumber("Battery %", Math.round(RobotController.getBatteryVoltage()));
       
    }

    @Override
    public void simulationPeriodic() {
        m_robotContainer.simulationPeriodic();
    }
}
