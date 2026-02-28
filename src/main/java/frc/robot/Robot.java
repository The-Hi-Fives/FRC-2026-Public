// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.stream.IntStream;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.SolenoidSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
    

    private enum AnimationType {
        None,
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
    }

    private AnimationType m_anim0State = AnimationType.None;
    private AnimationType m_anim1State = AnimationType.None;

    private static final int kSlot0StartIdx = 8;
    private static final int kSlot0EndIdx = 37;

    private static final int kSlot1StartIdx = 38;
    private static final int kSlot1EndIdx = 67;

    private final SendableChooser<AnimationType> m_anim0Chooser = new SendableChooser<AnimationType>();
    private final SendableChooser<AnimationType> m_anim1Chooser = new SendableChooser<AnimationType>();

     private final CANdle m_candle = new CANdle(22, "rio");

    private final RGBWColor kGreen = new RGBWColor(0, 255, 0, 0);
    private final RGBWColor kRed = new RGBWColor(255, 0, 0, 0);
    private final RGBWColor kBlue = new RGBWColor(0, 0, 255, 0);

    

    private final SolidColor ledStatusColorGreen = new SolidColor(0, 399).withColor(kGreen);
    private final SolidColor ledStatusColorRed = new SolidColor(0,399).withColor(kRed);

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() { {
        Hood hood = new Hood();
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        SmartDashboard.putData(CommandScheduler.getInstance());
        RobotController.setBrownoutVoltage(Volts.of(6.1));
        SmartDashboard.putNumber("Current Hood Percent: ", hood.getCurrentPercent());

        /* Configure CANdle */
        var cfg = new CANdleConfiguration();
        /* set the LED strip type and brightness */
        cfg.LED.BrightnessScalar = 0.5;
        /* disable status LED when being controlled */
        cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

        m_candle.getConfigurator().apply(cfg);

        /* clear all previous animations */
        for (int i = 0; i < 8; ++i) {
            m_candle.setControl(new EmptyAnimation(i));
        }
        /* set the onboard LEDs to a solid color */

        /* add animations to chooser for slot 0 */
        m_anim0Chooser.setDefaultOption("Color Flow", AnimationType.ColorFlow);
        m_anim0Chooser.addOption("Rainbow", AnimationType.Rainbow);
        m_anim0Chooser.addOption("Twinkle", AnimationType.Twinkle);
        m_anim0Chooser.addOption("Twinkle Off", AnimationType.TwinkleOff);
        m_anim0Chooser.addOption("Fire", AnimationType.Fire);

        /* add animations to chooser for slot 1 */
        m_anim1Chooser.setDefaultOption("Larson", AnimationType.Larson);
        m_anim1Chooser.addOption("RGB Fade", AnimationType.RgbFade);
        m_anim1Chooser.addOption("Single Fade", AnimationType.SingleFade);
        m_anim1Chooser.addOption("Strobe", AnimationType.Strobe);
        m_anim1Chooser.addOption("Fire", AnimationType.Fire);

        SmartDashboard.putData("Animation 0", m_anim0Chooser);
        SmartDashboard.putData("Animation 1", m_anim1Chooser);
    }


   
    for (int i = 0; i < 8; ++i) {
            m_candle.setControl(new EmptyAnimation(i));

    }
}

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
         isHubActive();
        // m_candle.setControl(ledStatusColorRed);

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
        SmartDashboard.putString("gameData", gameData);
        SmartDashboard.putNumber("matchTime", matchTime);
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
                m_candle.setControl(ledStatusColorGreen);
            } else {
                m_candle.setControl(ledStatusColorRed);
            }
            // return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            if(!redInactiveFirst) {
                m_candle.setControl(ledStatusColorGreen);
            } else {
                m_candle.setControl(ledStatusColorRed);
            }
            // return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            if(redInactiveFirst) {
                m_candle.setControl(ledStatusColorGreen);
            } else {
                m_candle.setControl(ledStatusColorRed);
            }
            // return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            if(!redInactiveFirst) {
                m_candle.setControl(ledStatusColorGreen);
            } else {
                m_candle.setControl(ledStatusColorRed);
            }
            // return !shift1Active;
        } else {
            // End game, hub always active.
            return;
        }
    }

    @Override
    public void autonomousInit() {

    final var anim0Selection = m_anim0Chooser.getSelected();
        if (m_anim0State != anim0Selection) {
            m_anim0State = anim0Selection;

            switch (m_anim0State) {
                default:
                case Fire:
                    m_candle.setControl(
                        new FireAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                    );
                    break;
            }

        }

    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        switch (m_anim1State) {
            default:
            case SingleFade:
                    m_candle.setControl(
                        new SingleFadeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                            .withColor(kGreen)
                            .withColor(kBlue)
                    );
                    break;
        }
    }

}


