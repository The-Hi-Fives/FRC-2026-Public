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
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Hood;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    private final RobotContainer m_robotContainer;

    public enum LEDState {
    START,
    DISABLED,
    DISABLED_LOW_BATTERY,
    AUTONOMOUS,
    ENABLED,

    // Extra states kept from your original list (not automatically used here)
    INTAKING,
    FEEDING,
    CLIMBING,
    SHOOTING,
  }

  private LEDState currentState = LEDState.START;

  // Optional: allow other code to force a state (e.g., from commands)
  private boolean overrideEnabled = false;
  private LEDState overrideState = LEDState.ENABLED;

  // === Colors ===
  private static final RGBWColor BLACK   = new RGBWColor(0,   0,   0,   0);
  private static final RGBWColor WHITE   = new RGBWColor(255, 255, 255, 0);
  private static final RGBWColor RED     = new RGBWColor(255, 0,   0,   0);
  private static final RGBWColor GREEN   = new RGBWColor(0,   255, 0,   0);
  private static final RGBWColor BLUE    = new RGBWColor(0,   0,   255, 0);
  private static final RGBWColor YELLOW  = new RGBWColor(255, 255, 0,   0);
  private static final RGBWColor CYAN    = new RGBWColor(0,   255, 240, 0);
  private static final RGBWColor BROWN   = new RGBWColor(166, 41,  41,  0);
  private static final RGBWColor PINK    = new RGBWColor(255, 60,  150, 0);
  private static final RGBWColor PURPLE  = new RGBWColor(170, 0,   255, 0);

  // === LED Segments (your original indices) ===
  // Note: CANdle onboard LEDs are 0-7. Strip LEDs are 8-399.
  private final LEDSegment candle = new LEDSegment(0,   7,   0); // 8 LEDs
  private final LEDSegment stripLeft = new LEDSegment(8, 32, 1); // 24 LEDs
  private final LEDSegment stripHood = new LEDSegment(33,  61,  2); // 28 LEDs
  private final LEDSegment stripRight = new LEDSegment(62,   86,  3); // 24 LEDs


  // === Segment helper ===
  private final class LEDSegment {
    final int start;     // inclusive
    final int end;       // inclusive
    final int slot;      // 0-7 (CANdle animation slot)

    LEDSegment(int startInclusive, int endInclusive, int slot) {
      this.start = startInclusive;
      this.end = endInclusive;
      this.slot = slot;
    }

    void clearSlot() {
      m_candle.setControl(new EmptyAnimation(start).withSlot(slot));
    }

    void setSolid(RGBWColor color) {
      clearSlot();
      m_candle.setControl(new SolidColor(start, end).withColor(color));
    }

    void setStrobe1Colors(RGBWColor color, double frameRateHz) {
      m_candle.setControl(
          new StrobeAnimation(start, end)
              .withSlot(slot)
              .withColor(color)
              .withFrameRate(frameRateHz));
    }

    void setStrobe2Colors(RGBWColor color, RGBWColor color2, double frameRateHz) {
      m_candle.setControl(
          new StrobeAnimation(start, end)
              .withSlot(slot)
              .withColor(color)
              .withColor(color2)
              .withFrameRate(frameRateHz));
    }

    void setColorFlow(RGBWColor color, double frameRateHz, AnimationDirectionValue dir) {
      m_candle.setControl(
          new ColorFlowAnimation(start, end)
              .withSlot(slot)
              .withColor(color)
              .withFrameRate(frameRateHz)
              .withDirection(dir));
    }

    void setLarson(RGBWColor color, double frameRateHz, LarsonBounceValue bounce, int size) {
      m_candle.setControl(
          new LarsonAnimation(start, end)
              .withSlot(slot)
              .withColor(color)
              .withFrameRate(frameRateHz)
              .withBounceMode(bounce)
              .withSize(size));
    }

    void setRainbow(double frameRateHz, boolean reverse) {
      m_candle.setControl(
          new RainbowAnimation(start, end)
              .withSlot(slot)
              .withFrameRate(frameRateHz)
              .withDirection(AnimationDirectionValue.Forward));
    }

    void setFire(double frameRateHz, boolean reverse) {
      m_candle.setControl(
          new FireAnimation(start, end)
              .withSlot(slot)
              .withFrameRate(frameRateHz)
              .withDirection(AnimationDirectionValue.Forward));
    }

    void off() {
      setSolid(BLACK);
    }
  }


    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {

         // Phoenix 6 CANdle config (same style as your Robot.java)
    CANdleConfiguration cfg = new CANdleConfiguration();
    cfg.LED.BrightnessScalar = 0.9;
    cfg.LED.StripType = StripTypeValue.GRB;

    m_candle.getConfigurator().apply(cfg);

    // Start with everything off
    fullClear();

        // /* Configure CANdle */
        // var cfg = new CANdleConfiguration();
        // /* set the LED strip type and brightness */
        // cfg.LED.StripType = StripTypeValue.GRB;
        // cfg.LED.BrightnessScalar = 0.5;
        // /* disable status LED when being controlled */
        // cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

        // m_candle.getConfigurator().apply(cfg);

        // /* clear all previous animations */
        // for (int i = 0; i < 8; ++i) {
        //     m_candle.setControl(new EmptyAnimation(i));
        // }

        Hood hood = new Hood();
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        SmartDashboard.putData(CommandScheduler.getInstance());
        RobotController.setBrownoutVoltage(Volts.of(6.1));
        // SmartDashboard.putNumber("Current Hood Percent: ", hood.getCurrentPercent());
    }

    public void fullClear() {
    // Clear all slots we used and turn all segments off
    for (int slot = 0; slot <= 7; slot++) {
      m_candle.setControl(new EmptyAnimation(0).withSlot(slot));
    }
    candle.off();
    stripHood.off();
    stripRight.off();
    stripLeft.off();
  }
    
    private final CANdle m_candle = new CANdle(22, "rio");

    private final RGBWColor statusGreen = new RGBWColor(0, 255, 0, 0);
    private final RGBWColor statusRed = new RGBWColor(255, 0, 0, 0);

    private final SolidColor ledStatusColorGreen = new SolidColor(8, 84).withColor(statusGreen);
    private final SolidColor ledStatusColorRed = new SolidColor(8, 84).withColor(statusRed);
    private final StrobeAnimation ledStatusFlashingRed = new StrobeAnimation(8, 84).withFrameRate(100).withColor(statusRed);
    private final StrobeAnimation ledStatusFlashingGreen = new StrobeAnimation(8, 84).withFrameRate(100).withColor(statusRed);


      private LEDState decideState() {
    if (overrideEnabled) {
      return overrideState;
    }

    if (DriverStation.isDisabled()) {
        stripHood.setStrobe2Colors(GREEN, BLUE, 8.0);
        stripLeft.setStrobe2Colors(GREEN, BLUE, 8.0);
        stripRight.setStrobe2Colors(GREEN, BLUE, 8.0);
    } else {
        isHubActive();
    }


    if (DriverStation.isAutonomousEnabled()) {
        candle.off();
        // Fire on verticals, white chassis/strip
        stripLeft.setFire(35.0, true);
        stripRight.setFire(35.0, false);
    } else {
        isHubActive();
    }

    if (DriverStation.isTeleopEnabled()) {
        candle.off();
        stripHood.setSolid(WHITE);
        stripLeft.setSolid(WHITE);
        stripRight.setSolid(WHITE);
    } else {
        isHubActive();
    }

    // Default while enabled in teleop
        return LEDState.START;
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
    LEDState newState = decideState();

    if (newState != currentState) {
      applyState(newState);
      currentState = newState;
    }



    // Default while enabled in teleop
   
       
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        isHubActive();
        // m_candle.setControl(ledStatusColorRed);

        double distanceFromTag = 0.0;
        int[] validTags = {25, 26, 27, 20, 24, 21, 19, 20, 18, 17, 31};
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

    private void applyState(LEDState state) {
    switch (state) {
      case DISABLED: {
        stripHood.setStrobe2Colors(GREEN, BLUE, 8.0);
        stripLeft.setStrobe2Colors(GREEN, BLUE, 8.0);
        stripRight.setStrobe2Colors(GREEN, BLUE, 8.0);
        break;
      }

      case DISABLED_LOW_BATTERY: {
        candle.off();
        // Fast brown strobes everywhere
        stripLeft.setStrobe1Colors(BROWN, 8.0);
        stripRight.setStrobe1Colors(BROWN, 8.0);
        stripHood.setStrobe1Colors(BROWN, 8.0);
        break;
      }

      case AUTONOMOUS: {
        candle.off();
        // Fire on verticals, white chassis/strip
        stripLeft.setFire(35.0, true);
        stripRight.setFire(35.0, false);
        break;
      }

      case ENABLED: {
        candle.off();
        stripHood.setSolid(WHITE);
        stripLeft.setSolid(WHITE);
        stripRight.setSolid(WHITE);
        break;
      }
      case START:
      default: {
        fullClear();
        break;
      }
    }
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

        if (DriverStation.isDisabled()) {

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
        // If we have no game data, assume hub is active early in teleop.
        if (gameData.isEmpty()) {
            m_candle.setControl(ledStatusColorGreen);
            return;
        }
        boolean weAreInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> weAreInactiveFirst = true;
            case 'B' -> weAreInactiveFirst = false;
            default -> {
            m_candle.setControl(ledStatusColorGreen);
            // If we have invalid game data, assume hub is active.
            return;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !weAreInactiveFirst;
            case Blue -> weAreInactiveFirst;
        };

        if (matchTime > 135) {
            m_candle.setControl(ledStatusColorGreen);
        } else if (matchTime > 130) {
        if (shift1Active) {
            m_candle.setControl(ledStatusFlashingGreen);
        } else {
            m_candle.setControl(ledStatusFlashingRed);
        }
        } else if (matchTime > 110) {
             // Shift 1
        if (shift1Active) {
            m_candle.setControl(ledStatusColorGreen);
        } else {
            m_candle.setControl(ledStatusColorRed);
        }
        } else if (matchTime > 105) {
        if (!shift1Active) {
            m_candle.setControl(ledStatusFlashingGreen);
        } else {
            m_candle.setControl(ledStatusFlashingRed);
        }
        } else if (matchTime > 85) {
        if (!shift1Active) {
            m_candle.setControl(ledStatusColorGreen);
        } else {
            m_candle.setControl(ledStatusColorRed);
        }
        } else if (matchTime > 80) {
            //Shift 2
        if (shift1Active) {
            m_candle.setControl(ledStatusFlashingGreen);
        } else {
            m_candle.setControl(ledStatusFlashingRed);
        }
        } else if (matchTime > 60) {
        if (shift1Active) {
            m_candle.setControl(ledStatusColorGreen);
        } else {
            m_candle.setControl(ledStatusColorRed);
        }
        } else if (matchTime > 55) {
            //Shift 3
        if (!shift1Active) {
            m_candle.setControl(ledStatusFlashingGreen);
        } else {
            m_candle.setControl(ledStatusFlashingRed);
        }
        } else if (matchTime > 35) {
        if (!shift1Active) {
            m_candle.setControl(ledStatusColorGreen);
        } else {
            m_candle.setControl(ledStatusColorRed);
        }
        } else if (matchTime > 30) {
            //Shift 4
        if (shift1Active) {
            m_candle.setControl(ledStatusFlashingGreen);
        } else {
            m_candle.setControl(ledStatusFlashingRed);
        }

        } else if (matchTime > 15) {
        if (shift1Active) {
            m_candle.setControl(ledStatusColorGreen);
        } else {
            m_candle.setControl(ledStatusColorRed);
        }

        } else if (matchTime > 5) {
        m_candle.setControl(ledStatusFlashingGreen);
        } else {
        m_candle.setControl(ledStatusColorGreen);
        }
    }
}
