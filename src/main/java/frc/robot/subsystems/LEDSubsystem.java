// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

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

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SubsystemCommands;

/**
 * Phoenix 6 CANdle LED subsystem.
 */
public class LEDSubsystem extends SubsystemBase {

  // === CANdle (matches your Robot.java) ===
  private static final int CANDLE_ID = 22;
  private static final String CANDLE_BUS = "rio";
    private static final Swerve swerve = new Swerve();
    private static final Intake intake = new Intake();
    private static final Floor floor = new Floor();
    private static final Feeder feeder = new Feeder();
    private static final Shooter shooter = new Shooter();
    private static final Hood hood = new Hood();
    private static final Hanger hanger = new Hanger();
    private final CANdle m_candle = new CANdle(CANDLE_ID, CANDLE_BUS);
   
    private final SubsystemCommands subsystemCommands = new SubsystemCommands(swerve, intake, floor, feeder, shooter, hood, hanger);




  // === Robot LED States (you can expand later) ===
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
  private final LEDSegment stripHood = new LEDSegment(32,  60,  2); // 28 LEDs
  private final LEDSegment stripRight = new LEDSegment(61,   85,  3); // 24 LEDs


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
              .withDirection(AnimationDirectionValue.Backward));
    }

    void off() {
      setSolid(BLACK);
    }
  }

  private RGBWColor getAllianceColor(RGBWColor ifUnknown) {
  Optional<Alliance> ally = DriverStation.getAlliance();
  if (ally.isPresent()) {
    if (ally.get() == Alliance.Blue) return BLUE;
    if (ally.get() == Alliance.Red) return RED;
  }
  return ifUnknown;
}

  public LEDSubsystem() {
    // Phoenix 6 CANdle config (same style as your Robot.java)
    CANdleConfiguration cfg = new CANdleConfiguration();
    cfg.LED.BrightnessScalar = 0.9;
    cfg.LED.StripType = StripTypeValue.GRB;

    m_candle.getConfigurator().apply(cfg);

    // Start with everything off
    fullClear();
  }

  @Override
  public void periodic() {
    LEDState newState = decideState();

    if (newState != currentState) {
      applyState(newState);
      currentState = newState;
    }

    if (currentState == LEDState.ENABLED) {
    stripHood.setSolid(getAllianceColor(WHITE));
}
  }

  private LEDState decideState() {
    if (overrideEnabled) {
      return overrideState;
    }

    if (DriverStation.isDisabled()) {
      return (RobotController.getBatteryVoltage() < 11.8)
          ? LEDState.DISABLED_LOW_BATTERY
          : LEDState.DISABLED;
    }


    if (DriverStation.isAutonomousEnabled()) {
      return LEDState.AUTONOMOUS;
    }

    // Default while enabled in teleop
    return LEDState.DISABLED;
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
        

        stripHood.setSolid(getAllianceColor(WHITE));
        break;
      }

      case ENABLED: {
        candle.off();
        stripHood.setSolid(WHITE);
        stripLeft.setSolid(WHITE);
        stripRight.setSolid(WHITE);
        break;
      }

      // --- Extra states (kept so you can call setOverrideState(...) from commands) ---
      case INTAKING: {
        candle.off();
        stripHood.setStrobe1Colors(WHITE, 6.0);
        stripRight.setStrobe1Colors(WHITE, 6.0);
        break;
      }

      case FEEDING: {
        candle.off();
        stripLeft.setLarson(PINK, 40.0, LarsonBounceValue.Back, 6);
        stripRight.setLarson(PINK, 40.0, LarsonBounceValue.Back, 6);
        stripHood.setSolid(PINK);
        break;
      }

      case CLIMBING: {
        candle.off();
        stripLeft.setFire(35.0, true);
        stripRight.setFire(35.0, false);
        break;
      }

      case START:
      default: {
        fullClear();
        break;
      }
    }
  }

  // === Public helpers ===

  /** 0.0 to 1.0 */
  public void setBrightness(double percent) {
    CANdleConfiguration cfg = new CANdleConfiguration();
    cfg.LED.BrightnessScalar = clamp(percent, 0.0, 1.0);
    m_candle.getConfigurator().apply(cfg);
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

  public LEDState getCurrentState() {
    return currentState;
  }

  /**
   * Force LEDs into a state from elsewhere (commands, RobotContainer, etc.)
   * Call clearOverride() to return to automatic state selection.
   */
  public void setOverrideState(LEDState state) {
    overrideEnabled = true;
    overrideState = state;
  }

  public void clearOverride() {
    overrideEnabled = false;
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }
}