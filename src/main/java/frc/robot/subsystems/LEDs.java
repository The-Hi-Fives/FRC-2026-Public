package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.SubsystemCommands;

public class LEDs extends SubsystemBase {

  // ========================
  // STATES
  // ========================
  public enum LEDState {
    START,
    DISABLED,
    DISABLED_LOW_BATTERY,
    AUTONOMOUS,
    ENABLED,
    SHOOTING,
    INTAKING,
    FULL
  }

  // ========================
  // COLORS
  // ========================
  private static final RGBWColor BLACK   = new RGBWColor(0, 0, 0, 0);
  private static final RGBWColor WHITE   = new RGBWColor(255, 255, 255, 0);
  private static final RGBWColor RED     = new RGBWColor(255, 0, 0, 0);
  private static final RGBWColor GREEN   = new RGBWColor(0, 255, 0, 0);
  private static final RGBWColor BLUE    = new RGBWColor(0, 0, 255, 0);
  private static final RGBWColor PURPLE  = new RGBWColor(170, 0, 255, 0);
  private static final RGBWColor YELLOW  = new RGBWColor(255, 255, 0, 0);

  private final RGBWColor statusGreen = new RGBWColor(0, 255, 0, 0);
  private final RGBWColor statusRed = new RGBWColor(255, 0, 0, 0);

  private final SolidColor ledStatusColorGreen = new SolidColor(8, 102).withColor(statusGreen);
  private final SolidColor ledStatusColorRed = new SolidColor(8, 102).withColor(statusRed);


  private RGBWColor getAllianceColorVerticals() {
  var alliance = DriverStation.getAlliance();

  if (alliance.isPresent()) {
    switch (alliance.get()) {
      case Red:
        return RED;
      case Blue:
        return BLUE;
      default:
        return GREEN;
    }
  }

  return GREEN; // fallback if unknown
}

private RGBWColor getAllianceColorHood() {
  var alliance = DriverStation.getAlliance();

  if (alliance.isPresent()) {
    switch (alliance.get()) {
      case Red:
        return RED;
      case Blue:
        return BLUE;
      default:
        return BLUE;
    }
  }

  return BLUE; // fallback if unknown
}

  // ========================
  // HARDWARE
  // ========================
  private final CANdle m_candle = new CANdle(22, "rio");

  private final LEDSegment candle     = new LEDSegment(0, 7, 0);
  private final LEDSegment stripLeft  = new LEDSegment(8, 31, 1);
  private final LEDSegment stripHood  = new LEDSegment(32, 77, 2);
  private final LEDSegment stripRight = new LEDSegment(78, 101, 3);

  private LEDState currentState = null;

  // ========================
  // SEGMENT CLASS
  // ========================
  private final class LEDSegment {
    final int start;
    final int end;
    final int slot;

    LEDSegment(int start, int end, int slot) {
      this.start = start;
      this.end = end;
      this.slot = slot;
    }

    void clearSlot() {
      m_candle.setControl(new EmptyAnimation(start).withSlot(slot));
    }

    void setSolid(RGBWColor color) {
      clearSlot();
      m_candle.setControl(new SolidColor(start, end).withColor(color));
    }

    void setStrobe(RGBWColor color, double fps) {
      m_candle.setControl(
          new StrobeAnimation(start, end)
              .withSlot(slot)
              .withColor(color)
              .withFrameRate(fps));
    }

    void setLarson(RGBWColor color, double fps, LarsonBounceValue bounce, int size) {
      m_candle.setControl(
          new LarsonAnimation(start, end)
              .withSlot(slot)
              .withColor(color)
              .withFrameRate(fps)
              .withBounceMode(bounce)
              .withSize(size));
    }

    void setFire(double fps, boolean reverse) {
      m_candle.setControl(
          new FireAnimation(start, end)
              .withSlot(slot)
              .withFrameRate(fps)
              .withDirection(AnimationDirectionValue.Forward));
    }

    void off() {
      setSolid(BLACK);
    }
  }

  // ========================
  // INIT
  // ========================
  public LEDs() {
    CANdleConfiguration cfg = new CANdleConfiguration();
    cfg.LED.BrightnessScalar = 0.9;
    cfg.LED.StripType = StripTypeValue.GRB;

    m_candle.getConfigurator().apply(cfg);
    fullClear();
  }

  // ========================
  // CLEAR
  // ========================
  public void fullClear() {
    for (int slot = 0; slot <= 7; slot++) {
      m_candle.setControl(new EmptyAnimation(0).withSlot(slot));
    }

    candle.off();
    stripLeft.off();
    stripHood.off();
    stripRight.off();
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
            m_candle.setControl(ledStatusColorGreen);
        } else {
            m_candle.setControl(ledStatusColorRed);
        }   //Shift 1
        } else if (matchTime > 105) {
        if (!shift1Active) {
            m_candle.setControl(ledStatusColorGreen);
        } else {
            m_candle.setControl(ledStatusColorRed);
        }
        } else if (matchTime > 80) {
            //Shift 2
        if (shift1Active) {
            m_candle.setControl(ledStatusColorGreen);
        } else {
            m_candle.setControl(ledStatusColorRed);
        }
        } else if (matchTime > 55) {
            //Shift 3
        if (!shift1Active) {
            m_candle.setControl(ledStatusColorGreen);
        } else {
            m_candle.setControl(ledStatusColorRed);
        }
        } else if (matchTime > 30) {
            //Shift 4
        if (shift1Active) {
            m_candle.setControl(ledStatusColorGreen);
        } else {
            m_candle.setControl(ledStatusColorRed);
        }
        } else {
        m_candle.setControl(ledStatusColorGreen);
        }
    }

  // ========================
  // STATE DECIDER
  // ========================
  private LEDState decideState() {

    if (DriverStation.isDisabled()) {
      return LEDState.DISABLED;
    }

    if (DriverStation.isAutonomousEnabled()) {
      return LEDState.AUTONOMOUS;
    }

    if (DriverStation.isTeleopEnabled()) {

      if (SubsystemCommands.isFeeding) {
        return LEDState.SHOOTING;
      }
      if (Intake.intakeRunning) {
        return LEDState.INTAKING;
      }

      return LEDState.ENABLED;
    }

    return LEDState.START;
  }

  // ========================
  // APPLY STATE
  // ========================
  private void applyState(LEDState state) {
    if (state == currentState) return;

    fullClear();

    switch (state) {

      case DISABLED:
        stripLeft.setLarson(getAllianceColorVerticals(), 25, LarsonBounceValue.Front, 5);
        stripRight.setLarson(getAllianceColorVerticals(), 25, LarsonBounceValue.Front, 5);
        stripHood.setSolid(getAllianceColorHood());
        break;

      case AUTONOMOUS:
        stripLeft.setFire(35, true);
        stripRight.setFire(35, false);
        stripHood.setSolid(WHITE);
        break;

      case ENABLED:
        isHubActive();
        break;

      case SHOOTING:
        stripLeft.setStrobe(YELLOW, 30);
        stripRight.setStrobe(YELLOW, 30);
        stripHood.setStrobe(YELLOW, 30);
        break;

     case INTAKING:
        stripLeft.setStrobe(YELLOW, 10);
        stripRight.setStrobe(YELLOW, 10);
        stripHood.setStrobe(YELLOW, 10);
        break;

     case FULL:
        stripLeft.setStrobe(GREEN, 30);
        stripRight.setStrobe(GREEN, 30);
        stripHood.setStrobe(GREEN, 30);

      case START:
      default:
        fullClear();
        break;
    }

    currentState = state;
  }

  // ========================
  // PERIODIC
  // ========================
  @Override
  public void periodic() {
    LEDState newState = decideState();
    applyState(newState);
  }
}