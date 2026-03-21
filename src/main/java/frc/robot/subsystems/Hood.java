package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Ports;

public class Hood extends SubsystemBase {

  /* ==================== USER TUNING ==================== */

  // Percent range (kept from your servo version)
  private static final double kMinPercent = 0.00; //0.01
  private static final double kMaxPercent = 0.52; //0.77
  private static final double kPercentTolerance = 0.01; //0.01

  // Hood mechanical range (MECHANISM rotations)
  // Example: 90° hood travel = 0.25 rotations
  private static final double kMinMechRot = 0.0; //0.0
  private static final double kMaxMechRot = 0.0625; //0.25

  // Gear ratio: motor rotations per hood rotation
  // CHANGE THIS to match your real gearing
  private static final double kMotorRotsPerMechRot = 60.07; //100.0

  // PID (start conservative)
  private static final double kP = 15.0; //40.0
  private static final double kI = 0.0; //0.0
  private static final double kD = 0; //0.0

  /* ===================================================== */

  private final TalonFX hoodMotor;
  private final PositionVoltage positionRequest = new PositionVoltage(0); //0
  private final VoltageOut hoodVoltageRequest = new VoltageOut(0);

  

  private double targetPercent = 0.5; //0.5
  private double currentPercent = 0.0; 
  private boolean isHomed = false;


  private final StatusSignal<Angle> motorPosition;

  public Hood() {
    hoodMotor = new TalonFX(Ports.kHoodKrakenId, Ports.kCANivoreCANBus);
    // hoodMotor.setPosition(0.0);

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    cfg.Slot0.kP = kP;
    cfg.Slot0.kI = kI;
    cfg.Slot0.kD = kD;

    hoodMotor.getConfigurator().apply(cfg);

    motorPosition = hoodMotor.getPosition();

    setPercent(targetPercent);
    SmartDashboard.putData(this);
  }

  /** Set hood position as a percent [0.0 – 1.0] */
  public void setPercent(double percent) {
    targetPercent = MathUtil.interpolate(kMinPercent, kMaxPercent, percent);
    currentPercent = percent;

    double mechRot = MathUtil.interpolate(kMinMechRot, kMaxMechRot, targetPercent);
    double motorRot = mechRot * kMotorRotsPerMechRot;

    hoodMotor.setControl(positionRequest.withPosition(motorRot));
  }

  /** Backwards-compatible name with the old servo version */
  public void setPosition(double position) {
    setPercent(position);
  }

  public Command percentCommand(double percent) {
    return runOnce(() -> setPercent(percent))
        .andThen(Commands.waitUntil(this::isWithinTolerance));
  }

  /** Backwards-compatible name with the old servo version */
  public Command positionCommand(double position) {
    return percentCommand(position);
  }

  public boolean isWithinTolerance() {
    // return MathUtil.isNear(targetPercent, getCurrentPercent(), kPercentTolerance);
    return true;
  }

  public double getCurrentPercent() {
    return currentPercent;
    // double motorRot = motorPosition.getValue().in(Units.Rotations);
    // double mechRot = motorRot / kMotorRotsPerMechRot;

    // // Convert mechRot back to percent (and clamp for safety)
    // double percent = (mechRot - kMinMechRot) / (kMaxMechRot - kMinMechRot);
    // return MathUtil.clamp(percent, 0.0, 1.0); //0.0, 1.0
  }

  @Override
  public void periodic() {
    motorPosition.refresh();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty(
        "Command",
        () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null",
        null);

    builder.addDoubleProperty("Current Percent", this::getCurrentPercent, null);
    builder.addDoubleProperty("Target Percent", () -> targetPercent, this::setPercent);

    builder.addDoubleProperty(
        "Motor Position (rot)",
        () -> motorPosition.getValue().in(Units.Rotations),
        null);
  }

  public Command homingCommand() {
        // hoodMotor.setPosition(0);
        return Commands.sequence(
            // run(() -> SmartDashboard.putNumber("hood Stall Current", hoodMotor.getSupplyCurrent().getValue().in(Amps))),
            runOnce(() -> setHoodPercentOutput(-0.1)),
            Commands.waitUntil(() -> getHoodCurrent()),
            runOnce(() -> {
                hoodMotor.setPosition(0);
                setPercent(.01);
                isHomed = true;
            })
        )
        .unless(() -> isHomed)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    private Boolean getHoodCurrent()
    {
      SmartDashboard.putNumber("hood Stall Current", hoodMotor.getSupplyCurrent().getValue().in(Amps));
      if (hoodMotor.getSupplyCurrent().getValue().in(Amps) > 2)
      {
        return true;
      }
      return false;
    }

    private void setHoodPercentOutput(double percentOutput) {
        hoodMotor.setControl(
            hoodVoltageRequest
                .withOutput(Volts.of(percentOutput * 12.0))
        );
    }
}
