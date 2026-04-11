// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Amps;
// import static edu.wpi.first.units.Units.Degrees;
// import static edu.wpi.first.units.Units.RPM;
// import static edu.wpi.first.units.Units.RotationsPerSecond;
// import static edu.wpi.first.units.Units.Second;
// import static edu.wpi.first.units.Units.Volts;

// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.FeedbackConfigs;
// import com.ctre.phoenix6.configs.MotionMagicConfigs;
// import com.ctre.phoenix6.configs.MotorOutputConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.configs.VoltageConfigs;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.VelocityDutyCycle;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.units.measure.Voltage;
// import edu.wpi.first.util.sendable.SendableBuilder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.KrakenX60;
// import frc.robot.subsystems.Floor.Speed;
// import frc.robot.Ports;

// public class Intake extends SubsystemBase {

//     public enum Speed {
//         STOP(0);

//         private final double percentOutput;

//         private Speed(double percentOutput) {
//             this.percentOutput = percentOutput;
//         }

//         public Voltage voltage() {
//             return Volts.of(percentOutput * 12.0);
//         }

//     }

//     public enum Position {
//         HOMED(100),
//         STOWED(90),
//         INTAKE(-45),
//         AGITATE(-19),
//         CENTERTOCLIMB(60),
//         CAMERAANGLE(0);

//         private final double degrees;

//         private Position(double degrees) {
//             this.degrees = degrees;
//         }

//         public Angle angle() {
//             return Degrees.of(degrees);
//         }
//     }

//     private static final double kPivotReduction = 50.0;
//     private static final AngularVelocity kMaxPivotSpeed = KrakenX60.kFreeSpeed.div(kPivotReduction);
//     private static final Angle kPositionTolerance = Degrees.of(5);

//     private final TalonFX pivotMotor, rollerMotor;
//     private final VoltageOut pivotVoltageRequest = new VoltageOut(0);
//     private final VoltageOut rollerVoltageRequest = new VoltageOut(0);
//     private final MotionMagicVoltage pivotMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
//     private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

//     private boolean isHomed = false;

//     public Intake() {
//         pivotMotor = new TalonFX(Ports.kIntakePivot, Ports.kCANivoreCANBus);
//         rollerMotor = new TalonFX(Ports.kIntakeRollers, Ports.kCANivoreCANBus);
//         configurePivotMotor();
//         configureRollerMotor();
//         SmartDashboard.putData(this);
//     }

//     private void configurePivotMotor() {
//         final TalonFXConfiguration config = new TalonFXConfiguration()
//             .withMotorOutput(
//                 new MotorOutputConfigs()
//                     .withInverted(InvertedValue.CounterClockwise_Positive)
//                     .withNeutralMode(NeutralModeValue.Brake)
//             )
//             .withCurrentLimits(
//                 new CurrentLimitsConfigs()
//                     .withStatorCurrentLimit(Amps.of(120))
//                     .withStatorCurrentLimitEnable(true)
//                     .withSupplyCurrentLimit(Amps.of(70))
//                     .withSupplyCurrentLimitEnable(true)
//             )
//             .withFeedback(
//                 new FeedbackConfigs()
//                     .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
//                     .withSensorToMechanismRatio(kPivotReduction)
//             )
//             .withMotionMagic(
//                 new MotionMagicConfigs()
//                     .withMotionMagicCruiseVelocity(kMaxPivotSpeed)
//                     .withMotionMagicAcceleration(kMaxPivotSpeed.per(Second))
//             )
//             .withSlot0(
//                 new Slot0Configs()
//                     .withKP(300)
//                     .withKI(0)
//                     .withKD(0)
//                     .withKV(12.0 / kMaxPivotSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
//             );
//         pivotMotor.getConfigurator().apply(config);
//     }

//     private void configureRollerMotor() {
//         final TalonFXConfiguration config = new TalonFXConfiguration()
//             .withMotorOutput(
//                 new MotorOutputConfigs()
//                     .withInverted(InvertedValue.CounterClockwise_Positive)
//                     .withNeutralMode(NeutralModeValue.Brake)
//             )
//             .withVoltage(
//                 new VoltageConfigs()
//                     .withPeakReverseVoltage(Volts.of(0))
//             )
//             .withCurrentLimits(
//                 new CurrentLimitsConfigs()
//                     .withStatorCurrentLimit(Amps.of(120))
//                     .withStatorCurrentLimitEnable(true)
//                     .withSupplyCurrentLimit(Amps.of(70))
//                     .withSupplyCurrentLimitEnable(true)
//             )
//              .withSlot0(
//                 new Slot0Configs()
//                     .withKP(0.5)
//                     .withKI(0)
//                     .withKD(0)
//                     .withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
//             );
//         rollerMotor.getConfigurator().apply(config);
//     }

    
//     public void setRPM(double rpm) {
//             rollerMotor.setControl(
//                 velocityRequest
//                     .withVelocity(RPM.of(rpm))
//             );
//         }

//     public void stop() {
//         // setRPM(0);
//         rollerMotor.setControl(new VoltageOut(0));
//         intakerunning = false;
//     }

//     private boolean isPositionWithinTolerance() {
//         final Angle currentPosition = pivotMotor.getPosition().getValue();
//         final Angle targetPosition = pivotMotionMagicRequest.getPositionMeasure();
//         return currentPosition.isNear(targetPosition, kPositionTolerance);
//     }

//     private void setPivotPercentOutput(double percentOutput) {
//         pivotMotor.setControl(
//             pivotVoltageRequest
//                 .withOutput(Volts.of(percentOutput * 12.0))
//         );
//     }

//     public void set(Position position) {
//         pivotMotor.setControl(
//             pivotMotionMagicRequest
//                 .withPosition(position.angle())
//         );
//     }

//     public void set(Speed speed) {
//         rollerMotor.setControl(rollerVoltageRequest.withOutput(speed.voltage()));
//     }

//     public Command cameraIntakePosition() {
//         return startEnd(
//             () -> {
//                 set(Position.CAMERAANGLE);
//             },
//             () ->set(Speed.STOP)
//         );
//     }
//     public static boolean intakerunning = false;
//      public Command intakeCommand() {
//         return startEnd(
//             () -> {
//                 set(Position.INTAKE);
                
//             },
//             () -> set(Speed.STOP)
//         );
//     }

//     public Command agitateCommand() {
//         return runOnce(() -> set(Speed.INTAKE))
//             .andThen(
//                 Commands.sequence(
//                     runOnce(() -> set(Position.AGITATE)),
//                     Commands.waitUntil(this::isPositionWithinTolerance),
//                     runOnce(() -> set(Position.INTAKE)),
//                     Commands.waitUntil(this::isPositionWithinTolerance)
//                 )
//                 .repeatedly()
//             )
//             .handleInterrupt(() -> {
//                 set(Position.INTAKE);
//                 set(Speed.STOP);
//             });
//     }

//     public Command reverseIntakeCommand() {
//         return startEnd(
//             () -> {
//                 setRPM(-2000);
//             },
//             () -> stop()
//         );
//     }

//     public Command agitateCommand() {
//         return runOnce(() -> setRPM(6000))
//             .andThen(
//                 Commands.sequence(
//                     runOnce(() -> set(Position.AGITATE)),
//                     Commands.waitUntil(this::isPositionWithinTolerance),
//                     runOnce(() -> set(Position.INTAKE)),
//                     Commands.waitUntil(this::isPositionWithinTolerance)
//                 )
//                 .repeatedly()
//             )
//             .handleInterrupt(() -> {
//                 set(Position.INTAKE);
//                 stop();
//             });
//     }

//     public Command homingCommand() {
//         return Commands.sequence(
//             runOnce(() -> setPivotPercentOutput(0.1)),
//             Commands.waitUntil(() -> pivotMotor.getSupplyCurrent().getValue().in(Amps) > 6),
//             runOnce(() -> {
//                 pivotMotor.setPosition(Position.HOMED.angle());
//                 isHomed = true;
//                 set(Position.STOWED);
//             })
//         )
//         .unless(() -> isHomed)
//         .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
//     }

//     @Override
//     public void initSendable(SendableBuilder builder) {
//         builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
//         builder.addDoubleProperty("Angle (degrees)", () -> pivotMotor.getPosition().getValue().in(Degrees), null);
//         builder.addDoubleProperty("RPM", () -> rollerMotor.getVelocity().getValue().in(RPM), null);
//         builder.addDoubleProperty("Pivot Supply Current", () -> pivotMotor.getSupplyCurrent().getValue().in(Amps), null);
//         builder.addDoubleProperty("Roller Supply Current", () -> rollerMotor.getSupplyCurrent().getValue().in(Amps), null);
//     }

//     public void setPosition(double angle) {
//         pivotMotor.setPosition(Degrees.of(angle));
//     }

//     public double getPosition() {
//         return pivotMotor.getPosition().getValue().in(Degrees);
//     }
// }
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.TorqueUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;

public class Intake extends SubsystemBase {

private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

    public enum Speed {
        STOP(0),
        INTAKE(0.8),
        REVERSEINTAKE(-0.8);

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            if (percentOutput == 0) {
                intakeRunning = false;
            } else {
                intakeRunning = true;
            }
            return Volts.of(percentOutput * 12.0);
        }
    }


    // public enum Current{
    //     FULL(60),
    //     EMPTY(9);


    //     private final double current;

    //     private Current(double current) {
    //         this.current = current;
    //     }

    //     public edu.wpi.first.units.measure.Current getCurrent() {
    //         if (current >= 40) {
    //             hopperIsFull = false;
    //         } else {
    //             hopperIsFull = true;
    //         }
    //         return Amps.of(current * 12.0);
    //     }
    // }


    public enum Position {
       HOMED(100),
        STOWED(90),
        INTAKE(-45),
        AGITATE(-15),
        CENTERTOCLIMB(60),
        CAMERAANGLE(0);

        private final double degrees;

        private Position(double degrees) {
            this.degrees = degrees;
        }

        public Angle angle() {
            return Degrees.of(degrees);
        }
    }

    private static final double kPivotReduction = 50.0;
    private static final AngularVelocity kMaxPivotSpeed = KrakenX60.kFreeSpeed.div(kPivotReduction);
    private static final Angle kPositionTolerance = Degrees.of(5);

    private final TalonFX pivotMotor, rollerMotor;
    private final VoltageOut pivotVoltageRequest = new VoltageOut(0);
    private final MotionMagicVoltage pivotMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0);
    // private final TorqueCurrentFOC rollerCurrentRequest = new TorqueCurrentFOC(0);

    private boolean isHomed = false;
    public static boolean intakeRunning = false;
    
    public Intake() {
        pivotMotor = new TalonFX(Ports.kIntakePivot, Ports.kCANivoreCANBus);
        rollerMotor = new TalonFX(Ports.kIntakeRollers, Ports.kCANivoreCANBus);
        configurePivotMotor();
        configureRollerMotor();
        SmartDashboard.putData(this);
    }

    public void setRPM(double rpm) {
            rollerMotor.setControl(
                velocityRequest
                    .withVelocity(RPM.of(rpm))
            );
        }

    private void configurePivotMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(kPivotReduction)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(kMaxPivotSpeed)
                    .withMotionMagicAcceleration(kMaxPivotSpeed.per(Second))
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.5)
                    .withKI(2)
                    .withKD(0)
                    .withKV(12.0 / kMaxPivotSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );
        pivotMotor.getConfigurator().apply(config);
    }

    private void configureRollerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(80))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            );
        rollerMotor.getConfigurator().apply(config);
    }

    private void configureVerlocityVoltageRoller() {
         final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(150))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(kPivotReduction)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(kMaxPivotSpeed)
                    .withMotionMagicAcceleration(kMaxPivotSpeed.per(Second))
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(300)
                    .withKI(0)
                    .withKD(0)
                    .withKV(12.0 / kMaxPivotSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );
        rollerMotor.getConfigurator().apply(config);
    }

    private boolean isPositionWithinTolerance() {
        final Angle currentPosition = pivotMotor.getPosition().getValue();
        final Angle targetPosition = pivotMotionMagicRequest.getPositionMeasure();
        return currentPosition.isNear(targetPosition, kPositionTolerance);
    }

    private void setPivotPercentOutput(double percentOutput) {
        pivotMotor.setControl(
            pivotVoltageRequest
                .withOutput(Volts.of(percentOutput * 12.0))
        );
    }

    public void set(Position position) {
        pivotMotor.setControl(
            pivotMotionMagicRequest
                .withPosition(position.angle())
        );
    }

    public void set(Speed speed) {
        SmartDashboard.putNumber("roller speed", speed.voltage().in(Volts));
        rollerMotor.setControl(
            rollerVoltageRequest
                .withOutput(speed.voltage())
        );
    }

    // public void Current(Current current) {
    //     SmartDashboard.putNumber("roller current", current.getCurrent().in(Amps));
    //     rollerMotor.getTorqueCurrent(true);
    // }

    public Command intakeCommand() {
        return startEnd(
            () -> {
                set(Speed.INTAKE);
                set(Position.INTAKE);
            },
            () -> set(Speed.STOP)

        );
    }

    public Command intaketeleopCommand() {
        return startEnd(
            () -> {
                setRPM(6000);
                set(Position.INTAKE);
            },
            () -> setRPM(0)

        );

    }

        

    public Command intakeRollers() {
         return startEnd(
            () -> {
                set(Speed.INTAKE);
            },
            () -> set(Speed.STOP)

        );
    }

    public Command reverseIntakeCommand() {
        return startEnd(
            () -> {
                set(Speed.REVERSEINTAKE);
            },
            () -> set(Speed.STOP)
        );
    }


    public Command agitateCommand() {
        return runOnce(() -> set(Speed.INTAKE))
            .andThen(
                Commands.sequence(
                    runOnce(() -> set(Position.AGITATE)),
                    Commands.waitUntil(this::isPositionWithinTolerance),
                    runOnce(() -> set(Position.INTAKE)),
                    Commands.waitUntil(this::isPositionWithinTolerance)
                )
                .repeatedly()
            )
            .handleInterrupt(() -> {
                set(Position.INTAKE);
                set(Speed.STOP);
            });
    }

    public Command homingCommand() {
        return Commands.sequence(
            runOnce(() -> setPivotPercentOutput(0.1)),
            Commands.waitUntil(() -> pivotMotor.getSupplyCurrent().getValue().in(Amps) > 6),
            runOnce(() -> {
                pivotMotor.setPosition(Position.HOMED.angle());
                isHomed = true;
                set(Position.STOWED);
            })
        )
        .unless(() -> isHomed)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    // public boolean hopperIsFull() {
    //     double current = rollerMotor.getSupplyCurrent().getValue().in(Amps);

    //     if (current > 1) {
    //         return hopperIsFull = true;
    //     } else {
    //         return hopperIsFull = false;
    //     }
    // }
    

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Angle (degrees)", () -> pivotMotor.getPosition().getValue().in(Degrees), null);
        builder.addDoubleProperty("RPM", () -> rollerMotor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty("Pivot Supply Current", () -> pivotMotor.getSupplyCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty("Roller Supply Current", () -> rollerMotor.getSupplyCurrent().getValue().in(Amps), null);
    }
}
