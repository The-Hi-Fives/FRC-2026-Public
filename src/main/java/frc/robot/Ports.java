package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    // CAN Buses
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    public static final CANBus kCANivoreCANBus = new CANBus("main");

    // Talon FX IDs
    public static final int kIntakePivot = 13;
    public static final int kIntakeRollers = 14;
    public static final int kFloor = 15;
    public static final int kFeeder = 16;
    public static final int kShooterLeft = 17;
    public static final int kShooterMiddle = 18;
    public static final int kShooterRight = 19;
    public static final int kHanger = 20;

    // PWM Ports
    public static final int kHoodLeftServo = 3;
    public static final int kHoodRightServo = 4;
}
