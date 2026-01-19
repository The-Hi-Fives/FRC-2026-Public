// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shooter extends Command {
  private final TalonFX m_shooter = new TalonFX(5);
  public Shooter() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

   public void enableFeeder(Feeder m_feeder) {
    System.out.println("running");
    double currentSpeed = getVelocity();
    System.out.println(currentSpeed);
    if(currentSpeed > 85) {
      System.out.println("working");
      m_feeder.feedFuel();
    }

  }

   public double getVelocity() {
    return m_shooter.getVelocity().getValueAsDouble();
  }

  public void periodic() {
    SmartDashboard.putNumber("shooter rpm", getVelocity()*60);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  public void shoot() {
     m_shooter.setControl(new VelocityDutyCycle(16000));
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
