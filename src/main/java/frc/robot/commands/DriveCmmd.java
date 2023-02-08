// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.ConstantsFolder.RobotConstants.Drive.*;

/** An example command that uses an example subsystem. */
public class DriveCmmd extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  private final DoubleSupplier m_fowDoubleSupplier;
  private final DoubleSupplier m_thetaDoubleSupplier;
  private final  Boolean m_turbo;
  private final SlewRateLimiter m_slewRateLimiter;
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCmmd(DriveSubsystem subsystem, DoubleSupplier f, DoubleSupplier t, Boolean turbo) {
    m_subsystem = subsystem;
    m_fowDoubleSupplier = f;
    m_thetaDoubleSupplier = t;
    m_turbo = turbo;
    m_slewRateLimiter = new SlewRateLimiter(3, -3, 0);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double m_foward = m_slewRateLimiter.calculate( m_fowDoubleSupplier.getAsDouble());
    double m_theta =  m_slewRateLimiter.calculate(  m_thetaDoubleSupplier.getAsDouble());
    
    if(m_turbo){
      m_subsystem.TurboJoystickDrive(m_foward, m_theta);
    }
    if(!m_turbo){
      m_subsystem.NormalDrive(m_foward, m_theta);
    }
    
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
