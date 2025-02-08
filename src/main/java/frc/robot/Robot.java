// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.*;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public Pose2d pose = new Pose2d(0.89, -1.6, new Rotation2d(-36));
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 
  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 

    SmartDashboard.putData(CommandScheduler.getInstance());

    if(m_robotContainer.joystick.a().getAsBoolean()) {
      m_robotContainer.drivetrain.kalman.resetPose(new Pose2d());
    }


    SmartDashboard.putNumber("x", m_robotContainer.drivetrain.kalman.getEstimatedPosition().getX());
    SmartDashboard.putNumber("y", m_robotContainer.drivetrain.kalman.getEstimatedPosition().getY());
    SmartDashboard.putNumber("rot", m_robotContainer.drivetrain.kalman.getEstimatedPosition().getRotation().getDegrees());

    SmartDashboard.putNumber("target X", m_robotContainer.target.getX());
    SmartDashboard.putNumber("target Y", m_robotContainer.target.getY());        
    SmartDashboard.putNumber("target ROT", m_robotContainer.target.getRotation().getDegrees());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.drivetrain.kalman.resetPose(new Pose2d());
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
