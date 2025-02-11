// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.*;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 
  private final RobotContainer m_robotContainer;




  public StructPublisher<Pose2d> publisher2 = NetworkTableInstance.getDefault()
        .getStructTopic("Target Pose", Pose2d.struct).publish();
  public Robot() {
    m_robotContainer = new RobotContainer();
    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 

    SmartDashboard.putData(CommandScheduler.getInstance());

    if(m_robotContainer.joystick.a().getAsBoolean()) {
      m_robotContainer.drivetrain.resetKalman(new Pose2d());
    }

    if(m_robotContainer.joystick.b().getAsBoolean()) {
      m_robotContainer.target = m_robotContainer.drivetrain.getRobotPose();
    } 

    if(m_robotContainer.joystick.y().getAsBoolean()) {
      m_robotContainer.drivetrain.seenTag = false;
      m_robotContainer.drivetrain.multiplier = 1;
    }


    m_robotContainer.joystick.x().onTrue(m_robotContainer.drivetrain.alignToPose(m_robotContainer.target).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    publisher2.set(m_robotContainer.target);

    // SmartDashboard.putNumber("x", m_robotContainer.drivetrain.kalman.getEstimatedPosition().getX());
    // SmartDashboard.putNumber("y", m_robotContainer.drivetrain.kalman.getEstimatedPosition().getY());
    // SmartDashboard.putNumber("rot", m_robotContainer.drivetrain.kalman.getEstimatedPosition().getRotation().getDegrees());

    // SmartDashboard.putNumber("target X", m_robotContainer.target.getX());
    // SmartDashboard.putNumber("target Y", m_robotContainer.target.getY());        
    // SmartDashboard.putNumber("target ROT", m_robotContainer.target.getRotation().getDegrees());

    // SmartDashboard.putBoolean("reached position", m_robotContainer.drivetrain.getRobotPose().getTranslation().getDistance(m_robotContainer.target.getTranslation()) < 0.05);
          

  
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
