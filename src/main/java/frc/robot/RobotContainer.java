



package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 
   
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) 
            .withDriveRequestType(DriveRequestType.Velocity); 

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public final CommandXboxController joystick = new CommandXboxController(0);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    public Pose2d target = new Pose2d();

    public RobotContainer() {;
        configureBindings();

        // AutoBuilder.configure(
        //     drivetrain::getRobotPose,
        //     drivetrain::resetKalman,
        //     drivetrain::gimmeSpeed,
        //     drivetrain::iwantItBack,
        //     new PPHolonomicDriveController(drivetrain.vrooom, drivetrain.skrrrt),
        //     ,
        //     null,
        //     null
        // );
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive
            .withVelocityX(-joystick.getLeftY() * MaxSpeed*0.85) 
            .withVelocityY(-joystick.getLeftX() * MaxSpeed *0.85) 
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) 
        ));

        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
