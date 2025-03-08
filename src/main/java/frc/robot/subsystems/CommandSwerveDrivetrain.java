package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.studica.frc.AHRS;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; 
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
   
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
   
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

    public AHRS navx = new AHRS(AHRS.NavXComType.kUSB1);



    private boolean m_hasAppliedOperatorPerspective = false;
   
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    public SwerveDrivePoseEstimator kalman;


    

    public PIDController translationPID = new PIDController(2, 0, 0);
    public PIDController headingPID = new PIDController(0.045, 0, 0);


    public void resetKalman(Pose2d newPose) {
        kalman = new SwerveDrivePoseEstimator(super.getKinematics(), new Rotation2d(), super.getState().ModulePositions, newPose);
    }

    public Command alignToPose(Pose2d targetPose) {
        return applyRequest(() -> drive
            .withVelocityX((translationPID.calculate(getRobotPose().getX(), targetPose.getX()) * MaxSpeed) * multiplier)
            .withVelocityY((translationPID.calculate(getRobotPose().getY(), targetPose.getY()) * MaxSpeed) * multiplier)
            .withRotationalRate((headingPID.calculate(getRobotPose().getRotation().getDegrees(), targetPose.getRotation().getDegrees()) * MaxAngularRate))
        ).until(() -> getRobotPose().getTranslation().getDistance(targetPose.getTranslation()) < 0.05);
    }

    public float multiplier = 0.4f;
    public boolean seenTag = false;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
        .getStructTopic("Robot Pose", Pose2d.struct).publish();

        

    public Pose2d getRobotPose() {
        return kalman.getEstimatedPosition();
    }
     
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(4), null,state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
        new SysIdRoutine.Mechanism(output -> setControl(m_translationCharacterization.withVolts(output)), null, this)
    );

    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(7), null, state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
        new SysIdRoutine.Mechanism(volts -> setControl(m_steerCharacterization.withVolts(volts)),null, this)
    );
   
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(Math.PI / 6).per(Second), Volts.of(Math.PI),null, state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
        new SysIdRoutine.Mechanism(output -> {
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            }, null, this)
    );

    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);

        if (Utils.isSimulation()) {
            startSimThread();
        }
        resetKalman(new Pose2d());
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        resetKalman(new Pose2d());
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        resetKalman(new Pose2d());
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {  

        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        SmartDashboard.putBoolean("seen tag", seenTag);

        kalman.update(navx.getRotation2d(), super.getState().ModulePositions);


        


        // PoseEstimate frontEstimate = DriverStation.getAlliance().get() == Alliance.Red ? 
        //             LimelightHelpers.getBotPoseEstimate_wpiRed("limelight-front") : LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front");

      //  PoseEstimate backEstimate = DriverStation.getAlliance().get() == Alliance.Red ? 
        //            LimelightHelpers.getBotPoseEstimate_wpiRed("limelight-back") : LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-back");

        // if (frontEstimate.tagCount > 0) {
        //     Pose2d lala = new Pose2d(frontEstimate.pose.getX(), frontEstimate.pose.getY(), getPigeon2().getRotation2d());
        //     kalman.addVisionMeasurement(lala, frontEstimate.timestampSeconds);

        //     multiplier = 0.2f;
        //     seenTag = true;
        // }

        // SmartDashboard.putNumber("backestimatetagcount", backEstimate.tagCount);
                
        // if (backEstimate.tagCount > 0) {
        //     Pose2d lala = new Pose2d(backEstimate.pose.getX(), backEstimate.pose.getY(), getPigeon2().getRotation2d());
        //     kalman.addVisionMeasurement(lala, frontEstimate.timestampSeconds);

        //     SmartDashboard.putNumber("back tag X", backEstimate.pose.getX());
        //     SmartDashboard.putNumber("back tag Y", backEstimate.pose.getY());

        //     multiplier = 0.2f;
        //     seenTag = true;
        // }

        publisher.set(getRobotPose());

        SmartDashboard.updateValues();
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
           
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }



}
