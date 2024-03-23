package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private final SwerveRequest.RobotCentric driveRobotCentricNoDeadband = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);

        if (Utils.isSimulation()) {
            startSimThread();
        }


    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }


    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public void applyRequest1(Supplier<SwerveRequest> requestSupplier) {
        this.setControl(requestSupplier.get());
    }

    public void fromChassisSpeeds(ChassisSpeeds speeds) {
        // var states = m_kinematics.toSwerveModuleStates(speeds, new Translation2d());
        // for (int i = 0; i < this.Modules.length; i++) {
        //     this.Modules[i].apply(states[i], 
        //     SwerveModule.DriveRequestType.OpenLoopVoltage, SwerveModule.SteerRequestType.MotionMagic);
        // }
        applyRequest1(() -> driveRobotCentricNoDeadband.
        withVelocityX(speeds.vxMetersPerSecond).
        withVelocityY(speeds.vyMetersPerSecond).
        withRotationalRate(speeds.omegaRadiansPerSecond).
        withDriveRequestType(DriveRequestType.Velocity));
    }

    public void configurePathPlanner() {

        double driveBaseRadius = 0;
        for (var moduleLocatoin: m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocatoin.getNorm());
        }
        AutoBuilder.configureHolonomic(
            () -> this.getState().Pose, // Robot pose supplier
            (pose) -> seedFieldRelative(pose), // Method to reset odometry (will be called if your auto has a starting pose)
            this::getCurrentRobotChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds)  -> fromChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(10, 0.0, 0.0), // Translation PID constants
                new PIDConstants(10, 0.0, 0.0), // Rotation PID constants
                TunerConstants.kSpeedAt12VoltsMps, // Max module speed, in m/s
                driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig(true, false) // Default path replanning config. See the API for the options here
            ),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE ClimberSide

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
    );
    }

    public Command getAutoPath(String pathname, String autoname) {
        configurePathPlanner();
        //return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathname));
        seedFieldRelative(PathPlannerPath.fromPathFile(pathname).getPreviewStartingHolonomicPose());
        return new PathPlannerAuto(autoname);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
