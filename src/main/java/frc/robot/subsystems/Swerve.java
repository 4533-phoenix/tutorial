package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Constants.SwerveConstants;

public final class Swerve extends SubsystemBase {
    private static Swerve swerve = null;

    private final SwerveModule frontLeftSwerveModule = new SwerveModule(
        0, 
        0, 
        false, 
        false, 
        0, 
        0, 
        false
    );

    private final SwerveModule frontRightSwerveModule = new SwerveModule(
        0, 
        0, 
        false, 
        false, 
        0, 
        0, 
        false
    );

    private final SwerveModule backLeftSwerveModule = new SwerveModule(
        0, 
        0, 
        false, 
        false, 
        0, 
        0, 
        false
    );

    private final SwerveModule backRightSwerveModule = new SwerveModule(
        0, 
        0, 
        false, 
        false, 
        0, 
        0, 
        false
    );

    private final SwerveModule[] swerveModules = new SwerveModule[]{
        frontLeftSwerveModule,
        frontRightSwerveModule,
        backLeftSwerveModule,
        backRightSwerveModule
    };

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private SwerveDrivePoseEstimator poseEstimator = null;

    private final HolonomicDriveController swerveController = new HolonomicDriveController(
        new PIDController(
            SwerveConstants.X_KP, 
            SwerveConstants.X_KI, 
            SwerveConstants.X_KD
        ), 
        new PIDController(
            SwerveConstants.Y_KP, 
            SwerveConstants.Y_KI, 
            SwerveConstants.Y_KD
        ), 
        new ProfiledPIDController(
            SwerveConstants.THETA_KP, 
            SwerveConstants.THETA_KI, 
            SwerveConstants.THETA_KD, 
            new Constraints(
                SwerveConstants.MAX_ROTATIONAL_VELOCITY, 
                SwerveConstants.MAX_ROTATIONAL_ACCELERATION
            )
        )
    );

    public static Swerve getInstance() {
        if (swerve == null) {
            swerve = new Swerve();
        }

        return swerve;
    }

    private Swerve() {
        swerveController.getThetaController().enableContinuousInput(-Math.PI, Math.PI);
    }

    public HolonomicDriveController getHolonomicDriveController() {
        return swerveController;
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void registerPoseEstimator(Pose2d initialPose) {
        poseEstimator = new SwerveDrivePoseEstimator(
            SwerveConstants.SWERVE_DRIVE_KINEMATICS, 
            Rotation2d.fromDegrees(-gyro.getYaw()), 
            new SwerveModulePosition[]{
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            }, 
            initialPose
        );
    }

    public Rotation2d getRobotAngle() {
        return Rotation2d.fromDegrees(-gyro.getYaw());
    }

    public Pose2d getRobotPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void drive() {
        Translation2d translation = new Translation2d(
            RobotContainer.getController().getLeftX(), 
            RobotContainer.getController().getLeftY()
        );
        double rotation = RobotContainer.getController().getRightX();

        double xVelocity = translation.getX() * SwerveConstants.MAX_VELOCITY;
        double yVelocity = translation.getY() * SwerveConstants.MAX_VELOCITY;
        double rotationalVelocity = rotation * SwerveConstants.MAX_ROTATIONAL_VELOCITY;

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xVelocity, 
            yVelocity, 
            rotationalVelocity, 
            Rotation2d.fromDegrees(-gyro.getYaw())
        );

        SwerveModuleState[] swerveModuleStates = SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        setSwerveModuleStates(swerveModuleStates);
    }

    public void setSwerveModuleStates(SwerveModuleState[] swerveModuleStates) {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setState(swerveModuleStates[i]);
        }
    }

    @Override
    public void periodic() {
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];

        for (int i = 0; i < 4; i++) {
            SwerveModule swerveModule = swerveModules[i];

            swerveModulePositions[i] = swerveModule.getSwerveModulePosition();
        }

        if (poseEstimator != null) {
            poseEstimator.update(getRobotAngle(), swerveModulePositions);
        }
    }
}
