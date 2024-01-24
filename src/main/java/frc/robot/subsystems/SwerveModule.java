package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.subsystems.Constants.SwerveModuleConstants;

public final class SwerveModule {
    private CANSparkMax driveMotor;
    private CANSparkMax steerMotor;

    private RelativeEncoder driveEncoder;
    private CANcoder steerEncoder;

    private double steerEncoderOffset;
    private boolean steerEncoderReversed;

    private SimpleMotorFeedforward driveFeedforward;
    private SimpleMotorFeedforward steerFeedforward;

    private PIDController drivePIDController;
    private ProfiledPIDController steerPIDController;

    public SwerveModule(int driveMotorID, int steerMotorID, boolean driveMotorReversed, boolean steerMotorReversed, 
        int steerEncoderID, double steerEncoderOffset, boolean steerEncoderReversed
    ) {
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        steerMotor = new CANSparkMax(steerMotorID, MotorType.kBrushless);
        
        driveEncoder = driveMotor.getEncoder();

        driveMotor.setInverted(driveMotorReversed);
        steerMotor.setInverted(steerMotorReversed);

        driveEncoder.setPositionConversionFactor(1.0);
        driveEncoder.setVelocityConversionFactor(1.0);

        steerEncoder = new CANcoder(steerEncoderID);

        this.steerEncoderOffset = steerEncoderOffset;
        this.steerEncoderReversed = steerEncoderReversed;

        driveFeedforward = new SimpleMotorFeedforward(
            SwerveModuleConstants.DRIVE_MOTOR_KS, 
            SwerveModuleConstants.DRIVE_MOTOR_KV
        );
        steerFeedforward = new SimpleMotorFeedforward(
            SwerveModuleConstants.STEER_MOTOR_KS, 
            SwerveModuleConstants.STEER_MOTOR_KV
        );
        
        drivePIDController = new PIDController(
            SwerveModuleConstants.DRIVE_MOTOR_KP, 
            SwerveModuleConstants.DRIVE_MOTOR_KI, 
            SwerveModuleConstants.DRIVE_MOTOR_KD
        );
        steerPIDController = new ProfiledPIDController(
            SwerveModuleConstants.STEER_MOTOR_KP, 
            SwerveModuleConstants.STEER_MOTOR_KI, 
            SwerveModuleConstants.STEER_MOTOR_KD, 
            new Constraints(
                SwerveModuleConstants.STEER_MOTOR_MAX_VELOCITY, 
                SwerveModuleConstants.STEER_MOTOR_MAX_ACCELERATION
            )
        );

        steerPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public double getSteerEncoderValue() {
        double angle = steerEncoder.getAbsolutePosition().getValueAsDouble();

        angle *= (Math.PI / 180.0);
        angle %= (2 * Math.PI);
        angle -= steerEncoderOffset;

        angle *= steerEncoderReversed ? -1.0 : 1.0;

        return angle;
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromRadians(getSteerEncoderValue()));
    }

    public void setState(SwerveModuleState state) {
        SwerveModuleState.optimize(state, Rotation2d.fromRadians(getSteerEncoderValue()));

        driveMotor.setVoltage(
            driveFeedforward.calculate(state.speedMetersPerSecond) 
            + drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond)
        );

        double steerPIDValue = steerPIDController.calculate(getSteerEncoderValue(), state.angle.getRadians());

        steerMotor.setVoltage(
            steerFeedforward.calculate(steerPIDController.getSetpoint().velocity)
            + steerPIDValue
        );
    }
}
