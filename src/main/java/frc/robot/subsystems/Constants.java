package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
    public static final class SwerveModuleConstants {
        public static final double DRIVE_MOTOR_KS = 0.0;
        public static final double DRIVE_MOTOR_KV = 0.0;

        public static final double DRIVE_MOTOR_KP = 0.0;
        public static final double DRIVE_MOTOR_KI = 0.0;
        public static final double DRIVE_MOTOR_KD = 0.0;

        public static final double STEER_MOTOR_KS = 0.0;
        public static final double STEER_MOTOR_KV = 0.0;

        public static final double STEER_MOTOR_KP = 0.0;
        public static final double STEER_MOTOR_KI = 0.0;
        public static final double STEER_MOTOR_KD = 0.0;

        public static final double DRIVE_MOTOR_MAX_VELOCITY = 0.0; // m/s
        public static final double DRIVE_MOTOR_MAX_ACCELERATION = 0.0; // m/s^2

        public static final double STEER_MOTOR_MAX_VELOCITY = 0.0; // rad/s
        public static final double STEER_MOTOR_MAX_ACCELERATION = 0.0; // rad/s^2
    }

    public static final class SwerveConstants {
        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d[]{
                new Translation2d(-1.0, 1.0), 
                new Translation2d(1.0, 1.0), 
                new Translation2d(-1.0, -1.0),
                new Translation2d(1.0, -1.0)
            }
        );

        public static final double X_KP = 0.0;
        public static final double X_KI = 0.0;
        public static final double X_KD = 0.0;

        public static final double Y_KP = 0.0;
        public static final double Y_KI = 0.0;
        public static final double Y_KD = 0.0;

        public static final double THETA_KP = 0.0;
        public static final double THETA_KI = 0.0;
        public static final double THETA_KD = 0.0;

        public static final double MAX_VELOCITY = 0.0; // m/s
        public static final double MAX_ACCELERATION = 0.0; // m/s^2

        public static final double MAX_ROTATIONAL_VELOCITY = 0.0; // rad/s
        public static final double MAX_ROTATIONAL_ACCELERATION = 0.0; // rad/s^2
    }

    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 0;

        public static final double INTAKE_MOTOR_VOLTAGE = 5.0;
    }
}
