package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {

    public static final class AmpConstants {
        public static final int AMP_SWITCH_ONE_CHANNEL = 0;
        public static final int AMP_SWITCH_TWO_CHANNEL = 1;

        public static final int AMP_MOTOR_ONE_ID = 0;
        public static final int AMP_MOTOR_TWO_ID = 1;

        public static final double AMP_MOTOR_VOLTAGE = 0.0;
    }

    public static final class ClimberMotorConstants {
        public static final double CLIMBER_VOLTAGE = 0;
        public static final int CLIMBER_MOTOR_ONE_ID = 0;
        // second climb motor
        public static final int CLIMBER_MOTOR_TWO_ID = 0;
    }

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

    public static final class ShooterConstants {
        public static final int SHOOTER_MOTOR_ID = 16;

        public static final double SHOOTER_MOTOR_VOLTAGE = 0.5;
    }

    public static final class ControllerConstants {
        // drive controller button IDs
        public static final int BUTTON_A = 1;
        public static final int BUTTON_B = 2;
        public static final int BUTTON_X = 3;
        public static final int BUTTON_Y = 4;
        public static final int BUTTON_LB = 5;
        public static final int BUTTON_RB = 6;
        public static final int BUTTON_BACK = 7;
        public static final int BUTTON_START = 8;
        public static final int LEFT_STICK_PRESS_DOWN = 9;
        public static final int RIGHT_STICK_PRESS_DOWN = 10;

        // shooter is using A, X, and right trigger
        // Climb wants go use RB and LB

        // drive controller axis IDs
        public static final int LEFT_STICK_AXIS = 1;
        public static final int RIGHT_STICK_AXIS = 5;
        public static final int LEFT_TRIGGER_AXIS = 2;
        public static final int RIGHT_TRIGGER_AXIS = 3; 
    }
}
