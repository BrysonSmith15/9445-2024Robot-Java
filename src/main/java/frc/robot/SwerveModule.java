package frc.robot;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DrivetrainConstants;

import com.ctre.phoenix6.hardware.CANcoder;

public class SwerveModule {
    public SwerveModule(int driveMotorCANID, int turnMotorCANID, int turnEncoderCANID, boolean driveInverted,
            boolean turnInverted) {
        m_driveSparkMax = new CANSparkMax(turnMotorCANID, MotorType.kBrushless);
        m_turnSparkMax = new CANSparkMax(driveMotorCANID, MotorType.kBrushless);

        m_driveEncoder = m_driveSparkMax.getEncoder();
        m_turnEncoder = new CANcoder(turnEncoderCANID);

        m_turnPIDController = new PIDController(6e-1, 0.0, 0.0);

        m_turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
        m_driveEncoder.setPosition(0);
        driveRamper = new SlewRateLimiter(1.0, -3.0, 0.0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getVelocity(),
                new Rotation2d(m_turnEncoder.getAbsolutePosition().getValue()));
    }

    public void setIdleState(boolean coast) {
        m_driveSparkMax.setIdleMode(coast ? IdleMode.kCoast : IdleMode.kBrake);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveEncoder.getPosition(),
                new Rotation2d(m_turnEncoder.getAbsolutePosition().getValue()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedState = new SwerveModuleState();
        correctedState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        SwerveModuleState optimizedState = SwerveModuleState.optimize(correctedState,
                new Rotation2d(m_turnEncoder.getPosition().getValue()));

        double driveOut = driveRamper.calculate(optimizedState.speedMetersPerSecond / DrivetrainConstants.maxSpeedMPS);

        double turnOut = 0.0;
        if (Math.abs(optimizedState.angle.getRadians() - getTurnAngle()) < Math.PI / 32) {
            turnOut = m_turnPIDController.calculate(getTurnAngle(), optimizedState.angle.getRadians());
        } else {
            m_turnPIDController.calculate(getTurnAngle(), optimizedState.angle.getRadians());
        }

        driveOut = driveOut > 1.0 ? 1.0 : driveOut;
        driveOut = driveOut < -1.0 ? -1.0 : driveOut;

        turnOut = turnOut > 1.0 ? 1.0 : turnOut;
        turnOut = turnOut < -1.0 ? -1.0 : turnOut;

        m_driveSparkMax.set(driveOut);
        m_turnSparkMax.set(turnOut);
    }

    public double getTurnAngle() {
        return -m_turnEncoder.getAbsolutePosition().getValue();
    }

    private final CANSparkMax m_driveSparkMax;
    private final CANSparkMax m_turnSparkMax;

    private final CANcoder m_turnEncoder;
    private final RelativeEncoder m_driveEncoder;
    private final PIDController m_turnPIDController;
    private final SlewRateLimiter driveRamper;
}
