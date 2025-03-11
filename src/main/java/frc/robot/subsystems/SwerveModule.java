// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;

/** 
 * Class for managing and manipulating a swerve module. 
 */
public class SwerveModule {
    private static final double CONVERSION_FACTOR_ROTATION = Math.toRadians(150 / 7);
    private static final double CONVERSION_FACTOR_MOVEMENT = 6.75 / (0.09525 * Math.PI);

    private static final double PID_P = 0.70;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0;

    private static final double ROCK_PID_P = 0.05;
    private static final double ROCK_PID_I = 0.0;
    private static final double ROCK_PID_D = 0.0;

    private final TalonFX rotationMotor;   // Motor responsible for rotating the module.
    private final TalonFX movementMotor;   // Motor responsible for creating movement in the module.
    private final CANcoder angularEncoder; // Cancoder tracking the angle of the module.

    private final PIDController rotationController;
    private final PIDController rockController;

    private final Angle canCoderOffset;

    private SwerveModuleState desiredState;

    // Set to NaN if not in rock mode, NaN does not equal itself by definition
    // (see some IEEE standard or something) and so this is how rock mode is 
    // checked.
    private double rockPos = Double.NaN;

    public void run() {
        desiredState.optimize(new Rotation2d(getAngle().radians()));

        double desiredAngle = ((desiredState.angle.getRadians() % Angle.TAU) + Angle.TAU) % Angle.TAU;
        double rotationSpeed = rotationController.calculate(getAngle().radians(), desiredAngle);
        double propulsionSpeed = Double.isNaN(rockPos)
            ? desiredState.speedMetersPerSecond
            : rockController.calculate(getDistanceTraveled(), rockPos);
    
        movementMotor.set(propulsionSpeed);
        rotationMotor.set(rotationSpeed);
    }
    
    public SwerveModule(
        int movementMotorID, 
        int rotationalMotorID, 
        int canCoderID, 
        Angle canCoderOffset
    ) {
        this.canCoderOffset = canCoderOffset.clone();

        angularEncoder = new CANcoder(canCoderID);
        angularEncoder.getConfigurator().apply(new CANcoderConfiguration());

        // Rotation Motor
        rotationMotor = new TalonFX(rotationalMotorID);
        TalonFXConfigurator rotationConfigurator = rotationMotor.getConfigurator();

        rotationConfigurator.apply(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
        );

        rotationConfigurator.apply(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(30.0)
                .withSupplyCurrentLimitEnable(true)
        );

        rotationConfigurator.apply(
            new FeedbackConfigs()
                .withFeedbackRemoteSensorID(canCoderID)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                .withSensorToMechanismRatio(CONVERSION_FACTOR_ROTATION)
        );

        rotationController = new PIDController(PID_P, PID_I, PID_D);
        rotationController.enableContinuousInput(0.0, Angle.TAU);
        rotationController.setTolerance(Angle.TAU / 360.0);

        // Movement Motor
        movementMotor = new TalonFX(movementMotorID);
        TalonFXConfigurator movementConfigurator = movementMotor.getConfigurator();

        movementConfigurator.apply(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
        );

        movementConfigurator.apply(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(40.0)
                .withSupplyCurrentLimitEnable(true)
        );

        movementConfigurator.apply(
            new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(70.0)
                .withPeakForwardTorqueCurrent(-70.0)
        );

        movementConfigurator.apply(
            new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(CONVERSION_FACTOR_MOVEMENT)
        );
        
        rockController = new PIDController(ROCK_PID_P, ROCK_PID_I, ROCK_PID_D);
        rockController.setTolerance(1);

        desiredState = new SwerveModuleState();
    }

    /**
     * Sets the desired module state for this module. This must be run 
     * repeatedly to continue PID calculations.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        while (desiredState.angle.getRadians() < 0.0) {
            desiredState.angle = new Rotation2d(desiredState.angle.getRadians() + Angle.TAU);
        }

        this.desiredState = desiredState;
    }

    /**
     * Gets the actual state of this module, as read and calculated from 
     * encoders.
     */
    public SwerveModuleState getActualState() {
        return new SwerveModuleState(
            movementMotor.getVelocity().getValueAsDouble(),
            new Rotation2d(getAngle().radians())
        );
    }

    /**
     * Gets the desired state of this module, as given previously.
     */
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    /**
     * Gets the position of the module. 
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            movementMotor.getPosition().getValueAsDouble(), 
            new Rotation2d(getAngle().radians())
        );
    }

    /**
     * True if the module should be in rock mode.
     */
    public void setRockMode(boolean shouldHold) {
        if (!shouldHold) {
            rockPos = Double.NaN;
        } else if (rockPos != rockPos) {
            rockPos = getDistanceTraveled();
        }
    }

    /**
     * True if in rock mode.
     */
    public boolean inRockMode() {
        return rockPos == rockPos;
    }

    /**
     * Gets distance traveled, should not be used for ablsolute distances as 
     * this function currently makes no guarantee as to the starting position
     * of the module. (This can be mitigated if you zero positions, but it will
     * interupt odometry).
     */
    public double getDistanceTraveled() {
        return movementMotor.getPosition().getValueAsDouble();
    }

    public Angle getAngle() {
        var radians = angularEncoder.getPosition().getValueAsDouble() * Angle.TAU;
        var offsetRadians = radians + canCoderOffset.radians();
        return new Angle().setRadians(((offsetRadians % Angle.TAU) + Angle.TAU) % Angle.TAU);
    }
    
    /**
     * Gets the reported tempurature of the rotation motor in celsius.
     */
    public double getRotationMotorTemp() {
        return rotationMotor.getDeviceTemp().getValueAsDouble();
    }

    /**
     * Gets the reported tempurature of the movement motor in celsius.
     */
    public double getMovementMotorTemp() {
        return movementMotor.getDeviceTemp().getValueAsDouble();
    }

    /**
     * Gets the reported torque current, proportional to the motor's output torque, of the movement
     * motor. Value is in amps and may be negative, where sign indicates forward/backward.
     */
    public double getMovementTorqueCurrent() {
        return movementMotor.getTorqueCurrent().getValueAsDouble();
    }

    /**
     * Gets the reported torque current, proportional to the motor's output torque, of the rotation
     * motor. Value is in amps and may be negative, where sign indicates forward/backward.
     */
    public double getRotationTorqueCurrent() {
        return rotationMotor.getTorqueCurrent().getValueAsDouble();
    }

    /**
     * Gets the reported motor voltage of the movement motor. Value is in volts, may be negative.
     */
    public double getMovementVoltage() {
        return movementMotor.getMotorVoltage().getValueAsDouble();
    }

    /**
     * Gets the reported motor voltage of the rotation motor. Value is in volts, may be negative.
     */
    public double getRotationVoltage() {
        return rotationMotor.getMotorVoltage().getValueAsDouble();
    }
}
