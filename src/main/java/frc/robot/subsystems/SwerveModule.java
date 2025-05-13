// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

/** 
 * Class for managing and manipulating a swerve module. 
 */
public class SwerveModule {
    private static final double CONVERSION_FACTOR_ROTATION = 150 / 7;
    private static final double CONVERSION_FACTOR_MOVEMENT = 6.75 / (0.09525 * Math.PI);

    private static final double PID_ROTATION_P = 38.0; // Basically, request 20 volts per rotation of error.
    private static final double PID_ROTATION_I = 0.0;
    private static final double PID_ROTATION_D = 0.0;

    private static final double PID_PROPULSION_P = 8.0;
    private static final double PID_PROPULSION_I = 0.0;
    private static final double PID_PROPULSION_D = 0.025;

    private final TalonFX rotationMotor;   // Motor responsible for rotating the module.
    private final TalonFX propulsionMotor; // Motor responsible for creating movement in the module.
    private final CANcoder angularEncoder; // Cancoder tracking the angle of the module.

    private final Angle canCoderOffset;
    private SwerveModuleState desiredState;

    private boolean rockMode = false;

    public void run() {
        desiredState.optimize(new Rotation2d(rotationMotor.getPosition().getValueAsDouble() * Angle.TAU));
        rotationMotor.setPosition(getAngle().degrees() / 360.0);

        PositionVoltage rotationRequest = new PositionVoltage(desiredState.angle.getRotations());
        rotationMotor.setControl(rotationRequest);

        if (rockMode) {
            VelocityVoltage propulsionRequest = new VelocityVoltage(0.0);
            propulsionMotor.setControl(propulsionRequest);
        } else {
            VelocityVoltage propulsionRequest = new VelocityVoltage(desiredState.speedMetersPerSecond);
            propulsionMotor.setControl(propulsionRequest);
        }
    }
    
    public SwerveModule(
        int movementMotorID, 
        int rotationalMotorID, 
        int canCoderID, 
        Angle canCoderOffset
    ) {
        desiredState = new SwerveModuleState();
        this.canCoderOffset = canCoderOffset.clone();

        angularEncoder = new CANcoder(canCoderID);
        angularEncoder.getConfigurator().apply(new CANcoderConfiguration());

        // Rotation Motor
        rotationMotor = new TalonFX(rotationalMotorID);

        ClosedLoopGeneralConfigs rotationClosedLoopConfigs = new ClosedLoopGeneralConfigs();
        rotationClosedLoopConfigs.ContinuousWrap = true;
        
        TalonFXConfiguration rotationConfig = new TalonFXConfiguration()
            .withClosedLoopGeneral(rotationClosedLoopConfigs)
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(30.0)
                    .withSupplyCurrentLimitEnable(true)
            )
            .withFeedback(
                new FeedbackConfigs()
                    // .withFeedbackRemoteSensorID(canCoderID)
                    // .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                    .withSensorToMechanismRatio(CONVERSION_FACTOR_ROTATION)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(PID_ROTATION_P)
                    .withKI(PID_ROTATION_I)
                    .withKD(PID_ROTATION_D)
            );

        rotationMotor.getConfigurator().apply(rotationConfig);
        rotationMotor.setPosition(getAngle().degrees() / 360.0);

        // Movement Motor
        propulsionMotor = new TalonFX(movementMotorID);
        TalonFXConfiguration propulsionConfiguration = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(40.0)
                    .withSupplyCurrentLimitEnable(true)
            )
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(60.0)
                    .withPeakForwardTorqueCurrent(-60.0)
            )
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(CONVERSION_FACTOR_MOVEMENT)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(PID_PROPULSION_P)
                    .withKI(PID_PROPULSION_I)
                    .withKD(PID_PROPULSION_D)
            );
        
        propulsionMotor.getConfigurator().apply(propulsionConfiguration);
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
            propulsionMotor.getVelocity().getValueAsDouble(),
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
            propulsionMotor.getPosition().getValueAsDouble(), 
            new Rotation2d(getAngle().radians())
        );
    }

    /**
     * True if the module should be in rock mode.
     */
    public void setRockMode(boolean shouldHold) {
        rockMode = shouldHold;
    }

    /**
     * True if in rock mode.
     */
    public boolean inRockMode() {
        return rockMode;
    }

    /**
     * Gets distance traveled, should not be used for ablsolute distances as 
     * this function currently makes no guarantee as to the starting position
     * of the module. (This can be mitigated if you zero positions, but it will
     * interupt odometry).
     */
    public double getDistanceTraveled() {
        return propulsionMotor.getPosition().getValueAsDouble();
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
    public double getPropulsionMotorTemp() {
        return propulsionMotor.getDeviceTemp().getValueAsDouble();
    }

    /**
     * Gets the reported torque current, proportional to the motor's output torque, of the movement
     * motor. Value is in amps and may be negative, where sign indicates forward/backward.
     */
    public double getPropulsionTorqueCurrent() {
        return propulsionMotor.getTorqueCurrent().getValueAsDouble();
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
        return propulsionMotor.getMotorVoltage().getValueAsDouble();
    }

    /**
     * Gets the reported motor voltage of the rotation motor. Value is in volts, may be negative.
     */
    public double getRotationVoltage() {
        return rotationMotor.getMotorVoltage().getValueAsDouble();
    }
}
