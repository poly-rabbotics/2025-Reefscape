package frc.robot.subsystems.Algae;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;

import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import frc.robot.subsystems.Angle;
import frc.robot.systems.Algae.AlgaePose;

public class AlgaeIntake {
    private static final Angle PIVOT_STOW = new Angle().setDegrees(180.0);
    private static final Angle PIVOT_INTAKE = new Angle().setDegrees(0.0);
    private static final Angle PIVOT_HANDOFF = new Angle().setDegrees(115.0);
    private static final Angle PIVOT_PROCESSOR = new Angle().setDegrees(0.0);

    private final SparkMax pivotMotor;
    private final PIDController controller = new PIDController(0.035, 0.0, 0.0);

    private AlgaePose pose = AlgaePose.Stow;

    public AlgaeIntake(int pivotMotorId) {
        pivotMotor = new SparkMax(pivotMotorId, MotorType.kBrushless);
        pivotMotor.configure(
            new SparkMaxConfig()
                .inverted(true)
                .apply(new AbsoluteEncoderConfig()
                    .zeroOffset(355.0 / 360.0)
                    .positionConversionFactor(360.0)
                    .velocityConversionFactor(360.0)
                    .inverted(false)
                    .zeroCentered(true)
                )
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12.0),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

    /**
     * Run the algae intake, should be run periodically.
     */
    public void run() {
        double speed = controller.calculate(getAngle().degrees(), getSetAngle().degrees());
        pivotMotor.set(speed);
    }

    /**
     * Set the desired `AlgaePose`.
     */
    public void setPose(AlgaePose pose) {
        this.pose = pose;
    }
    
    /**
     * Get the set `AlgaePose` for the intake's pivot.
     */
    public AlgaePose getSetPose() {
        return pose;
    }

    /**
     * Get the set angle for the intake's pivot.
     */
    public Angle getSetAngle() {
        return getPoseAngle(pose);
    }

    /**
     * Get the current angle of the intake's pivot.
     */
    public Angle getAngle() {
        double degrees = pivotMotor.getAbsoluteEncoder().getPosition();

        if (degrees < -80.0) {
            degrees += 360.0;
        }
        
        return new Angle().setDegrees(degrees);
    }

    /**
     * Get the speed of the pivot motor.
     */
    public double getPivotSpeed() {
        return pivotMotor.get();
    }

    /**
     * Get the pivot's velocity.
     */
    public double getPivotVelocity() {
        return pivotMotor.getAbsoluteEncoder().getVelocity();
    }

    public void _setPivotSpeed(double speed) {
        pivotMotor.set(speed);
    }

    /**
     * Gets an angle for the given `AlgaePose`.
     */
    private static Angle getPoseAngle(AlgaePose pose) {
        switch (pose) {
            case Handoff:
                return PIVOT_HANDOFF;
            case Intake:
                return PIVOT_INTAKE;
            case Processor:
                return PIVOT_PROCESSOR;
            default:
            case Stow:
                return PIVOT_STOW;
        }
    }
}
