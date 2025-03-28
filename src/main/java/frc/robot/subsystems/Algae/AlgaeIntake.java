package frc.robot.subsystems.Algae;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;

import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import frc.robot.subsystems.Angle;
import frc.robot.systems.Algae.AlgaePose;

public class AlgaeIntake {
    private static final Angle PIVOT_STOW = new Angle().setDegrees(90.0);
    private static final Angle PIVOT_INTAKE = new Angle().setDegrees(-25.0);
    private static final Angle PIVOT_HANDOFF = new Angle().setDegrees(90.0);
    private static final Angle PIVOT_PROCESSOR = new Angle().setDegrees(45.0);

    private final SparkMax pivotMotor;
    private final PIDController controller = new PIDController(0.0075, 0.0, 0.0);

    private final SparkMax frontMotor;
    private final SparkMax backMotor;

    private AlgaePose pose = AlgaePose.Stow;

    public AlgaeIntake(int frontId, int backId, int pivotMotorId) {
        pivotMotor = new SparkMax(pivotMotorId, MotorType.kBrushless);
        pivotMotor.configure(
            new SparkMaxConfig()
                .inverted(true)
                .apply(new AbsoluteEncoderConfig()
                    // .zeroOffset(0.46)
                    .positionConversionFactor(360.0)
                    .velocityConversionFactor(360.0)
                    .inverted(true)
                    .zeroCentered(true)
                ),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        frontMotor = new SparkMax(frontId, MotorType.kBrushless);
        backMotor = new SparkMax(backId, MotorType.kBrushless);

        frontMotor.configure(
            new SparkMaxConfig().inverted(false),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        backMotor.configure(
            new SparkMaxConfig().inverted(true),
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
     * Sets the motors controlling the front and back pairs of mecanum wheels to speed in order to
     * achieve the given vector in cartesian form. This function will normalize the given vector if
     * it has a magnitude greater than 1.0 in order to maintain the angle of the vector, this will
     * result in vectors which have a magnitude greater than 1.0 all being taken as though they have
     * a magnitude of exactly 1.0.
     */
    public void setIntakeVector(double x, double y) {
        // basis vector along x points out of the intake.
        final double basisFrontX = 1.0;
        final double basisBackX = 1.0;

        // basis vector along y point up from the intake when the intake is at zero degrees.
        final double basisFrontY = -1.0;
        final double basisBackY = 1.0;

        double magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        double angle = Math.atan2(y, x);

        if (magnitude > 1.0) {
            magnitude = 1.0;
        }

        double speedFront
            = magnitude * (basisFrontX * Math.cos(angle) + basisFrontY * Math.sin(angle));
        double speedBack
            = magnitude * (basisBackX * Math.cos(angle) + basisBackY * Math.sin(angle));

        frontMotor.set(speedFront);
        backMotor.set(speedBack);
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
        return new Angle().setDegrees(pivotMotor.getAbsoluteEncoder().getPosition());
    }

    /**
     * Gets the speed of the motor controlling the front pair of mecanum wheels
     */
    public double getFrontPairMotorSpeed() {
        return frontMotor.get();
    }
    
    /**
     * Gets the speed of the motor controlling the back pair of mecanum wheels
     */
    public double getBackPairMotorSpeed() {
        return backMotor.get();
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
