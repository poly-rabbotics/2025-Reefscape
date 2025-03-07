package frc.robot.subsystems.Algae;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import frc.robot.subsystems.Angle;
import frc.robot.systems.Algae.AlgaePose;

public class AlgaeIntake {
    private static final Angle PIVOT_STOW = new Angle().setDegrees(90.0);
    private static final Angle PIVOT_INTAKE = new Angle().setDegrees(-20.0);
    private static final Angle PIVOT_HANDOFF = new Angle().setDegrees(110.0);
    private static final Angle PIVOT_PROCESSOR = new Angle().setDegrees(60.0);

    private final SparkMax intakeMotor;
    private final SparkMax pivotMotor;
    private final PIDController controller = new PIDController(0.005, 0.0, 0.0);

    private AlgaePose pose = AlgaePose.Stow;

    public AlgaeIntake(int intakeMotorId, int pivotMotorId) {
        intakeMotor = new SparkMax(intakeMotorId, MotorType.kBrushless);
        pivotMotor = new SparkMax(pivotMotorId, MotorType.kBrushless);

        intakeMotor.configure(
            new SparkMaxConfig()
                .idleMode(IdleMode.kBrake),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        pivotMotor.configure(
            new SparkMaxConfig()
                .inverted(true)
                .apply(new AbsoluteEncoderConfig()
                    .zeroOffset(0.46)
                    .positionConversionFactor(360.0)
                    .velocityConversionFactor(360.0)
                    .inverted(true)
                    .zeroCentered(true)
                ),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

    public void run() {
        double speed = controller.calculate(getAngle().degrees(), getSetAngle().degrees());
        pivotMotor.set(speed);
    }

    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public AlgaePose getSetPose() {
        return pose;
    }

    public void setPose(AlgaePose pose) {
        this.pose = pose;
    }
    
    public Angle getSetAngle() {
        return getPoseAngle(pose);
    }

    public Angle getAngle() {
        return new Angle().setDegrees(pivotMotor.getAbsoluteEncoder().getPosition());
    }
    
    public double getPivotSpeed() {
        return pivotMotor.get();
    }

    public double getIntakeSpeed() {
        return intakeMotor.get();
    }

    public double getPivotVelocity() {
        return pivotMotor.getAbsoluteEncoder().getVelocity();
    }

    public void _setPivotSpeed(double speed) {
        pivotMotor.set(speed);
    }

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
