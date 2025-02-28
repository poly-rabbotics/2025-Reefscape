package frc.robot.subsystems.Algae;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Angle;
import frc.robot.systems.Algae.AlgaePose;

public class AlgaeIntake {
    private static final double PID_P = 0.0075;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0;

    private static final double PIVOT_TOLERANCE = 10.0;
    private static final double PIVOT_OFFSET = -160.2;
    private static final double PIVOT_MAX_SPEED = 0.4;

    private static final Angle PIVOT_STOW = new Angle().setDegrees(90.0);
    private static final Angle PIVOT_INTAKE = new Angle().setDegrees(-20.0);
    private static final Angle PIVOT_HANDOFF = new Angle().setDegrees(110.0);
    private static final Angle PIVOT_PROCESSOR = new Angle().setDegrees(60.0);

    private final PIDController controller = new PIDController(PID_P, PID_I, PID_D);
    private final SparkMax intakeMotor;
    private final SparkMax pivotMotor;

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
                .smartCurrentLimit(20),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        controller.setTolerance(PIVOT_TOLERANCE);
    }

    public void run() {
        double pivotSpeed = controller.calculate(getAngle().degrees(), getSetAngle().degrees());

        if (Math.abs(pivotSpeed) > PIVOT_MAX_SPEED) {
            pivotSpeed = PIVOT_MAX_SPEED * Math.signum(pivotSpeed);
        } 

        pivotMotor.set(pivotSpeed);
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
        double encoderPose = pivotMotor.getAbsoluteEncoder().getPosition() * 360.0;
        return new Angle().setDegrees(-1.0 * ((encoderPose + PIVOT_OFFSET) % 360.0));
    }
    
    public double getPivotSpeed() {
        return pivotMotor.get();
    }

    public double getIntakeSpeed() {
        return intakeMotor.get();
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
