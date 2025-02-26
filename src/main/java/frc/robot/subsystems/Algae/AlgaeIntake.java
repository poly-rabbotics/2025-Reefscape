package frc.robot.subsystems.Algae;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Angle;
import frc.robot.systems.Algae.AlgaePose;

public class AlgaeIntake {
    private static final double PID_P = 0.0;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0;

    private static final double PIVOT_OFFSET = 0.0;

    private static final Angle PIVOT_STOW = new Angle().setDegrees(0.0);
    private static final Angle PIVOT_INTAKE = new Angle().setDegrees(0.0);
    private static final Angle PIVOT_HANDOFF = new Angle().setDegrees(0.0);
    private static final Angle PIVOT_PROCESSOR = new Angle().setDegrees(0.0);

    private final PIDController controller = new PIDController(PID_P, PID_I, PID_D);
    private final SparkMax intakeMotor;
    private final SparkMax pivotMotor;

    private AlgaePose pose = AlgaePose.Stow;

    public AlgaeIntake(int intakeMotorId, int pivotMotorId) {
        intakeMotor = new SparkMax(intakeMotorId, MotorType.kBrushless);
        pivotMotor = new SparkMax(pivotMotorId, MotorType.kBrushless);
    }

    public void run() {
        double pivotSpeed = controller.calculate(getAngle().degrees(), getSetAngle().degrees());
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
        return new Angle().setDegrees((encoderPose + PIVOT_OFFSET) % 360.0);
    }
    
    public double getPivotSpeed() {
        return pivotMotor.get();
    }

    public double getIntakeSpeed() {
        return intakeMotor.get();
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
