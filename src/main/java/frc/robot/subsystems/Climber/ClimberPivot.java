package frc.robot.subsystems.Climber;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Angle;

public class ClimberPivot {
    private static final double CLIMB_HIGH_POINT = 60.02;
    private static final double CLIMB_LOW_POINT = 39.66;
    private static final double GRABBING_POINT = 91.7;

    private static final double PID_P = 0.5;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0;

    private final SparkMax motor;
    private final PIDController controller = new PIDController(
        PID_P,
        PID_I,
        PID_D
    );

    private double speed = 0.0;
    private Angle setAngle = null;

    public ClimberPivot(int id, boolean inverted, boolean encInverted) {
        motor = new SparkMax(id, MotorType.kBrushless);
        SparkMaxConfig motorLeftConfig = new SparkMaxConfig();
        AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();

        // Convert to degrees
        encoderConfig.positionConversionFactor(360.0);
        encoderConfig.inverted(encInverted);
        motorLeftConfig.apply(encoderConfig);
        motorLeftConfig.inverted(inverted);

        motor.configure(
            motorLeftConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

    public void run(boolean runPID) {
        if (setAngle != null && runPID) {
            var angle = setAngle.bound180();
            var pose = new Angle().setDegrees(motor.getAbsoluteEncoder().getPosition());
            pose = pose.bound180();
            speed = controller.calculate(angle.degrees(), pose.degrees());
        } 

        motor.set(speed);
    }

    /**
     * Sets the speed and discards any previously given set angle.
     */
    public void set(double speed) {
        setAngle = null;
        this.speed = speed;
    }

    /**
     * Gives a set angle and begins closed loop control, setting speed after this will disable set
     * speed control until this method is called again.
     */
    public void goTo(Angle angle) {
        setAngle = angle;
    }

    /**
     * Gets the speed of the pivot, as set either by a user of this class or by closed loop control
     * internal to this class.
     */
    public double get() {
        return motor.get();
    }

    /**
     * Gets the position reported by the absolute encoder.
     */
    public Angle getPosition() {
        double enc = motor.getAbsoluteEncoder().getPosition();
        return new Angle().setDegrees(enc).bound180();
    }
}
