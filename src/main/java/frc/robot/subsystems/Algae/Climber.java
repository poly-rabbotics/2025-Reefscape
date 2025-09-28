package frc.robot.subsystems.Algae;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.*;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import frc.robot.subsystems.Angle;
import frc.robot.systems.Algae.ClimberPose;
import frc.robot.Robot;
import frc.robot.systems.Pose;
public class Climber {
    Joystick controlPanelClimber;
    private static final Angle PIVOT_CLIMB_HOLD = new Angle().setDegrees(140.0);
    private static final Angle PIVOT_CLIMB_APROACH = new Angle().setDegrees(0.0);
    private static final Angle PIVOT_CLIMB_ALGAEXFER = new Angle().setDegrees(90);
    private static final Angle PIVOT_CLIMB_ALGAEINTAKE = new Angle().setDegrees(45);
    private static final Angle PIVOT_CLIMB_PROCESSOR = new Angle().setDegrees(0.0);
    private static final Angle PIVOT_ALGAE_GRAB = new Angle().setDegrees(0);
    private static final Angle PIVOT_CORAL_GRAB = new Angle().setDegrees(0);
    private static final Angle PIVOT_ALGAE_TRANSFER = new Angle().setDegrees(0.0);
    private static final Angle PIVOT_STOW = PIVOT_CLIMB_HOLD;

    private static final double PIVOT_DEGREES_TOLERANCE = 1.0;

    private static final double ACTUATOR_LENGTH = 3.75;
    private static final double ACTUATOR_HOLD = 1.5;
    private static final double ACTIATOR_STOW = 0.0;
    
    private final SparkMax pivotMotor;
    SparkMaxConfig pivotMotorConfig;
    SparkAbsoluteEncoder absoluteEncoder;
    //private final PIDController controller = new PIDController(0.035, 0.0, 0.0);

    private final double P_0 = 0.035;
    private final double I_0 = 0.0;
    private final double D_0 = 0.0;

    private ClimberPose pose = ClimberPose.Stow;

    public Climber(int pivotMotorId, int actuatorChannel) {
        /*pivotMotor = new SparkMax(pivotMotorId, MotorType.kBrushless);
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
        );*/

        pivotMotor = new SparkMax(pivotMotorId, MotorType.kBrushless);
        pivotMotorConfig = new SparkMaxConfig();
        absoluteEncoder = pivotMotor.getAbsoluteEncoder();

        pivotMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        pivotMotorConfig.absoluteEncoder.positionConversionFactor(360);
        pivotMotorConfig.absoluteEncoder.inverted(false);
        pivotMotorConfig.inverted(false);
        pivotMotorConfig.closedLoop.pid(P_0,I_0,D_0);
        pivotMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        pivotMotorConfig.closedLoop.positionWrappingInputRange(-180, 180);
        pivotMotorConfig.closedLoop.positionWrappingEnabled(true);
        pivotMotor.configure(pivotMotorConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        

        controlPanelClimber = new Joystick(3);
    }

    public void run() {
        
        pivotMotor.getClosedLoopController().setReference(Pose.climberPos,ControlType.kPosition);
       /* double speed = controller.calculate(getAngle().degrees(), getSetAngle().degrees());
        pivotMotor.set(speed); */

        /*if (
            pose == ClimberPose.ClimbHold
            && Math.abs(getAngle().degrees() - getSetAngle().degrees()) < PIVOT_DEGREES_TOLERANCE
        ) {
            linearActuator.set(ACTUATOR_HOLD / ACTUATOR_LENGTH);
        } else {
            linearActuator.set(ACTIATOR_STOW / ACTUATOR_LENGTH);
        } */
       /*if climb button pressed set pose to climb
       else: approach button pressed go to approach
       */
      
      }
    public void test() {
        Joystick testJoy = new Joystick(4);

        double power = testJoy.getRawAxis(1);
        double deadband = 0.1;
        if(power < Math.abs(deadband))
            power = 0;
        pivotMotor.set(0);
    }
    

    public void setPose(ClimberPose pose) {
        this.pose = pose;
    }
    
    public ClimberPose getSetPose() {
        return pose;
    }

    public Angle getSetAngle() {
        return getPoseAngle(pose);
    }

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
    private static Angle getPoseAngle(ClimberPose pose) {
        switch (pose) {
            case ClimbHold:
                return PIVOT_CLIMB_HOLD;
            case ClimbAproach:
                return PIVOT_CLIMB_APROACH;
            default:
            case Stow:
                return PIVOT_STOW;
        }
    }
}
