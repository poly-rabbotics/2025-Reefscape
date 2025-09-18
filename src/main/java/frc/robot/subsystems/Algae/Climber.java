package frc.robot.subsystems.Algae;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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
    
    private final Servo linearActuator;
    private final SparkMax pivotMotor;
    private final PIDController controller = new PIDController(0.035, 0.0, 0.0);

    private ClimberPose pose = ClimberPose.Stow;

    public Climber(int pivotMotorId, int actuatorChannel) {
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

        linearActuator = new Servo(actuatorChannel);
        linearActuator.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        controlPanelClimber = new Joystick(3);
    }

    public void run() {
        
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
      if(controlPanelClimber.getRawButton(5)){
            Pose.pose = "Climb";

      
      }
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

    public double getActuatorSet() {
        return linearActuator.get();
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
            case AlgaeGrab:
                return PIVOT_ALGAE_GRAB;
            case CoralGrab:
                return PIVOT_CORAL_GRAB;
            case AlgaeTransfer:
                return PIVOT_ALGAE_TRANSFER;
            default:
            case Stow:
                return PIVOT_STOW;
        }
    }
}
