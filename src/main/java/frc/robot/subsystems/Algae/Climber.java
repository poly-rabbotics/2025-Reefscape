package frc.robot.subsystems.Algae;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.*;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import frc.robot.systems.Pose;
public class Climber {
    Joystick controlPanelClimber;
    //private static final Angle PIVOT_CLIMB_HOLD = new Angle().setDegrees(140.0);
    /*private static final Angle PIVOT_CLIMB_APROACH = new Angle().setDegrees(0.0);
    private static final Angle PIVOT_CLIMB_ALGAEXFER = new Angle().setDegrees(90);
    private static final Angle PIVOT_CLIMB_ALGAEINTAKE = new Angle().setDegrees(45);
    private static final Angle PIVOT_CLIMB_PROCESSOR = new Angle().setDegrees(0.0);
    private static final Angle PIVOT_ALGAE_GRAB = new Angle().setDegrees(0);
    private static final Angle PIVOT_CORAL_GRAB = new Angle().setDegrees(0);
    private static final Angle PIVOT_ALGAE_TRANSFER = new Angle().setDegrees(0.0);
    private static final Angle PIVOT_STOW = PIVOT_CLIMB_HOLD;*/

 
    private final SparkMax pivotMotor;
    SparkMaxConfig pivotMotorConfig;
    SparkAbsoluteEncoder absoluteEncoder;
    SparkClosedLoopController pivotController;
  
    private final double P_0 = 0.04;
    private final double I_0 = 0.0;
    private final double D_0 = 0.0;
    
    public Climber(int pivotMotorId) {


        pivotMotor = new SparkMax(pivotMotorId, MotorType.kBrushless);
        pivotMotorConfig = new SparkMaxConfig();

        pivotMotorConfig.apply(new AbsoluteEncoderConfig()
            .positionConversionFactor(360.0)
            .inverted(false)
            .zeroOffset(0.1823656)
            .zeroCentered(true)
                );
        
        pivotMotorConfig.apply(new ClosedLoopConfig()
            .pid(P_0,I_0,D_0)
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .positionWrappingInputRange(-180, 180)
            .positionWrappingEnabled(true)
                );

        pivotMotorConfig.idleMode(IdleMode.kBrake);
        pivotMotorConfig.inverted(true);
        
        pivotMotor.configure(pivotMotorConfig,ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);

        absoluteEncoder = pivotMotor.getAbsoluteEncoder();
        pivotController = pivotMotor.getClosedLoopController();       

        controlPanelClimber = new Joystick(3);
    }

    public void run() {
        
        pivotController.setReference(Pose.climberPos,SparkMax.ControlType.kPosition,ClosedLoopSlot.kSlot0); 
      
      }
    public void test() {
        Joystick testJoy = new Joystick(4);

        double power = testJoy.getRawAxis(1);
        double deadband = 0.1;
        if(Math.abs(power) < deadband)
            power = 0;
        pivotMotor.set(power);
        
    } 


    public void print()
    {
        SmartDashboard.putNumber("Climber Target", Pose.climberPos);
        SmartDashboard.putNumber("Power",pivotMotor.get());
        SmartDashboard.putNumber("Climber Position", pivotMotor.getAbsoluteEncoder().getPosition());
    }
}
