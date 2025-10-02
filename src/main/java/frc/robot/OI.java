package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.systems.Pose;

public class OI {

    Joystick leftControl, rightControl;

    boolean[] leftPressed = new boolean[13];
    boolean[] leftReleased = new boolean[13];
    boolean[] leftRaw = new boolean[13];
    boolean[] rightPressed = new boolean[13];
    boolean[] rightReleased = new boolean[13];
    boolean[] rightRaw = new boolean[13];


    public OI()
    {
        leftControl = new Joystick(2);
        rightControl = new Joystick(3);
    }

    public void run()
    {
        for(int i=1;i<=12;i++)
        {
            leftPressed[i] = leftControl.getRawButtonPressed(i);
            leftReleased[i] = leftControl.getRawButtonReleased(i);
            leftRaw[i] = leftControl.getRawButton(i);
            rightPressed[i] = rightControl.getRawButtonPressed(i);
            rightReleased[i] = rightControl.getRawButtonReleased(i);
            rightRaw[i] = rightControl.getRawButton(i);
        }

        if(leftRaw[5])
            Pose.pose = "Climb";
        else if(leftRaw[4])
            Pose.pose = "L4";
        else if(leftRaw[3])
            Pose.pose = "L3";
        else if(leftRaw[2])
            Pose.pose = "L2";
        else if(leftRaw[1])
            Pose.pose = "L1";
        else if(leftRaw[8])
            Pose.pose = "ClimbApproach";
        
        else if(rightRaw[4])
            Pose.pose = "GrndAl";

        else if(leftRaw[9])
            Pose.pose = "AlXFR";
        else if(rightRaw[3])
            Pose.pose = "Processor";

        else Pose.pose = "FldNav";

        
    }
    
}
