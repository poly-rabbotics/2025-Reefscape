// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.systems.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Coral.*;
import frc.robot.subsystems.Algae.Climber;
import frc.robot.subsystems.Algae.GroundIntake;
import frc.robot.systems.Algae.ClimberPose;
import frc.robot.systems.Coral.ScoringPoint;
import frc.robot.systems.SwerveDrive.DriveOrientation;
import frc.robot.subsystems.Coral.CoralIntake.IntakeState;

import java.util.Optional;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;


import com.ctre.phoenix6.signals.InvertedValue;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    GroundIntake groundIntake;
 
    private static final int CONTROLLER_PORT_DRIVER = 0;
    private static final int CONTROLLER_PORT_OPERATOR = 1;
    private static final int CONTROL_PANEL_PORT_CORAL = 2; // Elevator
    private static final int CONTROL_PANEL_PORT_ALGAE = 3; // Algae and switches

    private static final XboxController controllerDriver
        = new XboxController(CONTROLLER_PORT_DRIVER);
    private static final XboxController controllerOperator
        = new XboxController(CONTROLLER_PORT_OPERATOR);
    private static final Joystick controlPanelCoral
        = new Joystick(CONTROL_PANEL_PORT_CORAL);
    private static final Joystick controlPanelAlgae
        = new Joystick(CONTROL_PANEL_PORT_ALGAE);

    @SuppressWarnings("unused")
    private static PowerDistribution powerDistributionHub
        = new PowerDistribution(1, ModuleType.kRev);

    private static Optional<AutonomousProcedure> autonomousProcedure;
    
    private static boolean invertedTurn = true;

    Pose robotPose;
    OI oi;
    public static Climber climber;
    public static Elevator elevator;
    public static CoralPivot coralPivot;
    public static Dealgaeficator algae;
    public static CoralIntake coralIntake;
    public static boolean visionIsGood = false;
    int autoSelection;
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        climber = new Climber(15);
        groundIntake=new GroundIntake(20,21); 
        
        Logger.recordMetadata("ProjectName", "7042 Theseus");
        robotPose = new Pose();
        elevator = new Elevator(14, InvertedValue.Clockwise_Positive, 3);
        coralPivot = new CoralPivot(3);
        algae = new Dealgaeficator(0);
        coralIntake = new CoralIntake(11, 7, 1, 2);

        oi = new OI();

        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            try {
                // Try to replay from a log file.
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                setUseTiming(false);

                System.out.println("Replaying from '" + logPath + "' in simulation ...");
            } catch (Exception e) {
                // Simulate if no log file exists.
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());

                System.out.println("Could not find log to replay from ...");
                System.out.println("Starting non-replay simulation ...");
            }
        }

        Logger.start();
        SwerveDrive.recordState();

        // Log driver station data
        //DataLogManager.start();
        //DriverStation.startDataLog(DataLogManager.getLog());
        
        //PoseEstimator.reset(new Pose2d(7.576,3.989, new Rotation2d(Pigeon.getYaw().radians())));
    }

    @Override
    public void robotPeriodic() {

        //Run these all the time when the robot is powered on
         
        Pose.run();
        
        LEDLights.run();

        if(DriverStation.isTeleop())
            oi.run();
        

        recordLogFrame();

        //Logger.recordOutput("Alliance", DriverStation.getAlliance().orElse(Alliance.Blue));
        /*Logger.recordOutput("Selected Auto Number", AutonomousManager.getSwitchSelection(
            controlPanelAlgae.getRawButton(1),
            controlPanelAlgae.getRawButton(2),
            controlPanelAlgae.getRawButton(3),
            controlPanelAlgae.getRawButton(4)
        ));*/
        //Print useful data to the Dashboard
        groundIntake.print();
        Pose.print();
        elevator.print();
        climber.print();
        AutonomousManager.print();

        if(DriverStation.isAutonomous()&&DriverStation.isDisabled()&&!visionIsGood){
            Pose2d pose;
            Rotation2d rot;

            Optional<Alliance> ally = DriverStation.getAlliance();
            if(ally.get() == Alliance.Blue)
            {
                Pigeon.setYaw(new Angle().setDegrees(180));
                SwerveDrive.setDriveOrientation(DriveOrientation.BlueDriver);
            }
            else
            {
                Pigeon.setYaw(new Angle().setDegrees(0));
                SwerveDrive.setDriveOrientation(DriveOrientation.RedDriver);
            }
            rot = new Rotation2d(0);
            Pose2d leftPose = new Pose2d(6.94,6.168, rot);
            Pose2d rightPose = new Pose2d(6.94,1.911, rot);
            Pose2d centerPose = new Pose2d(6.94,4.17, rot);

            
            switch (AutonomousManager.getSwitchSelection())
            {
            case 0:
                pose =  centerPose;    
                break;
            case 1:
                pose = centerPose;
                break;
            case 2:
                pose = rightPose;
                break;
            case 3:
                pose = leftPose;
                break;
            case 4:
                pose = centerPose;
                break;
            case 5:
                pose = centerPose;
                break;
            default:
                pose = centerPose;  
                break;
            }

            pose = DrivePath.makeAllianceSideTranslation(pose,DriverStation.getAlliance().get());
            PoseEstimator.reset(pose);

            SmartDashboard.putNumber("Starting Pos",AutonomousManager.getSwitchSelection());
        }
        else {
            SmartDashboard.putNumber("Starting Pos",-1);
        }

        PoseEstimator.update();
        


   

        if (DriverStation.isJoystickConnected(CONTROLLER_PORT_DRIVER)) {
            int dPadAngle = controllerDriver.getPOV();

            if (dPadAngle != -1) {
                Angle yaw = new Angle().setDegrees(-(double)dPadAngle + 90.0);
                Pigeon.setYaw(yaw);

                // Prevents fast rotation when setting heading.
                if (SwerveDrive.getMode() == SwerveMode.SetAngle) {
                    SwerveDrive.setMode(SwerveMode.Headless);
                }

                PoseEstimator.reinstantiateDrive();
            }

            if (controllerDriver.getRightBumperButtonReleased()) {
                SwerveDrive.setDriveOrientation(DriveOrientation.RedDriver);
                //Pigeon.setYaw(new Angle().setDegrees(0));
            }

            if (controllerDriver.getLeftBumperButtonReleased()) {
                SwerveDrive.setDriveOrientation(DriveOrientation.BlueDriver);
                //Pigeon.setYaw(new Angle().setDegrees(180));
            }

            if (controllerDriver.getStartButtonReleased()) {
                SwerveDrive.setDriveOrientation(DriveOrientation.NeutralObserver);
            }
        }
        /*PoseEstimator.swervePoseEstimator.resetPosition(new Rotation3d(
            Pigeon.getRoll().radians(),
            Pigeon.getPitch().radians(),
            Pigeon.getYaw().radians()
        ),
        SwerveDrive.positions,new Pose3d(5, 6, 0, new Rotation3d(0, 0, 1.5)));*/

        
    }

    @Override
    public void autonomousInit() {
        //Coral.setIntakeState(IntakeState.Idle);
        Optional<Alliance> ally = DriverStation.getAlliance();
        if(ally.get() == Alliance.Blue)
        {
            Pigeon.setYaw(new Angle().setDegrees(180));
            SwerveDrive.setDriveOrientation(DriveOrientation.BlueDriver);
        }
        else
        {
            Pigeon.setYaw(new Angle().setDegrees(0));
            SwerveDrive.setDriveOrientation(DriveOrientation.RedDriver);
        }

        AutonomousManager.reset();
        SwerveDrive.setMode(SwerveMode.FollowPath);

        autonomousProcedure = AutonomousManager.getProcedureFromSwitches(
            controlPanelAlgae.getRawButton(1),
            controlPanelAlgae.getRawButton(2),
            controlPanelAlgae.getRawButton(3),
            controlPanelAlgae.getRawButton(4)
        );
        Pose.pose="FldNav";
        
        //PoseEstimator.reset(new Pose2d(7.576,3.989, new Rotation2d(Pigeon.getYaw().radians())));
    }

    @Override
    public void autonomousPeriodic() {
        //Coral.run();
        //Algae.run();
        coralPivot.run();
        elevator.run();
        coralIntake.run();

        autonomousProcedure.ifPresent(proc -> {
            proc.run();
            proc.recordState();
        });
    }

    @Override
    public void teleopInit() {
        SwerveDrive.setMode(SwerveMode.Headless);
        SwerveDrive.setRotationCurve(Controls::turnCurveRohan);
        SwerveDrive.setTranslationCurve(Controls::quinticCurveTwoDimensional);

        if (invertedTurn) {
            SwerveDrive.setRotationCurve((x) -> Controls.turnCurveRohan(-x));
        }

       
    }

    @Override
    public void teleopPeriodic() {
        //oi.run();
        groundIntake.run();
        climber.run();
        elevator.run();
        coralPivot.run();
        algae.run();
        coralIntake.run();

       

        
    
        if (DriverStation.isJoystickConnected(CONTROLLER_PORT_DRIVER)) {
            driverTeleop();
        }
            

        if (DriverStation.isJoystickConnected(CONTROLLER_PORT_OPERATOR)
            || DriverStation.isJoystickConnected(CONTROL_PANEL_PORT_CORAL)
            || DriverStation.isJoystickConnected(CONTROL_PANEL_PORT_ALGAE)) {
           // operatorTeleop();
        }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    private void recordLogFrame() {
        SwerveDrive.recordState();
        Pigeon.recordState();
        PoseEstimator.recordState();
        
        Coral.recordState();
        Algae.recordState();
    }

    /**
     * To be called periodically during teleop, this method handles all behavior that requires input
     * from the driver's controller.
     */
    private void driverTeleop() {
        boolean isFieldOriented
            = SwerveDrive.getMode() == SwerveMode.Headless
            || SwerveDrive.getMode() == SwerveMode.SetAngle;

        // Left stick changes between headless and relative control modes.
        if (controllerDriver.getLeftStickButtonReleased()) {
            SwerveDrive.setMode(
                isFieldOriented ? SwerveMode.Relative : SwerveMode.Headless
            );
        }

        // Invert turn on right stick.
        if (controllerDriver.getRightStickButtonReleased()) {
            if (!invertedTurn) {
                SwerveDrive.setRotationCurve(x -> Controls.turnCurveRohan(-x));
                invertedTurn = true;
            } else {
                SwerveDrive.setRotationCurve(Controls::turnCurveRohan);
                invertedTurn = false;
            }
        }

        // double pov = controllerDriver.getPOV();

        // // If a D-Pad button is pressed, snap to that angle.
        // if (pov != -1 && isFieldOriented) {
        //     Angle angle = new Angle().setDegrees((double) pov);

        //     SwerveDrive.setMode(SwerveMode.SetAngle);
        //     SwerveDrive.setTargetAngle(angle);
        // }

        // If the driver is not actively turning the robot, enable automated turn control to
        // compensate for drift.
        SmartDashboard.putString("Swerve Mode", SwerveDrive.getMode().toString());
        SmartDashboard.putNumber("Pigeon Yaw",Pigeon.getYaw().degrees());
        if (
            SwerveDrive.getMode() == SwerveMode.Headless &&
            Math.abs(controllerDriver.getRightX()) < 0.15
        ) {
            SwerveDrive.setTargetAngle(Pigeon.getYaw());
            SwerveDrive.setMode(SwerveMode.SetAngle);
        }

        // If the driver tries to trun the robot, drop out of automated turn control.
        if (
            SwerveDrive.getMode() == SwerveMode.SetAngle &&
            Math.abs(controllerDriver.getRightX()) > 0.15
        ) {
            SwerveDrive.setMode(SwerveMode.Headless);
        }

        SwerveDrive.conditionalTempTranslationCurve(
            Controls.cardinalLock(Controls::defaultCurveTwoDimensional),
            controllerDriver.getXButton()
        ); // Lock to cardinal directions.
        // SwerveDrive.conditionalTempTranslationCurve(
        //     (x, y) -> Controls.defaultCurveTwoDimensional(x, y) / 3.0,
        //     controllerDriver.getRightBumperButton()
        // ); // Lessen translation speed.
        // SwerveDrive.conditionalTempTranslationCurve(
        //     Controls.cardinalLock((x, y) -> Controls.defaultCurveTwoDimensional(x, y) / 3.0),
        //     controllerDriver.getRightBumperButton() &&
        //     controllerDriver.getXButton()
        // ); // Lessen translation speed and lock cardinal directions.

        SwerveDrive.conditionalTempMode(SwerveMode.Rock, controllerDriver.getBButton());
        SwerveDrive.conditionalTempMode(SwerveMode.Relative, controllerDriver.getYButton());
        SwerveDrive.conditionalTempMode(SwerveMode.FollowPath, controllerDriver.getAButton());
        SwerveDrive.setPath(new DrivePath(new Pose2d(5.75, 3.85, new Rotation2d(Math.PI))));

        double speedX = controllerDriver.getLeftX();
        double speedY = -controllerDriver.getLeftY();
        SwerveDrive.run(speedX, speedY, -controllerDriver.getRightX());

        
    }

    /**
     * To be called periodically during teleop, this method handles all behavior that requires input
     * from the operator's controller.
     */
    public void operatorTeleop() {

        
        double dealgaeSpeed = -0.2;
        ScoringPoint scoringPoint = ScoringPoint.Stowed;
        ClimberPose algaePose = ClimberPose.Stow;

        if (controlPanelAlgae.getRawButton(7)) {
            dealgaeSpeed = -1;
        } else if (controlPanelAlgae.getRawButton(6)){
            dealgaeSpeed = 1;
        }

        if (controlPanelCoral.getRawButton(1)) {
            scoringPoint = ScoringPoint.ScoringL1;
        } else if (controlPanelCoral.getRawButton(2)) {
            scoringPoint = ScoringPoint.ScoringL2;
        } else if (controlPanelCoral.getRawButton(3)) {
            scoringPoint = ScoringPoint.ScoringL3;
        } else if (controlPanelCoral.getRawButton(4)) {
            scoringPoint = ScoringPoint.ScoringL4;
        } else if (controlPanelCoral.getRawButton(6)) {
            scoringPoint = ScoringPoint.DealgaeficateL2;
        } else if (controlPanelCoral.getRawButton(5)) {
            scoringPoint = ScoringPoint.DealgaeficateL3;
        } else if (controlPanelCoral.getRawButton(7)) {
            scoringPoint = ScoringPoint.AlgaeNet;
        } else if (controlPanelAlgae.getRawButton(10)) {
            scoringPoint = ScoringPoint.Intaking;
        }

        if (controlPanelAlgae.getRawButtonPressed(10) || controlPanelAlgae.getRawButtonPressed(9)) {
            if (Coral.getIntakeState() != IntakeState.Intaking) {
                Coral.setIntakeState(IntakeState.Intaking);
            }
        } else if (controlPanelAlgae.getRawButtonReleased(10) || controlPanelAlgae.getRawButtonReleased(9)) {
            if (Coral.getIntakeState() != IntakeState.Storing) {
                Coral.setCoralPosition(2.6);
            }
        } else if (controlPanelAlgae.getRawButtonPressed(11)) {
            Coral.setIntakeState(IntakeState.Outtaking);
        } else if (controlPanelAlgae.getRawButtonReleased(11)) {
            Coral.setIntakeState(IntakeState.Idle);
        }

        if (controlPanelCoral.getRawButton(8)) {
            algaePose = ClimberPose.ClimbAproach;
        }

        if (controlPanelAlgae.getRawButton(5)) {
            algaePose = ClimberPose.ClimbHold;
            scoringPoint = ScoringPoint.Handoff;
        }

        Coral.setDealgaeficatorSpeed(dealgaeSpeed);
        Coral.setScoringPoint(scoringPoint);

        Algae.setPose(algaePose);

        //Algae.run();
        //Coral.run();
        
        double mainOverrideSpeed = Controls.defaultCurve(-controllerOperator.getLeftY());
        // double secondaryOverrideSpeed = Controls.defaultCurve(-controllerOperator.getRightY());
        double intakeOverrideSpeed
            = Controls.defaultCurve(controllerOperator.getRightTriggerAxis())
            - Controls.defaultCurve(controllerOperator.getLeftTriggerAxis());

        if (controllerOperator.getRightBumperButton()) {
            Coral._setElevatorSpeed(mainOverrideSpeed);
            Coral._setIntakeSpeed(intakeOverrideSpeed);
        }

        if (controllerOperator.getXButton()) {
            Algae._setPivotSpeed(mainOverrideSpeed);
        }

        if (controllerOperator.getStartButtonReleased()) {
            Coral._setElevatorPose(0.0);
        }

        if (controlPanelCoral.getRawButton(9)) {
            Coral._setElevatorSpeed(-1.0);
        }
    }

    XboxController simController = new XboxController(5);
    Pose2d simPose = new Pose2d();
    double simFrameTime = DriverStation.getMatchTime();
    double simDeltaTime = 0;

    @Override
    public void simulationPeriodic() {
        double rotationSpeed = simController.getRightX();

        if (Math.abs(rotationSpeed) < 0.1) {
            rotationSpeed = 0;
        }

        double translationSpeedX = simController.getLeftX();

        if (Math.abs(translationSpeedX) < 0.1) {
            translationSpeedX = 0;
        }

        double translationSpeedY = simController.getLeftY();

        if (Math.abs(translationSpeedY) < 0.1) {
            translationSpeedY = 0;
        }

        double dx = Math.pow(translationSpeedX, 3) * 0.1;
        double dy = Math.pow(-translationSpeedY, 3) * 0.1;
        double dTheta = Math.pow(-rotationSpeed, 5) * 0.1;

        simPose = new Pose2d(
            simPose.getX() + dx,
            simPose.getY() + dy,
            simPose.getRotation().plus(new Rotation2d(dTheta))
        );

        Logger.recordOutput("Simulation Pose", simPose);
        Logger.recordOutput("Simulation Delta Time", simDeltaTime);
    }
}
