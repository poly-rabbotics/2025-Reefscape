// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.subsystems.*;
import frc.robot.systems.*;
import frc.robot.systems.Algae.AlgaePose;

import java.util.Optional;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private static final int CONTROLLER_PORT_DRIVER = 0;
    private static final int CONTROLLER_PORT_OPERATOR = 1;
    private static final int CONTROL_PANEL_PORT_ELEVATOR = 2; // CONTROLLER PANEL WITH ELEVATOR SETPOINTS
    private static final int CONTROL_PANEL_PORT_CLIMB = 3; // CONTROLLER PANEL WITH SWITCHES, INTAKE, & CLIMB

    private static Compressor comp = new Compressor(0, PneumaticsModuleType.CTREPCM);

    private static final XboxController controllerDriver
        = new XboxController(CONTROLLER_PORT_DRIVER);
    private static final XboxController controllerOperator
        = new XboxController(CONTROLLER_PORT_OPERATOR);
    private static final Joystick controlPanelElevator
        = new Joystick(CONTROL_PANEL_PORT_ELEVATOR);
    private static final Joystick controlPanelClimb
        = new Joystick(CONTROL_PANEL_PORT_CLIMB);

    @SuppressWarnings("unused")
    private static PowerDistribution powerDistributionHub
        = new PowerDistribution(1, ModuleType.kRev);

    private static Optional<AutonomousProcedure> autonomousProcedure;
    
    private static boolean invertedTurn = false;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        Logger.recordMetadata("ProjectName", "7042 Theseus");

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

        // Log driver station data
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        Pigeon.setYaw(new Angle().setDegrees(0.0));

        // Compressor stuffs
        comp.enableDigital();
    }

    @Override
    public void robotPeriodic() {
        PoseEstimator.update();

        if (DriverStation.isJoystickConnected(CONTROLLER_PORT_DRIVER)) {
            if (controllerDriver.getBackButtonReleased()) {
                var pose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
                Pigeon.setFeildOrientation(new Angle().setRadians(pose.getRotation().getRadians()));
                PoseEstimator.reset();

                // Prevents fast rotation when setting heading.
                if (SwerveDrive.getMode() == SwerveMode.SetAngle) {
                    SwerveDrive.setMode(SwerveMode.Headless);
                }
            }

            if (controllerDriver.getStartButtonReleased()) {
                if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) {
                    Pigeon.setYaw(new Angle().setDegrees(0.0));
                } else {
                    Pigeon.setYaw(new Angle().setDegrees(180.0));
                }

                // Prevents fast rotation when setting heading.
                if (SwerveDrive.getMode() == SwerveMode.SetAngle) {
                    SwerveDrive.setMode(SwerveMode.Headless);
                }
            }
        }

        recordLogFrame();
    }

    @Override
    public void autonomousInit() {
        AutonomousManager.reset();
        SwerveDrive.setMode(SwerveMode.FollowPath);

        autonomousProcedure = AutonomousManager.getProcedureFromSwitches(
            // insert switch gets
        );
    }

    @Override
    public void autonomousPeriodic() {
        autonomousProcedure.ifPresent(proc -> {
            proc.run();
            proc.recordState();
        });
        
        SwerveDrive.run();
        Coral.run();
    }

    @Override
    public void teleopInit() {
        SwerveDrive.setMode(SwerveMode.Headless);
    }

    @Override
    public void teleopPeriodic() {
        if (DriverStation.isJoystickConnected(CONTROLLER_PORT_DRIVER)) {
            driverTeleop();
        }

        if (DriverStation.isJoystickConnected(CONTROLLER_PORT_OPERATOR)) {
            operatorTeleop();
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
        // Reset field oriented heading, based on alliance.
        if (controllerDriver.getStartButtonReleased()) {
            if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) {
                Pigeon.setYaw(new Angle().setDegrees(0.0));
            } else {
                Pigeon.setYaw(new Angle().setDegrees(180.0));
            }
        }

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

        double pov = controllerDriver.getPOV();

        // If a D-Pad button is pressed, snap to that angle.
        if (pov != -1 && isFieldOriented) {
            Angle angle = new Angle().setDegrees(-(double) pov);

            SwerveDrive.setMode(SwerveMode.SetAngle);
            SwerveDrive.setTargetAngle(angle);
        }

        // If the driver is not actively turning the robot, enable automated turn control to
        // compensate for drift.
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
        SwerveDrive.conditionalTempTranslationCurve(
            (x, y) -> Controls.defaultCurveTwoDimensional(x, y) / 3.0,
            controllerDriver.getRightBumperButton()
        ); // Lessen translation speed.
        SwerveDrive.conditionalTempTranslationCurve(
            Controls.cardinalLock((x, y) -> Controls.defaultCurveTwoDimensional(x, y) / 3.0),
            controllerDriver.getRightBumperButton() &&
            controllerDriver.getXButton()
        ); // Lessen translation speed and lock cardinal directions.

        SwerveDrive.conditionalTempMode(SwerveMode.Rock, controllerDriver.getBButton());
        SwerveDrive.conditionalTempMode(SwerveMode.Relative, controllerDriver.getYButton());

        SwerveDrive.conditionalTempMode(SwerveMode.FollowPath, controllerDriver.getAButton());

        SwerveDrive.run(
            -controllerDriver.getLeftY(),
            -controllerDriver.getLeftX(),
            -controllerDriver.getRightX()
        );
    }

    /**
     * To be called periodically during teleop, this method handles all behavior that requires input
     * from the operator's controller.
     */
    public void operatorTeleop() {
        double algaeSpeed = 0.0;

        if (controllerOperator.getPOV() == 180) {
            algaeSpeed = -1.0;
        } else {
            algaeSpeed = 1.0;
        }

        Coral.run(
            controlPanelClimb.getRawButton(10), //Intaking
            controlPanelClimb.getRawButton(9), //Outtaking
            false, // Stow
            controlPanelElevator.getRawButton(8), // Intake
            controlPanelElevator.getRawButton(1),
            controlPanelElevator.getRawButton(2), // L2
            controlPanelElevator.getRawButton(3), // L3
            false, //controlPanelElevator.getRawButton(4),     // L4
            algaeSpeed,
            controlPanelElevator.getRawButton(6),
            controlPanelElevator.getRawButton(5)
        );

        Algae.setIntakeSpeed(0.0);
        Algae.setPose(AlgaePose.Stow);
        Algae.run();
        
        double pivotSpeed = controllerOperator.getRightX();
        double elevatorSpeed = -controllerOperator.getRightY();

        if (Math.abs(pivotSpeed) > 0.15) {
            Coral._setPivotSpeed(pivotSpeed);
        }

        if (Math.abs(elevatorSpeed) > 0.15) {
            Coral._setElevatorSpeed(elevatorSpeed);
        }

        if (controlPanelElevator.getRawButton(8)) {
            Coral._setElevatorPose(0.0);
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
