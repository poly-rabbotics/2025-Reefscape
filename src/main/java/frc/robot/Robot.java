// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import frc.robot.subsystems.*;
import frc.robot.systems.*;
import frc.robot.systems.CoralIntake.IntakeState;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private static final int CONTROLLER_PORT_DRIVER = 0;
    private static final int CONTROLLER_PORT_OPERATOR = 1;
    
    private static final XboxController controllerDriver = new XboxController(CONTROLLER_PORT_DRIVER);
    private static final XboxController controllerOperator = new XboxController(CONTROLLER_PORT_OPERATOR);
    private static boolean invertedTurn = false;
    
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @SuppressWarnings("resource") // Suppress warning about PDP instantiation.
    @Override
    public void robotInit() {
        Logger.recordMetadata("ProjectName", "7042 Prototype");

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
            new PowerDistribution(1, ModuleType.kRev);
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
                new PowerDistribution(1, ModuleType.kRev);

                System.out.println("Could not find log to replay from ...");
                System.out.println("Starting non-replay simulation ...");
            }
        }

        Logger.start();

        // Log driver station data
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        Pigeon.setYaw(new Angle().setDegrees(0.0));
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
                if (SwerveDrive.getMode() == SwerveMode.SET_ANGLE) {
                    SwerveDrive.setMode(SwerveMode.HEADLESS);
                }
            }

            if (controllerDriver.getStartButtonReleased()) {
                if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) {
                    Pigeon.setYaw(new Angle().setDegrees(0.0));
                } else {
                    Pigeon.setYaw(new Angle().setDegrees(180.0));
                }

                // Prevents fast rotation when setting heading.
                if (SwerveDrive.getMode() == SwerveMode.SET_ANGLE) {
                    SwerveDrive.setMode(SwerveMode.HEADLESS);
                }
            }
        }

        recordLogFrame();
    }
    
    @Override
    public void autonomousInit() {
        SwerveDrive.setMode(SwerveMode.SIDEWALK_WALK);
    }

    @Override
    public void autonomousPeriodic() {
        SwerveDrive.run();
    }

    @Override
    public void teleopInit() {
        SwerveDrive.setMode(SwerveMode.HEADLESS);
    }

    @Override
    public void teleopPeriodic() {
        if (DriverStation.isJoystickConnected(CONTROLLER_PORT_DRIVER)) {
            driverTeleop();
        }

        if (DriverStation.isJoystickConnected(CONTROLLER_PORT_OPERATOR)) {
            if (controllerOperator.getAButton()) {
                CoralIntake.setIntakeState(IntakeState.INTAKING);
            } else if (controllerOperator.getBButton()) {
                CoralIntake.setIntakeState(IntakeState.OUTTAKING);
            } else {
                CoralIntake.setIntakeState(IntakeState.IDLE);
            }
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
            = SwerveDrive.getMode() == SwerveMode.HEADLESS
            || SwerveDrive.getMode() == SwerveMode.SET_ANGLE;

        // Left stick changes between headless and relative control modes.
        if (controllerDriver.getLeftStickButtonReleased()) {
            SwerveDrive.setMode(isFieldOriented ? SwerveMode.RELATIVE : SwerveMode.HEADLESS);
        }

        // Invert turn on right stick.
        if (controllerDriver.getRightStickButtonReleased()){       
            if (!invertedTurn) {
                SwerveDrive.setRotationCurve((x) -> Controls.turnCurveRohan(-x));
                invertedTurn = true;
            } else {
                SwerveDrive.setRotationCurve(Controls::turnCurveRohan);
                invertedTurn = false;
            }
        }

        double pov = controllerDriver.getPOV();

        // If a D-Pad button is pressed, snap to that angle.
        if (pov != -1 && isFieldOriented) {
            Angle angle = new Angle().setDegrees(-(double)pov);

            SwerveDrive.setMode(SwerveMode.SET_ANGLE);
            SwerveDrive.setTargetAngle(angle);
        }
        
        // If the driver is not actively turning the robot, enable automated turn control to
        // compensate for drift.
        if (
            SwerveDrive.getMode() == SwerveMode.HEADLESS
            && Math.abs(controllerDriver.getRightX()) < 0.15
        ) {
            SwerveDrive.setTargetAngle(Pigeon.getYaw());
            SwerveDrive.setMode(SwerveMode.SET_ANGLE);
        }

        // If the driver tries to trun the robot, drop out of automated turn control.
        if (
            Math.abs(controllerDriver.getRightX()) > 0.15
            && SwerveDrive.getMode() == SwerveMode.SET_ANGLE
        ) {
            SwerveDrive.setMode(SwerveMode.HEADLESS);
        }

        SwerveDrive.conditionalTempTranslationCurve(
            Controls.cardinalLock(Controls::defaultCurveTwoDimensional), 
            controllerDriver.getXButton()
        ); // Lock to cardinal directions.
        SwerveDrive.conditionalTempTranslationCurve(
            (x, y) -> Controls.defaultCurveTwoDimensional(x, y) / 3.0,
            controllerDriver.getRightBumperButton()
        ); // Half translation speed.
        SwerveDrive.conditionalTempTranslationCurve(
            Controls.cardinalLock((x, y) -> Controls.defaultCurveTwoDimensional(x, y) / 2.0),
            controllerDriver.getRightBumperButton() && controllerDriver.getXButton()
        ); // Half translation speed.

        SwerveDrive.conditionalTempMode(SwerveMode.ROCK, controllerDriver.getBButton());
        SwerveDrive.conditionalTempMode(SwerveMode.RELATIVE, controllerDriver.getYButton());

        var setPose = new Pose2d(16.522, 4.745, new Rotation2d(Math.PI));
        SwerveDrive.setTargetPathPosition(new PathPosition(setPose, 0.0));
        SwerveDrive.conditionalTempMode(SwerveMode.SIDEWALK_WALK, controllerDriver.getAButton());

        SwerveDrive.run(
            -controllerDriver.getLeftY(),
            -controllerDriver.getLeftX(),
            -controllerDriver.getRightX()
        );
    }
}
