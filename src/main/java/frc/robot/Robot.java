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
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.signals.InvertedValue;

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
    private static final XboxController controllerDriver = new XboxController(0);
    private static boolean invertedTurn = false;
    
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @SuppressWarnings("resource") // Suppress warning about PDP instantiation.
    @Override
    public void robotInit() {
        //https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/INSTALLATION.md#new-projects
        Logger.recordMetadata("ProjectName", "7042 Prototype");

        if (isReal()) {
            // Log for real robot runs
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
            new PowerDistribution(1, ModuleType.kRev);
        } else {
            // in simulation mode replay from logs
            setUseTiming(false);
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        }

        Logger.start();

        // Log driver station data
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        Pigeon.setFeildZero();
    }
    
    @Override
    public void robotPeriodic() {
        PoseEstimator.update();

        if (controllerDriver.getBackButtonReleased()) {
            var pose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
            Pigeon.setFeildOrientation(new Angle().setRadians(pose.getRotation().getRadians()));
            // TODO: Reset pose estimation here.
            
            // Prevents fast rotation when setting heading.
            if (SwerveDrive.getMode() == SwerveMode.SET_ANGLE) {
                SwerveDrive.setMode(SwerveMode.HEADLESS);
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
        if (controllerDriver.getAButton()) {
            CoralIntake.setIntakeState(IntakeState.INTAKING);
        } else if (controllerDriver.getYButton()) {
            CoralIntake.setIntakeState(IntakeState.OUTTAKING);
        } else {
            CoralIntake.setIntakeState(IntakeState.IDLE);
        }

        if (controllerDriver.getStartButtonReleased()) {
            Pigeon.setFeildZero();
        }
        
        // Left stick changes between headless and relative control modes.
        if (controllerDriver.getLeftStickButtonReleased()) {
            SwerveDrive.setMode(
                SwerveDrive.getMode() == SwerveMode.HEADLESS 
                    ? SwerveMode.RELATIVE 
                    : SwerveMode.HEADLESS
            );
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

        if (pov != -1 && (SwerveDrive.getMode() == SwerveMode.HEADLESS || SwerveDrive.getMode() == SwerveMode.SET_ANGLE)) {
            double angle = -((double)pov);

            SwerveDrive.setMode(SwerveMode.SET_ANGLE);
            SwerveDrive.setTargetAngle(new Angle().setDegrees(angle));
        } else if (Math.abs(controllerDriver.getRightX()) > 0.15 && SwerveDrive.getMode() == SwerveMode.SET_ANGLE) {
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

        var setPose = new Pose2d(16.522, 4.745, new Rotation2d(Math.PI));
        SwerveDrive.setTargetPathPosition(new PathPosition(setPose, 0.0));
        SwerveDrive.conditionalTempMode(SwerveMode.SIDEWALK_WALK, controllerDriver.getAButton());

        if (SwerveDrive.getMode() == SwerveMode.HEADLESS && Math.abs(controllerDriver.getRightX()) < 0.15) {
            SwerveDrive.setTargetAngle(Pigeon.getYaw());
            SwerveDrive.setMode(SwerveMode.SET_ANGLE);
        }

        SwerveDrive.run(
            -controllerDriver.getLeftY(),
            -controllerDriver.getLeftX(),
            -controllerDriver.getRightX()
        );
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
}
