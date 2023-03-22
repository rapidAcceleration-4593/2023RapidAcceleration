package frc.robot.auton;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.armCommands.armExtensionIn;
import frc.robot.commands.armCommands.armExtensionOut;
import frc.robot.commands.armCommands.armStop;
import frc.robot.commands.armCommands.moveDownPoint;
import frc.robot.commands.armCommands.moveUpPoint;
import frc.robot.commands.armCommands.scorerStop;
import frc.robot.commands.armCommands.theReverseScorer;
import frc.robot.commands.armCommands.theScorer;
import frc.robot.commands.armCommands.wristDown;
import frc.robot.commands.armCommands.wristStop;
import frc.robot.commands.armCommands.wristUp;
import frc.robot.commands.armCommands.manualArmControl.armDown;
import frc.robot.commands.armCommands.manualArmControl.armUp;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.arm;

import java.util.List;

public class Autos extends SequentialCommandGroup {
  public Autos(Swerve s_Swerve, arm m_arm, String autoSelector) {
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.SwerveConstants.swerveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1.5, 0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(2.9, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController,
            0,
            0,
            Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            s_Swerve::getPose,
            Constants.SwerveConstants.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);
    Trajectory plzTurn =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(2.9, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(2.8, 0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(2.9, 0, new Rotation2d(.1)),
            config);

    SwerveControllerCommand turnPlz =
        new SwerveControllerCommand(
            plzTurn,
            s_Swerve::getPose,
            Constants.SwerveConstants.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    if(autoSelector == "noMove") {
        addCommands(    
        new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
        new moveUpPoint(m_arm).withTimeout(1),
        new armStop(m_arm).withTimeout(.25),
        new armExtensionIn(m_arm).withTimeout(.75),
        new armStop(m_arm).withTimeout(.25),
        new wristDown(m_arm).withTimeout(.5),
        new wristStop(m_arm).withTimeout(.25),
        new theReverseScorer(m_arm).withTimeout(1),
        new scorerStop(m_arm).withTimeout(.25),
        new armExtensionOut(m_arm).withTimeout(.75),
        new armStop(m_arm).withTimeout(.25),
        new wristUp(m_arm).withTimeout(.5),
        new wristStop(m_arm).withTimeout(.25),
        new armStop(m_arm).withTimeout(.25),
        new moveDownPoint(m_arm).withTimeout(1.75),
        new armStop(m_arm).withTimeout(.1));
    }
    else {      
    addCommands(    
        new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
        new moveUpPoint(m_arm).withTimeout(1),
        new armExtensionIn(m_arm).withTimeout(1),
        new armStop(m_arm).withTimeout(.5),
        new wristDown(m_arm).withTimeout(.5),
        new wristStop(m_arm).withTimeout(.5),
        new theReverseScorer(m_arm).withTimeout(1),
        new scorerStop(m_arm).withTimeout(.5),
        new armExtensionOut(m_arm).withTimeout(1),
        new armStop(m_arm).withTimeout(.5),
        new wristUp(m_arm).withTimeout(.5),
        new wristStop(m_arm).withTimeout(.5),
        new armStop(m_arm).withTimeout(.5),
        new moveDownPoint(m_arm).withTimeout(1.75),
        new armStop(m_arm).withTimeout(.1),
        swerveControllerCommand,
        turnPlz);
  }
}
}