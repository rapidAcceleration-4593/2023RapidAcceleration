package frc.robot.auton;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.arm;

import java.util.HashMap;

@SuppressWarnings("unused")
public final class Autos {

  private final Swerve swerve;
  private final arm arm;
  private final SendableChooser<Command> autonChooser;
  private final HashMap<String, Command> eventMap;
  private final SwerveAutoBuilder autonBuilder;

  public Autos(Swerve swerve, arm arm) {
    this.swerve = swerve;
    this.arm = arm;

    eventMap = new HashMap<>();
    setMarkers();

    autonBuilder =
        new SwerveAutoBuilder(
            swerve::getPose,
            swerve::resetOdometry,
            new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0),
            new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0),
            swerve::setChassisSpeeds,
            eventMap,
            true,
            swerve);

    autonChooser = new SendableChooser<Command>();
   // autonChooser.setDefaultOption("No-op", new InstantCommand());
    autonChooser.setDefaultOption("Simple 1", simplePath());
    // autonChooser.addOption("Score 2 Money Zone", score2MoneyZone());
    // autonChooser.addOption("Score 3 Money Zone", score3MoneyZone());
    // autonChooser.addOption("Score 2 Far Zone", score2FarZone());
    // autonChooser.addOption("Score 3 Far Zone", score3FarZone());

    SmartDashboard.putData("Auton Chooser", autonChooser);
  
  }

  private void setMarkers() {
    eventMap.put("Wait a Second", new WaitCommand(1.5));
    eventMap.put("Reset Gyro", new InstantCommand(() -> swerve.zeroGyro()));
  }

    public Command simplePath(){
      return autonBuilder.fullAuto(
        PathPlanner.loadPath("simple1", new PathConstraints(
        Constants.AutoConstants.kMaxSpeedMetersPerSecond, 
        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)));
    }

  public Command getSelected() {
    return simplePath();
  }
}