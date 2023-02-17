package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision;

public class armVisionCombo extends CommandBase{

    private final vision m_vision;

    public armVisionCombo(vision visionPassedIn) {
        m_vision = visionPassedIn;
        addRequirements(visionPassedIn);
    }

    @Override
    public void execute() {
        m_vision.armAutoUp();
    }
    
}
