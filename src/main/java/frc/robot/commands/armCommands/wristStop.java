package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm;

public class wristStop extends CommandBase{

    private final arm m_arm;

    public wristStop(arm armPassedIn) {
        m_arm = armPassedIn;
        addRequirements(armPassedIn);
    }

    @Override
    public void execute() {
        m_arm.wristStop();
    }
    
}
