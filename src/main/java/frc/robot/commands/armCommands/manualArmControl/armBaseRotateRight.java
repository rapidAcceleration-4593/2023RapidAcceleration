package frc.robot.commands.armCommands.manualArmControl;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm;

public class armBaseRotateRight extends CommandBase{

    private final arm m_arm;

    public armBaseRotateRight(arm armPassedIn) {
        m_arm = armPassedIn;
        addRequirements(armPassedIn);
    }

    @Override
    public void initialize() {
        m_arm.baseRotateRight();
    }
    
}
