package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class vision extends SubsystemBase {
    
NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
NetworkTableEntry tx = table.getEntry("tx");
NetworkTableEntry ty = table.getEntry("ty");
NetworkTableEntry ta = table.getEntry("ta");
NetworkTableEntry tv = table.getEntry("tv");
    public vision() {

    }
    public double horizontalOffset () {

        double x = tx.getDouble(0);

        System.out.println(x);

        return x;

    }
    public double verticalOffset () {

        double y = ty.getDouble(0);

        return y;

    }
    public double percentArea () {

        double a = ta.getDouble(0);

        return a;

    }
    public double isTarget () {

        double v = tv.getDouble(0);

        return v;

    }

}
