package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.armCommands.manualArmControl.armUp;
import edu.wpi.first.wpilibj.Relay;

public class arm extends SubsystemBase {

    private AnalogPotentiometer m_armPotentiometer;

    private CANSparkMax m_extensionMotor;
    private PWM m_armMotorP2;
    private PWM m_armRotaionMotor;
    private PWMSparkMax m_scorer;
    private PWMTalonSRX m_wrist;

    private final Relay m_relay;

    private SparkMaxPIDController m_ArmExtenPidController;
    private CANSparkMax m_motorTest;

    //double armPotVal = m_armPotentiometer.get();

    public arm() {
      //m_basePotentiometer = new AnalogPotentiometer(1, 314, 0);
      m_armPotentiometer = new AnalogPotentiometer(1, 314, 0);
      //m_baseMotor = new PWM(1);
     
      m_armMotorP2 = new PWM(1); 
      m_armRotaionMotor = new PWM(2);
      
      m_scorer = new PWMSparkMax(3);
      
      m_wrist = new PWMTalonSRX(0);
      
      m_extensionMotor = new CANSparkMax(14, MotorType.kBrushless);

      m_relay = new Relay(0);

      
    } 

    public void baseRotateLeft () {
      //m_baseMotor.setSpeed(.66);
    }

    public void baseRotateRight (){
      //m_baseMotor.setSpeed(-.66);

      //System.out.println("Base Potentiometer" + m_basePotentiometer.get());
    }

    public void armRotateUp () {
      m_armRotaionMotor.setSpeed(-.5);
      m_armMotorP2.setSpeed(-.5);

      System.out.println("arm Potentiometer" + m_armPotentiometer.get());
    }

    public void armRotateDown () {
      m_armRotaionMotor.setSpeed(.5);
      m_armMotorP2.setSpeed(.5);

      System.out.println("arm Potentiometer" + m_armPotentiometer.get());
    }

    public void extesionOut() {
      m_extensionMotor.set(Constants.ArmConstants.armExtensionSpeed);
      m_relay.set(Relay.Value.kReverse);
    }

    public void extesionIn() {
      m_extensionMotor.set(-Constants.ArmConstants.armExtensionSpeed);
      m_relay.set(Relay.Value.kReverse);
    }

    public void stopArm(){
      m_armRotaionMotor.setSpeed(0);
      m_armMotorP2.setSpeed(0);
      //m_baseMotor.setSpeed(0);
      m_extensionMotor.set(0);
      m_scorer.set(0);
      //m_wrist.set(0);
    }

    public void moveUpPoint(){

      if(m_armPotentiometer.get() < 150){

        m_armRotaionMotor.setSpeed(-.4);
        m_armMotorP2.setSpeed(-.4);

        System.out.println("Arm Potentiometer" + m_armPotentiometer.get());

      }
      else if (m_armPotentiometer.get() > 150){
        
       m_armRotaionMotor.setSpeed(0);
       m_armMotorP2.setSpeed(0);
        //System.out.println("more that set point " + m_armPotentiometer.get());
      }
      else {
       m_armRotaionMotor.setSpeed(0);
       m_armMotorP2.setSpeed(0);
       // System.out.println("at set point" + m_armPotentiometer.get());
      }
    }

    public void moveDownPoint() {
 

      if (m_armPotentiometer.get() > 69){
        m_armRotaionMotor.setSpeed(.5);
        m_armMotorP2.setSpeed(.5);

        
        
      }
      else if (m_armPotentiometer.get() < 69){
        m_armRotaionMotor.setSpeed(0);
        m_armMotorP2.setSpeed(0);
      }
      else {
        m_armRotaionMotor.setSpeed(0);
        m_armMotorP2.setSpeed(0);
      }

    }

    public void theScorer(){
     m_scorer.set(1);
     m_relay.set(Relay.Value.kReverse);
    }

    public void theReverseScorer(){
      m_scorer.set(-1);
      m_relay.set(Relay.Value.kReverse);
    }

    public void scorerStop(){
      m_scorer.set(0);
      m_relay.set(Relay.Value.kForward);
    }

    public void wristUp(){
      m_wrist.set(.5);
      m_relay.set(Relay.Value.kReverse);
    }
    public void wristDown(){
      m_wrist.set(-.5);
      m_relay.set(Relay.Value.kReverse);
    }
    public void wristStop(){
      m_wrist.set(0);
    }

    public void periodic(){


     // SmartDashboard.putNumber("Base pot ", m_basePotentiometer.get());
      SmartDashboard.putNumber("Arm Pot ", m_armPotentiometer.get());

      //System.out.println("Arm Potentiometer" + m_armPotentiometer.get());
  }
}