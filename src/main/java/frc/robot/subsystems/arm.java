package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.PWM;
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

public class arm extends SubsystemBase {

    private AnalogPotentiometer m_basePotentiometer;
    private AnalogPotentiometer m_armPotentiometer;

    private CANSparkMax m_extensionMotor;
    private PWM m_baseMotor;
    private PWM m_armRotaionMotor;
    private PWM m_scorer;
    private PWM m_wrist;

    private SparkMaxPIDController m_ArmExtenPidController;
    private CANSparkMax m_motorTest;

    public arm() {
      m_basePotentiometer = new AnalogPotentiometer(1, 314, 0);
      m_armPotentiometer = new AnalogPotentiometer(2, 314, 0);
      m_baseMotor = new PWM(1);
      m_armRotaionMotor = new PWM(2);
      m_scorer = new PWM(4);
      m_wrist = new PWM(3);
      m_extensionMotor = new CANSparkMax(14, MotorType.kBrushless);
    }

    public void baseRotateLeft () {
      m_baseMotor.setSpeed(Constants.ArmConstants.baseRotateSpeed);
    }

    public void baseRotateRight (){
      m_baseMotor.setSpeed(-Constants.ArmConstants.baseRotateSpeed);

      System.out.println("Base Potentiometer" + m_basePotentiometer.get());
    }

    public void armRotateUp () {
      m_armRotaionMotor.setSpeed(Constants.ArmConstants.armRotateSpeed);

      System.out.println("arm Potentiometer" + m_armPotentiometer.get());
    }

    public void armRotateDown () {
      m_armRotaionMotor.setSpeed(-Constants.ArmConstants.armRotateSpeed);

      System.out.println("arm Potentiometer" + m_armPotentiometer.get());
    }

    public void extesionOut() {
      m_extensionMotor.set(Constants.ArmConstants.armExtensionSpeed);
    }

    public void extesionIn() {
      m_extensionMotor.set(-Constants.ArmConstants.armExtensionSpeed);
    }

    public void stopArm(){
      m_armRotaionMotor.setSpeed(0);
      m_baseMotor.setSpeed(0);
      m_extensionMotor.set(0);
      m_scorer.setSpeed(0);
      m_wrist.setSpeed(0);
    }

    public void moveUpPoint(){

      if(m_armPotentiometer.get() < 120){

        m_armRotaionMotor.setSpeed(Constants.ArmConstants.armRotateSpeed);

      }
      else if (m_armPotentiometer.get() > 130){
        
       m_armRotaionMotor.setSpeed(-Constants.ArmConstants.baseRotateSpeed);
        //System.out.println("more that set point " + m_armPotentiometer.get());
      }
      else {
       m_armRotaionMotor.setSpeed(0);
       // System.out.println("at set point" + m_armPotentiometer.get());
      }
    }

    public void moveDownPoint() {
      m_armRotaionMotor.setSpeed(-.5);
    }

    public void movePointRight(){
      
      double speedAdjust = 1;

      // if(m_basePotentiometer.get() >  120 || m_basePotentiometer.get() < 150){
      //   speedAdjust = .66;
      // }
      // else {
      //   speedAdjust = 1;
      // }

      if(m_basePotentiometer.get() < 135){

        m_baseMotor.setSpeed(speedAdjust*Constants.ArmConstants.baseRotateSpeed);
        //System.out.println("less than set point " + m_armPotentiometer.get());
      }
      else if (m_basePotentiometer.get() > 140){
        
        m_baseMotor.setSpeed(speedAdjust*-Constants.ArmConstants.baseRotateSpeed);
        //System.out.println("more that set point " + m_armPotentiometer.get());
      }
      else {
        m_baseMotor.setSpeed(0);
       // System.out.println("at set point" + m_armPotentiometer.get());
      }
    }

    public void movePointLeft(){
      
      double speedAdjust = 1;

      // if(m_armPotentiometer.get() >  190 || m_armPotentiometer.get() < 210){
      //   speedAdjust = .66;
      // }
      // else {
      //   speedAdjust = 1;
      // }
      
      if(m_basePotentiometer.get() < 200){
        m_baseMotor.setSpeed(speedAdjust*Constants.ArmConstants.baseRotateSpeed);
        //System.out.println("less than set point " + m_armPotentiometer.get());
      }
      else if (m_basePotentiometer.get() > 205){
        m_baseMotor.setSpeed(speedAdjust*-Constants.ArmConstants.baseRotateSpeed);
        //System.out.println("more that set point " + m_armPotentiometer.get());
      }
      else {
        m_baseMotor.setSpeed(0);
       // System.out.println("at set point" + m_armPotentiometer.get());
      }
    }

    public void theScorer(){
     m_scorer.setSpeed(1);

    }

    public void theReverseScorer(){
      m_scorer.setSpeed(-1);
    }

    public void wristUp(){
      m_wrist.setSpeed(.775);
    }
    public void wristDown(){
      m_wrist.setSpeed(-.775);
    }

    public void periodic(){

      SmartDashboard.putNumber("Base pot ", m_basePotentiometer.get());
      SmartDashboard.putNumber("Arm Pot ", m_armPotentiometer.get());

      //System.out.println("Arm Potentiometer" + m_armPotentiometer.get());
  }
}