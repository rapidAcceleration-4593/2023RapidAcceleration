package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.PWM;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.armCommands.armUp;

public class arm extends SubsystemBase {

    private AnalogPotentiometer m_basePotentiometer;
    private AnalogPotentiometer m_armPotentiometer;

    private CANSparkMax m_extensionMotor;
    private PWM m_baseMotor;
    private PWM m_armRotaionMotor;
    private SparkMaxPIDController m_ArmExtenPidController;

  
    public arm() {
      m_basePotentiometer = new AnalogPotentiometer(1, 180, 0);
      m_armPotentiometer = new AnalogPotentiometer(2, 180, 0);
      m_baseMotor = new PWM(1);
      m_armRotaionMotor = new PWM(2);
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
    }
    public void periodic(){

      System.out.println("Base Potentiometer" + m_basePotentiometer.get());

    }
  }