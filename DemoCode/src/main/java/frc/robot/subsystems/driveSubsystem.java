// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;




import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class driveSubsystem extends SubsystemBase {
  /** Yeni bir Drive Alt sistemi (Subsystem) Oluşturalım */

  /* Öcelikle SparkMax'leri tanımlıyoruz ve motor tiplerini ayarlıyoruz  */
  //private CANSparkMax leftMotor = new CANSparkMax(0,MotorType.kBrushed);
  //private CANSparkMax rightMotor = new CANSparkMax(1,MotorType.kBrushed);
  private PWMSparkMax leftMotor = new PWMSparkMax(0);
  private PWMSparkMax rightMotor = new PWMSparkMax(1);

  
  /* Motorlardaki encoder'ları tanımlıyoruz, böylelikle motorların dönüşlerine dair bilgi edinebileceğiz */


  /* Differential Drive tanımlıyoruz  */

  private DifferentialDrive differentialDrive = new DifferentialDrive(leftMotor,rightMotor);




  public driveSubsystem() {



    /* Eğer iki motora da aynı komutu verirsek daireler çizer, bir tarafın ters olması gerekir */
    rightMotor.setInverted(false);
    leftMotor.setInverted(true);




  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //SmartDashboard.putNumber("kP", kP);
    //SmartDashboard.putNumber("kI", kI);
    //SmartDashboard.putNumber("kD", kD);
    //SmartDashboard.putNumber("Left Encoder", leftMotorEncoder.getPosition());
    // SmartDashboard.putNumber("Right Encoder", rightMotorEncoder.getPosition());



  }


  public void set(double joystickDrive, double joystickTurn){

    differentialDrive.arcadeDrive(-joystickDrive, joystickTurn);
    SmartDashboard.putNumber("Guc", joystickDrive);
  }
  
  public void stop(){
    differentialDrive.stopMotor();
  }
}
