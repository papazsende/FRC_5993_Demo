// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class driveSubsystem extends SubsystemBase {
  /** Yeni bir Drive Alt sistemi (Subsystem) Oluşturalım */

  /* Öcelikle SparkMax'leri tanımlıyoruz ve motor tiplerini ayarlıyoruz  */
  private CANSparkMax leftMotor = new CANSparkMax(0,MotorType.kBrushless);
  private CANSparkMax rightMotor = new CANSparkMax(2,MotorType.kBrushless);
  
  /* Motorlardaki encoder'ları tanımlıyoruz, böylelikle motorların dönüşlerine dair bilgi edinebileceğiz */

  private RelativeEncoder leftMotorEncoder = leftMotor.getEncoder();
  private RelativeEncoder rightMotorEncoder = rightMotor.getEncoder();

  /* Differential Drive tanımlıyoruz  */

  private DifferentialDrive differentialDrive = new DifferentialDrive(leftMotor,rightMotor);

  public driveSubsystem() {

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    /* Eğer iki motora da aynı komutu verirsek daireler çizer, bir tarafın ters olması gerekir */
    rightMotor.setInverted(true);
    leftMotor.setInverted(false);

    /*Subsystem içinde encoder pozisyonunu 0'a ayarlıyoruz ve resetliyoruz */
    rightMotorEncoder.setPosition(0);
    leftMotorEncoder.setPosition(0);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Right Encoder", getRightEncoderValue());
    SmartDashboard.putNumber("Left Encoder", getLeftEncoderValue());

  }

  public double getRightEncoderValue(){
    return rightMotorEncoder.getPosition();
  }

  public double getLeftEncoderValue(){
    return leftMotorEncoder.getPosition();
  }

  public void set(double drive,double turn){
    differentialDrive.arcadeDrive(drive, turn);
  }

  public void tankmode(double left, double right){

    differentialDrive.tankDrive(left, right);
  }

  public void stop(){
    differentialDrive.stopMotor();
  }
}
