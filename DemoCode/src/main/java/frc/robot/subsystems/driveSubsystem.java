// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class driveSubsystem extends SubsystemBase {
  /** Creates a new driveSubsystem. */

  private CANSparkMax leftMotor = new CANSparkMax(0,MotorType.kBrushless);
  private CANSparkMax rightMotor = new CANSparkMax(2,MotorType.kBrushless);
  
  private RelativeEncoder leftMotorEncoder = leftMotor.getEncoder();
  private RelativeEncoder rightMotorEncoder = rightMotor.getEncoder();

  private DifferentialDrive differentialDrive = new DifferentialDrive(leftMotor,rightMotor);
  
  public driveSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
