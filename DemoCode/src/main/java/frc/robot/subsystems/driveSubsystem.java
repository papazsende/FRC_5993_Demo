// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



  /* Motorlardaki encoder'ları tanımlıyoruz, böylelikle motorların dönüşlerine dair bilgi edinebileceğiz */


  /* Differential Drive tanımlıyoruz  */

  private DifferentialDrive differentialDrive = new DifferentialDrive(leftMotor,rightMotor);



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
