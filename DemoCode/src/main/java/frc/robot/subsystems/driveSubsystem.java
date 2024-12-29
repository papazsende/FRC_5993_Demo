// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class driveSubsystem extends SubsystemBase {
  /** Yeni bir Drive Alt sistemi (Subsystem) Oluşturalım */

  /* Öcelikle SparkMax'leri tanımlıyoruz ve motor tiplerini ayarlıyoruz  */
  private CANSparkMax leftMotor = new CANSparkMax(0,MotorType.kBrushed);
  private CANSparkMax rightMotor = new CANSparkMax(1,MotorType.kBrushed);
  
  //private PWMSparkMax leftMotor = new PWMSparkMax(0);
  //private PWMSparkMax rightMotor = new PWMSparkMax(1);
  
  /* Motorlardaki encoder'ları tanımlıyoruz, böylelikle motorların dönüşlerine dair bilgi edinebileceğiz */

  private RelativeEncoder leftMotorEncoder = leftMotor.getEncoder();
  private RelativeEncoder rightMotorEncoder = rightMotor.getEncoder();

  /* Differential Drive tanımlıyoruz  */

  private DifferentialDrive differentialDrive = new DifferentialDrive(leftMotor,rightMotor);
  private double kP = 0.1;  // Proportional parametresi
  private double kI = 0.01; // Integral parametrei
  private double kD = 0.01; // Derivative parametresi
  
  // Mevcut hızlar (drive ve turn için)
  private double currentDrive = 0;
  private double currentTurn = 0;

  // Hedef hızlar (Joystick'ten gelen)
  private double targetDrive = 0;
  private double targetTurn = 0;
  
  private double integral = 0.0;     // Toplam hata (Integral terimi için)
  private double previousError = 0.0; // Bir önceki hata (Derivative terimi için)

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

    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);
    SmartDashboard.putNumber("Target Drive", targetDrive);
    SmartDashboard.putNumber("Target Turn", targetTurn);
    //SmartDashboard.putNumber("Left Encoder", leftMotorEncoder.getPosition());
   // SmartDashboard.putNumber("Right Encoder", rightMotorEncoder.getPosition());



  }
  // PID Kontrol Metodu

  public double calculatePID(double targetSpeed, double currentSpeed) {
      // Hata hesaplama (Hedef hız - Mevcut hız)
      double error = targetSpeed - currentSpeed;

      // Integral terimi: Hata birikimi (0.02 saniye döngü süresi)
      integral += error * 0.02;

      // Derivative terimi: Hatanın değişim oranı
      double derivative = (error - previousError) / 0.02;

      // Hatanın bir önceki değerini güncelle
      previousError = error;

      // PID formülü: P + I + D
      return kP * error + kI * integral + kD * derivative;
  }

  
  public void setSmoothDrive(double joystickDrive, double joystickTurn) {
    // Joystick girişlerini yumuşatmak için bir katsayı kullanıyoruz (örnek: 0.1)
    targetDrive = targetDrive + 0.1 * (joystickDrive - targetDrive);
    targetTurn = targetTurn + 0.1 * (joystickTurn - targetTurn);

    // PID hata (error) hesaplaması
    double driveOutput = calculatePID(targetDrive, currentDrive);
    double turnOutput = calculatePID(targetTurn, currentTurn);

    // Motor yönleri ters olduğundan, sağ motorun yönünü değiştireceğiz
    double leftOutput = driveOutput + turnOutput;   // Sol motorun çıkışı
    double rightOutput = driveOutput + turnOutput;  // Sağ motorun çıkışı

    // Motorlara komut gönder
    differentialDrive.tankDrive(leftOutput, rightOutput);

    // Mevcut hızları güncelle
    currentDrive = leftOutput; // Sol motorun çıkışı
    currentTurn = turnOutput;  // Dönüş çıkışı

    // SmartDashboard için PID çıktılarını ekleyelim
    SmartDashboard.putNumber("Drive Output", driveOutput);
    SmartDashboard.putNumber("Turn Output", turnOutput);
  }

  public void stop(){
    differentialDrive.stopMotor();
  }
}
