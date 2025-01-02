// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;




import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class driveSubsystem extends SubsystemBase {
  /** Yeni bir Drive Alt sistemi (Subsystem) Oluşturalım */

  /* Öcelikle SparkMax'leri tanımlıyoruz ve motor tiplerini ayarlıyoruz  */
  //private CANSparkMax leftMotor = new CANSparkMax(0,MotorType.kBrushed);
  //private CANSparkMax rightMotor = new CANSparkMax(1,MotorType.kBrushed);
  private PWMSparkMax leftMotor = new PWMSparkMax(0);
  private PWMSparkMax rightMotor = new PWMSparkMax(1);
  //private PWMSparkMax leftMotor = new PWMSparkMax(0);
  //private PWMSparkMax rightMotor = new PWMSparkMax(1);
  
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
  // PID Kontrol Metodu
/* 
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
*/
  /* 
  public void setSmoothDrive(double joystickDrive, double joystickTurn) {
   // Joystick girişlerini yumuşatmak için bir katsayı kullanıyoruz (örnek: 0.1)
    targetDrive = joystickDrive;
    targetTurn = joystickTurn;
    targetDrive = -targetDrive;

    // PID hata (error) hesaplaması
    double driveOutput = joyfilter.calculate(targetDrive);
    double turnOutput = targetTurn * 0.7;

    // Motor yönleri ters olduğundan, sağ motorun yönünü değiştireceğiz
    double leftOutput = driveOutput + turnOutput;   // Sol motorun çıkışı
    double rightOutput = driveOutput - turnOutput;  // Sağ motorun çıkışı

    // Motorlara komut gönder
    differentialDrive.tankDrive(leftOutput, rightOutput);

    // Mevcut hızları güncelle
    currentDrive = leftOutput; // Sol motorun çıkışı
    currentTurn = turnOutput;  // Dönüş çıkışı

    // SmartDashboard için PID çıktılarını ekleyelim
    SmartDashboard.putNumber("Joysstick Y Axis", joystickDrive);
    SmartDashboard.putNumber("Joystick X Axis", joystickTurn);
    SmartDashboard.putNumber("Target Drive", targetDrive);
    SmartDashboard.putNumber("Target Turn", targetTurn);
    SmartDashboard.putNumber("Turn Output", turnOutput);
    SmartDashboard.putNumber("Drive Output", leftOutput);
    SmartDashboard.putNumber("left Output", leftOutput);
    SmartDashboard.putNumber("Right Output", rightOutput);
 


  }
   */
  public void set(double joystickDrive, double joystickTurn){

    differentialDrive.arcadeDrive(-joystickDrive, joystickTurn);
  }

  public void stop(){
    differentialDrive.stopMotor();
  }
}
