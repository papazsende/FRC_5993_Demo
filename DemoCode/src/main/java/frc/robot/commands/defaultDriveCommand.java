// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.driveSubsystem;

public class defaultDriveCommand extends Command {
  /** Creates a new defaultDriveCommand. */
  private driveSubsystem DRIVE_SUBSYSTEM;
  private Joystick JOYSTICK;
  double drive;
  double turn;
  SlewRateLimiter driveFilter = new SlewRateLimiter(1);
  SlewRateLimiter turnFilter = new SlewRateLimiter(1);
  LinearFilter testFilter = LinearFilter.singlePoleIIR(0.2, 0.2);
  public defaultDriveCommand(driveSubsystem drive,Joystick joy) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DRIVE_SUBSYSTEM = drive;
    this.JOYSTICK = joy;
    addRequirements(DRIVE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    drive = 0;
    turn = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double targetDrive = JOYSTICK.getRawAxis(1);
    double targetTurn = JOYSTICK.getRawAxis(0);

    double clampedDrive = MathUtil.clamp(targetDrive,-0.8, 0.8);
    double clampedTurn = MathUtil.clamp(targetTurn,-0.8, 0.8);

    //DRIVE_SUBSYSTEM.setSmoothDrive(targetDrive, targetTurn);
    DRIVE_SUBSYSTEM.set(testFilter.calculate(clampedDrive),clampedTurn);
    SmartDashboard.putNumber("Calculated Output", driveFilter.calculate(targetDrive));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
