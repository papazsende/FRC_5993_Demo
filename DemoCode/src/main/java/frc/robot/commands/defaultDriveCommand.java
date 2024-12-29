// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.driveSubsystem;

public class defaultDriveCommand extends Command {
  /** Creates a new defaultDriveCommand. */
  private driveSubsystem DRIVE_SUBSYSTEM;
  private Joystick JOYSTICK;
  double drive;
  double turn;

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
    DRIVE_SUBSYSTEM.setSmoothDrive(targetDrive, targetTurn);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DRIVE_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
