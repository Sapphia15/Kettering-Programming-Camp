/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TeleopDriveCommand extends Command {
  public TeleopDriveCommand() {
    requires(Robot.driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //Robot.driveSubsystem.tankDrive(Robot.m_oi.stick.getRawAxis(1), Robot.m_oi.stick.getRawAxis(3));
    Robot.driveSubsystem.mechDrive(Robot.m_oi.stick.getRawAxis(0), Robot.m_oi.stick.getRawAxis(2)*.5,Robot.m_oi.stick.getRawAxis(1));
    //Robot.driveSubsystem.wristMove(Robot.m_oi.stick.getRawAxis(1));
    //System.out.println(Robot.m_oi.stick.getRawAxis(1));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
