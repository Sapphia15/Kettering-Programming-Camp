/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class HatchToCargoShip extends Command {

  private int state;


  public HatchToCargoShip() {
    
    
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveSubsystem);
    state=0;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    while (state<100){
      Robot.updateSDB();
      switch(state){
        case 0:
          //init
          //Drive forward
          //Robot.driveSubsystem.mechDrive(0, 0, -0);
          state=1;
        break;

        case 1:
          //drive up to cargo
          double percentF=-.015*(RobotMap.targCargo1-Robot.driveSubsystem.getAverageEncoderValue());
          if (percentF>.1){
            percentF=.1;
          }
          double percentR;
          if (Math.abs(Robot.driveSubsystem.gyro.getAngleY())>.5){
            percentR=0;
          } else {
            percentR=.015*(0-Robot.driveSubsystem.gyro.getAngleY());
            if (percentR>.1){
              percentR=.1;
            }
          }

          Robot.driveSubsystem.mechDrive(0, percentR, percentF);
          if (Robot.driveSubsystem.getAverageEncoderValue()>=RobotMap.targCargo1){
            terminateSM();
          }
        break;
      }
    }
        
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

  /**Terminates the state machine process
   * 
   */
  public void terminateSM(){
    Robot.driveSubsystem.mechDrive(0, 0, 0);
    state=100;
    System.out.println("Autonomus Process Complete");
  }
}
