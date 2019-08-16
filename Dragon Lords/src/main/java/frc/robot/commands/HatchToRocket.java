/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class HatchToRocket extends Command {

  private int state;
  private boolean isFinished;
  public HatchToRocket() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveSubsystem);
    
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    state=0;

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double percent;
    long startTime=0;
    while (state<100){
      Robot.updateSDB();
      switch(state){
        case 0:
          //init
          //Drive forward
          Robot.driveSubsystem.mechDrive(0, 0, -0);
          state=1;
        break;

        case 1:
          //drive until first line
          percent=-.015*(RobotMap.targLine1-Robot.driveSubsystem.getAverageEncoderValue());
          if (percent>.07){
            percent=.07;
          }
          Robot.driveSubsystem.mechDrive(0, 0, percent);
          if (Robot.driveSubsystem.getAverageEncoderValue()>RobotMap.targLine1){
            startTime=System.currentTimeMillis();

            state=2;
          }
        break;
        case 2:
          Robot.driveSubsystem.mechDrive(0, 0, (-.1*(RobotMap.targLine1-Robot.driveSubsystem.getAverageEncoderValue())));
          if (System.currentTimeMillis()>startTime+1000){
            state=3;
          }
        break;
        case 3:
          //drive until second line
          percent=-.012*(RobotMap.targLine2-Robot.driveSubsystem.getAverageEncoderValue());
          if (percent>.02){
            percent=.02;
          }
          Robot.driveSubsystem.mechDrive(0, 0, percent);
          if (Robot.driveSubsystem.getAverageEncoderValue()>RobotMap.targLine2){
            startTime=System.currentTimeMillis();

            state=4;
          }
        break;
        case 4:
          Robot.driveSubsystem.mechDrive(0, 0, (-.1*(RobotMap.targLine2-Robot.driveSubsystem.getAverageEncoderValue())));
          if (System.currentTimeMillis()>startTime+1000){
            Robot.driveSubsystem.mechDrive(0, 0, 0);
            Robot.driveSubsystem.zeroEncoders();
            /*System.err.println("Recalibrating Gyro");
            Robot.driveSubsystem.gyro.calibrate();
            try {
              Thread.sleep(2000);
            } catch (InterruptedException e) {
              // TODO Auto-generated catch block
              e.printStackTrace();
            }
            System.err.println("Gyro Recalibrated");
            state=5;*/
          }
        break;
        case 5:
          /*percent=.12*(RobotMap.targAngle1-Robot.driveSubsystem.gyro.getAngleY());
          if (percent>.1){
            percent=.1;
          }*/

          //Robot.driveSubsystem.mechDrive(0, percent/*((RobotMap.targLeftTurn1-Robot.driveSubsystem.getAverageLeftEncoderValue())+(RobotMap.targRightTurn1-Robot.driveSubsystem.getAverageRightEncoderValue()))/2*/, 0);
          state=Robot.driveSubsystem.mechPRot(6,.2,RobotMap.targAngle1);
          /*if (Robot.driveSubsystem.gyro.getAngleY() < RobotMap.targAngle1 /*Robot.driveSubsystem.getAverageLeftEncoderValue() > RobotMap.targLeftTurn1 && Robot.driveSubsystem.getAverageRightEncoderValue() > RobotMap.targRightTurn1*///) {
            /*Robot.driveSubsystem.mechDrive(0, 0, 0);
            startTime=System.currentTimeMillis();
            state=6;
          }*/
        break;
        case 6:
          //state=Robot.driveSubsystem.mechPRot(6,.2,RobotMap.targAngle1);
          //Robot.driveSubsystem.mechDrive(0, percent/*((RobotMap.targLeftTurn1-Robot.driveSubsystem.getAverageLeftEncoderValue())+(RobotMap.targRightTurn1-Robot.driveSubsystem.getAverageRightEncoderValue()))/2*/, 0);
          //if (System.currentTimeMillis()>startTime+2000){
            Robot.driveSubsystem.zeroEncoders();
            state=7;
          //}
        break;
        case 7:
          percent=-.015*(RobotMap.targRocket1-Robot.driveSubsystem.getAverageEncoderValue());
          if (percent>.5){
            percent=.05;
          }
          Robot.driveSubsystem.mechDrive(0, 0, percent);
          if (Robot.driveSubsystem.getAverageEncoderValue()>RobotMap.targRocket1/2){
            //startTime=System.currentTimeMillis();

            state=99;
          }
        break;
        case 8:
          state=99;
        break;
        case 99:
            terminateSM();
        break;

      }
       
    }
    isFinished=true;
    
       
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    cancel();
  }

  /**Terminates the state machine process
   * 
   */
  public void terminateSM(){
    Robot.driveSubsystem.mechDrive(0, 0, 0);
    state=100;
    System.out.println("Autonomus Process Complete");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
