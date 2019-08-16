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
import frc.robot.subsystems.DriveSubsystem;

public class AroundTheCargoShip extends Command {
private boolean isFinished;
private int state;
private int runs;
  public AroundTheCargoShip() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveSubsystem);
    state=0;
    runs=0;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
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
          //Robot.driveSubsystem.mechDrive(0, 0, -0);
          startTime=System.currentTimeMillis();
          state=1;
        break;

        case 1:
          //drive until other side of cargo ship
          state=Robot.driveSubsystem.mechP(1,.3,0.2, RobotMap.targCargoEdge1,0,.05);
          if (System.currentTimeMillis()>startTime+10500){
            System.out.println("State 1 timed out.");
            state=2;
          }
        break;
        case 2:
          Robot.driveSubsystem.zeroEncoders();
          //Robot.driveSubsystem.gyro.calibrate();
          //Robot.driveSubsystem.gyro.reset();
          /*System.out.println("Recalibrating gyro...");
          try {
            Thread.sleep(2000);
          } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
          } 
          System.out.println("Gyro recalibrated!");*/
          startTime=System.currentTimeMillis();
          state=3;
        break;
        case 3:
          //turn 90 degrees right
          //System.out.println("In state 2");
          state=Robot.driveSubsystem.mechPRot(3, .25,RobotMap.targAngle2,.03);
          if (System.currentTimeMillis()>startTime+3500){
            System.out.println("State 3 timed out.");
            state=4;
          }
        break;
        case 4:
          Robot.driveSubsystem.zeroEncoders();
         //Robot.driveSubsystem.gyro.reset();
          /*Robot.driveSubsystem.gyro.calibrate();
          System.out.println("Recalibrating gyro...");
          try {
            Thread.sleep(2000);
          } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
          } 
          System.out.println("Gyro recalibrated!");*/
          startTime=System.currentTimeMillis();
          System.out.println("Entering state 5");
          state=5;
        break;
        case 5:
          Robot.driveSubsystem.mechP(5, .3, .2, RobotMap.targCargoEdge2,90);
          if (System.currentTimeMillis()>startTime+10500){
            System.out.println("State 5 timed out.");
            startTime=System.currentTimeMillis();
            state=6;
          }
        break;
        case 6:
        //turn 90 degrees right
          //System.out.println("In state 6");
          state=Robot.driveSubsystem.mechPRot(6, .25,RobotMap.targAngle3,.03);
          if (System.currentTimeMillis()>startTime+3500){
            System.out.println("State 6 timed out.");
            state=7;
          }
        break;
        case 7:
          Robot.driveSubsystem.zeroEncoders();
          startTime=System.currentTimeMillis();
          state=8;
        break;
        case 8:
          //drive until other side of cargo ship
          if (runs==0){
            state=Robot.driveSubsystem.mechP(8,.3,0.25, RobotMap.targCargoEdge3,RobotMap.targAngle3,.05);
          } else {
            state=Robot.driveSubsystem.mechP(8,.3,0.25, RobotMap.targCargoEdge3+15,RobotMap.targAngle3,.05);
          }
          if (System.currentTimeMillis()>startTime+10500){
            System.out.println("State 8 timed out.");
            state=9;
          }
        break;
        case 9:
          startTime=System.currentTimeMillis();
          state=10;
        break;
        case 10:
          state=Robot.driveSubsystem.mechPRot(10, .25,RobotMap.targAngle4,.03);
          if (System.currentTimeMillis()>startTime+3500){
            System.out.println("State 10 timed out.");
            state=11;
          }
        break;
        case 11:
          Robot.driveSubsystem.zeroEncoders();
          startTime=System.currentTimeMillis();
          state=12;
        break;
        case 12:
          //drive until other side of cargo ship
          state=Robot.driveSubsystem.mechP(12,.3,0.25, RobotMap.targCargoEdge4,RobotMap.targAngle4,.05);
          if (System.currentTimeMillis()>startTime+10500){
            System.out.println("State 12 timed out.");
            state=13;
          }
        break;
        case 13:
          startTime=System.currentTimeMillis();
          state=14;
        break;
        case 14:
          state=Robot.driveSubsystem.mechPRot(14, .25,RobotMap.targAngle5,.03);
          if (System.currentTimeMillis()>startTime+3500){
            System.out.println("State 14 timed out.");
            state=15;
          }
        break;
        case 15:
          Robot.driveSubsystem.zeroEncoders();
          Robot.driveSubsystem.gyro.reset();
          //System.out.println("State 13");
          startTime=System.currentTimeMillis();
          state=16;
        break;
        case 16:
          if (runs<1){
            runs++;
            state=1;
          } else {
            state=99;
          }
        break;
        case 99:
          //Robot.driveSubsystem.mechDrive(0, 0, (-.12*(RobotMap.targLine2-Robot.driveSubsystem.getAverageEncoderValue())));
          //if (System.currentTimeMillis()>startTime+1000){
            Robot.driveSubsystem.mechDrive(0, 0, 0);
            state=100;
            System.out.println("Autonomus Process Complete");
            Robot.kill();
          //}
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

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
