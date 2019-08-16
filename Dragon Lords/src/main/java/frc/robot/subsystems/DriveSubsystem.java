/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ADIS16448;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.TeleopDriveCommand;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {

  public CANSparkMax rft, rfb, rrt, rrb, lft, lfb, lrt, lrb;
  private TalonSRX wrist;
  public ADIS16448 gyro;

  public DriveSubsystem() {
    rft = new CANSparkMax(RobotMap.R_F_T, MotorType.kBrushless);
    rfb = new CANSparkMax(RobotMap.R_F_B, MotorType.kBrushless);

    rrt = new CANSparkMax(RobotMap.R_R_T, MotorType.kBrushless);
    rrb = new CANSparkMax(RobotMap.R_R_B, MotorType.kBrushless);

    lft = new CANSparkMax(RobotMap.L_F_T, MotorType.kBrushless);
    lfb = new CANSparkMax(RobotMap.L_F_B, MotorType.kBrushless);

    lrt = new CANSparkMax(RobotMap.L_R_T, MotorType.kBrushless);
    lrb = new CANSparkMax(RobotMap.L_R_B, MotorType.kBrushless);

    wrist = new TalonSRX(16);

    gyro = new ADIS16448();

    gyro.calibrate();
    zeroEncoders();

  }

  public void tankDrive(double leftPower, double rightPower) {
    rightPower = -rightPower;
    rft.set(rightPower);
    rfb.set(rightPower);

    rrt.set(rightPower);
    rrb.set(rightPower);

    lft.set(leftPower);
    lfb.set(leftPower);

    lrt.set(leftPower);
    lrb.set(leftPower);
  }

  /**
   * Cartesian drive method that specifies speeds in terms of the field
   * longitudinal and lateral directions, using the drive's angle sensor to
   * automatically determine the robot's orientation relative to the field.
   * <p>
   * Using this method, the robot will move away from the drivers when the
   * joystick is pushed forwards, and towards the drivers when it is pulled
   * towards them - regardless of what direction the robot is facing.
   * 
   * @param x        The speed that the robot should drive in the X direction.
   *                 [-1.0..1.0]
   * @param y        The speed that the robot should drive in the Y direction.
   *                 This input is inverted to match the forward == -1.0 that
   *                 joysticks produce. [-1.0..1.0]
   * @param rotation The rate of rotation for the robot that is completely
   *                 independent of the translation. [-1.0..1.0]
   */
  public void mechDrive(double x, double y, double rotation) {
    double xIn = dzify(x);
    double yIn = dzify(y);
    rotation = dzify(rotation);
    // Negate y for the joystick.
    yIn = -yIn;
    // Compensate for gyro angle.
    double r = Math.hypot(xIn, yIn);
    double robotAngle = Math.atan2(yIn, xIn) - Math.PI / 4;
    double rightX = rotation;
    final double v1 = r * Math.cos(robotAngle) + rightX; // Left Front
    final double v2 = r * Math.sin(robotAngle) - rightX; // Right Front
    final double v3 = r * Math.sin(robotAngle) + rightX; // Left Rear
    final double v4 = r * Math.cos(robotAngle) - rightX; // Right Rear

    lft.set(v1);
    lfb.set(v1);
    lrt.set(v3);
    lrb.set(v3);
    rft.set(v2);
    rfb.set(v2);
    rrt.set(v4);
    rrb.set(v4);
  }

  public void wristMove(double power) {
    wrist.set(ControlMode.PercentOutput, power);
  }

  /**
   * Disable all drive motors
   */

  public void disable() {
    lft.set(0);
    lfb.set(0);
    lrt.set(0);
    lrb.set(0);
    rft.set(0);
    rfb.set(0);
    rrt.set(0);
    rrb.set(0);

    wrist.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Enables all drive motors
   */
  public void enable() {
    lft.set(0);
    lfb.set(0);
    rft.set(0);
    rfb.set(0);
    lrt.set(0);
    lrb.set(0);
    rrt.set(0);
    rrb.set(0);

    wrist.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Deadzonifies a double to the set deadzone in RobotMap
   * 
   * @param A double between -1 and 1
   * @return A double that is now been deadzoned
   */
  private double dzify(double value) {
    double deadzone = RobotMap.deadzone;
    if (value > deadzone || value < -deadzone) {
      return value;
    }
    return 0.0;
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new TeleopDriveCommand());
  }

  /**
   * Returns the encoder value of a specific spark max motor
   * 
   * @param Motor
   * @return
   */
  public double getEncoderValue(CANSparkMax Motor) {
    return Motor.getEncoder().getPosition();
  }

  /**
   * Returns the average of the right encoder values and the inverted left encoder
   * values (since they are facing opposite directions)
   * 
   * @return
   */
  public double getAverageEncoderValue() {
    return ((getEncoderValue(lft) + getEncoderValue(lfb) + getEncoderValue(lrt) + getEncoderValue(lrb)) * -1
        + getEncoderValue(rft) + getEncoderValue(rfb) + getEncoderValue(rrt) + getEncoderValue(rrb)) / 8;
  }

  public double getAverageLeftEncoderValue() {
    return (getEncoderValue(lft) + getEncoderValue(lfb) + getEncoderValue(lrt) + getEncoderValue(lrb)) / 4;
  }

  public double getAverageRightEncoderValue() {
    return (getEncoderValue(rft) + getEncoderValue(rfb) + getEncoderValue(rrt) + getEncoderValue(rrb)) / 4;
  }

  /**
   * Resets all encoder values
   * 
   */
  public void zeroEncoders() {
    rft.getEncoder().setPosition(0);
    rfb.getEncoder().setPosition(0);
    rrt.getEncoder().setPosition(0);
    rrb.getEncoder().setPosition(0);
    lft.getEncoder().setPosition(0);
    lfb.getEncoder().setPosition(0);
    lrt.getEncoder().setPosition(0);
    lrb.getEncoder().setPosition(0);
  }

  /**
   * Returns the number of rotations required to move a specified amount of inches
   * forward
   * 
   * @param inches the number of inches to move
   * @return
   */
  public double inchesToRotations(double inches) {
    return inches / RobotMap.FF;
  }

  public int mechP(int currentState, double maxPercentF, double maxPercentR, double targetF, double targetR, double P,
      double minPercentF) {
    double position = getAverageEncoderValue();
    double percentF;
    // SmartDashboard.putNumber("percent F", percentF);

    if (position < targetF + .12 && position > targetF - .12) {
      percentF = 0;
      System.out.println("Deadband F");
    } else {
      percentF = -P * (targetF - position);
      if (Math.abs(percentF) > maxPercentF) {
        percentF = maxPercentF * (percentF / Math.abs(percentF));
        // SmartDashboard.putNumber("percent F", percentF);
      }
      // if (percentF<minPercentF){
      // percentF=minPercentF*(percentF/Math.abs(percentF));
      // }
    }

    double percentR;
    if (Robot.driveSubsystem.gyro.getAngleY() < targetR + .2 && Robot.driveSubsystem.gyro.getAccelY() > targetR - .2) {
      percentR = 0;
      System.out.println("Deadband R");
    } else {
      percentR = P * (targetR - Robot.driveSubsystem.gyro.getAngleY());
      if (Math.abs(percentR) > maxPercentR) {
        percentR = maxPercentR*(percentR/Math.abs(percentR));
      }
    }

    // System.out.println("Pos: "+position+" targF: "+targetF+" percentF:
    // "+percentF);

    mechDrive(0, percentR, percentF);

    // SmartDashboard.putNumber("percent F", percentF);
    // SmartDashboard.putNumber("percent R", percentR);

    if (Robot.driveSubsystem.getAverageEncoderValue() >= targetF - .3
        && Robot.driveSubsystem.getAverageEncoderValue() <= targetF + .3) {
      return currentState + 1;
    } else {
      return currentState;
    }
  }

  public int mechP(int currentState, double maxPercentF, double maxPercentR, double targetF, double targetR, double P) {
    return mechP(currentState, maxPercentF, maxPercentR, targetF, targetR, .015, 0);
  }

  public int mechP(int currentState, double maxPercentF, double maxPercentR, double targetF, double targetR) {
    return mechP(currentState, maxPercentF, maxPercentR, targetF, targetR, .015);
  }

  public int mechP(int currentState, double maxPercentF, double maxPercentR, double targetF) {
    return mechP(currentState, maxPercentF, maxPercentR, targetF, 0);
  }

  public int mechP(int currentState, double maxPercentF, double targetF) {
    return mechP(currentState, maxPercentF, .1, targetF, 0);
  }

  /**
   * 
   * @param currentState
   * @param targetF
   * @return
   */
  public int mechP(int currentState, double targetF) {
    return mechP(currentState, .1, .1, targetF, 0);
  }

  public int mechPRot(int currentState, double maxPercentR, double targetR, double P) {
    // SmartDashboard.putNumber("percent F", percentF);

    double percentR;
    if (Robot.driveSubsystem.gyro.getAngleY() < targetR + .2 && Robot.driveSubsystem.gyro.getAccelY() > targetR - .2) {
      percentR = 0;
      System.out.println("Deadband R");
    } else {
      percentR = P * (targetR - Robot.driveSubsystem.gyro.getAngleY());
      if (Math.abs(percentR) > maxPercentR) {
        percentR = maxPercentR*(percentR/Math.abs(percentR));
      }
    }

     System.out.println("targR: "+targetR+" percentR:"+percentR);

    mechDrive(0, percentR, 0);

    // SmartDashboard.putNumber("percent F", percentF);
    // SmartDashboard.putNumber("percent R", percentR);

    if (gyro.getAngleY() > targetR - .25 && gyro.getAngleY() < targetR + .25) {
      return currentState + 1;
    } else {
      return currentState;
    }
  }

  public int mechPRot(int currentState, double maxPercentR, double targetR) {
    return mechPRot(currentState, maxPercentR, targetR, .3);
  }

  public int mechPRot(int currentState, double targetR) {
    return mechPRot(currentState, 1, targetR, .03);
  }
}
// &&