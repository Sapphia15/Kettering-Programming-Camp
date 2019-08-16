/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  //Deadzone
  public static double deadzone = 0.2;

  // Spark Brushless Motors
  public final static int R_F_T = 1; 
  public final static int R_F_B = 10; //Original was 0
   
  public final static int R_R_T = 3; 
  public final static int R_R_B = 4; 

  public final static int L_F_T = 8; 
  public final static int L_F_B = 17; //Original was 7

  public final static int L_R_T = 6; 
  public final static int L_R_B = 5; //Original was 5

  public final static int hab_pegs = 11;

  public final static int elevator = 12;

  public final static double FF=1.66666666666;

  //Target Positions
  public static final double targLine1=116/FF;
  public static final double targLine2=252/FF;
  //public static final double targLeftTurn1=3.84523475170135/*4.642852067947385*/;
  //public static final double targRightTurn1=7.11310529708862/*4.82142329216003*/;
  public static final double targAngle1=-30;
  public static final double targRocket1=12/FF;
  public static final double targCargo1=54.4; //100 for hab climb  54.4 for hatch to cargo
  public static final double targCargoEdge1=370/FF;
  public static final double targAngle2=86; //first turn around cargo ship
  public static final double targCargoEdge2=160/FF;
  public static final double targAngle3=176;
  public static final double targCargoEdge3=325/FF;
  public static final double targAngle4=266;
  public static final double targCargoEdge4=160/FF;
  public static final double targAngle5=356;
  public static final double targCargoEdge5=325/FF;
  
  


}
