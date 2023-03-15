// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SmartDashboardHandler extends SubsystemBase {

    

  RobotContainer thisrobot;
  public SmartDashboardHandler(RobotContainer mainbrain) {
    thisrobot = mainbrain;

    bootupPersistents();
  }



  private void bootupPersistents() {

    if(!SmartDashboard.containsKey("Jow Speed Multiplier"))
        {
            SmartDashboard.putNumber("Jow Speed Multiplier", .25);
            SmartDashboard.setPersistent("Jow Speed Multiplier");
        }
        if(!SmartDashboard.containsKey("Jow Rotation Multiplier"))
        {
            SmartDashboard.putNumber("Jow Rotation Multiplier", .25);
            SmartDashboard.setPersistent("Jow Rotation Multiplier");
        }
  }

 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getdataToDashboard();
   // VibeOnZero();
    
  }
 
  public void getdataToDashboard()
  {
    SmartDashboard.putBoolean("teleOpEnable", DriverStation.isTeleopEnabled());
    SmartDashboard.putBoolean("AutonEnable", DriverStation.isAutonomousEnabled());
    SmartDashboard.putString("Alliance", DriverStation.getAlliance().toString());
    SmartDashboard.putString("CowboyMode", thisrobot.cowboyMode.toString());
  }


}
