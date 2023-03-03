// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.*;


public class AlignZToTargetCMD extends CommandBase {

  //variables
  Limelight3Subsystem limelight3Subsystem;
  PIDController AlignZController;//this will go forward and back to align. 
  
  double kZAprilP = .6;
  double kZAprilI = 0.001;
  double kZAprilD = 0.0001;

  double kZRetroLowP = 6.0;
  double kZRetroLowI = 0.020;
  double kZRetroLowD = 0.0001;

  //constructor method called when object is made.
  public AlignZToTargetCMD(Limelight3Subsystem ThisLimelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    limelight3Subsystem = ThisLimelight;
    addRequirements(limelight3Subsystem);
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    AlignZController =  new PIDController(kZAprilP,kZAprilI,kZAprilD); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double min_forward_command = 0.1f;

    double tagArea_error = limelight3Subsystem.getArea();
    double steering_adjust = 0.0;
    double distance_adjust =  0.0;
    
    double RetroHighMustBeThisSizeBeforeAutomating = .021;
    double RetroHigh_tagArea_setpoint = .23;
    double RetroLowMustBeThisSizeBeforeAutomating = .06;
    double RetroLow_tagArea_setpoint = .30;
    double AprilMustBeThisSizeBeforeAutomating = .4;
    double April_tagArea_setpoint = 3.0;
    
    //these are the variables that will get used of the 3 options. 
    double SizeNeededForAuto = 0;
    double tagArea_setpoint = 0;

    //depending on the pipeline set the setpoint and outer limit.
    if(limelight3Subsystem.getPipeline() == limelight3Subsystem.kpipelineAprilTags){
      tagArea_setpoint = April_tagArea_setpoint;
      SizeNeededForAuto = AprilMustBeThisSizeBeforeAutomating;
      AlignZController.setPID(kZAprilP,kZAprilI,kZAprilD);
    }
    else if(limelight3Subsystem.getPipeline() == limelight3Subsystem.kpipelineRetroflectiveHighRung){
      tagArea_setpoint = RetroHigh_tagArea_setpoint;
      SizeNeededForAuto = RetroHighMustBeThisSizeBeforeAutomating;
      AlignZController.setPID(kZRetroLowP, kZRetroLowI, kZRetroLowD);
    }
    else if(limelight3Subsystem.getPipeline() == limelight3Subsystem.kpipelineRetroflectiveLowerRung){
      tagArea_setpoint = RetroLow_tagArea_setpoint;
      SizeNeededForAuto = RetroLowMustBeThisSizeBeforeAutomating;
      AlignZController.setPID(kZRetroLowP, kZRetroLowI, kZRetroLowD);
    }

    if (Math.abs(tagArea_error) > SizeNeededForAuto)
    {
      distance_adjust = AlignZController.calculate(tagArea_error,tagArea_setpoint); //KpDistance * distance_error;/// we are ignoring Y (up down) for now. 

    }
    
    //driveSubsystem.ArcadeDrivemotors(-distance_adjust, 0);
    SmartDashboard.putNumber("distance_adjust", distance_adjust);
    SmartDashboard.putNumber("steering_adjust", steering_adjust);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
