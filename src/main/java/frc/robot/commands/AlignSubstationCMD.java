// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
//import frc.robot.Constants;
import frc.robot.Subsystems.*;


public class AlignSubstationCMD extends CommandBase {
  




  /** Creates a new ArmStopCMD. */
  Limelight3Subsystem limelight3Subsystem;
  Swerve s_Swerve;

  //LL POSE X is forward and backward toward target in field space
  double k_PoseX_P = 0.20;
  double k_PoseX_I = 0.0025;
  double k_PoseX_D = 0.0025;
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private final PIDController AlignXController = new PIDController(k_PoseX_P,k_PoseX_I,k_PoseX_D);

  //LL POSE Y Is left to right translation in field space
  double k_PoseY_P = 0.3;
  double k_PoseY_I = 0.00;
  double k_PoseY_D = 0.0000; 
  private static final TrapezoidProfile.Constraints Z_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private final PIDController AlignPoseYController = new PIDController(k_PoseY_P,k_PoseY_I,k_PoseY_D);
  
  //LL pose RZ is our rotation relative to the target in field space
  double k_RZ_P = 0.025;
  double k_RZ_I = 0.00;
  double k_RZ_D = 0.00;
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(8, 8);
  private final PIDController AlignRZController = new PIDController(k_RZ_P,k_RZ_I,k_RZ_D);

  
  double minXposeErrorToCorrect = .04;//.04;
  double minYposeErrorToCorrect = .02;//.04;
  double minRZErrorToCorrect = .2;//.04;

  double min_xpose_command = 0.04;
  double min_Ypose_command = 0.0;
  double min_RZ_command = 0;//0.004;

  boolean robotCentric = true;
  
  public AlignSubstationCMD(Swerve Thiss_Swerve, Limelight3Subsystem ThisLimelight) {
    s_Swerve = Thiss_Swerve;
    limelight3Subsystem = ThisLimelight;
    addRequirements(s_Swerve,limelight3Subsystem);    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      
    //LL POSE X is forward and backward toward target in field space
    AlignXController.setSetpoint(-6.8250);
    //AlignXController.setTolerance(minXposeErrorToCorrect);
    //LL POSE Y Is left to right translation in field space
    AlignPoseYController.setSetpoint(2.19);
    //AlignPoseYController.setTolerance(minYposeErrorToCorrect);

    //LL pose RZ is our rotation relative to the target in field space
    AlignRZController.setSetpoint(180);
    //AlignRZController.enableContinuousInput(-180, 180);
    //AlignRZController.setTolerance(minRZErrorToCorrect);


  }
  int pipeline = 0;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //get our alliance red or blue
   Alliance CurrentAlliance = DriverStation.getAlliance();
   //make sure we are in april tag pipeline before checking
   //
   //get if we have any targets
   boolean HasTarget = limelight3Subsystem.hasValidTarget();
   //get the target number
   int targetID = limelight3Subsystem.getTargetID();
   //get if the closest target is for our team(ignore others obviously)
   boolean WeSeeourSubstationTag = false;
   if ( (CurrentAlliance == Alliance.Red) && targetID == Constants.AllianceAprilTags.Red.substation)
   {
      WeSeeourSubstationTag = true;
   }
   if ( (CurrentAlliance == Alliance.Blue) && targetID == Constants.AllianceAprilTags.Blue.substation)
   {
      WeSeeourSubstationTag = true;
   }
   if(!WeSeeourSubstationTag){
       //if substation is at X area size then switch our speed to substation movde
       //when we are at X area set bool to true.
       s_Swerve.drive(
             new Translation2d(0,0), 
             0  , 
             !robotCentric, 
           true
         ); 
    return;
   }
  

    
    //LL pose RZ is our rotation relative to the target in field space
    double RZCurrent = limelight3Subsystem.getRZ() *-1.0;//rotation Y targetspace is ROtation Z field space?  
    
    double RZAdjust = GetRZPoseAdjust(RZCurrent, min_RZ_command);
    
    //LL POSE X is forward and backward toward target in field space
    double xpose = limelight3Subsystem.getXPos() ;

    double xpose_adjust = GetXPoseAdjust(xpose, min_xpose_command);

    //LL POSE Y Is left to right translation in field space
    double Ypose = limelight3Subsystem.getYPos();

    double Ypose_adjust =  GetYPoseAdjust(Ypose, min_Ypose_command );



    
    double alignmentspeed = 1.0;//5.0 * Constants.Swerve.maxSpeed;
    double rotationspeed = 1.0;//1.0 * Constants.Swerve.maxAngularVelocity;
    
    double YposeAxis = Ypose_adjust * alignmentspeed;
    double XposeAxis = xpose_adjust * alignmentspeed;
    double RZposeAxis = RZAdjust * rotationspeed;
    
     
    s_Swerve.drive(
             new Translation2d(XposeAxis,YposeAxis), 
             RZposeAxis  , 
             !robotCentric, 
           true
         );

    //s_Swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(translationAxis, strafeAxis, rotationAxis,s_Swerve.swerveOdometry.getPoseMeters().getRotation() ));
    SmartDashboard.putNumber("RZ_Current", RZCurrent);
    SmartDashboard.putNumber("RZ_PID", RZAdjust);
    
   
    SmartDashboard.putNumber("Ypose_Current", Ypose);
    SmartDashboard.putNumber("Ypose_PID", Ypose_adjust);

    SmartDashboard.putNumber("Xpose", xpose);
    SmartDashboard.putNumber("Xpose_PID", XposeAxis);
    
  }

  //LL POSE Y Is left to right translation in field space
private double GetYPoseAdjust(double Ypose, double min_PoseY_command) {
  double ypose_adjust;
  //if(Ypose == 0){return 0;}
  //if (Math.abs(Ypose) > minYposeErrorToCorrect )
  //{
    if (Ypose < 0)
    {
            ypose_adjust = AlignPoseYController.calculate(Ypose) + min_PoseY_command;
    }
    else
    {
            ypose_adjust = AlignPoseYController.calculate(Ypose) - min_PoseY_command;
    }
 // }
  //else 
 // {
  //  ypose_adjust = 0;
  //}
  return ypose_adjust *-1.0;
}

  //LL POSE X is forward and backward toward target in field space
  private double GetXPoseAdjust(double fwdReverse_error, double min_Fwd_command) {
    double xpose_adjust;
    
    //if (Math.abs(fwdReverse_error) > minXposeErrorToCorrect ) //!AlignXController.atGoal()
    //{
      if (fwdReverse_error < 0)
      {
              xpose_adjust = AlignXController.calculate(fwdReverse_error) - min_Fwd_command;
      }
      else
      {
              xpose_adjust = AlignXController.calculate(fwdReverse_error) + min_Fwd_command;
      }
    //}
    //else 
    //{
    //  xpose_adjust = 0;
    //}
    //if(fwdReverse_error == 0){return 0;}
    return xpose_adjust *-1.0;
  }

  //LL pose RZ is our rotation relative to the target in field space
  private double GetRZPoseAdjust(double RZ, double min_spin_command) {
    double RZ_adjust = 0;
    //if(RZ == 0){return 0;}
    //double error = Math.abs(AlignRZController.getGoal().position - RZ);
    //if (Math.abs(RZ_adjust) > minRZErrorToCorrect)//!AlignRotationController.atGoal()
    //{
      if (RZ < 0)
      {
        RZ_adjust = AlignRZController.calculate(RZ*-1.0) - min_spin_command;
      }
      else
      {
        RZ_adjust = AlignRZController.calculate(RZ)*-1.0 + min_spin_command;
      }
      
      //our own tolerance script. 
      // if(Math.abs(RZ_adjust) < minRZErrorToCorrect)
      // {
      //   RZ_adjust = 0;
      // }
    //}
    //else
    //{
    //  RZ_adjust = 0;
    //}
    return RZ_adjust;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {super.end(interrupted);}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
