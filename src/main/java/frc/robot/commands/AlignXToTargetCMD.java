// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.*;


public class AlignXToTargetCMD extends CommandBase {
  double kXP = 0.001;
  double kXI = 0.001;
  double kXD = 0.000;

  double kZP = 0.07;
  double kZI = 0.01;
  double kZD = 0.0000; 

  double kZRetroLowP = 3.0;
  double kZRetroLowI = 0.010;
  double kZRetroLowD = 0.001;

  double kZRetroHighP = 5.0;
  double kZRetroHighI = 0.050;
  double kZRetroHighD = 0.0001;

  double kRP = 0.007;
  double kRI = 0.003;
  double kRD = 0.000;
  /** Creates a new ArmStopCMD. */
  Limelight3Subsystem limelight3Subsystem;
  Swerve s_Swerve;
  public AlignXToTargetCMD(Swerve Thiss_Swerve, Limelight3Subsystem ThisLimelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Swerve = Thiss_Swerve;
    limelight3Subsystem = ThisLimelight;
    addRequirements(limelight3Subsystem);
    addRequirements(s_Swerve);
    
  }

  PIDController AligXController;// this will turn left or right to align
  PIDController AlignZController;//this will go forward and back to align. 
  PIDController AlignRotationController;//this will spin us. 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  
       
    AligXController =  new PIDController(kXP,kXI,kXD);


       
    AlignZController =  new PIDController(kZP,kZI,kZD);


       
    AlignRotationController =  new PIDController(kRP,kRI,kRD);
    
  }
  int pipeline = 0;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double KpAim = -0.1f;
    //double KpDistance = -0.1f;
    double min_aim_command = 0.008;
    

    double x = limelight3Subsystem.getXOffset(); 
    double y = limelight3Subsystem.getYOffset();
    double area = limelight3Subsystem.getArea();

    NetworkTableEntry targetpose_robotsSpace = limelight3Subsystem.latestInfo.getEntry("targetpose_robotspace");
    double[] tPoseRS = 	targetpose_robotsSpace.getDoubleArray(new double[]{});
    SmartDashboard.putNumberArray("TposeRS", tPoseRS);
    double targetOffsetAngle_Vertical = y;

    double targetrotationyoffset = tPoseRS[4];//we want to get the 3d yaw correction we need for our swerve
    SmartDashboard.putNumber("TposeRY", targetrotationyoffset);//rotation Y targetspace is ROtation Z field space?


    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 0.0;

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 17.0;

    // distance from the target to the floor
    double goalHeightInches = 11.0;

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);

    
    
    //////////////
    double RotationY_error = x * 1.0;//rotation Y targetspace is ROtation Z field space?
    double rotation_adjust = 0.0;
    double min_spin_command = 0.004;
    if (Math.abs(RotationY_error) > .04)
    {
      if (RotationY_error < 0)
      {
        rotation_adjust = AlignRotationController.calculate(RotationY_error,0) + min_spin_command;
      }
      else
      {
        rotation_adjust = AlignRotationController.calculate(RotationY_error,0) - min_spin_command;
      }
    }
    //////////////
    //////////////
    double heading_error = x * 1.0;
    double steering_adjust = 0.0;
    if (Math.abs(heading_error) > .04)
    {
      if (heading_error < 0)
      {
              steering_adjust = AligXController.calculate(heading_error,0) + min_aim_command;
      }
      else
      {
              steering_adjust = AligXController.calculate(heading_error,0) - min_aim_command;
      }
    }
    //////////////
    double min_forward_command = 0.03;

    double RetroHighMustBeThisSizeBeforeAutomating = .020;
    double RetroHigh_tagArea_setpoint = .10;
    double RetroLowMustBeThisSizeBeforeAutomating = .06;
    double RetroLow_tagArea_setpoint = .27;
    double AprilMustBeThisSizeBeforeAutomating = .4;
    double April_tagArea_setpoint = 3.0;

    double SizeNeededForAuto = 0;
    double tagArea_error = area;
    double tagArea_setpoint = 3.01;
    //double distance_error = y * 1.0;
    double distance_adjust =  0.0;

    //depending on the pipeline set the setpoint and outer limit.
    if(limelight3Subsystem.getPipeline() == limelight3Subsystem.kpipelineAprilTags){
      tagArea_setpoint = April_tagArea_setpoint;
      SizeNeededForAuto = AprilMustBeThisSizeBeforeAutomating;
      AlignZController.setPID(kZP, kZI, kZD);
    }
    else if(limelight3Subsystem.getPipeline() == limelight3Subsystem.kpipelineRetroflectiveHighRung){
      tagArea_setpoint = RetroHigh_tagArea_setpoint;
      SizeNeededForAuto = RetroHighMustBeThisSizeBeforeAutomating;
      AlignZController.setPID(kZRetroHighP, kZRetroHighI, kZRetroHighD);
    }
    else if(limelight3Subsystem.getPipeline() == limelight3Subsystem.kpipelineRetroflectiveLowerRung){
      tagArea_setpoint = RetroLow_tagArea_setpoint;
      SizeNeededForAuto = RetroLowMustBeThisSizeBeforeAutomating;
      AlignZController.setPID(kZRetroLowP, kZRetroLowI, kZRetroLowD);
    }

    if (Math.abs(tagArea_error) > SizeNeededForAuto)
    {
      distance_adjust = AlignZController.calculate(tagArea_error,tagArea_setpoint); //KpDistance * distance_error;/// we are ignoring Y (up down) for now. 

      if (heading_error > 0)
      {
        distance_adjust += min_forward_command;
      }
      else
      {
        distance_adjust -= min_forward_command; //KpDistance * distance_error;/// we are ignoring Y (up down) for now. 
      }
    }
    // double left_command = steering_adjust + distance_adjust;
    // double right_command = steering_adjust + distance_adjust;
    //driveSubsystem.ArcadeDrivemotors(0, steering_adjust);
    boolean robotCentric = true;
    double translationAxis = distance_adjust;
    double strafeAxis = steering_adjust;
    double rotationAxis = rotation_adjust;
    
    s_Swerve.drive(
            new Translation2d(translationAxis, strafeAxis).times(Constants.Swerve.maxSpeed), 
            rotationAxis * Constants.Swerve.maxAngularVelocity, 
            !robotCentric, 
            true
        );
    
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
