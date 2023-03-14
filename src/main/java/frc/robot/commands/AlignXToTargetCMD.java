// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.*;


public class AlignXToTargetCMD extends CommandBase {
  double kXP = 1.5;
  double kXI = 0.000;
  double kXD = 0.000;

  double kZP = 0.07;
  double kZI = 0.00;
  double kZD = 0.0000; 

  double kZRetroLowP = 3.0;
  double kZRetroLowI = 0.000;
  double kZRetroLowD = 0.001;

  double kZRetroHighP = 5.0;
  double kZRetroHighI = 0.000;
  double kZRetroHighD = 0.0001;

  double kRP = 0.07;
  double kRI = 0.00;
  double kRD = 0.00;
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

  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(8, 8);
  private final ProfiledPIDController AlignXController = new ProfiledPIDController(kXP,kXI,kXD, X_CONSTRAINTS);
  private final ProfiledPIDController AlignZController = new ProfiledPIDController(kZP,kZI,kZD, Y_CONSTRAINTS);
  private final ProfiledPIDController AlignRotationController = new ProfiledPIDController(kRP,kRI,kRD, OMEGA_CONSTRAINTS);

  //PIDController AligXController;// this will turn left or right to align
  //PIDController AlignZController;//this will go forward and back to align. 
  //PIDController AlignRotationController;//this will spin us. 
  //min amounts to tolerate 
  //double minXErrorToCorrect = .1;//.04
  double minRYErrorToCorrect = .5;//.04;
  double minStrafeXErrorToCorrect = .08;//.04;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      
    AlignXController.setTolerance(minStrafeXErrorToCorrect);
    AlignZController.setTolerance(minRYErrorToCorrect);
    AlignRotationController.setTolerance(minRYErrorToCorrect);
    //AlignRotationController.setTolerance(Units.degreesToRadians(3));
    //AlignRotationController.enableContinuousInput(-Math.PI, Math.PI);

    double xoffsetinmeters = .11;
    // Drive
    AlignXController.setGoal(0-.11);
    AlignZController.setGoal(0);
    AlignRotationController.setGoal(minRYErrorToCorrect);
  }
  int pipeline = 0;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double KpAim = -0.1f;
    //double KpDistance = -0.1f;
    
    

    //double x = limelight3Subsystem.getXOffset(); 
    //double y = limelight3Subsystem.getYOffset();
    double area = limelight3Subsystem.getArea();

    //NetworkTableEntry targetpose_robotsSpace = limelight3Subsystem.latestInfo.getEntry("targetpose_robotspace");
    //double[] tPoseRS = 	targetpose_robotsSpace.getDoubleArray(new double[]{});
    //SmartDashboard.putNumberArray("TposeRS", tPoseRS);
    //double targetOffsetAngle_Vertical = y;

    //double targetrotationyoffset = tPoseRS[4];//we want to get the 3d yaw correction we need for our swerve
    //SmartDashboard.putNumber("TposeRY", targetrotationyoffset);//rotation Y targetspace is ROtation Z field space?


    // how many degrees back is your limelight rotated from perfectly vertical?
    //double limelightMountAngleDegrees = 0.0;

    // distance from the center of the Limelight lens to the floor
    //double limelightLensHeightInches = 17.0;

    // distance from the target to the floor
    //double goalHeightInches = 11.0;

    //double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    //double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    //double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);

    
    //////////////
    //rotation is the bots rotation to the target
    //////////////
    double RotationY_error = limelight3Subsystem.getYaw() * -1.0;//rotation Y targetspace is ROtation Z field space?
    double rotation_adjust = 0.0;
    double min_spin_command = 0;//0.004;
    
    if (Math.abs(RotationY_error) > minRYErrorToCorrect)//!AlignRotationController.atGoal()
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
    else
    {
      rotation_adjust = 0;
    }
    //////////////
    //heading is currently the left and right alingment to the target
    //////////////
    double stafe_error = limelight3Subsystem.getXPos() * -1.0;
    double strafe_adjust = 0.0;
    double min_strafe_command = 0.008;
    
    if (Math.abs(stafe_error) > minStrafeXErrorToCorrect ) //!AlignXController.atGoal()
    {
      if (stafe_error < 0)
      {
              strafe_adjust = AlignXController.calculate(stafe_error,0) + min_strafe_command;
      }
      else
      {
              strafe_adjust = AlignXController.calculate(stafe_error,0) - min_strafe_command;
      }
    }
    else 
    {
      strafe_adjust = 0;
    }
    //////////////
    //////////////
    //distance is currently the left and right alingment to the target
    //////////////
    
    double RetroHighMustBeThisSizeBeforeAutomating = .020;
    double RetroHigh_tagArea_setpoint = .10;
    double RetroLowMustBeThisSizeBeforeAutomating = .06;
    double RetroLow_tagArea_setpoint = .27;
    double AprilMustBeThisSizeBeforeAutomating = .4;
    double April_tagArea_setpoint = 3.0;


    double Selected_tagArea_error = limelight3Subsystem.getZPos();//area;
    double Selected_tagArea_setpoint = 3.01;

    double min_forward_command = 0.003;
    double distance_adjust =  0.0;
    double minAreaErrorToCorrect = .00;//.04;
    double SizeNeededForAuto = 0;
    //depending on the pipeline set the setpoint and outer limit.
    if(limelight3Subsystem.getPipeline() == limelight3Subsystem.kpipelineAprilTags){
      Selected_tagArea_setpoint = April_tagArea_setpoint;
      SizeNeededForAuto = AprilMustBeThisSizeBeforeAutomating;
      AlignZController.setPID(kZP, kZI, kZD);
    }
    else if(limelight3Subsystem.getPipeline() == limelight3Subsystem.kpipelineRetroflectiveHighRung){
      Selected_tagArea_setpoint = RetroHigh_tagArea_setpoint;
      SizeNeededForAuto = RetroHighMustBeThisSizeBeforeAutomating;
      AlignZController.setPID(kZRetroHighP, kZRetroHighI, kZRetroHighD);
    }
    else if(limelight3Subsystem.getPipeline() == limelight3Subsystem.kpipelineRetroflectiveLowerRung){
      Selected_tagArea_setpoint = RetroLow_tagArea_setpoint;
      SizeNeededForAuto = RetroLowMustBeThisSizeBeforeAutomating;
      AlignZController.setPID(kZRetroLowP, kZRetroLowI, kZRetroLowD);
    }


    
    
    if (Math.abs(Selected_tagArea_error) > SizeNeededForAuto )//!AlignZController.atGoal() &&
    {
      distance_adjust = AlignZController.calculate(Selected_tagArea_error,Selected_tagArea_setpoint); //KpDistance * distance_error;/// we are ignoring Y (up down) for now. 

      if (Selected_tagArea_error > minAreaErrorToCorrect)
      {
        distance_adjust += min_forward_command;
      }
      else
      {
        distance_adjust -= min_forward_command; //KpDistance * distance_error;/// we are ignoring Y (up down) for now. 
      }
    }
    /////////////////

    // double left_command = steering_adjust + distance_adjust;
    // double right_command = steering_adjust + distance_adjust;
    //driveSubsystem.ArcadeDrivemotors(0, steering_adjust);
    boolean robotCentric = true;
    double translationAxis = 0;//distance_adjust;
    double strafeAxis = strafe_adjust;//strafe_adjust ;
    double rotationAxis = rotation_adjust;//rotation_adjust;
    double alignmentspeed = .1;//5.0 * Constants.Swerve.maxSpeed;
    double rotationspeed = .1;//1.0 * Constants.Swerve.maxAngularVelocity;
     s_Swerve.drive(
             new Translation2d(translationAxis, strafeAxis).times(alignmentspeed), 
             rotationAxis * rotationspeed , 
             !robotCentric, 
           true
         );

    //s_Swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(translationAxis, strafeAxis, rotationAxis,s_Swerve.swerveOdometry.getPoseMeters().getRotation() ));
    SmartDashboard.putNumber("rotation_adjust", RotationY_error);
    SmartDashboard.putNumber("distance_adjust", Selected_tagArea_error);
    SmartDashboard.putNumber("steering_adjust", stafe_error);
    SmartDashboard.putNumber("rotation_PID", rotation_adjust);
    SmartDashboard.putNumber("distance_PID", distance_adjust);
    SmartDashboard.putNumber("steering_PID", strafeAxis);
    
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
