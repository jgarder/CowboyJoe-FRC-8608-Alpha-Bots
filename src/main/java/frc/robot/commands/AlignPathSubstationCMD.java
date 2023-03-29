// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.List;

import javax.lang.model.util.ElementScanner14;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
//import frc.robot.Constants;
import frc.robot.Subsystems.*;


public class AlignPathSubstationCMD extends CommandBase {
  




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

  double XP_buffer = 0;
  double YP_buffer = 0;
  double RZ_buffer = 0;

  double XP_Setpoint = 0;
  double YP_Setpoint = 0;
  double RZ_Setpoint = 0;

  boolean robotCentric = true;
  int pipeline = 0;
  // Called every time the scheduler runs while the command is scheduled.
  int debounceloops = 0;
  int loopsoffbeforestopping = 50;

  Alliance CurrentAlliance;
  double xymulti = -1.0;

  public AlignPathSubstationCMD(Swerve Thiss_Swerve, Limelight3Subsystem ThisLimelight) {
    s_Swerve = Thiss_Swerve;
    limelight3Subsystem = ThisLimelight;
    addRequirements(s_Swerve,limelight3Subsystem);    
  }

  int targetID = 0;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      
    //get our alliance red or blue
    CurrentAlliance = DriverStation.getAlliance();
    //get the target number
    targetID = limelight3Subsystem.getTargetID();

    if ( (CurrentAlliance == Alliance.Red) && targetID == Constants.AllianceAprilTags.Red.substation)
    {  
      xymulti = 1.0;
      SetPidControlersToRedSubstation();
    }
    else if ( (CurrentAlliance == Alliance.Blue) && targetID == Constants.AllianceAprilTags.Blue.substation)
    {
      xymulti = -1.0;
      SetPidControlersToBlueSubstation();
    }else{
      end(true);
      return;
    }

    XP_buffer = 0;
    YP_buffer = 0;
    RZ_buffer = 0;
    FillBuffers();
    double Xpose_Offset = 0;
    double Ypose_Offset = 0;
    double RZ_Offset = 0;
    if(YP_buffer != 0.00 & XP_buffer != 0.00 & RZ_buffer != 0.00) 
    {
        //SUBTRACT where we need to go, from where we are. this will give us the translations we need to make 
        Xpose_Offset = XP_buffer - XP_Setpoint;
        Ypose_Offset = YP_buffer - YP_Setpoint;
        RZ_Offset = RZ_buffer - RZ_Setpoint;
    }
    SmartDashboard.putNumber("RZ_Offset", RZ_Offset);
    SmartDashboard.putNumber("Ypose_Offset", Ypose_Offset);
    SmartDashboard.putNumber("Xpose_Offset", Xpose_Offset);

    
    SmartDashboard.putNumber("Xpose", XP_buffer);
    SmartDashboard.putNumber("Ypose_Current", YP_buffer); 
    SmartDashboard.putNumber("RZ_Current", RZ_buffer); 
    
    // More complex path with holonomic rotation. Non-zero starting velocity of 2 m/s. Max velocity of 4 m/s and max accel of 3 m/s^2
     PathPlannerTrajectory traj3 = PathPlanner.generatePath(
     new PathConstraints(2, 2), 
     new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
     new PathPoint(new Translation2d(Xpose_Offset*xymulti, Ypose_Offset*xymulti), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(RZ_Offset), 0) // position, heading(direction of travel), holonomic rotation, velocity override
      // position, heading(direction of travel), holonomic rotation
    // new PathPoint(new Translation2d(5.0, 3.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-30)) // position, heading(direction of travel), holonomic rotation
     );

    //  TrajectoryConfig config =
    //         new TrajectoryConfig(
    //                 Constants.AutoConstants.kMaxSpeedMetersPerSecond,
    //                 Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //             .setKinematics(Constants.Swerve.swerveKinematics);

    //  Trajectory exampleTrajectory =
    //       TrajectoryGenerator.generateTrajectory(
    //           // Start at the origin facing the +X direction
    //           new Pose2d(0, 0, new Rotation2d(0)),
    //           // Pass through these two interior waypoints, making an 's' curve path
    //           //List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //           List.of(new Translation2d(Xpose_Offset, Ypose_Offset)),
    //           // End 3 meters straight ahead of where we started, facing forward
    //           new Pose2d(Xpose_Offset, Ypose_Offset, new Rotation2d(0)),
    //           config);

              var thetaController =
              new ProfiledPIDController(
                  Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
          thetaController.enableContinuousInput(-Math.PI, Math.PI);
  
          SwerveControllerCommand swerveControllerCommand =
              new SwerveControllerCommand(
                  traj3,
                  s_Swerve::getPose,
                  Constants.Swerve.swerveKinematics,
                  new PIDController(1, 0, 0),
                  new PIDController(1, 0, 0),
                  thetaController,
                  s_Swerve::setModuleStates,
                  s_Swerve);

      s_Swerve.resetOdometry(traj3.getInitialPose());
      swerveControllerCommand.schedule();

    end(true);
    //LL POSE X is forward and backward toward target in field space
    //AlignXController.setSetpoint(XP_Setpoint);
    //LL POSE Y Is left to right translation in field space
    //AlignPoseYController.setSetpoint(YP_Setpoint);
    //LL pose RZ is our rotation relative to the target in field space
    //AlignRZController.setSetpoint(RZ_Setpoint);
  }
  
  
  @Override
  public void execute() {
    //get the target number
    targetID = limelight3Subsystem.getTargetID();
  
    
  }

private void SetPidControlersToRedSubstation() {
  //LL POSE X is forward and backward toward target in field space
  //AlignXController.setSetpoint(-6.8250);
  XP_Setpoint = -6.8250;
  //LL POSE Y Is left to right translation in field space
  //AlignPoseYController.setSetpoint(2.00);
  YP_Setpoint = 2.00;
  //LL pose RZ is our rotation relative to the target in field space
  //AlignRZController.setSetpoint(180);
  RZ_Setpoint = 180;
}
private void SetPidControlersToBlueSubstation() {
  //LL POSE X is forward and backward toward target in field space
  //AlignXController.setSetpoint(6.8250);
  XP_Setpoint = 6.8250;
  //LL POSE Y Is left to right translation in field space
  //AlignPoseYController.setSetpoint(3.50);
  YP_Setpoint = 3.50;
  //LL pose RZ is our rotation relative to the target in field space
  //AlignRZController.setSetpoint(0);
  RZ_Setpoint = 0;
}
private void FillBuffers()
{
  double RZCurrent = limelight3Subsystem.getRZ() *-1.0;//rotation Y targetspace is ROtation Z field space?
  double xpose = limelight3Subsystem.getXPos() ;
  double Ypose = limelight3Subsystem.getYPos();
  if(Ypose != 0.00 & xpose != 0.00 & RZCurrent != 0.00) 
  {
    XP_buffer = xpose;
    YP_buffer = Ypose;
    RZ_buffer = RZCurrent;
  }
  else{debounceloops++;}
}
 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ( (CurrentAlliance == Alliance.Red) && targetID == Constants.AllianceAprilTags.Red.substation)
    {  
      return false;
    }
    else if ( (CurrentAlliance == Alliance.Blue) && targetID == Constants.AllianceAprilTags.Blue.substation)
    {
      return false;
    }
    else{
      return true;
    }
    
  }
}
