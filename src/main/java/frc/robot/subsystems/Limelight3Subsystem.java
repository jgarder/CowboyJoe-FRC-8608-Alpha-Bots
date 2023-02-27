// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight3Subsystem extends SubsystemBase {
  /** Creates a new Limelight3Subsystem. */
  public Limelight3Subsystem(XboxController controllerUsedToScore) {
    this.ControllerUsedToScore = controllerUsedToScore;
  }

  XboxController ControllerUsedToScore;
  private NetworkTable latestInfo;

  public static int kpipelineAprilTags = 0;
  public static int kpipelineRetroflectiveHighRung = 1;
  public static int kpipelineRetroflectiveLowerRung = 2;

  //Create variables
	double targetD;
	boolean hasTarget;
	double xOffset;
	double yOffset;
	double area;
	double skew;
	double LEDMode;
	double camMode;
	double pipeline;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getdataToDashboard();
   // VibeOnZero();
    
  }
  public NetworkTable getlatestinfo() {
    return latestInfo; 
  }
  public void VibeOnZero() {
    
    NetworkTableEntry tx = latestInfo.getEntry("tx");
    NetworkTableEntry ty = latestInfo.getEntry("ty");
    NetworkTableEntry ta = latestInfo.getEntry("ta");

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    
    double kXClosenessForRumble = 5.0;
    double kYClosenessForRumble = 1.0;
    double kTagAreaofScreenBeforeRumbleOn = .4;
    if(area > kTagAreaofScreenBeforeRumbleOn)
    {
      if(Math.abs(x) < kXClosenessForRumble && Math.abs(x) > 0.0)
      {
        ControllerUsedToScore.setRumble(RumbleType.kRightRumble, .3);
      }
      else
      {
        ControllerUsedToScore.setRumble(RumbleType.kRightRumble, 0);
      }
      if(Math.abs(y) < kYClosenessForRumble && Math.abs(y) > 0.0)
      {
        ControllerUsedToScore.setRumble(RumbleType.kLeftRumble, .2);
      }
      else {
        ControllerUsedToScore.setRumble(RumbleType.kLeftRumble, 0);
      }
    } 
    else 
    {
      ControllerUsedToScore.setRumble(RumbleType.kBothRumble, 0);
    }
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
  }
  public NetworkTable getdataToDashboard()
  {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    latestInfo = table;

    //post to smart dashboard periodically
    //SmartDashboard.putNumber("LimelightX", getXOffset());
    //SmartDashboard.putNumber("LimelightY", getYOffset());
    SmartDashboard.putNumber("LimelightArea", getArea());
    return latestInfo;
  }
  public boolean getHasTarget() {
		targetD = latestInfo.getEntry("tv").getDouble(0);
		if(targetD == 0) {
			hasTarget = false;
		}else if(targetD == 1) {
			hasTarget = true;
		}
		return hasTarget;
	}
	
	public double getXOffset() {
		xOffset = latestInfo.getEntry("tx").getDouble(0);
		return xOffset;
	}
	
	public double getYOffset() {
		yOffset = latestInfo.getEntry("ty").getDouble(0);
		return yOffset;
	}
	
	public double getArea() {
		area = latestInfo.getEntry("ta").getDouble(0);
		return area;
	}
	
	public double getSkew() {
		skew = latestInfo.getEntry("ts").getDouble(0);
		return skew;
	}
	
	public double getLEDMode() {
		LEDMode = latestInfo.getEntry("ledMode").getDouble(1);
		return LEDMode;
	}
	
	public double getCamMode() {
		camMode = latestInfo.getEntry("camMode").getDouble(0);
		return camMode;
	}
	
	public double getPipeline() {
		pipeline = latestInfo.getEntry("pipeline").getDouble(0);
		return pipeline;
	}
	
	public void switchLED() {
		if(getLEDMode() == 0) {
			latestInfo.getEntry("ledMode").setDouble(1);
			SmartDashboard.putString("LED Mode", "Off");
		}else if(getLEDMode() == 1) {
			latestInfo.getEntry("ledMode").setDouble(0);
			SmartDashboard.putString("LED Mode", "On");
		}else if(getLEDMode() == 2) {
			latestInfo.getEntry("ledMode").setDouble(1);
			SmartDashboard.putString("LED Mode", "Off");
		}
	}
	
	public void switchCamera() {
		if(getCamMode() == 0) {
			latestInfo.getEntry("camMode").setDouble(1);
			SmartDashboard.putString("Camera Mode", "Camera");
		}else if(getCamMode() == 1) {
			latestInfo.getEntry("camMode").setDouble(0);
			SmartDashboard.putString("Camera Mode", "Vision");
		}
	}

  public NetworkTable switchPipeline()
  {
    double currentpipeline = getPipeline();
    if(currentpipeline == kpipelineAprilTags)
    {
      setPipeline(kpipelineRetroflectiveHighRung);
    }
    else if (currentpipeline == kpipelineRetroflectiveHighRung)
    {
      setPipeline(kpipelineRetroflectiveLowerRung);
    }
    else if (currentpipeline == kpipelineRetroflectiveLowerRung)
    {
      setPipeline(kpipelineAprilTags);
    }

    getdataToDashboard();
    return latestInfo;
  }
	
	public void setPipeline(double pipeline) {
		latestInfo.getEntry("pipeline").setDouble(pipeline);
		SmartDashboard.putNumber("Camera Mode", pipeline);
	}

  public void SwitchToCameraModeWithClosestTarget()
  {
    //goto pipeline 0
    setPipeline(kpipelineAprilTags);
    var apriltaginfo = getdataToDashboard();
    //scan for targets
    boolean AprilTargeted = getHasTarget();
    //goto pipeline 1 
    setPipeline(kpipelineRetroflectiveHighRung);
    var RetroHighRunginfo = getdataToDashboard();
    //scan for target..
    boolean HighRungTargeted = getHasTarget();
    //if target found in only 1 choose that. 

    

    //if tartget found in both, choose the one with the lowest x value (left right offset from center)
  }
}
