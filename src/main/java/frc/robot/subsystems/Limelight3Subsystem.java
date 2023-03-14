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

    private NetworkTable limelight;//Table for the limelight
    private NetworkTableEntry tx;//Table for the x-coordinate
    private NetworkTableEntry ty;//Table for the y-coordnate
    private NetworkTableEntry ta;//Table for the area
    private NetworkTableEntry ts;//Table for the skew
    private NetworkTableEntry tv;//Table to see if there are valid targets
    private NetworkTableEntry tl;//Table for latency
    private NetworkTableEntry tshort;//Table for short side length
    private NetworkTableEntry tlong;//Table for long side length
    private NetworkTableEntry thoriz;//Table for width
    private NetworkTableEntry tvert;//Table for height
    private NetworkTableEntry ledMode;//Table to set blinking leds
    private NetworkTableEntry camMode;//Table to set camera mode
    private NetworkTableEntry pipeline;//Table to switch pipelines
    private NetworkTableEntry solvePNP;
    double[] defaultArray = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  /** Creates a new Limelight3Subsystem. */
  public Limelight3Subsystem(XboxController controllerUsedToScore) {
    this.ControllerUsedToScore = controllerUsedToScore;

    limelight = NetworkTableInstance.getDefault().getTable("limelight");//Instantiate the tables
        tx = limelight.getEntry("tx");//x angle offset
        ty = limelight.getEntry("ty");// y angle offset 
        ta = limelight.getEntry("ta");//target is x area of screen
        ts = limelight.getEntry("ts");// this is the skew of the target
        tv = limelight.getEntry("tv");//is there a visible target
        tl = limelight.getEntry("tl");
        tshort = limelight.getEntry("tshort");
        tlong = limelight.getEntry("tlong");
        thoriz = limelight.getEntry("thor");//thoriz
        tvert = limelight.getEntry("tvert");
        ledMode = limelight.getEntry("ledMode");
        camMode = limelight.getEntry("camMode");
        pipeline = limelight.getEntry("pipeline");
        solvePNP = limelight.getEntry("camerapose_targetspace");//("camtran");// this is old. need to find out : is this camera translation in target space?
  }

  XboxController ControllerUsedToScore;

  public static int kpipelineAprilTags = 0;
  public static int kpipelineRetroflectiveHighRung = 1;
  public static int kpipelineRetroflectiveLowerRung = 2;

  //Create variables
	double targetD;
	boolean hasTarget;


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getdataToDashboard();
   // VibeOnZero();
    
  }
  public NetworkTable getlatestinfo() {
    return limelight; 
  }
  public void VibeOnZero() {
    

    
    double kXClosenessForRumble = 5.0;
    double kYClosenessForRumble = 1.0;
    double kTagAreaofScreenBeforeRumbleOn = .4;
    if(getArea() > kTagAreaofScreenBeforeRumbleOn)
    {
      if(Math.abs(getXOffset()) < kXClosenessForRumble && Math.abs(getXOffset()) > 0.0)
      {
        ControllerUsedToScore.setRumble(RumbleType.kRightRumble, .3);
      }
      else
      {
        ControllerUsedToScore.setRumble(RumbleType.kRightRumble, 0);
      }
      if(Math.abs(getYOffset()) < kYClosenessForRumble && Math.abs(getYOffset()) > 0.0)
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
    //SmartDashboard.putNumber("LimelightX", x);
    //SmartDashboard.putNumber("LimelightY", y);
    //SmartDashboard.putNumber("LimelightSkew", getSkew());
  }
  public NetworkTable getdataToDashboard()
  {
    //NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    //latestInfo = table;

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", getXOffset());
    SmartDashboard.putNumber("LimelightY", getYOffset());
    SmartDashboard.putNumber("LimelightArea", getArea());
    SmartDashboard.putNumber("LimelightSkew", getSkew());
    SmartDashboard.putNumber("LL Pose Y", getYPos());
    SmartDashboard.putNumber("LL Pose Yaw", getYaw());
    SmartDashboard.putNumber("LL Pose Z", getZPos());
    return limelight;
  }

  /**
     * This function uses the Limelight's Solve3D function to compute the distance from the target in inches. The limelight must be in Solve3D mode with High-Res enabled.
     * @return Distance from the target
     */
    public double getDistance() {
      return Math.sqrt(Math.pow(getXPos(), 2) + Math.pow(getYPos(), 2));
  }

  /**
   * This function uses the Limelight's Solve3D function to compute the x-distance from the target in inches. The limelight must be in Solve3D mode with High-Res enabled.
   * @return x-distance from the target in inches
   */
  public double getXPos() {
      return solvePNP.getDoubleArray(defaultArray)[0];
  }

  /**
   * This function uses the Limelight's Solve3D function to compute the y-distance from the target in inches. The limelight must be in Solve3D mode with High-Res enabled.
   * @return y-distance from the target in inches
   */
  public double getYPos() {
      return solvePNP.getDoubleArray(defaultArray)[1];
  }

  public double getZPos() {
      return solvePNP.getDoubleArray(defaultArray)[2];
  }

  public double getPitch() {
      return solvePNP.getDoubleArray(defaultArray)[3];
  }

  public double getYaw() {
      return solvePNP.getDoubleArray(defaultArray)[4];
  }

  public double getRoll() {
      return solvePNP.getDoubleArray(defaultArray)[5];
  }
	
	public double getXOffset() {
		//xOffset = latestInfo.getEntry("tx").getDouble(0);
		return tx.getDouble(0.0);
	}
	
	public double getYOffset() {
		//yOffset = latestInfo.getEntry("ty").getDouble(0);
		return ty.getDouble(0.0);
	}
	
	public double getArea() {
		//area = latestInfo.getEntry("ta").getDouble(0);
		//return area;
    return ta.getDouble(0.0);
	}
	
	public double getSkew() {
		//skew = latestInfo.getEntry("ts").getDouble(0);
		//return skew;
    return ts.getDouble(0.0);
	}
  public boolean hasValidTarget() {
    return tv.getDouble(0) == 1.0;//return true if true false if false.
  }
  // public boolean getHasTarget() {
	// 	targetD = latestInfo.getEntry("tv").getDouble(0);
	// 	if(targetD == 0) {
	// 		hasTarget = false;
	// 	}else if(targetD == 1) {
	// 		hasTarget = true;
	// 	}
	// 	return hasTarget;
	// }
  public double getLatency() {
    return tl.getDouble(0.0);
  }

  public double getShortSide() {
    return tshort.getDouble(0.0);
  }

  public double getLongSide() {
    return tlong.getDouble(0.0);
  }

  public double getWidth() {
    return thoriz.getDouble(0.0);
  }

  public double getHeight() {
    return tvert.getDouble(0.0);
  }

  public void setPipeline(int id) {
    pipeline.setNumber(id);
  }
	
	public double getLEDMode() {
		//LEDMode = latestInfo.getEntry("ledMode").getDouble(1);
		//return LEDMode;
    return ledMode.getDouble(1);
	}
      /**
     * Set the state of the LEDs
     * @param mode
     *  0- Pipeline default
     *  1- Force off
     *  2- Force blink
     *  3- Force on
     */
    public void setLedMode(int mode) {
      ledMode.setNumber(mode);
  }

  public void setCamMode(int mode) {
      camMode.setNumber(mode);
  }
	
	public double getCamMode() {
		//camMode = latestInfo.getEntry("camMode").getDouble(0);
		//return camMode;
    return camMode.getDouble(0.0);
	}
	
	public double getPipeline() {
		//pipeline = latestInfo.getEntry("pipeline").getDouble(0);
		//return pipeline;
    return pipeline.getDouble(0.0);

	}
	
	public void switchLED() {
		if(getLEDMode() == 0) {
			limelight.getEntry("ledMode").setDouble(1);
			SmartDashboard.putString("LED Mode", "Off");
		}else if(getLEDMode() == 1) {
			limelight.getEntry("ledMode").setDouble(0);
			SmartDashboard.putString("LED Mode", "On");
		}else if(getLEDMode() == 2) {
			limelight.getEntry("ledMode").setDouble(1);
			SmartDashboard.putString("LED Mode", "Off");
		}
	}
	
	public void switchCamera() {
		if(getCamMode() == 0) {
			limelight.getEntry("camMode").setDouble(1);
			SmartDashboard.putString("Camera Mode", "Camera");
		}else if(getCamMode() == 1) {
			limelight.getEntry("camMode").setDouble(0);
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

    //getdataToDashboard();
    return limelight;
  }
	
	public void setPipeline(double pipeline) {
		limelight.getEntry("pipeline").setDouble(pipeline);
		SmartDashboard.putNumber("Camera Mode", pipeline);
	}

  public void SwitchToCameraModeWithClosestTarget()
  {
    //goto pipeline 0
    setPipeline(kpipelineAprilTags);
    var apriltaginfo = getdataToDashboard();
    //scan for targets
    boolean AprilTargeted = hasValidTarget();
    //goto pipeline 1 
    setPipeline(kpipelineRetroflectiveHighRung);
    var RetroHighRunginfo = getdataToDashboard();
    //scan for target..
    boolean HighRungTargeted = hasValidTarget();
    //if target found in only 1 choose that. 

    

    //if tartget found in both, choose the one with the lowest x value (left right offset from center)
  }
}
