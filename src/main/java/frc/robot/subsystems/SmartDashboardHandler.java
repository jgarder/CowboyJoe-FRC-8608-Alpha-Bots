package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SmartDashboardHandler extends SubsystemBase {

  //Autonomous choose options
  public static final String kDefaultAuto = "Default";
  public static final String kCalibrateYesAuto = "Calibrate With A ConeCube";
  public static final String kCalibrateNoAuto = "Calibrate with NO ConeCube";
  public static final String kScoreOnlyAuto = "calibrate then score NO MOVEment";
  public static final String kDropBackChargeAuto = "DropBackCharge";
  public static final String kDropAndbackupEZsideAuto = "DropAndbackupEZside";
  public static final String kDropBackBumpSideAuto = "DropBackBumpSide";
  public static final String kDropBackPullUpChargeAuto = "DropBackPullUpCharge";
  public static final String kBumpSideSpin = "Bump Side Spin Move";
  public static final String kChargePadSpin = "Charge Pad Spin Move";
  //private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final SendableChooser<String> m_pickupChooser = new SendableChooser<>();

  RobotContainer thisrobot;
  public SmartDashboardHandler(RobotContainer mainbrain) {
    thisrobot = mainbrain;

    bootupPersistents();
    BuildAutonomousChooser();
    BuildConeCubeChooser();
  }

  public String getChosenAutoString()
  {
    return m_chooser.getSelected();
  }

  public static final String SpeedMultiplierName = "Jow Speed Multiplier";
  public static final String RotationMultiplierName = "Jow Rotation Multiplier";


  public static final double defaultspeed = .25;
  public static final double FloorHuntSpeed = .25;
  public static final double ScoreSpeed = .5;
  public static final double SubstationSpeed = .5;
  public static final double CompetitionSpeed = 1.0;

  public static final double SpinCompetitionSpeed = .5;
  public static final double SpinSubstationSpeed = .25;
  public static final double SpinScoreSpeed = .25;
  public static final double SpinFloorHuntSpeed = .25;


  private void bootupPersistents() {

    if(!SmartDashboard.containsKey("Jow Speed Multiplier"))
        {
            SmartDashboard.putNumber("Jow Speed Multiplier", defaultspeed);
            SmartDashboard.setPersistent("Jow Speed Multiplier");
        }
        if(!SmartDashboard.containsKey("Jow Rotation Multiplier"))
        {
            SmartDashboard.putNumber("Jow Rotation Multiplier", defaultspeed);
            SmartDashboard.setPersistent("Jow Rotation Multiplier");
        }
  }

  private void BuildAutonomousChooser() {
    // build Autonomous selector. 
    m_chooser.setDefaultOption("No Auto", kDefaultAuto);
    m_chooser.addOption("Calibrate Only Yes ConeCube", kCalibrateYesAuto);
    m_chooser.addOption("Calibrate Only NO Conecube", kCalibrateNoAuto);
    m_chooser.addOption("calibrate/Score Only", kScoreOnlyAuto);
    m_chooser.addOption("Score/Backup EZ side", kDropAndbackupEZsideAuto);
    m_chooser.addOption("Score/Backup Bump side", kDropBackBumpSideAuto);
    m_chooser.addOption("Score/Backup Over Charge station", kDropBackChargeAuto);      
    m_chooser.addOption("drive on Charging after score/backup", kDropBackPullUpChargeAuto);
    m_chooser.addOption("Bump Side Spin Move", kBumpSideSpin);
    //m_chooser.addOption("Charge Pad Spin Move", kChargePadSpin);
    SmartDashboard.putData("Auto choices", m_chooser);
    //SmartDashboard.setPersistent("Auto choices");
    
}
public String getChosenPickupString()
{
  return m_pickupChooser.getSelected();
}

public static final String kConeCubeModeName = "Pickup Controls";
public static final String kConeCubeModeCubeMode = "Cube";
public static final String kConeCubeModeConeMode = "Cone";
private void BuildConeCubeChooser() {
  // build Autonomous selector. 
  //m_pickupChooser.setDefaultOption(kConeCubeModeConeMode, kConeCubeModeConeMode);
  //m_pickupChooser.addOption(kConeCubeModeCubeMode, kConeCubeModeCubeMode);
  SmartDashboard.putString(kConeCubeModeName, kConeCubeModeConeMode);
  //SmartDashboard.setPersistent(kConeCubeModeName);
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

    SmartDashboard.putString("LassoMode", thisrobot.PIDLassoSubsystem.lassoState.toString());

    SmartDashboard.putBoolean("Lasso OT", !thisrobot.PIDLassoSubsystem.isMotorOvertemp());
    SmartDashboard.putBoolean("Lifter OT", !thisrobot.PIDArmLifterSubsystem.isMotorOvertemp());
    SmartDashboard.putBoolean("Extension OT", !thisrobot.PIDArmExtensionSubsystem.isMotorOvertemp());
    SmartDashboard.putBoolean("RDY2PICKUP", thisrobot.limelight3Subsystem.iscurrentTagAtSubstationPickupSize());
    
  }


  public static ShuffleboardTab CowboyJowTab;
  public static GenericEntry SpeedAdjustSlider;
  public void SetupShuffleboard()
  {
        CowboyJowTab = Shuffleboard.getTab("CowboyJoe");
        //GenericEntry maxSpeed = tab.add("Joe Speed Adjustment", 0).getEntry();
        SpeedAdjustSlider =  CowboyJowTab
        .add("Joe Speed Adjustment", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
        .getEntry();
        //s_Swerve.inverseGyro();
        //smartdashboard implementation of speed controller
  }


}
