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
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  RobotContainer thisrobot;
  public SmartDashboardHandler(RobotContainer mainbrain) {
    thisrobot = mainbrain;

    bootupPersistents();
    BuildAutonomousChooser();
  }

  public String getChosenAutoString()
  {
    return m_chooser.getSelected();
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

  private void BuildAutonomousChooser() {
    // build Autonomous selector. 
    m_chooser.setDefaultOption("No Auto", kDefaultAuto);
    m_chooser.setDefaultOption("Calibrate Only Yes ConeCube", kCalibrateYesAuto);
    m_chooser.setDefaultOption("Calibrate Only NO Conecube", kCalibrateNoAuto);
    m_chooser.setDefaultOption("calibrate/Score Only", kScoreOnlyAuto);
    m_chooser.addOption("Score/Backup EZ side", kDropAndbackupEZsideAuto);
    m_chooser.addOption("Score/Backup Bump side", kDropBackBumpSideAuto);
    m_chooser.addOption("Score/Backup Over Charge station", kDropBackChargeAuto);      
    m_chooser.addOption("drive on Charging after score/backup", kDropBackPullUpChargeAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
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
