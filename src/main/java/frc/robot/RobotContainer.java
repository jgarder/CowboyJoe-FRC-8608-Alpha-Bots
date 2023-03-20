package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.*;
import frc.robot.Constants.XboxControllerMap;
import frc.robot.Subsystems.*;
import frc.robot.autos.*;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.GenericEntry;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    //
    private static final String kDefaultAuto = "Default";
    private static final String kDropBackChargeAuto = "DropBackCharge";
    private static final String kDropAndbackupEZsideAuto = "DropAndbackupEZside";
    private static final String kDropBackBumpSideAuto = "DropBackBumpSide";
    private static final String kDropBackPullUpChargeAuto = "DropBackPullUpCharge";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();
    //
    public enum CowboyMode{
        STARTUP,
        CALIBRATED,
        AUTONOMOUS,
        READYTOSTART,
        FLOORHUNTING,
        FLOORGRABBING,
        SUBSTATIONHUNTING,
        SUBSTATIONGRABING,
        PICKED,//LIKE READYTOSTART but with a cone/cube
        CONEPICKED,
        CUBEPICKED,
        SCOREHUNTING,
        SCORECONE,
        SCORECUBE,
        BALANCING,
        ENDGAME,


    }
    public CowboyMode cowboyMode = CowboyMode.STARTUP;
    int stage = 0; //scoreing stages 0 - empty, 1-rdy to grab, 2-grabbed, 3 readying to score, 4-score and reset, 10-balancing
    String currentlyHolding = "";
    
    /* Controllers */
    private final XboxController driveController = new XboxController(0);

    


    /* Subsystems */
    public JoePowerDistributionPanel PDP= new JoePowerDistributionPanel();

    private final NavxSubsystem navx = new NavxSubsystem();
    public static Swerve s_Swerve;
    
    public final JoeColorSensor CSensor= new JoeColorSensor();
    public final Limelight3Subsystem limelight3Subsystem = new Limelight3Subsystem(driveController);
    public final PIDLassoSubsystem PIDLassoSubsystem = new PIDLassoSubsystem(CSensor);
    public final PIDArmExtensionSubsystem PIDArmExtensionSubsystem = new PIDArmExtensionSubsystem();
    public final PIDArmLifterSubsystem PIDArmLifterSubsystem = new PIDArmLifterSubsystem(PIDLassoSubsystem::isLassoinOpenState);

    ArmStateHandler ArmStateHandler = new ArmStateHandler(PIDArmLifterSubsystem, PIDArmExtensionSubsystem, PIDLassoSubsystem,this);
    public static ShuffleboardTab CowboyJowTab;
    public static GenericEntry SpeedAdjustSlider;

    public final SmartDashboardHandler mySmartDashboardHandler = new SmartDashboardHandler(this);
    /* Controller 1 Declarations and instiantiainted  */

    /* Drive Controls */
    

    private final JoystickButton aButton = new JoystickButton(driveController, XboxController.Button.kA.value);
    private final JoystickButton xButton = new JoystickButton(driveController, XboxController.Button.kX.value);
    private final JoystickButton bButton = new JoystickButton(driveController, XboxController.Button.kB.value);
    private final JoystickButton yButton = new JoystickButton(driveController, XboxController.Button.kY.value);

    private final Axis LeftForwardBackAxis = Axis.kLeftX;
    private final Axis LeftLeftRightAxis = Axis.kLeftY;

    private final Axis RightForwardBackAxis = Axis.kRightX;
    private final Axis RightLeftRightAxis = Axis.kRightY;

    private final Trigger LeftTrigger = new Trigger(()->driveController.getRawAxis(Constants.OperatorConstants.kDRIVELeftTriggerAxis) > 0.7);
    private final Trigger RightTrigger = new Trigger(()->driveController.getRawAxis(Constants.OperatorConstants.kDRIVERightTriggerAxis) > 0.7);
    
    private final JoystickButton LeftBumperButton = new JoystickButton(driveController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton robotCentric = new JoystickButton(driveController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton RightBumperButton = new JoystickButton(driveController, XboxController.Button.kRightBumper.value);
    
    private final JoystickButton LeftStickButton = new JoystickButton(driveController, XboxController.Button.kLeftStick.value);
    private final JoystickButton RightStickButton = new JoystickButton(driveController, XboxController.Button.kRightStick.value);

    private final JoystickButton Startbutton = new JoystickButton(driveController, XboxController.Button.kStart.value);
    private final JoystickButton BackButton = new JoystickButton(driveController, XboxController.Button.kBack.value);


    private final POVButton UpHatPOV = new POVButton(driveController, XboxControllerMap.kPOVDirectionUP);
    private final POVButton DownHatPOV = new POVButton(driveController, XboxControllerMap.kPOVDirectionDOWN);
    private final POVButton RightHatPOV = new POVButton(driveController, XboxControllerMap.kPOVDirectionRIGHT);
    private final POVButton LeftHatPOV = new POVButton(driveController, XboxControllerMap.kPOVDirectionLeft);
    
    private final int translationAxis = LeftLeftRightAxis.value;
    private final int strafeAxis = LeftForwardBackAxis.value;
    private final int rotationAxis = RightForwardBackAxis.value;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        //Autonomous selector. 
        m_chooser.setDefaultOption("No Auto", kDefaultAuto);
        m_chooser.addOption("Score/Backup Over Charge station", kDropBackChargeAuto);
        m_chooser.addOption("Score/Backup EZ side", kDropAndbackupEZsideAuto);
        m_chooser.addOption("Score/Backup Bump side", kDropBackBumpSideAuto);
        m_chooser.addOption("drive on Charging after score/backup", kDropBackPullUpChargeAuto);
        SmartDashboard.putData("Auto choices", m_chooser);
        //
        s_Swerve = new Swerve(navx);
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driveController.getRawAxis(translationAxis), 
                () -> -driveController.getRawAxis(strafeAxis), 
                () -> -driveController.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
        //System.out.println("wheel circum: " + Constants.Swerve.wheelCircumference + "gear ratio : " + Constants.Swerve.driveGearRatio);
        //lasso is backwards to everyones brain. 
        //PIDLassoSubsystem.setDefaultCommand(new LassoJoystickCmd(PIDLassoSubsystem,CSensor,()->driveController.getRawAxis(Constants.OperatorConstants.klassoMotorAxis)));

        //Configure Controller 1

        // CONFIGURE DRIVER
        configureButtonBindingsDefault();

        //on boot might as well start up the camera server
        CameraServer.startAutomaticCapture();
        // CowboyJowTab = Shuffleboard.getTab("CowboyJoe");
        // //GenericEntry maxSpeed = tab.add("Joe Speed Adjustment", 0).getEntry();
        // SpeedAdjustSlider =  CowboyJowTab
        // .add("Joe Speed Adjustment", 0)
        // .withWidget(BuiltInWidgets.kNumberSlider)
        // .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
        // .getEntry();
        //s_Swerve.inverseGyro();
        //smartdashboard implementation of speed controller
        


    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    public boolean isReadyToStart(){
        return cowboyMode == CowboyMode.READYTOSTART;
    }
    private void configureButtonBindingsDefault() {

        /* Driver Buttons */
        aButton.onTrue(new InstantCommand(PIDLassoSubsystem::RunLasso,PIDLassoSubsystem));
        //here is one way to handle trigger/axis inputs that act as a button but accept their analog input.
        //this also has the onFalse for both axis triggers that executes when while true is done handled but the Trigger Class.
        // LeftTrigger.whileTrue(new ArmLifterJoystickCmd(PIDArmLifterSubsystem, () -> -driveController.getRawAxis(Constants.OperatorConstants.kDRIVELeftTriggerAxis)))
        // .onFalse(new InstantCommand(PIDArmLifterSubsystem::setSetpointAtCurrentPoint,PIDArmLifterSubsystem));
        // RightTrigger.whileTrue(new ArmLifterJoystickCmd(PIDArmLifterSubsystem, () -> driveController.getRawAxis(Constants.OperatorConstants.kDRIVERightTriggerAxis)))
        // .onFalse(new InstantCommand(PIDArmLifterSubsystem::setSetpointAtCurrentPoint,PIDArmLifterSubsystem));
        LeftTrigger.onTrue(new InstantCommand(ArmStateHandler::resetArmState,ArmStateHandler));
        RightTrigger.onTrue(new InstantCommand(ArmStateHandler::runArmState,ArmStateHandler));
        
        LeftBumperButton.onTrue(new InstantCommand(PIDArmExtensionSubsystem::runArmExtensionStages,PIDArmExtensionSubsystem));
        BooleanSupplier isInScoreMode =  ()->cowboyMode == CowboyMode.SCOREHUNTING;
        //BooleanSupplier isReadyToStart =  ()->;

        xButton.onTrue(
            new ParallelCommandGroup(
                new InstantCommand(()->{cowboyMode = CowboyMode.SCOREHUNTING;}),
                new InstantCommand(PIDArmExtensionSubsystem::setSetpointMidScore,PIDArmExtensionSubsystem),
                new InstantCommand(PIDArmLifterSubsystem::setSetpointScore,PIDArmLifterSubsystem)
                ));
        yButton.onTrue(
            new ParallelCommandGroup(
                new InstantCommand(()->{cowboyMode = CowboyMode.SUBSTATIONHUNTING;}),
                new InstantCommand(PIDArmExtensionSubsystem::setSetpointSubstation,PIDArmExtensionSubsystem),
                new InstantCommand(PIDLassoSubsystem::setSetpointLassoOut,PIDLassoSubsystem),
                new InstantCommand(PIDArmLifterSubsystem::setSetpointSubstationHunt,PIDArmLifterSubsystem)
                ));
        bButton.onTrue(
            new ParallelCommandGroup(
                new InstantCommand(()->{cowboyMode = CowboyMode.FLOORHUNTING;}),
                new InstantCommand(PIDArmExtensionSubsystem::setSetpointIn,PIDArmExtensionSubsystem),
                new InstantCommand(PIDLassoSubsystem::setSetpointLassoOut,PIDLassoSubsystem),
                new InstantCommand(PIDArmLifterSubsystem::setSetpointFloorHunt,PIDArmLifterSubsystem)
                ));

        //UpHatPOV.whileTrue(new StartEndCommand(PIDArmLifterSubsystem::slowWindInBeyondSoftLimit, PIDArmLifterSubsystem::resetEncoder,PIDArmLifterSubsystem));
        UpHatPOV.onTrue(
            new ParallelCommandGroup(
                new InstantCommand(()->{cowboyMode = CowboyMode.READYTOSTART;}),
                new ZeroLifterCmd(PIDArmLifterSubsystem)
                ));

        DownHatPOV.onTrue(new ZeroExtensionCmd(PIDArmExtensionSubsystem));
        
        //RightHatPOV.whileTrue(new StartEndCommand(PIDLassoSubsystem::slowWindInBeyondSoftLimit, PIDLassoSubsystem::resetEncoder,PIDLassoSubsystem));
        RightHatPOV.onTrue(new ZeroLassoCmd(PIDLassoSubsystem));

        //Startbutton.onTrue(new InstantCommand(driveSubsystem::changeturbomode,driveSubsystem));
        Startbutton.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        BackButton.onTrue(new InstantCommand(limelight3Subsystem::switchPipeline,limelight3Subsystem));


        //AlignXButton.whileTrue(new AlignXToTargetCMD(s_Swerve,limelight3Subsystem));   
        //AlignXButton.toggleOnTrue(new AlignXToTargetCMD(s_Swerve,limelight3Subsystem));

        //BalanceButton.whileTrue(new PidBalanceCmd(s_Swerve,navx));     //new JoystickButton(joystick1, Constants.OperatorConstants.kresetLassoEncoderButton).whileTrue(new StartEndCommand(LassoSubsystem::slowWindInBeyondSoftLimit, LassoSubsystem::resetEncoder,LassoSubsystem));
        
        //checkStageControls();

    }

    public void checkStageControls()
    {
        SmartDashboard.putNumber("ActionStage", stage);
        SmartDashboard.putString("currentlyHolding", currentlyHolding);

        //run auto
        //Proceedbutton.onTrue(new myfirstAuto(s_Swerve));
        //Proceedbutton.onTrue(new DriveFollowPath("Test Path 2",4,4));
        //Proceedbutton.onTrue(new DriveFollowPath("back",2,2));
       // Retrybutton.onTrue(new DriveFollowPath("translate left",2,2));
        
        //if pressing proceed then try the next thing
        //Proceedbutton.and(()->{return stage == 0;})
        //.whileTrue(new  SequentialCommandGroup(
            // new openLasso(),
            // new dropArmLiftToPickupReadyHeigh(),

        //   new DriveToGoal(m_drive),
        //    new ScoreTube(m_wrist)););



        //if pressing retry, try the last thing
        //if pressing both, reset?
    }



    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        //Before running any commands do these setup steps.
        navx.ahrs.zeroYaw();


        Command ZeroLassoStartupCmd = new SequentialCommandGroup(
            new InstantCommand(()->PIDLassoSubsystem.HoldAutoLoaded(),PIDLassoSubsystem)
            );
            
        Command ZeroLifterCmd = new ParallelCommandGroup(
                    new InstantCommand(()->{cowboyMode = CowboyMode.READYTOSTART;}),
                    new ZeroLifterCmd(PIDArmLifterSubsystem),
                    new ZeroExtensionCmd(PIDArmExtensionSubsystem)
                    )
               ;
        
        // while(!command.isFinished() && DriverStation.isAutonomousEnabled())
        // {
        //     Timer.delay(1);
        //     System.out.println("WAITING TO AUTO" + command.isFinished() + DriverStation.isAutonomousEnabled());
        // }

        //now get which autonomous is selected?
        m_autoSelected = m_chooser.getSelected();
        System.out.println("Auto selected: " + m_autoSelected);

        //select the script and return it to whatever called this method. 
        switch (m_autoSelected) {
            case kDropBackChargeAuto:
              // Put custom auto code here
              
              break;
              case kDropBackPullUpChargeAuto:
              // Put custom auto code here
              break;
              case kDropAndbackupEZsideAuto:
              // Put custom auto code here
              return ZeroLassoStartupCmd.andThen(ZeroLifterCmd).andThen(EZSideDropBackAutoCMD());
              case kDropBackBumpSideAuto:
              // Put custom auto code here
              break;
            case kDefaultAuto:
            default:
                return new WaitCommand(10);
          }



          return new WaitCommand(10);
        //zero the yaw when we begin. 
        //2023 this is flipped 180
        
        //"backupforwardchargepad","clockwisesquare","straightsquare","spintest","DropAndbackupEZside"
       
    }

    private Command EZSideDropBackAutoCMD() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("DropAndbackupEZside",1,2);
        return 
            new ParallelCommandGroup(
                new InstantCommand(PIDArmExtensionSubsystem::setSetpointHighestScore,PIDArmExtensionSubsystem),
                new InstantCommand(PIDArmLifterSubsystem::setSetpointScore,PIDArmLifterSubsystem)
                )
                .andThen(new WaitCommand(1.8))
                //.andThen(new LassoOutCmd(PIDLassoSubsystem))
                .andThen(new InstantCommand(()->PIDLassoSubsystem.setSetpoint(PIDLassoSubsystem.lassoEncoderValue+40)))
                .andThen(new WaitCommand(.5))
                .andThen(
                    new ParallelCommandGroup(
                        new InstantCommand(PIDArmExtensionSubsystem::setSetpointIn,PIDArmExtensionSubsystem),
                        new InstantCommand(PIDArmLifterSubsystem::setSetpointVertical,PIDArmLifterSubsystem),
                        new SequentialCommandGroup(
                            new LassoInCmd(PIDLassoSubsystem),
                            new ZeroLassoCmd(PIDLassoSubsystem))                        
                    )
            );
            //.andThen(s_Swerve.followTrajectoryCommand(trajectory, true));//ALWAYS RESETS ODOMETRY RN
        //return new DriveFollowPath("clockwisesquare",1,1);//.andThen(new DriveFollowPath("translate right",2,2));
    }
    private Command BumpSideDropBackAutoCMD() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("DropBackBumpSide",1,2);
        return new ParallelCommandGroup(
            new InstantCommand(PIDArmExtensionSubsystem::setSetpointHighestScore,PIDArmExtensionSubsystem),
            new InstantCommand(PIDArmLifterSubsystem::setSetpointScore,PIDArmLifterSubsystem)
            )
            .andThen(new WaitCommand(1.8))
            .andThen(new LassoOutCmd(PIDLassoSubsystem))
            .andThen(
                new ParallelCommandGroup(
                    new InstantCommand(PIDArmExtensionSubsystem::setSetpointIn,PIDArmExtensionSubsystem),
                    new InstantCommand(PIDArmLifterSubsystem::setSetpointVertical,PIDArmLifterSubsystem),
                    new SequentialCommandGroup(
                        new LassoInCmd(PIDLassoSubsystem),
                        new ZeroLassoCmd(PIDLassoSubsystem))                        
                ));
            //.andThen(s_Swerve.followTrajectoryCommand(trajectory, true));//ALWAYS RESETS ODOMETRY RN
        //return new DriveFollowPath("clockwisesquare",1,1);//.andThen(new DriveFollowPath("translate right",2,2));
    }
}
