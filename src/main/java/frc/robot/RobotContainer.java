package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.*;
import frc.robot.Constants.XboxControllerMap;
import frc.robot.Subsystems.*;
import frc.robot.autos.*;

import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    //
    int stage = 0; //scoreing stages 0 - empty, 1-rdy to grab, 2-grabbed, 3 readying to score, 4-score and reset, 10-balancing
    String currentlyHolding = "";
    
    /* Controllers */
    private final XboxController driveController = new XboxController(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;


    /* Subsystems */
    public JoePowerDistributionPanel PDP= new JoePowerDistributionPanel();

    private final NavxSubsystem navx = new NavxSubsystem();
    public static Swerve s_Swerve;
    
    public final JoeColorSensor CSensor= new JoeColorSensor();
    public final Limelight3Subsystem limelight3Subsystem = new Limelight3Subsystem(driveController);
    public final PIDLassoSubsystem PIDLassoSubsystem = new PIDLassoSubsystem(CSensor);
    public final PIDArmExtensionSubsystem PIDArmExtensionSubsystem = new PIDArmExtensionSubsystem();
    public final PIDArmLifterSubsystem PIDArmLifterSubsystem = new PIDArmLifterSubsystem();

    public static ShuffleboardTab CowboyJowTab;
    public static GenericEntry SpeedAdjustSlider;

    /* Controller 1 Declarations and instiantiainted  */
    private final JoystickButton aButton = new JoystickButton(driveController, XboxController.Button.kA.value);
    private final JoystickButton xButton = new JoystickButton(driveController, XboxController.Button.kX.value);
    private final JoystickButton bButton = new JoystickButton(driveController, XboxController.Button.kB.value);
    private final JoystickButton yButton = new JoystickButton(driveController, XboxController.Button.kY.value);


    private final Trigger LeftTrigger = new Trigger(()->driveController.getRawAxis(Constants.OperatorConstants.kDRIVELeftTriggerAxis) > 0.05);
    private final Trigger RightTrigger = new Trigger(()->driveController.getRawAxis(Constants.OperatorConstants.kDRIVERightTriggerAxis) > 0.05);
    
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
    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve = new Swerve(navx.ahrs);
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driveController.getRawAxis(translationAxis), 
                () -> -driveController.getRawAxis(strafeAxis), 
                () -> -driveController.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
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

        //smartdashboard implementation of speed controller
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

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */

    private void configureButtonBindingsDefault() {

        /* Driver Buttons */
        aButton.onTrue(new InstantCommand(PIDLassoSubsystem::RunLasso,PIDLassoSubsystem));
        //here is one way to handle trigger/axis inputs that act as a button but accept their analog input.
        //this also has the onFalse for both axis triggers that executes when while true is done handled but the Trigger Class.
        LeftTrigger.whileTrue(new ArmLifterJoystickCmd(PIDArmLifterSubsystem, () -> -driveController.getRawAxis(Constants.OperatorConstants.kDRIVELeftTriggerAxis)))
        .onFalse(new InstantCommand(PIDArmLifterSubsystem::setSetpointAtCurrentPoint,PIDArmLifterSubsystem));
        RightTrigger.whileTrue(new ArmLifterJoystickCmd(PIDArmLifterSubsystem, () -> driveController.getRawAxis(Constants.OperatorConstants.kDRIVERightTriggerAxis)))
        .onFalse(new InstantCommand(PIDArmLifterSubsystem::setSetpointAtCurrentPoint,PIDArmLifterSubsystem));


        LeftBumperButton.onTrue(new InstantCommand(PIDArmExtensionSubsystem::runArmExtensionStages,PIDArmExtensionSubsystem));
        
        xButton.onTrue(new InstantCommand(PIDArmLifterSubsystem::setSetpointScore,PIDArmLifterSubsystem));


        
        UpHatPOV.whileTrue(new StartEndCommand(PIDArmLifterSubsystem::slowWindInBeyondSoftLimit, PIDArmLifterSubsystem::resetEncoder,PIDArmLifterSubsystem));
        DownHatPOV.whileTrue(new StartEndCommand(PIDArmExtensionSubsystem::slowWindInBeyondSoftLimit, PIDArmExtensionSubsystem::resetEncoder,PIDArmExtensionSubsystem));
        RightHatPOV.whileTrue(new StartEndCommand(PIDLassoSubsystem::slowWindInBeyondSoftLimit, PIDLassoSubsystem::resetEncoder,PIDLassoSubsystem));


        //Startbutton.onTrue(new InstantCommand(driveSubsystem::changeturbomode,driveSubsystem));
        Startbutton.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        BackButton.onTrue(new InstantCommand(limelight3Subsystem::switchPipeline,limelight3Subsystem));


        //AlignXButton.whileTrue(new AlignXToTargetCMD(s_Swerve,limelight3Subsystem));   
        //AlignXButton.toggleOnTrue(new AlignXToTargetCMD(s_Swerve,limelight3Subsystem));

        //BalanceButton.whileTrue(new PidBalanceCmd(s_Swerve,navx));     //new JoystickButton(joystick1, Constants.OperatorConstants.kresetLassoEncoderButton).whileTrue(new StartEndCommand(LassoSubsystem::slowWindInBeyondSoftLimit, LassoSubsystem::resetEncoder,LassoSubsystem));
        
        checkStageControls();

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
        // An ExampleCommand will run in autonomous
        //return new exampleAuto(s_Swerve);
        return new DriveFollowPath("translate left",2,2);
    }
}
