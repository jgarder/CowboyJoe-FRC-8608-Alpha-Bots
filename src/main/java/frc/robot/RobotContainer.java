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

    /* Driver Buttons */
    private final JoystickButton robotCentric = new JoystickButton(driveController, XboxController.Button.kLeftBumper.value);

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
        
        PIDLassoSubsystem.setDefaultCommand(new LassoJoystickCmd(PIDLassoSubsystem,CSensor,()->driveController.getRawAxis(Constants.OperatorConstants.klassoMotorAxis)));

        // Configure the button bindings
        configureButtonBindings();

        //on boot might as well start up the camera server
        CameraServer.startAutomaticCapture();
        CowboyJowTab = Shuffleboard.getTab("CowboyJoe");
        //GenericEntry maxSpeed = tab.add("Joe Speed Adjustment", 0).getEntry();
        SpeedAdjustSlider =  CowboyJowTab
    .add("Joe Speed Adjustment", 0)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
    .getEntry();

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        

        //here is one way to handle trigger/axis inputs that act as a button but accept their analog input.
        //this also has the onFalse for both axis triggers that executes when while true is done handled but the Trigger Class.
        Trigger ArmUpTrigger = new Trigger(()->driveController.getRawAxis(Constants.OperatorConstants.kDRIVELeftTriggerAxis) > 0.2);
        ArmUpTrigger.whileTrue(new ArmLifterJoystickCmd(PIDArmLifterSubsystem, () -> -driveController.getRawAxis(Constants.OperatorConstants.kDRIVELeftTriggerAxis))).and(()->PIDArmLifterSubsystem.isLiftArmVerticalOrCloser());
        ArmUpTrigger.onFalse(new InstantCommand(PIDArmLifterSubsystem::setSetpointAtCurrentPoint,PIDArmLifterSubsystem));
        Trigger ArmDownTrigger = new Trigger(()->driveController.getRawAxis(Constants.OperatorConstants.kDRIVERightTriggerAxis) > 0.2);
        ArmDownTrigger.whileTrue(new ArmLifterJoystickCmd(PIDArmLifterSubsystem, () -> driveController.getRawAxis(Constants.OperatorConstants.kDRIVERightTriggerAxis)));
        ArmDownTrigger.onFalse(new InstantCommand(PIDArmLifterSubsystem::setSetpointAtCurrentPoint,PIDArmLifterSubsystem));

        int kArmOutHighestPoleButton = XboxController.Button.kY.value;//this is the Y button, the triangle button or the Top button of the xbox controller
        int kArmOutMidestPoleButton = XboxController.Button.kB.value;//this is the B button, the Square button or the Right button of the xbox controller
        int kArmin = XboxController.Button.kA.value;// A button, bottom of 4 buttons
        JoystickButton armHighestout = new JoystickButton(driveController, kArmOutHighestPoleButton);
        JoystickButton armMidestOut = new JoystickButton(driveController, kArmOutMidestPoleButton);
        armHighestout.onTrue(new InstantCommand(PIDArmExtensionSubsystem::setSetpointHighestScore,PIDArmExtensionSubsystem));
        armMidestOut.onTrue(new InstantCommand(PIDArmExtensionSubsystem::setSetpointMidScore,PIDArmExtensionSubsystem));

        JoystickButton armIn = new JoystickButton(driveController, kArmin);
        armIn.onTrue(new InstantCommand(PIDArmExtensionSubsystem::setSetpointIn,PIDArmExtensionSubsystem));
        
        //new JoystickButton(joystick1, 10).whileTrue(new InstantCommand(PIDArmExtensionSubsystem::enable,PIDArmExtensionSubsystem));
        JoystickButton armliftToScoring = new JoystickButton(driveController, XboxController.Button.kX.value);
        armliftToScoring.onTrue(new InstantCommand(PIDArmLifterSubsystem::setSetpointScore,PIDArmLifterSubsystem));

        POVButton HomeLifterPOV = new POVButton(driveController, XboxControllerMap.kPOVDirectionUP);
        POVButton HomeExtensionPOV = new POVButton(driveController, XboxControllerMap.kPOVDirectionDOWN);
        POVButton HomeLassoPOV = new POVButton(driveController, XboxControllerMap.kPOVDirectionRIGHT);
        
        HomeLifterPOV.whileTrue(new StartEndCommand(PIDArmLifterSubsystem::slowWindInBeyondSoftLimit, PIDArmLifterSubsystem::resetEncoder,PIDArmLifterSubsystem));
        HomeExtensionPOV.whileTrue(new StartEndCommand(PIDArmExtensionSubsystem::slowWindInBeyondSoftLimit, PIDArmExtensionSubsystem::resetEncoder,PIDArmExtensionSubsystem));
        HomeLassoPOV.whileTrue(new StartEndCommand(PIDLassoSubsystem::slowWindInBeyondSoftLimit, PIDLassoSubsystem::resetEncoder,PIDLassoSubsystem));


        JoystickButton Startbutton = new JoystickButton(driveController, XboxController.Button.kStart.value);
        //Startbutton.onTrue(new InstantCommand(driveSubsystem::changeturbomode,driveSubsystem));
        Startbutton.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        JoystickButton switchpipelinebutton = new JoystickButton(driveController, XboxController.Button.kBack.value);
        switchpipelinebutton.onTrue(new InstantCommand(limelight3Subsystem::switchPipeline,limelight3Subsystem));


        JoystickButton AlignXButton = new JoystickButton(driveController, XboxController.Button.kLeftStick.value);
        //AlignXButton.whileTrue(new AlignXToTargetCMD(s_Swerve,limelight3Subsystem));   
        AlignXButton.toggleOnTrue(new AlignXToTargetCMD(s_Swerve,limelight3Subsystem));

        JoystickButton BalanceButton = new JoystickButton(driveController, XboxController.Button.kRightStick.value);
        //BalanceButton.whileTrue(new PidBalanceCmd(s_Swerve,navx));     //new JoystickButton(joystick1, Constants.OperatorConstants.kresetLassoEncoderButton).whileTrue(new StartEndCommand(LassoSubsystem::slowWindInBeyondSoftLimit, LassoSubsystem::resetEncoder,LassoSubsystem));
        
        checkStageControls();

    }

    public void checkStageControls()
    {
        SmartDashboard.putNumber("ActionStage", stage);
        SmartDashboard.putString("currentlyHolding", currentlyHolding);
        JoystickButton Proceedbutton = new JoystickButton(driveController, XboxController.Button.kRightBumper.value);
        JoystickButton Retrybutton = new JoystickButton(driveController, XboxController.Button.kLeftBumper.value);
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
