package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.*;
import frc.robot.Constants.XboxControllerMap;
import frc.robot.Subsystems.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Translation2d;

public class RobotContainer { 
 
    //possible Cowboy Modes
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
    
    private final XboxController driveController = new XboxController(0);
    private final GenericHID CoPilotController = new GenericHID(1);

    


    /* Subsystems */
    public PWM ourpwm;
    public Relay ourRelay;
    public JoePowerDistributionPanel PDP= new JoePowerDistributionPanel();
    public final NavxSubsystem navx = new NavxSubsystem();
    public static Swerve s_Swerve;
    public final JoeColorSensor CSensor= new JoeColorSensor();
    public final Limelight3Subsystem limelight3Subsystem = new Limelight3Subsystem(driveController);
    public final PIDLassoSubsystem PIDLassoSubsystem = new PIDLassoSubsystem(CSensor);
    public final PIDArmExtensionSubsystem PIDArmExtensionSubsystem = new PIDArmExtensionSubsystem(this);
    public final PIDArmLifterSubsystem PIDArmLifterSubsystem = new PIDArmLifterSubsystem(PIDLassoSubsystem::isLassoinOpenState);
    public final ArmStateHandler ArmStateHandler = new ArmStateHandler(PIDArmLifterSubsystem, PIDArmExtensionSubsystem, PIDLassoSubsystem,this);
    public final AutonomousCMDBuilder AutoCmdBuilder = new AutonomousCMDBuilder(this);
    public final SmartDashboardHandler mySmartDashboardHandler = new SmartDashboardHandler(this);
    /* co pilot Controller 1 Declarations and instiantiainted  */
    private final JoystickButton leftButton = new JoystickButton(CoPilotController, 11);
    private final JoystickButton RightButton = new JoystickButton(CoPilotController, 12);


    /* Main Driver Controller 0 Declarations and instiantiainted  */
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
        navx.ahrs.calibrate();
        //bootup the led pwm
        ourpwm = new PWM(8);
        ourRelay = new Relay(0);
       
        
        //Setup DriveTrain
        s_Swerve = new Swerve(navx);
        ourpwm.setRaw(1000);

        // CONFIGURE CONTROLLER
        configureButtonBindingsDefault();
        configureCopilotController();
        //on boot might as well start up the camera server
        CameraServer.startAutomaticCapture();

        
    }

    public boolean isReadyToStart(){
        return cowboyMode == CowboyMode.READYTOSTART;
    }

    private void configureCopilotController(){
        leftButton.onTrue(new InstantCommand(()->{SmartDashboard.putString(SmartDashboardHandler.kConeCubeModeName, SmartDashboardHandler.kConeCubeModeConeMode);})
            );
        RightButton.onTrue(new InstantCommand(()->{SmartDashboard.putString(SmartDashboardHandler.kConeCubeModeName, SmartDashboardHandler.kConeCubeModeCubeMode);})
        );
    }
    private void configureButtonBindingsDefault() {
        //JoySticks Left and right
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driveController.getRawAxis(translationAxis), 
                () -> -driveController.getRawAxis(strafeAxis), 
                () -> -driveController.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        
        //Back Triggers
        LeftTrigger.onTrue(new ParallelCommandGroup(
            new InstantCommand(ArmStateHandler::resetArmState,ArmStateHandler),
            new InstantCommand(()->{SmartDashboard.putNumber("Jow Speed Multiplier", SmartDashboardHandler.CompetitionSpeed);})
            ));
        RightTrigger.onTrue(new InstantCommand(ArmStateHandler::runArmState,ArmStateHandler));
        
        //Upper Bumper (shoulder) buttons
        LeftBumperButton.onTrue(new InstantCommand(PIDArmExtensionSubsystem::runArmExtensionStages,PIDArmExtensionSubsystem));
        //RightBumperButton.onTrue(new AlignPathSubstationCMD(s_Swerve, limelight3Subsystem));
        //RightBumperButton.toggleOnTrue(new AlignSubstationCMD(s_Swerve, limelight3Subsystem));
        RightBumperButton.whileTrue(new AlignSubstationCMD(s_Swerve, limelight3Subsystem).andThen(new ParallelCommandGroup(
            new InstantCommand(()->{cowboyMode = CowboyMode.SUBSTATIONHUNTING;}),
            new InstantCommand(()->{SmartDashboard.putNumber("Jow Speed Multiplier", SmartDashboardHandler.SubstationSpeed); }),
            new InstantCommand(PIDArmExtensionSubsystem::setSetpointSubstation,PIDArmExtensionSubsystem),
            new InstantCommand(PIDLassoSubsystem::setSetpointLassoOut,PIDLassoSubsystem),
            new InstantCommand(PIDArmLifterSubsystem::setSetpointSubstationHunt,PIDArmLifterSubsystem)
            )));

        // Basic Buttons (X,Y,B,A)
        xButton.onTrue(
            new ParallelCommandGroup(
                new InstantCommand(()->{cowboyMode = CowboyMode.SCOREHUNTING; }),
                new InstantCommand(()->{SmartDashboard.putNumber("Jow Speed Multiplier", SmartDashboardHandler.ScoreSpeed); }),
                new InstantCommand(PIDArmExtensionSubsystem::setSetpointMidScore,PIDArmExtensionSubsystem),
                new InstantCommand(PIDArmLifterSubsystem::setSetpointScore,PIDArmLifterSubsystem)
                ));
        yButton.onTrue(
            new ParallelCommandGroup(
                new InstantCommand(()->{cowboyMode = CowboyMode.SUBSTATIONHUNTING;}),
                new InstantCommand(()->{SmartDashboard.putNumber("Jow Speed Multiplier", SmartDashboardHandler.SubstationSpeed); }),
                new InstantCommand(PIDArmExtensionSubsystem::setSetpointSubstation,PIDArmExtensionSubsystem),
                new InstantCommand(PIDLassoSubsystem::setSetpointLassoOut,PIDLassoSubsystem),
                new InstantCommand(PIDArmLifterSubsystem::setSetpointSubstationHunt,PIDArmLifterSubsystem)
                ));
        bButton.onTrue(
            new ParallelCommandGroup(
                new InstantCommand(()->{cowboyMode = CowboyMode.FLOORHUNTING;}),
                new InstantCommand(()->{SmartDashboard.putNumber("Jow Speed Multiplier", SmartDashboardHandler.FloorHuntSpeed); }),
                new InstantCommand(PIDArmExtensionSubsystem::setSetpointIn,PIDArmExtensionSubsystem),
                new InstantCommand(PIDLassoSubsystem::setSetpointLassoOut,PIDLassoSubsystem),
                new InstantCommand(PIDArmLifterSubsystem::setSetpointFloorHunt,PIDArmLifterSubsystem)
                ));
        aButton.onTrue(new InstantCommand(PIDLassoSubsystem::RunLasso,PIDLassoSubsystem));
        
        //Hat PoV Dpad;
        UpHatPOV.onTrue(
            new ParallelCommandGroup(
                new InstantCommand(()->{cowboyMode = CowboyMode.READYTOSTART;}),
                new ZeroLifterCmd(PIDArmLifterSubsystem)
                ));
        DownHatPOV.onTrue(new ZeroExtensionCmd(PIDArmExtensionSubsystem));      
        RightHatPOV.onTrue(new ZeroLassoCmd(PIDLassoSubsystem));
        LeftHatPOV.onTrue(new InstantCommand(()->s_Swerve.resetModulesToAbsolute(),s_Swerve));
        //Central Menu buttons
        Startbutton.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        BackButton.onTrue(new InstantCommand(limelight3Subsystem::switchPipeline,limelight3Subsystem));
        //Startbutton.onTrue(new InstantCommand(driveSubsystem::changeturbomode,driveSubsystem));

        //Joystick Down Buttons
        //AlignXButton.whileTrue(new AlignXToTargetCMD(s_Swerve,limelight3Subsystem));   
        //AlignXButton.toggleOnTrue(new AlignXToTargetCMD(s_Swerve,limelight3Subsystem));
        //BalanceButton.whileTrue(new PidBalanceCmd(s_Swerve,navx));     //new JoystickButton(joystick1, Constants.OperatorConstants.kresetLassoEncoderButton).whileTrue(new StartEndCommand(LassoSubsystem::slowWindInBeyondSoftLimit, LassoSubsystem::resetEncoder,LassoSubsystem));

        // RightStickButton.toggleOnTrue(
        //     new SequentialCommandGroup(
        //         new PidBalanceCmd(s_Swerve, navx), 
        //         new WaitCommand(.10),
        //         new InstantCommand(() -> s_Swerve.drive(new Translation2d(0,0), 1, false,true),s_Swerve)
        //         ));
    }


    public Command getAutonomousCommand() {
        //Before running any commands do these setup steps.
        navx.ahrs.zeroYaw();
        RobotContainer.s_Swerve.resetModulesToAbsolute();
        //now get which autonomous is selected?
        String SelectedAuto = mySmartDashboardHandler.getChosenAutoString();
        System.out.println("Auto selected: " + SelectedAuto);

        //return command from command builder. 
        return AutoCmdBuilder.GetAutoCommand(SelectedAuto);
        
    }


}
