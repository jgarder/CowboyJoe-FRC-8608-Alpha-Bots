package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

  public static class NeoBrushless
  {
    public static int neo550safelimitAmps = 35;//lasso and arm extension use these
    public static int neo1650safelimitAmps = 45; //this is the main arm lift motor. 
  }

    public static class Canivore{
      public static final String CanivoreBus1Name = "8608ChassisCan";
    }

    public static class XboxControllerMap {
        //THIS MAP EXCEPT POV exist as Xboxcontroller.button.KBack.value or .Axis.kRightY ect ect ect. 
        //these are the pov directional buttons (this had 8 directions but we do not have them mapped.)
        public static final int kPOVDirectionUP = 0;//0 is up on xbox controller POV hat
        public static final int kPOVDirectionDOWN = 180;//180 is down on xbox controller POV hat
        public static final int kPOVDirectionRIGHT = 90;//90 is right on xbox controller POV hat
        public static final int kPOVDirectionLeft = 270;//270 is Left on xbox controller POV hat
    
      }
      public static class OperatorConstants {
        
        public static final int kDRIVEJoystickPort = 0;
        public static final int kDRIVEforwardReverseAxis = XboxController.Axis.kLeftY.value;
        public static final int kDRIVELeftRightAxis = XboxController.Axis.kLeftX.value;
        public static final int klassoMotorAxis = XboxController.Axis.kRightY.value;
        public static final int kDRIVERightTriggerAxis = XboxController.Axis.kRightTrigger.value;
        public static final int kDRIVELeftTriggerAxis = XboxController.Axis.kLeftTrigger.value;
        public static final int kArmupButton = XboxController.Button.kRightBumper.value;
        public static final int kArmdownButton = XboxController.Button.kStart.value;
        public static final int kArmoutButton = XboxController.Button.kBack.value;
        public static final int kArminButton = XboxController.Button.kLeftBumper.value; 
    
        //public static final int kresetLassoEncoderButton = 9;//#9 is the "select" button in the middle of the joystick
        
        public static final int RatchetLinearActuatorsparkMaxCanID = 15;
      }
      public static class DriveConstants {
        public static final int kFrontLeftSparkMaxCanID = 8;
        public static final int kFollowerLeftSparkMaxCanID = 10;
        public static final int kFrontRightSparkMaxCanID = 6;
        public static final int kFollowerRightSparkMaxCanID = 9;
        public static final boolean kleftInverted = false;
        public static final boolean krightInverted = true;
        
      }
    
      public static class ArmLifterConstants {
        public static final int kArmLifterSparkMaxCanID = 7;
    
        public static final double kminEncoderValue = 0; // this should when the arm is vertical and slightly leaning back if not straight.
        public static final double kmaxEncoderValue = 65;// this is when the arm is horizontal and the arm is extended all the way out.
        public static final double kmaxEncoderValueCollapsed=65; //this is when the amr is horizontal and the arm is collapsed all the way in
        public static final double kmaxEncoderValueExtended=60; //this is when the amr is horizontal and the arm is collapsed all the way in
    
        public static final double kArmLifterUpSpeed = 0.5;
        public static final double kArmLifterDownSpeed = -0.5;
        
        public static final int kArmLifterSlewRate =8;
        public static int kslewrate = 10;//will be the input slew rate for the arm lifter motor
    
        public static final double kGoalScoringEncoderValue = 36.7;
    
      }
    
      public static class ArmExtensionConstants {
        public static final int kArmExtensionSparkMaxCanID = 13;
        
        //double CubeSpeed = .25;
        //double ConeSpeed = .75;
    
        public static final double kminEncoderValue = 0;
        public static final double kmaxEncoderValue = 220;
    
        public static final double kArmOutSpeed = 0.75;
        public static final double kArmInSpeed = -0.75;
    
        public static final int kArmExtensionSlewRate = 10;
    
        public static final double kHighestGoalEncoderValue = 195;
        public static final double kMidestGoalEncoderValue = 30;
        public static final double kLowestGoalEncoderValue = 0;
      }
    
      public static class LassoConstants {
        public static final int klassoMotorCanID = 16;
        
        public static final double kCubeSpeed = .25;
        public static final double kConeSpeed = .75;
    
        //
        //double lassoinchesperrev = 1.5;
        //double lassoencodercountsperrev = 42;
        //double lassoencodercountsperinch = lassoencodercountsperrev/lassoinchesperrev;
    
       
        public static final double kminEncoderValue = 0;
        public static final double kminEncoderValueWithCube = 93;
        public static final double kEncoderValueLoopOut= 150;
        public static final double kmaxEncoderValue = 180;
    
        public static final double kminEncoderValueWithCone = 4;
    
      
      }

    public static final double stickDeadband = 0.10;

    public static final class Swerve {
        //public static final int pigeonID = 1;
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L1);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.5); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(26.5); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = .5;//4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 2;//10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 19;
            public static final int angleMotorID = 21;
            public static final int canCoderID = 29;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(336.09);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 23;
            public static final int angleMotorID = 18;
            public static final int canCoderID = 28;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(162.5);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 25;
            public static final int angleMotorID = 24;
            public static final int canCoderID = 26;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(231.06);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 22;
            public static final int angleMotorID = 20;
            public static final int canCoderID = 27;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(32);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
