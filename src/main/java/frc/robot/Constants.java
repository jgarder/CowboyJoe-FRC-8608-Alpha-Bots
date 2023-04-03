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
        public static final double kStickDeadband = 0.02;
        public static final int kDRIVEJoystickPort = 0;
        public static final int kDRIVEforwardReverseAxis = XboxController.Axis.kLeftY.value;
        public static final int kDRIVELeftRightAxis = XboxController.Axis.kLeftX.value;
        public static final int klassoMotorAxis = XboxController.Axis.kRightY.value;
        public static final int kDRIVERightTriggerAxis = XboxController.Axis.kRightTrigger.value;
        public static final int kDRIVELeftTriggerAxis = XboxController.Axis.kLeftTrigger.value;


        public static final int kArmOutHighestPoleButton = XboxController.Button.kY.value;//this is the Y button, the triangle button or the Top button of the xbox controller
        public static final int kArmOutMidestPoleButton = XboxController.Button.kB.value;//this is the B button, the Square button or the Right button of the xbox controller

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

        public static final double kEncoderValueMin = 0;//p // this should when the arm is laying back on the rest.
        public static final double kEncoderValueMax = 86.4;//p // this is when the arm is horizontal and the arm is extended all the way out.
        //public static final double kEncoderValueMaxCollapsed=76; //this is when the amr is horizontal and the arm is collapsed all the way in
        //public static final double kEncoderValueMaxExtended=71; //this is when the amr is horizontal and the arm is collapsed all the way in
        public static final double kEncoderValueGoalScoring = 49;//p this will change the angle we have our head at when we are scoring goals.
        public static final double kEncoderValueStartingConfig = kEncoderValueMin;// arm laying back and collapsed resting against backstop is 0 position
        public static final double kEncoderValueVertical = 16.0; //p this is when the arm is veritcal at the 90 degree angle

        public static final double kEncoderValueGroundPickupHUNT = 74.4;//p  //Arm IN, right above cube and cone
        public static final double kEncoderValueGroundPickupGRAB = 86.4;//p  //Arm IN , drop for cone and cube pickup

        public static final double kEncoderValueSubStationHunt = 46;//p  //just a lil lower then scoreing (higher angle)
        public static final double kEncoderValueSubStationGrab = kEncoderValueSubStationHunt + 10;  //substation lasso height

        public static final double kslowretractspeed = -.2;
        public static final double kArmLifterUpSpeed = 0.5;
        public static final double kArmLifterDownSpeed = -0.5;

        public static final int kArmLifterSlewRate =8;


        public static int kslewrate = 10;//will be the input slew rate for the arm lifter motor



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

        public static final double kHighestGoalEncoderValue = 205;
        public static final double kMidestGoalEncoderValue = 30;
        public static final double kLowestGoalEncoderValue = 0;

        public static final double kSubstationPickupEncoderValue = 90;
      }

      public static class LassoConstants {
        public static final int klassoMotorCanID = 16;

        public static final double kCubeSpeed = .25;
        public static final double kConeSpeed = .75;

        //
        //double lassoinchesperrev = 1.5;
        //double lassoencodercountsperrev = 42;
        //double lassoencodercountsperinch = lassoencodercountsperrev/lassoinchesperrev;

        //LOOP at Start/Home/Zero
        public static final double kminEncoderValue = 0;
        //lasso at cone or wait/ready
        public static final double kminEncoderValueWithCone = 3.5;//2.8;//p

        public static final double kminEncoderValueWithCube = 54.00;//67 is flat
        //public static final double kavgEncoderValueWithCube = 93;//93 is over inflated
        public static final double kEncoderValueLoopOut= 100.0;
        public static final double kmaxEncoderValue = 127;

        


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
        public static final double wheelCircumference = chosenModule.wheelCircumference;//Units.inchesToMeters(12.125);//OVERRODE by 8608 //chosenModule.wheelCircumference;


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
        public static final double driveKS = (0.45 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);
        // public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        // public static final double driveKV = (1.51 / 12);
        // public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double DRIVE_POS_ERROR_CONTROLLER_P = 0.1;
        public static final double DRIVE_AUTO_ROTATE_CONTROLLER_P = 0.1;
        public static final double DRIVE_MAX_ANGULAR_ACCEL = 4;
        public static final double DRIVE_MAX_ANGULAR_VELOCITY = 4;
        public static final double SWERVE_MAX_VELOCITY_METERS = 4;
        /** Meters per Second */
        public static final double maxSpeed = 1;//.1;//.5;//4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 2;//2;//10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 19;
            public static final int angleMotorID = 21;
            public static final int canCoderID = 29;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(336.17);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 23;
            public static final int angleMotorID = 18;
            public static final int canCoderID = 28;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(162.6);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 25;
            public static final int angleMotorID = 24;
            public static final int canCoderID = 26;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(229.57);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 22;
            public static final int angleMotorID = 20;
            public static final int canCoderID = 27;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(30.3);
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
    public static class PIDController {

      public static class BalancePID {
          public static final int kSetpoint = 0;
          public static final double kP = 0.1;
          public static final double kI = 0.05;
          public static final double kD = 0.5;
      }
    }

    public static class AllianceAprilTags{
      public static class Red {
        public static final int substation = 5;
        public static final int ezSideTag = 3;
        public static final int centerTag = 2;
        public static final int bumpSideTag = 1;
      }
      public static class Blue {
        public static final int substation = 4;
        public static final int ezSideTag = 6;
        public static final int centerTag = 7;
        public static final int bumpSideTag = 8;
      }

    }
}
