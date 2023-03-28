package frc.robot.Subsystems;

import frc.robot.SwerveModule;
import frc.robot.autoBalance;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

public class Swerve extends SubsystemBase {

    //Orchestra orchestra;
    //String[] songs = new String[] {"Imperial-March.chrp"};
    //int timeToPlaySong = 10;
   //ArrayList<TalonFX> instruments = new ArrayList<TalonFX>();
    public SwerveDriveKinematics kinematics = Constants.Swerve.swerveKinematics;
    public SwerveDriveOdometry swerveOdometry;
    boolean hasOdometryBeenSet;

    public SwerveModule[] mSwerveMods;
    //public Pigeon2 gyro;
    public NavxSubsystem ahrs;
    public Swerve(NavxSubsystem incomingahrs) {

        //booted music time
        // TalonFX[] motors = { new TalonFX(18),new TalonFX(19),new TalonFX(20),new TalonFX(21),new TalonFX(22),new TalonFX(23),new TalonFX(24)};

        // for (int i = 0; i < motors.length; ++i) {
        //     instruments.add(motors[i]);
        //     }
        // orchestra = new Orchestra(instruments);

        // loadSong(0);
        // orchestra.play(); 
        // Timer.delay(10.0);
        ahrs = incomingahrs;
        //ahrs = new AHRS(SPI.Port.kMXP); /* Alternatives:  SPI.Port.kMXP, I2C.Port.kMXP or SerialPort.Port.kUSB */

        //gyro = new Pigeon2(Constants.Swerve.pigeonID);
        //gyro.configFactoryDefault(); //AHRS does not seem to have this? maybe look for one though
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
        
        hasOdometryBeenSet = false;



    }

    // private void loadSong(int selection) {
    //     orchestra.loadMusic(songs[selection]);
    //     System.out.println("Song selected is: " + songs[selection]);
    // }

    /**
     * Sets the states of all swerve modules based on input velocities with NWU
     * coordinates
     * 
     * @param xVelMeters       the desired velocity of the robot in the x direction
     *                         in meters; relative to either the robot or the field.
     * @param yVelMeters       the desired velocity of the robot in the y direction
     *                         in meters; relative to either the robot or the field.
     * @param degreesPerSecond the desired angular velocity of the robot in degrees
     * @param isFieldRelative
     *                         <ul>
     *                         <li>true - the x and y velocities refer to an
     *                         absolute coordinate axis defined by the field;
     *                         <li>false - the x and y velocities refer to a
     *                         coordinate axis defined by the robot's current angle
     */
    public void drive(double xVelMeters, double yVelMeters, double degreesPerSecond, boolean isFieldRelative) {
        if (isFieldRelative) {
            drive(ChassisSpeeds.fromFieldRelativeSpeeds(xVelMeters, yVelMeters, Math.toRadians(degreesPerSecond),
                    getAngleRotation2d()));
        } else {
            drive(new ChassisSpeeds(xVelMeters, yVelMeters, Math.toRadians(degreesPerSecond)));
        }

    }
    /**
     * Sets the states of all swerve modules based on input
     * <code>ChassisSpeeds</code>
     * 
     * @param chassisSpeeds an object encapsulating desired dx, dy, and dθ of the
     *                      robot
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Swerve.SWERVE_MAX_VELOCITY_METERS);

        for (int i = 0; i < 4; i++) {
            SwerveModuleState currentState = mSwerveMods[i].getState();
            SwerveModuleState.optimize(moduleStates[i], currentState.angle);
            mSwerveMods[i].setDesiredState(moduleStates[i], hasOdometryBeenSet);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }
       /**
     * Gets the Robot's heading as a <code>Rotation2d</code> object
     * 
     * @return a Rotation2d object representing the robot's current heading
     */
    public Rotation2d getAngleRotation2d() {
        return Rotation2d.fromDegrees(getAngleDegrees());
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        ahrs.ahrs.zeroYaw();
        //gyro.setYaw(0);
    }
    
    // Can we use the yaw_offset_tracker minus 180 degrees to return the inverse yaw 
    //offset for corrections when the robot is heading directly towards driver station?
    boolean shouldInvertGyro = Constants.Swerve.invertGyro;
    // public void inverseGyro(){//set yaw when in autonomous facing driver station
    //     shouldInvertGyro = !shouldInvertGyro;
    //     //ahrs.zeroYaw();
    // }
    
    public double flipflipfused()
    {
        double answer = ahrs.ahrs.getFusedHeading();
        if(DriverStation.Alliance.Red == DriverStation.getAlliance())
        {
            answer = answer + 180;
        }
        return answer;
        //return ahrs.getFusedHeading();
    }
    
    public Rotation2d getYaw() {
        return (shouldInvertGyro) ? Rotation2d.fromDegrees(360 - ahrs.getYaw()) : Rotation2d.fromDegrees(ahrs.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }
    /**
     * Gets the IMU's current heading in degrees
     * 
     * @return the IMU's heading in degrees normalized between -180 and +180
     */
    public double getAngleDegrees() {
        double angle = ahrs.ahrs.getFusedHeading() % 360 ; //flipflipfused()
        if (angle > 180) {
            angle -= 360;
        } else if (angle <= -180) {
            angle += 360;
        }
        return -angle;
    }

    private int debounceCount = 0;
    private double debounceTime = 0.2;
    private boolean gottilted = false;
    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());  

        limitSpeedOnChargePadOrTilt();

         for(SwerveModule mod : mSwerveMods){
             SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
             //SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
             //SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
         }
    }

    private void limitSpeedOnChargePadOrTilt() {
        //if tilted slow vehicle in teleop
        if(Math.abs(ahrs.fliproll()) >= autoBalance.onChargeStationDegree)
        {
            gottilted = true;
            debounceCount = 0;
            SmartDashboard.putNumber("Jow Speed Multiplier", SmartDashboardHandler.FloorHuntSpeed);
        }
        else{
            if (debounceCount > autoBalance.secondsToTicks(debounceTime) && gottilted) {
                SmartDashboard.putNumber("Jow Speed Multiplier", SmartDashboardHandler.CompetitionSpeed);
                gottilted = false;
            }
        }
        debounceCount++;
        ///
    }

    // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
         new InstantCommand(() -> {
           // Reset odometry for the first path you run during auto
           if(isFirstPath){
               this.resetOdometry(traj.getInitialHolonomicPose());
           }
         }),
         new PPSwerveControllerCommand(
             traj, 
             this::getPose, // Pose supplier
             this.kinematics, // SwerveDriveKinematics
             new PIDController(0.07, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             new PIDController(0.07, 0, 0), // Y controller (usually the same values as X controller)
             new PIDController(0.2, 0.01, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             this::setModuleStates, // Module states consumer
             true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
             this // Requires this drive subsystem
         )
     );
    }
    
}