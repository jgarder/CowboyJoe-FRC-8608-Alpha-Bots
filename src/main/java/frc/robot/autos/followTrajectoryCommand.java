package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class followTrajectoryCommand extends CommandBase {
    Timer timer = new Timer();
    PathPlannerTrajectory trajectory;
    HolonomicDriveController controller;
    boolean resetOdometry;

    String pathName;
    static double SWERVE_MAX_VELOCITY_METERS = 4;
    static double SWERVE_MAX_ACCEL_METERS = 10;
    public followTrajectoryCommand(String pathname) {
        this(pathname, SWERVE_MAX_VELOCITY_METERS, SWERVE_MAX_ACCEL_METERS, true);
    }

    public followTrajectoryCommand(String pathname, double maxVel, double maxAccel) {
        this(pathname, maxVel, maxAccel, true);
    }

    public followTrajectoryCommand(String pathName, double maxVel, double maxAccel, boolean resetOdometry) {
        addRequirements(RobotContainer.s_Swerve);

        this.trajectory = PathPlanner.loadPath(pathName, maxVel, maxAccel);


        PIDController xController = new PIDController(Constants.Swerve.DRIVE_POS_ERROR_CONTROLLER_P, 0, 0);
        PIDController yController = new PIDController(Constants.Swerve.DRIVE_POS_ERROR_CONTROLLER_P, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            Constants.Swerve.DRIVE_AUTO_ROTATE_CONTROLLER_P, 0.01, 0,
                new TrapezoidProfile.Constraints(Constants.Swerve.DRIVE_MAX_ANGULAR_VELOCITY,
                Constants.Swerve.DRIVE_MAX_ANGULAR_ACCEL));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.controller = new HolonomicDriveController(xController, yController, thetaController);
        this.resetOdometry = resetOdometry;

        this.pathName = pathName;
    }

    @Override
    public void initialize() {
        //RobotContainer.s_Swerve.enableBrakeMode(true);
        timer.reset();
        timer.start();
        Pose2d initialPose = trajectory.getInitialPose();
        if(resetOdometry) RobotContainer.s_Swerve.resetOdometry(new Pose2d(initialPose.getTranslation(), RobotContainer.s_Swerve.getAngleRotation2d()));
    }

    @Override
    public void execute(){
        double time = timer.get();
        PathPlannerState desiredState = (PathPlannerState) trajectory.sample(time);
        //desiredState = PathPlannerTrajectory.transformStateForAlliance(desiredState,DriverStation.getAlliance());
        ChassisSpeeds targetSpeeds = controller.calculate(RobotContainer.s_Swerve.getPose(), desiredState, new Rotation2d(desiredState.holonomicRotation.getRadians()));

        targetSpeeds.vyMetersPerSecond = -targetSpeeds.vyMetersPerSecond;
        targetSpeeds.omegaRadiansPerSecond = -targetSpeeds.omegaRadiansPerSecond;

        // System.out.println("x:   " + RobotContainer.drive.getPoseMeters().getTranslation().getX() + " y:   " + RobotContainer.drive.getPoseMeters().getTranslation().getY() + " r: " + RobotContainer.drive.getPoseMeters().getRotation().getDegrees());
        // System.out.println("tx:  " + desiredState.poseMeters.getTranslation().getX() + " ty: " + desiredState.poseMeters.getTranslation().getY() + " tr:" + desiredState.holonomicRotation.getDegrees());
        // System.out.println("tvx: " + targetSpeeds.vxMetersPerSecond + " tvy: " + targetSpeeds.vyMetersPerSecond);
        // Position PID
        // SmartDashboard.putNumber("PIDTarget", 0);
        // SmartDashboard.putNumber("PIDActual", pathController.getPosError());

        // Rotation PID
        // SmartDashboard.putNumber("PIDTarget", desiredState.holonomicRotation.getDegrees());
        // SmartDashboard.putNumber("PIDACtual", RobotContainer.drive.getAngleDegrees());

        // Heading PID
        // SmartDashboard.putNumber("PIDTarget", desiredState.poseMeters.getRotation().getDegrees());
        // SmartDashboard.putNumber("PIDActual", pathController.getCurrentHeading().getDegrees());
        // System.out.println("tr:" + Math.round(desiredState.holonomicRotation.getDegrees()) + ", " + "r:" + Math.round(RobotContainer.drive.getAngleDegrees()) + " | th:" + Math.round(desiredState.poseMeters.getRotation().getDegrees()));

        Pose2d currentPose = RobotContainer.s_Swerve.getPose();
        String tString = " [" + Math.round(timer.get() * 100) / 100.0 + "]";
        //System.out.println(pathName + tString + " x error: " + (desiredState.poseMeters.getX() - currentPose.getX()));
        //System.out.println(pathName + tString + " y error: " + (desiredState.poseMeters.getY() - currentPose.getY()));
        //System.out.println(pathName + tString + " r error: " + (desiredState.holonomicRotation.getDegrees() - currentPose.getRotation().getDegrees()));

        RobotContainer.s_Swerve.drive(targetSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        RobotContainer.s_Swerve.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
}