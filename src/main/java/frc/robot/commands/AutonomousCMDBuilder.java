package frc.robot.Commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.CowboyMode;
import frc.robot.Subsystems.SmartDashboardHandler;

public class AutonomousCMDBuilder {

    RobotContainer Thisbrain;
    public AutonomousCMDBuilder(RobotContainer Thisbrain)
    {
        this.Thisbrain=Thisbrain;
    }

    public Command GetAutoCommand(String ChosenAuto)
    {
        //command to be used in auton
        Command ZeroLassoStartupCmd = GetStartupLassoBumpTight();
        //command to be used in auton 
        Command ZeroLifterCmd = GetZeroLifterCmd();
        //command to be used in auton
        Command DropHighestrun = GetDropHighestRungRoutine();

        Command ResetAfterScore = ResetRoutine();
        //select the script and return it to whatever called this method. 
        switch (ChosenAuto) {
            case SmartDashboardHandler.kDropBackPullUpChargeAuto:
                return ZeroLassoStartupCmd
                .andThen(ZeroLifterCmd)
                .andThen(DropHighestrun)
                .andThen(new ParallelCommandGroup(ResetAfterScore,DropBackPullUpChargeCMD()))
                .andThen(new SequentialCommandGroup(
                    new PidBalanceCmd(Thisbrain.s_Swerve, Thisbrain.navx), 
                    new WaitCommand(.10),
                    new InstantCommand(() -> Thisbrain.s_Swerve.drive(new Translation2d(0,0), 1, false,true),Thisbrain.s_Swerve)
                    ));
            case SmartDashboardHandler.kDropBackChargeAuto:
                return ZeroLassoStartupCmd.andThen(ZeroLifterCmd).andThen(DropHighestrun)
                .andThen(new ParallelCommandGroup(ResetAfterScore,DropBackChargeAutoCMD()));
            case SmartDashboardHandler.kDropAndbackupEZsideAuto://EZ side auto
                return ZeroLassoStartupCmd.andThen(ZeroLifterCmd).andThen(DropHighestrun)
                .andThen(new ParallelCommandGroup(ResetAfterScore,EZSideDropBackAutoCMD()));
            case SmartDashboardHandler.kDropBackBumpSideAuto:// Bump side Auto
                return ZeroLassoStartupCmd.andThen(ZeroLifterCmd).andThen(DropHighestrun)
                .andThen(new ParallelCommandGroup(ResetAfterScore,BumpSideDropBackAutoCMD()));
            case SmartDashboardHandler.kScoreOnlyAuto://calibrate conecube,lifter,extension, then drop to the highest score.
                return ZeroLassoStartupCmd.andThen(ZeroLifterCmd).andThen(DropHighestrun).andThen(ResetAfterScore);
            case SmartDashboardHandler.kCalibrateYesAuto://calibrate lasso to cone/cube, then calibrate lifter and extension no scoring
                return ZeroLassoStartupCmd.andThen(ZeroLifterCmd);
            case SmartDashboardHandler.kCalibrateNoAuto://calibrate lasso to 0, calibrate lifter and extension
                return ZeroLifterCmd.andThen(
                    new SequentialCommandGroup(
                    new LassoInCmd(Thisbrain.PIDLassoSubsystem),
                    new ZeroLassoCmd(Thisbrain.PIDLassoSubsystem)));
            case SmartDashboardHandler.kDefaultAuto:
            default:
                return new WaitCommand(10);
        }
    }
    
    public static Command EZSideDropBackAutoCMD() {
        //"backupforwardchargepad","clockwisesquare","straightsquare","spintest","DropAndbackupEZside"
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("DropAndbackupEZside",1,2);
        return RobotContainer.s_Swerve.followTrajectoryCommand(trajectory, true);//ALWAYS RESETS ODOMETRY RN
        //return new DriveFollowPath("clockwisesquare",1,1);//.andThen(new DriveFollowPath("translate right",2,2));
    }

    private static Command BumpSideDropBackAutoCMD() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("DropBackBumpSide",1,2);
        return RobotContainer.s_Swerve.followTrajectoryCommand(trajectory, true);//ALWAYS RESETS ODOMETRY RN
    }
    private static Command DropBackChargeAutoCMD() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("DropBackCharge",1,2);
        return RobotContainer.s_Swerve.followTrajectoryCommand(trajectory, true);//ALWAYS RESETS ODOMETRY RN
    }
    private static Command DropBackPullUpChargeCMD() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("DropBackPullUpCharge",1,2);
        return RobotContainer.s_Swerve.followTrajectoryCommand(trajectory, true);//ALWAYS RESETS ODOMETRY RN
    }

    private SequentialCommandGroup GetDropHighestRungRoutine() {
        return new ParallelCommandGroup(
                new InstantCommand(Thisbrain.PIDArmExtensionSubsystem::setSetpointHighestScore,Thisbrain.PIDArmExtensionSubsystem),
                new InstantCommand(Thisbrain.PIDArmLifterSubsystem::setSetpointScore,Thisbrain.PIDArmLifterSubsystem)
                )
                .andThen(new WaitCommand(1.8))
                //.andThen(new LassoOutCmd(PIDLassoSubsystem))
                .andThen(new InstantCommand(()->Thisbrain.PIDLassoSubsystem.setSetpoint(Thisbrain.PIDLassoSubsystem.lassoEncoderValue+40)))
                .andThen(new WaitCommand(.5))
                ;
    }
    private ParallelCommandGroup ResetRoutine() {
        return 
            new ParallelCommandGroup(
                new InstantCommand(Thisbrain.PIDArmExtensionSubsystem::setSetpointIn,Thisbrain.PIDArmExtensionSubsystem),
                new InstantCommand(Thisbrain.PIDArmLifterSubsystem::setSetpointVertical,Thisbrain.PIDArmLifterSubsystem),
                new SequentialCommandGroup(
                    new LassoInCmd(Thisbrain.PIDLassoSubsystem),
                    new ZeroLassoCmd(Thisbrain.PIDLassoSubsystem))                        
            );
    }



    private SequentialCommandGroup GetStartupLassoBumpTight() {
        return new SequentialCommandGroup(
            new InstantCommand(()->Thisbrain.PIDLassoSubsystem.HoldAutoLoaded(),Thisbrain.PIDLassoSubsystem)
            );
    }



    private Command GetZeroLifterCmd() {
        Command ZeroLifterCmd = new ParallelCommandGroup(
                //new InstantCommand(()->RobotContainer.s_Swerve.resetModulesToAbsolute(),RobotContainer.s_Swerve),
                    new InstantCommand(()->{Thisbrain.cowboyMode = CowboyMode.READYTOSTART;}),
                    new FastZeroLifterCmd(Thisbrain.PIDArmLifterSubsystem)
                    //new ZeroExtensionCmd(Thisbrain.PIDArmExtensionSubsystem)
                    )
               ;
        return ZeroLifterCmd;
    }
}
