package frc.robot.Commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.CowboyMode;
import frc.robot.Subsystems.SmartDashboardHandler;

public class AutonomousCMDBuilder {

    RobotContainer m_RobotContainer;
    public AutonomousCMDBuilder(RobotContainer Thisbrain)
    {
        this.m_RobotContainer=Thisbrain;
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
                .andThen(new InstantCommand(()->m_RobotContainer.s_Swerve.resetModulesToAbsolute(),m_RobotContainer.s_Swerve))
                .andThen(new ParallelCommandGroup(ResetAfterScore,DropBackPullUpChargeCMD()))
                .andThen(new SequentialCommandGroup(
                    new PidBalanceCmd(m_RobotContainer.s_Swerve, m_RobotContainer.navx), 
                    new WaitCommand(.10),
                    new InstantCommand(() -> m_RobotContainer.s_Swerve.drive(new Translation2d(0,0), 1, false,true),m_RobotContainer.s_Swerve)
                    ));
            case SmartDashboardHandler.kDropBackChargeAuto:
                return ZeroLassoStartupCmd.andThen(ZeroLifterCmd).andThen(DropHighestrun)
                .andThen(new ParallelCommandGroup(ResetAfterScore,DropBackChargeAutoCMD()));
            case SmartDashboardHandler.kDropAndbackupEZsideAuto://EZ side auto
                return ZeroLassoStartupCmd.andThen(ZeroLifterCmd).andThen(DropHighestrun)
                .andThen(new ParallelCommandGroup(ResetAfterScore,EZSideDropBackAutoCMD()))
                .andThen(GotoFloorHunt()).andThen(new WaitCommand(1.4))
                .andThen(new InstantCommand(m_RobotContainer.ArmStateHandler::runArmState,m_RobotContainer.ArmStateHandler))
                    //.andThen(EZSideDropBackReturnAutoCMD()).andThen(GetDropHighestRungRoutine()).andThen(ResetRoutine())
                ;

            case SmartDashboardHandler.kDropBackBumpSideAuto:// Bump side Auto
                return ZeroLassoStartupCmd.andThen(ZeroLifterCmd).andThen(DropHighestrun)
                .andThen(new ParallelCommandGroup(ResetAfterScore,BumpSideDropBackAutoCMD()))
                .andThen(GotoFloorHunt()).andThen(new WaitCommand(1.4))
                .andThen(new InstantCommand(m_RobotContainer.ArmStateHandler::runArmState,m_RobotContainer.ArmStateHandler))
                ;
               
               
            case SmartDashboardHandler.kBumpSideSpin:
                return ZeroLassoStartupCmd.andThen(ZeroLifterCmd).andThen(DropHighestrun)
                .andThen(new ParallelCommandGroup(ResetAfterScore,BumpSideDropBackAutoCMD()));
                //.and then hunt mode and then lasso in, and then reset arm, and then run BumpSide Return rotate cmd. AND THEN Drop Highest. 
            //case SmartDashboardHandler.kChargePadSpin:
                //return ZeroLassoStartupCmd.andThen(ZeroLifterCmd).andThen(new ParallelCommandGroup(ResetAfterScore,ChargepadSpinmoveCMD()));
            case SmartDashboardHandler.kScoreOnlyAuto://calibrate conecube,lifter,extension, then drop to the highest score.
                return ZeroLassoStartupCmd.andThen(ZeroLifterCmd).andThen(DropHighestrun).andThen(ResetAfterScore);
            case SmartDashboardHandler.kCalibrateYesAuto://calibrate lasso to cone/cube, then calibrate lifter and extension no scoring
                return ZeroLassoStartupCmd.andThen(ZeroLifterCmd);
            case SmartDashboardHandler.kCalibrateNoAuto://calibrate lasso to 0, calibrate lifter and extension
                return ZeroLifterCmd.andThen(
                    new SequentialCommandGroup(
                    new LassoInCmd(m_RobotContainer.PIDLassoSubsystem),
                    new ZeroLassoCmd(m_RobotContainer.PIDLassoSubsystem)));
            case SmartDashboardHandler.kDefaultAuto:
            default:
                return new WaitCommand(10);
        }
    }
    public ParallelCommandGroup GotoFloorHunt() {
               return new ParallelCommandGroup(
                new InstantCommand(()->{m_RobotContainer.cowboyMode = CowboyMode.FLOORHUNTING;}),
                new InstantCommand(()->{SmartDashboard.putNumber("Jow Speed Multiplier", SmartDashboardHandler.FloorHuntSpeed); }),
                new InstantCommand(()->{SmartDashboard.putNumber(SmartDashboardHandler.RotationMultiplierName, SmartDashboardHandler.SpinFloorHuntSpeed); }),
                new InstantCommand(m_RobotContainer.PIDArmExtensionSubsystem::setSetpointIn,m_RobotContainer.PIDArmExtensionSubsystem),
                new InstantCommand(m_RobotContainer.PIDLassoSubsystem::setSetpointLassoOut,m_RobotContainer.PIDLassoSubsystem),
                new InstantCommand(m_RobotContainer.PIDArmLifterSubsystem::setSetpointFloorHunt,m_RobotContainer.PIDArmLifterSubsystem)
                );     
            }
    
    public static Command EZSideDropBackAutoCMD() {
        //"backupforwardchargepad","clockwisesquare","straightsquare","spintest","DropAndbackupEZside"
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("DropCubeAndbackupEZside",1.3,2);
        return RobotContainer.s_Swerve.followTrajectoryCommand(trajectory, true);//ALWAYS RESETS ODOMETRY RN
        //return new DriveFollowPath("clockwisesquare",1,1);//.andThen(new DriveFollowPath("translate right",2,2));
    }
    public static Command EZSideDropBackReturnAutoCMD() {
        //"backupforwardchargepad","clockwisesquare","straightsquare","spintest","DropAndbackupEZside"
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("DropCubeAndbackupEZsideReturn",1.3,2);
        return RobotContainer.s_Swerve.followTrajectoryCommand(trajectory, true);//ALWAYS RESETS ODOMETRY RN
        //return new DriveFollowPath("clockwisesquare",1,1);//.andThen(new DriveFollowPath("translate right",2,2));
    }

    private static Command BumpSideDropBackAutoCMD() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("DropBackBumpSide",1.3,2);
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
    // private static Command BumpSideDropSlideRotateCMD() {
    //     PathPlannerTrajectory trajectory = PathPlanner.loadPath("DropBackBumpSide",1.3,2);
    //     return RobotContainer.s_Swerve.followTrajectoryCommand(trajectory, true);//ALWAYS RESETS ODOMETRY RN
    // }
    // private static Command ChargepadSpinmoveCMD() {
    //     PathPlannerTrajectory trajectory = PathPlanner.loadPath("chargepadspinmove",.5,2);
    //     return RobotContainer.s_Swerve.followTrajectoryCommand(trajectory, true);//ALWAYS RESETS ODOMETRY RN
    // }

    private SequentialCommandGroup GetDropHighestRungRoutine() {
        return new ParallelCommandGroup(
                new InstantCommand(m_RobotContainer.PIDArmExtensionSubsystem::setSetpointHighestScore,m_RobotContainer.PIDArmExtensionSubsystem),
                new InstantCommand(m_RobotContainer.PIDArmLifterSubsystem::setSetpointScore,m_RobotContainer.PIDArmLifterSubsystem)
                )
                .andThen(new WaitCommand(1.8))
                //.andThen(new LassoOutCmd(PIDLassoSubsystem))
                .andThen(new InstantCommand(()->m_RobotContainer.PIDLassoSubsystem.setSetpoint(m_RobotContainer.PIDLassoSubsystem.lassoEncoderValue+40)))
                .andThen(new WaitCommand(.5))
                ;
    }
    private ParallelCommandGroup ResetRoutine() {
        return 
            new ParallelCommandGroup(
                new InstantCommand(m_RobotContainer.PIDArmExtensionSubsystem::setSetpointIn,m_RobotContainer.PIDArmExtensionSubsystem),
                new InstantCommand(m_RobotContainer.PIDArmLifterSubsystem::setSetpointVertical,m_RobotContainer.PIDArmLifterSubsystem),
                new SequentialCommandGroup(
                    new LassoInCmd(m_RobotContainer.PIDLassoSubsystem),
                    new ZeroLassoCmd(m_RobotContainer.PIDLassoSubsystem))                        
            );
    }



    private SequentialCommandGroup GetStartupLassoBumpTight() {
        return new SequentialCommandGroup(
            new InstantCommand(()->m_RobotContainer.PIDLassoSubsystem.HoldAutoLoaded(),m_RobotContainer.PIDLassoSubsystem)
            );
    }



    private Command GetZeroLifterCmd() {
        Command ZeroLifterCmd = new ParallelCommandGroup(
                    new InstantCommand(()->{m_RobotContainer.cowboyMode = CowboyMode.READYTOSTART;}),
                    new FastZeroLifterCmd(m_RobotContainer.PIDArmLifterSubsystem),
                    new InstantCommand(()->RobotContainer.s_Swerve.resetModulesToAbsolute(),RobotContainer.s_Swerve)

                    //new ZeroExtensionCmd(Thisbrain.PIDArmExtensionSubsystem)
                    )
               ;
        return ZeroLifterCmd;
    }
}
