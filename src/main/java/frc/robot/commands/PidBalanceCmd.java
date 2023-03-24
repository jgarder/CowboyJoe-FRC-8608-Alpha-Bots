package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.autoBalance;
import frc.robot.Subsystems.NavxSubsystem;
import frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class PidBalanceCmd extends CommandBase {

    private double duration;   
    private Swerve s_Swerve;  
     private NavxSubsystem s_NavX; 
    // private PIDController balancePID;
    
    //balance subsystem
    public autoBalance mAutoBalance;

    //double lastknownpitch = 0;
    //double lastpidpitch = 0;
    // private DoubleSupplier translationSup;
    // private DoubleSupplier strafeSup;
    // private DoubleSupplier rotationSup;
    // private BooleanSupplier robotCentricSup;

    public PidBalanceCmd(Swerve s_Swerve, NavxSubsystem s_NavX) {
        //auto balance module
        mAutoBalance = new autoBalance(s_NavX);
        this.s_Swerve = s_Swerve;
        this.s_NavX = s_NavX;
        

        // this.balancePID = new PIDController(Constants.PIDController.BalancePID.kP, 
        //                                     Constants.PIDController.BalancePID.kI, 
        //                                     Constants.PIDController.BalancePID.kD);
        // this.balancePID.setSetpoint(Constants.PIDController.BalancePID.kSetpoint);//this should be the pitch we are going for. 
        // this.balancePID.setIntegratorRange(-1, 1);
        addRequirements(s_Swerve);
    }

    // public double getCalculatedBalancePID() {
    //     lastknownpitch = s_NavX.getPitch();
    //     double calculatedBalancePID = this.balancePID.calculate(lastknownpitch);
    //     if (calculatedBalancePID > 1) {
    //         calculatedBalancePID = 1.0;
    //     }
    //     SmartDashboard.putNumber("Power", calculatedBalancePID);
    //     SmartDashboard.putNumber("Balance Error", Constants.PIDController.BalancePID.kSetpoint - lastknownpitch);
    //     SmartDashboard.putNumber("Setpoint", Constants.PIDController.BalancePID.kSetpoint);
    //     return calculatedBalancePID;
    // }

    @Override
    public void execute() {
        double speed = mAutoBalance.autoBalanceRoutine();
        s_Swerve.drive(speed*1.0, 0, 0, false);
        SmartDashboard.putNumber("Balancer PID", speed);
        SmartDashboard.putNumber("Balancer Tilt", mAutoBalance.getTilt());
        //SmartDashboard.putNumber("Balancer Roll", mAutoBalance.getRoll());
        //SmartDashboard.putNumber("Balancer Pitch", mAutoBalance.getPitch());
        SmartDashboard.putNumber("navx roll", mAutoBalance.fliproll());
    }

    // called just before this Command runs the first time
    // calculates when to end Command
    public void initialize() {
        mAutoBalance.state = 0;
        double currentTime = System.currentTimeMillis();
        this.duration = (currentTime + 10000);
    }

    // make this return true when this Command no longer needs to run execute()
    // checks if the time has passed the set duration
    public boolean isFinished() {
        if (System.currentTimeMillis() >= this.duration || mAutoBalance.state == 4) {
            return true;
        }
        return false;
    }

    // Called once after isFinished returns true
    // drive train is stopped
    protected void end() {
        this.s_Swerve.drive(0, 0, 0, isFinished());;
    }

    protected void interrupted() {
        this.end();
    }


}