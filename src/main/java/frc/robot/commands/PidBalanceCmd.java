package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.Subsystems.NavxSubsystem;
import frc.robot.Subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class PidBalanceCmd extends CommandBase {

    private double duration;   
    private Swerve s_Swerve;  
    private NavxSubsystem s_NavX; 
    private PIDController balancePID;
    
    double lastknownpitch = 0;
    double lastpidpitch = 0;
    // private DoubleSupplier translationSup;
    // private DoubleSupplier strafeSup;
    // private DoubleSupplier rotationSup;
    // private BooleanSupplier robotCentricSup;

    public PidBalanceCmd(Swerve s_Swerve, NavxSubsystem s_NavX) {
        this.s_Swerve = s_Swerve;
        this.s_NavX = s_NavX;
        

        this.balancePID = new PIDController(Constants.PIDController.BalancePID.kP, 
                                            Constants.PIDController.BalancePID.kI, 
                                            Constants.PIDController.BalancePID.kD);
        this.balancePID.setSetpoint(Constants.PIDController.BalancePID.kSetpoint);//this should be the pitch we are going for. 
        this.balancePID.setIntegratorRange(-1, 1);
        addRequirements(s_NavX,s_Swerve);
    }

    public double getCalculatedBalancePID() {
        lastknownpitch = s_NavX.getPitch();
        double calculatedBalancePID = this.balancePID.calculate(lastknownpitch);
        if (calculatedBalancePID > 1) {
            calculatedBalancePID = 1.0;
        }
        SmartDashboard.putNumber("Power", calculatedBalancePID);
        SmartDashboard.putNumber("Balance Error", Constants.PIDController.BalancePID.kSetpoint - lastknownpitch);
        SmartDashboard.putNumber("Setpoint", Constants.PIDController.BalancePID.kSetpoint);
        return calculatedBalancePID;
    }

    @Override
    public void execute() {
        //get pitch/(roll if we spin) from navx

        //get pitch from pid
        lastpidpitch = getCalculatedBalancePID();
        //apply pitch to swerve
        this.s_Swerve.drive(
            new Translation2d(
                balancePID.calculate(0.0, 0),//rollpid required
                balancePID.calculate(0.0, lastpidpitch)
            ).times(Constants.Swerve.maxSpeed), 
            this.s_Swerve.getPose().getRotation().getDegrees(), 
            false, 
            true
        );

        System.out.println("Executing auto balancing");
        System.out.println("Pitch: " + this.lastknownpitch);
        System.out.println("PID Value: " + this.lastpidpitch);
    }

    // called just before this Command runs the first time
    // calculates when to end Command
    public void initialize() {
        double currentTime = System.currentTimeMillis();
        this.duration = (currentTime + 3000);
    }

    // make this return true when this Command no longer needs to run execute()
    // checks if the time has passed the set duration
    public boolean isFinished() {
        if (System.currentTimeMillis() >= this.duration && lastknownpitch == 0) {
            System.out.println("Executing auto balancing");
            System.out.println("Pitch: " + lastknownpitch);
            System.out.println("PID Value: " + lastpidpitch);
            return true;
        }
        return false;
    }

    // Called once after isFinished returns true
    // drive train is stopped
    protected void end() {
        //this.s_Swerve.stop();
    }

    protected void interrupted() {
        this.end();
    }


}