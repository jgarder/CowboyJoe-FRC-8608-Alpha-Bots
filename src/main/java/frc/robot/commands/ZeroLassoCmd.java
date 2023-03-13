package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.Subsystems.NavxSubsystem;
import frc.robot.Subsystems.PIDLassoSubsystem;
import frc.robot.Subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ZeroLassoCmd extends CommandBase {

    long timeoutmilliseconds = 6000;
    long startTime = 0;
    private PIDLassoSubsystem s_Lasso;  

    // private DoubleSupplier translationSup;
    // private DoubleSupplier strafeSup;
    // private DoubleSupplier rotationSup;
    // private BooleanSupplier robotCentricSup;

    public ZeroLassoCmd(PIDLassoSubsystem Lasso) {
        this.s_Lasso = Lasso;
        addRequirements(s_Lasso);
    }

    
    double amps = 0;
    double ampslimit = 30.0; // our motors are current limited to 35, so we shoot for about 28 on a okay//full battery 
    boolean firstpassdone = false;
    @Override
    public void execute() {
        if(!firstpassdone){
            s_Lasso.slowWindInBeyondSoftLimit();
        }
        else
        {
            s_Lasso.slowerWindInBeyondSoftLimit();
        }
        
    }

    // called just before this Command runs the first time
    // calculates when to end Command
    public void initialize() {
        startTime = System.currentTimeMillis(); 
        firstpassdone = false; 
        amps = 0;
    }

    // make this return true when this Command no longer needs to run execute()
    // checks if the time has passed the set duration
    @Override
    public boolean isFinished() {
        long end = startTime + timeoutmilliseconds;
        //System.out.println(String.format("end %d time %d",end,System.currentTimeMillis()));
        if (end < System.currentTimeMillis() ){
            return true;
        }
        amps = s_Lasso.getLassoAmps();
        
        if(amps > ampslimit && !firstpassdone){
            System.out.println(String.format("FP amps  %f",amps));
            firstpassdone = true;
            s_Lasso.resetEncoder();
            Timer.delay(.2);
            s_Lasso.resetEncoder();     
        }
        else if(amps > ampslimit && firstpassdone){
            System.out.println(String.format("SP amps  %f",amps));
            return true;
        }  
        return false;
        
    }

    // Called once after isFinished returns true
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        s_Lasso.resetEncoder();
        super.end(interrupted);
    }


    protected void interrupted() {
        this.end(true);
    }


}