package frc.robot.Commands;

import frc.robot.Subsystems.PIDLassoSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ZeroLassoCmd extends CommandBase {
    // our motors are current limited to 35, so we shoot for about 28 on a okay//full battery 
    private final double ampslimit = 30.0; 
    //timeout incase of something horrific happening (life)
    private final long timeoutmilliseconds = 6000;
    private long startTime = 0;
    private PIDLassoSubsystem s_Lasso;  
    private double amps = 0;
    private boolean firstpassdone = false;
    // private DoubleSupplier translationSup;
    // private DoubleSupplier strafeSup;
    // private DoubleSupplier rotationSup;
    // private BooleanSupplier robotCentricSup;

    public ZeroLassoCmd(PIDLassoSubsystem Lasso) {
        this.s_Lasso = Lasso;
        addRequirements(s_Lasso);
    }

    

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
        //in first 250 milliseconds we cant finish. 
        if ((startTime + 250) > System.currentTimeMillis()){
            return false;
        }
        amps = s_Lasso.getMotorAmps();
        
        if(amps > ampslimit && !firstpassdone){
            //System.out.println(String.format("FP amps  %f",amps));
            firstpassdone = true;
            s_Lasso.resetEncoder();
            Timer.delay(.2);
            s_Lasso.resetEncoder();     
        }
        else if(amps > ampslimit && firstpassdone){
            //System.out.println(String.format("SP amps  %f",amps));
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