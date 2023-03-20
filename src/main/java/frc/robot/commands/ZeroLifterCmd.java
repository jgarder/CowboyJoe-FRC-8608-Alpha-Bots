package frc.robot.Commands;

import frc.robot.Subsystems.PIDArmLifterSubsystem;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class ZeroLifterCmd extends CommandBase {
    // amp limit for neo brushless 1650 is 45 amps. so its smart limited. be aware. 
    private final double ampslimit = 31.0; // so we shoot for about 28 on a okay//full battery
    //timeout incase of something horrific happening (life)
    private final long timeoutmilliseconds = 8000;

    private long startTime = 0;
    private PIDArmLifterSubsystem s_Lifter;  
    private double amps = 0;

 
    private boolean firstpassdone = false;


    public ZeroLifterCmd(PIDArmLifterSubsystem Lifter) {
        this.s_Lifter = Lifter;
        addRequirements(s_Lifter);
    }

    

    @Override
    public void execute() {
        if(!firstpassdone){
            s_Lifter.slowWindInBeyondSoftLimit();
        }
        else
        {
            s_Lifter.slowerWindInBeyondSoftLimit();
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
        amps = s_Lifter.getMotorAmps();
        
        if(amps > ampslimit && !firstpassdone){
            //System.out.println(String.format("FP amps  %f",amps));
            firstpassdone = true;
            s_Lifter.resetEncoder();
            Timer.delay(.2);
            s_Lifter.resetEncoder();     
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
        s_Lifter.resetEncoder();
        super.end(interrupted);
    }


    protected void interrupted() {
        this.end(true);
    }


}