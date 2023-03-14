package frc.robot.Commands;

import frc.robot.Subsystems.PIDArmExtensionSubsystem;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class ZeroExtensionCmd extends CommandBase {

    // amp limit for neo brushless 550 is 35 amps. so its smart limited. be aware. 
    private final double ampslimit = 30.0; // so we shoot for about 28 on a okay//full battery 
    //timeout incase of something horrific happening (life)
    private final long timeoutmilliseconds = 5000;

    private long startTime = 0;
    private PIDArmExtensionSubsystem s_Extension;  
    private double amps = 0;

    
    private boolean firstpassdone = false;


    public ZeroExtensionCmd(PIDArmExtensionSubsystem thisSubsystem) {
        this.s_Extension = thisSubsystem;
        addRequirements(s_Extension);
    }

    

    @Override
    public void execute() {
        if(!firstpassdone){
            s_Extension.slowWindInBeyondSoftLimit();
        }
        else
        {
            s_Extension.slowerWindInBeyondSoftLimit();
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
        //in first 100 milliseconds we cant finish. 
        if ((startTime + 100) > System.currentTimeMillis()){
            return false;
        }
        amps = s_Extension.getMotorAmps();
        System.out.println(String.format("amps  %f",amps));
        if(amps > ampslimit && !firstpassdone){
            //System.out.println(String.format("FP amps  %f",amps));
            firstpassdone = true;
            s_Extension.resetEncoder();
            Timer.delay(.2);
            s_Extension.resetEncoder();     
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
        s_Extension.resetEncoder();
        super.end(interrupted);
    }


    protected void interrupted() {
        this.end(true);
    }


}