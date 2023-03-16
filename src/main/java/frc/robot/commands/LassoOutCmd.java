package frc.robot.Commands;

import frc.robot.Subsystems.PIDLassoSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class LassoOutCmd extends CommandBase {

    //timeout incase of something horrific happening (life)
    private final long timeoutmilliseconds = 4000;
    private long startTime = 0;
    private PIDLassoSubsystem s_Lasso;      
    long end = 0;
    public LassoOutCmd(PIDLassoSubsystem Lasso) {
        this.s_Lasso = Lasso;
        addRequirements(s_Lasso);
    }

    

    @Override
    public void execute() {
       
        
    }

    // called just before this Command runs the first time
    // calculates when to end Command
    public void initialize() {
        startTime = System.currentTimeMillis(); 
        end = startTime + timeoutmilliseconds;
        s_Lasso.setSetpointLassoOut();
    }

    // make this return true when this Command no longer needs to run execute()
    // checks if the time has passed the set duration
    @Override
    public boolean isFinished() {
        //check if timed out first
        if (end < System.currentTimeMillis() ){
            return true;
        }
        
        //if our motor encoder is at the setpoint, we are done
        if(s_Lasso.isLassoOut()){
            return true;
        }
        return false;
        
    }

    // Called once after isFinished returns true
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
        super.end(interrupted);
    }


    protected void interrupted() {
        this.end(true);
    }


}