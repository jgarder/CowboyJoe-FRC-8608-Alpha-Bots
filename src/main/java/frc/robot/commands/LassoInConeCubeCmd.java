package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.Subsystems.PIDLassoSubsystem;
import frc.robot.Subsystems.PIDLassoSubsystem.LassoState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class LassoInConeCubeCmd extends CommandBase {

    //timeout incase of something horrific happening (life)
    private final long timeoutmilliseconds = 4000;
    private long startTime = 0;
    private PIDLassoSubsystem s_Lasso;      
    long end = 0;
    public LassoInConeCubeCmd(PIDLassoSubsystem Lasso) {
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
        

        double wantedsetpoint = Constants.LassoConstants.kminEncoderValue;// default
        if (s_Lasso.ObjectInLasso() == "Cube")
            {
                s_Lasso.lassoState = LassoState.GOCUBE;
                wantedsetpoint = s_Lasso.setSetpointLassoCube();
            }
            else if(s_Lasso.ObjectInLasso() == "Cone")
            {
                s_Lasso.lassoState = LassoState.GOCONE;
                wantedsetpoint = s_Lasso.setSetpointLassoCone();
            }
            else if(s_Lasso.ObjectInLasso() == "RoomLight")
            {
              //nothing detected
              //retractSlowly();
              wantedsetpoint = s_Lasso.setSetpointLassoCone();
              s_Lasso.lassoState = LassoState.GOCONE;
            }
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
        if(s_Lasso.isLassoIn()){
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