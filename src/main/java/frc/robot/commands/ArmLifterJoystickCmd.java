package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.PIDArmLifterSubsystem;



public class ArmLifterJoystickCmd extends CommandBase {

    private final PIDArmLifterSubsystem PIDArmLifterSubsystem;
    private final Supplier<Double> speedFunction;

    public ArmLifterJoystickCmd(PIDArmLifterSubsystem PIDArmLifterSubsystem,
             Supplier<Double> speedFunction) {
        this.speedFunction = speedFunction;
        this.PIDArmLifterSubsystem = PIDArmLifterSubsystem;
        addRequirements(PIDArmLifterSubsystem);
    }
    @Override
    public void initialize() {
        //System.out.println("LassoJoystickCmd started!");
    }

    @Override
    public void execute() {
    double joystickAxis = speedFunction.get();
    //if we are already vertical then we dont need to fold back further.
    //this check will also only fire when the joystick is attempting a negatieve value
     if(PIDArmLifterSubsystem.isLiftArmVerticalOrCloser() && joystickAxis <=0) {
        System.out.println("Arm vertical");
         return;
     }
        //PIDArmLifterSubsystem.SetSpeed(speedFunction.get());
        double SetpointGain = 20;
        PIDArmLifterSubsystem.setSetpoint(PIDArmLifterSubsystem.getMeasurement() + (SetpointGain * joystickAxis));
    }

    @Override
    public void end(boolean interrupted) {
        //System.out.println("LassoJoystickCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
