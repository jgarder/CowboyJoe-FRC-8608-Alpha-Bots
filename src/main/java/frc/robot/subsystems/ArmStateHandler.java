// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.RobotContainer;
import frc.robot.Commands.*;
import frc.robot.RobotContainer.CowboyMode;


public class ArmStateHandler extends SubsystemBase {
 
  // public enum ArmLifterState {
  //   STARTUP,
  //   CALIBRATED,
  //   VERITCALREADY,
  //   SUBSTATIONHUNT,
  //   SUBSTATIONGRAB,
  //   SCOREHUNT,
  //   FLOORHUNT,
  //   FLOORGRAB
    
  // }
  //private ArmLifterState currentstate = ArmLifterState.STARTUP;

  private PIDArmLifterSubsystem s_ALifter;
  private PIDArmExtensionSubsystem s_AExtension;
  private PIDLassoSubsystem s_Lasso;
  private RobotContainer mainbrain;
  public ArmStateHandler(PIDArmLifterSubsystem s_ALifter,PIDArmExtensionSubsystem s_AExtension,PIDLassoSubsystem s_Lasso,RobotContainer mainbrain) {
    this.s_ALifter = s_ALifter;
    this.s_AExtension = s_AExtension;
    this.s_Lasso = s_Lasso;
    this.mainbrain = mainbrain;


  }


  public void resetArmState(){
    //go vertical
    //bring in the extension
    //if we are score mode then we bring lasso back in. 
    //if we are in substation or floor mode then we leave/let the lasso out.
    switch (mainbrain.cowboyMode){
      case STARTUP:
        new ZeroLifterCmd(s_ALifter).schedule();
        mainbrain.cowboyMode = CowboyMode.CALIBRATED;
        //do nothing
        break;
      case CALIBRATED:
      mainbrain.cowboyMode = CowboyMode.READYTOSTART;
        ArmResetting();
        break;
      case READYTOSTART:
        mainbrain.cowboyMode = CowboyMode.READYTOSTART;
        ArmResetting();
        break;
      case SUBSTATIONHUNTING:
      mainbrain.cowboyMode = CowboyMode.READYTOSTART;
        ArmResetting();
        break;
      case SUBSTATIONGRABING:
      mainbrain.cowboyMode = CowboyMode.READYTOSTART;
        ArmResetting();
      s_AExtension.setSetpointIn();
        break;
      case SCOREHUNTING: 
      mainbrain.cowboyMode = CowboyMode.READYTOSTART;
        ArmResetting();
        break;
      case FLOORHUNTING: 
      mainbrain.cowboyMode = CowboyMode.READYTOSTART;
        ArmResetting();
        break;
      case FLOORGRABBING:
      mainbrain.cowboyMode = CowboyMode.READYTOSTART;
        ArmResetting();
        s_AExtension.setSetpointIn();
        break;
    }
  }
  public void runArmState(){
    //if arm is in startup state then we need to calibrate the arm.

    //if arm is in calibrated state then we need to go vertical.
    //if the arm is veritcal and in substation mode then we need to go to the substation hunt position.
    //if the arm is veritcal and in floor mode then we need to go to the floor hunt position.
    //if the arm is veritcal and in score mode then we need to go to the score hunt position.
    
    
    
    switch (mainbrain.cowboyMode){
      case STARTUP:
        new ZeroLifterCmd(s_ALifter).andThen(new InstantCommand(()->s_ALifter.setSetpointVertical())).schedule();
        mainbrain.cowboyMode = CowboyMode.READYTOSTART;
        //do nothing
        break;
      case CALIBRATED:
      mainbrain.cowboyMode = CowboyMode.READYTOSTART;
        s_ALifter.setSetpointVertical();
        break;
      case READYTOSTART:
        //mainbrain.cowboyMode = CowboyMode.FLOORHUNTING;
        //s_ALifter.setSetpointFloorHunt();
        break;
      case SUBSTATIONHUNTING:
        //if the arm is in substation hunt position  then we need to go to the substation grab position.
        new InstantCommand(()->s_ALifter.setSetpointSubstationGrab())
        .andThen(new WaitCommand(.80))
        .andThen(new LassoInCmd(s_Lasso))
        .andThen(new InstantCommand(()->ArmResetting()))
        .andThen(new InstantCommand(()->{SmartDashboard.putNumber("Jow Speed Multiplier", SmartDashboardHandler.CompetitionSpeed);}))
        .schedule();
        mainbrain.cowboyMode = CowboyMode.READYTOSTART;
        break;
      case SUBSTATIONGRABING:
        //if the arm is in substation Grab position  then we need to go to the substation Hunt position.
        mainbrain.cowboyMode = CowboyMode.SUBSTATIONHUNTING;
        new LassoOutCmd(s_Lasso).raceWith(new WaitCommand(.1))
        .andThen(new InstantCommand(()->s_ALifter.setSetpointSubstationHunt()))
        .schedule();
        break;
      case SCOREHUNTING: 
        //if the arm is in score hunt position  then we need to Let lasso out. and auto reset the arm state.
        new LassoOutCmd(s_Lasso)
        .andThen(new InstantCommand(()->ArmResetting())
        .andThen(new WaitCommand(.20))
        .andThen(new LassoInCmd(s_Lasso)))
        .andThen(new ZeroLassoCmd(s_Lasso))
        .andThen(new InstantCommand(()->{SmartDashboard.putNumber("Jow Speed Multiplier", SmartDashboardHandler.CompetitionSpeed);}))
        .schedule();
        mainbrain.cowboyMode = CowboyMode.READYTOSTART;
        // new SequentialCommandGroup(
        //   new LassoOutCmd(s_Lasso),
        //   new ParallelCommandGroup( 
        //     new InstantCommand(()->ArmResetting()),
        //     new LassoInCmd(s_Lasso)).schedule());

        break;
      case FLOORHUNTING: 
        
        //if the arm is in floor hunt position  then we need to go to the floor grab position THEN Bring Lasso in.
        if(s_Lasso.getIsLassoIn())
        {
          //new WaitCommand(.5).schedule();
          new LassoOutCmd(s_Lasso).andThen(new InstantCommand(()->s_ALifter.setSetpointFloorGrab())).andThen(new WaitCommand(.80)).andThen(new LassoInCmd(s_Lasso)).schedule();
        }
        else{
          new InstantCommand(()->s_ALifter.setSetpointFloorGrab()).andThen(new WaitCommand(.80).andThen(new LassoInCmd(s_Lasso))).schedule();
        }
        mainbrain.cowboyMode = CowboyMode.FLOORGRABBING;
        break;
      case FLOORGRABBING:
        new LassoOutCmd(s_Lasso).raceWith(new WaitCommand(.1)).andThen(new InstantCommand(()->s_ALifter.setSetpointFloorHunt())).schedule();
        mainbrain.cowboyMode = CowboyMode.FLOORHUNTING;
        break;
    }
  }

  public void ArmResetting()
  {
    new InstantCommand(()->s_ALifter.setSetpointVertical(),s_ALifter)
    .andThen(new WaitCommand(.10))
    .andThen(new InstantCommand(()->s_AExtension.setSetpointIn(),s_AExtension))
    .schedule();
    
  }
  //LassoOutCmd lassoOut = new LassoOutCmd(s_Lasso);
  //SequentialCommandGroup lassoInZero = new SequentialCommandGroup( new LassoInCmd(s_Lasso),new ZeroLassoCmd(s_Lasso));
  // ParallelCommandGroup ArmResetCmd = new ParallelCommandGroup(
  //   new InstantCommand(()->s_AExtension.setSetpointIn(),s_AExtension),
  //   new InstantCommand(()->s_ALifter.setSetpointVertical(),s_ALifter)
  //   );

}
  
 