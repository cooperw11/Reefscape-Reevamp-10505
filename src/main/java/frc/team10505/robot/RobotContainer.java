package frc.team10505.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team10505.robot.subsystems.AlgaeSubsystem;
import frc.team10505.robot.subsystems.drive.DrivetrainSubsystem;
import frc.team10505.robot.subsystems.drive.generated.TunerConstants;

public class RobotContainer { 
    /*controllers */
    private CommandJoystick joystick;
    private CommandJoystick joystick2;

    private CommandXboxController driveController = new CommandXboxController(0);
    private CommandXboxController xbox2;

    /*Subsystems */
    private final DrivetrainSubsystem driveSubsys = TunerConstants.createDrivetrain();
    private final AlgaeSubsystem algaeSubsys = new AlgaeSubsystem();
    //TODO add other subsystems

    /*Sendable choosers */
    private SendableChooser<Double> polarityChooser = new SendableChooser<>(); 

    /*Constructor */
    public RobotContainer(){
        if(RobotBase.isSimulation()){
            joystick = new CommandJoystick(0);
            joystick2 = new CommandJoystick(1);

            simConfigButtonBindings();
        } else {
            driveController = new CommandXboxController(0);
            xbox2 = new CommandXboxController(1);

            configButtonBindings();
        }

        //TODO create and call pathplanner config
        configNamedCommands();
        configSendableChoosers();
    }

    private void simConfigButtonBindings(){

    }

    private void configButtonBindings(){
        if (Utils.isSimulation()) {
            joystick.button(1).onTrue(algaeSubsys.setAngle(0));
            joystick.button(2).onTrue(algaeSubsys.setAngle(-30));
            joystick.button(3).onTrue(algaeSubsys.setAngle(-90));
        } else {
            driveController.x().onTrue(algaeSubsys.setAngle(0));
            driveController.a().onTrue(algaeSubsys.setAngle(-30));
            driveController.b().onTrue(algaeSubsys.setAngle(-90));
            driveController.y().onTrue(algaeSubsys.setAngle(90));
        }
    }

    private void configSendableChoosers(){
        polarityChooser.setDefaultOption("Positive", 1.0);
        polarityChooser.addOption("Negative", -1.0);
        SmartDashboard.putData("Polarity Chooser", polarityChooser);

    }

    private void configNamedCommands(){

    }
     // Pivot Controls
     private void algaePivotControls() {
        
    }
}
