package frc.team10505.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team10505.robot.subsystems.drive.DrivetrainSubsystem;
import frc.team10505.robot.subsystems.drive.generated.TunerConstants;

public class RobotContainer { 
    /*controllers */
    private CommandJoystick joystick;
    private CommandJoystick joystick2;

    private CommandXboxController xbox;
    private CommandXboxController xbox2;

    /*Subsystems */
    private final DrivetrainSubsystem driveSubsys = TunerConstants.createDrivetrain();
    //TODO add other subsystems

    /*Sendable choosers */
    private SendableChooser<Double> polarityChooser = new SendableChooser<>(); 
    //TODO add sendable chooser <Command> for autons. Dont assign it a value though


    /*Constructor */
    public RobotContainer(){
        if(RobotBase.isSimulation()){
            joystick = new CommandJoystick(0);
            joystick2 = new CommandJoystick(1);

            simConfigButtonBindings();
        } else {
            xbox = new CommandXboxController(0);
            xbox2 = new CommandXboxController(1);

            configButtonBindings();
        }

        //TODO call autobuilder config once created
        configNamedCommands();
        configSendableChoosers();
    }

    private void simConfigButtonBindings(){

    }

    private void configButtonBindings(){

    }

    private void configSendableChoosers(){
        polarityChooser.setDefaultOption("Positive", 1.0);
        polarityChooser.addOption("Negative", -1.0);
        SmartDashboard.putData("Polarity Chooser", polarityChooser);

    }

    private void configNamedCommands(){

    }
}
