package frc.team10505.robot;

import com.ctre.phoenix6.Utils;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;


import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.team10505.robot.subsystems.AlgaeSubsystem;
import frc.team10505.robot.subsystems.CoralSubsystem;
import frc.team10505.robot.subsystems.drive.DrivetrainSubsystem;
import frc.team10505.robot.subsystems.drive.generated.TunerConstants;



public class RobotContainer { 

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDeadband(MaxSpeed*0.1).withRotationalDeadband(MaxAngularRate*0.1).withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    /*controllers */
    private CommandJoystick joystick;
    private CommandJoystick joystick2;

    private CommandXboxController xbox = new CommandXboxController(0);
    private CommandXboxController xbox2;

    /*Subsystems */
    private final DrivetrainSubsystem drivetrainSubsystem = TunerConstants.createDrivetrain();
    // private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
    private final CoralSubsystem coralSubsystem = new CoralSubsystem();

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
        if (Utils.isSimulation()) {
            joystick.button(1).onTrue(algaeSubsystem.setAngle(0));
            joystick.button(2).onTrue(algaeSubsystem.setAngle(-30));
            joystick.button(3).onTrue(algaeSubsystem.setAngle(-90));
        } else {
            xbox.x().onTrue(algaeSubsystem.setAngle(0));
            xbox.a().onTrue(algaeSubsystem.setAngle(-35));
            xbox.b().onTrue(algaeSubsystem.setAngle(-90));

            xbox.leftBumper().whileTrue(algaeSubsystem.runIntake(.6));
            xbox.rightBumper().whileTrue(algaeSubsystem.runIntake(-.6));
        }

         xbox.povUp().whileTrue(drivetrainSubsystem.applyRequest(() -> robotDrive.withVelocityX(0.0).withVelocityY(0.6).withRotationalRate(0.0))).onFalse(drivetrainSubsystem.stop());
         xbox.povDown().whileTrue(drivetrainSubsystem.applyRequest(() -> robotDrive.withVelocityX(0.4).withVelocityY(0.0).withRotationalRate(0.0))).onFalse(drivetrainSubsystem.stop());
         xbox.povLeft().whileTrue(drivetrainSubsystem.applyRequest(() -> robotDrive.withVelocityX(0.0).withVelocityY(0.6).withRotationalRate(0.0)).until(() -> !drivetrainSubsystem.seesLeftSensor()));
         xbox.povRight().whileTrue(drivetrainSubsystem.applyRequest(() -> robotDrive.withVelocityX(0.0).withVelocityY(-0.6).withRotationalRate(0.0)).until(() -> !drivetrainSubsystem.seesRightSensor())); 
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
