package frc.team10505.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import static frc.team10505.robot.Constants.HardwareConstants.*;
import static frc.team10505.robot.Constants.CoralConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//TODO import hardware constants AND coral constants, statically (instead of "import", write "import static" and before the semicolon, add ".*" to import all the vars in the class)

public class CoralSubsystem extends SubsystemBase {
        // Variables (Motors)
        private SparkMax flywheelRight = new SparkMax(CORAL_RIGHT_MOTOR_ID, MotorType.kBrushless);
        private SparkMax flywheelLeft = new SparkMax(CORAL_LEFT_MOTOR_ID, MotorType.kBrushless);

        // Mystery Stuph defining (Configurators)
        private SparkMaxConfig coralLeftConfig = new SparkMaxConfig();
        private SparkMaxConfig coralRightConfig = new SparkMaxConfig();

        // Konstructor
        public CoralSubsystem() {
                coralLeftConfig.idleMode(IdleMode.kBrake);
                coralRightConfig.idleMode(IdleMode.kBrake);
                coralLeftConfig.smartCurrentLimit(CORAL_MOTOR_CURRENT_LIMIT);
                coralRightConfig.smartCurrentLimit(CORAL_MOTOR_CURRENT_LIMIT);
                flywheelLeft.configure(coralLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                flywheelRight.configure(coralRightConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
        }

        //TODO add methods

        //TODO add periodic method(Should only put stuff to Smartdashboard)
}
