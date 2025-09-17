package frc.robot.commands.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.util.JoystickUtils;

public class ManualArmCommand extends Command {
    private final Arm arm;
    private final CommandXboxController controller;

    public ManualArmCommand(Arm arm, CommandXboxController controller) {
        this.arm = arm;
        this.controller = controller;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        Rotation2d setPoint = Rotation2d.fromRotations(JoystickUtils.smartDeadzone(controller.getLeftY()*3, 0.1));
        arm.setArmPosition(setPoint);
       // arm.holdArmPosition();
    }

    @Override
    public void end(boolean interrupted) {

        //arm.stopArm();
    }
}
