package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Averiguador de resenha")
public class Averiguador extends OpMode {
    GoBildaPinpointDriver odometria;

    @Override
    public void init() {
    odometria = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odometria.setOffsets(5.5F, 4.9F, DistanceUnit.CM);
        odometria.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometria.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odometria.resetPosAndIMU();
        odometria.initialize();
        odometria.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));
        odometria.update();
        Pose2D pose2D = odometria.getPosition();
    }

    @Override
    public void loop() {
        odometria.update();
        Pose2D pose2D = odometria.getPosition();
        double angX = pose2D.getX(DistanceUnit.MM);
        double angY = pose2D.getY(DistanceUnit.MM);
        telemetry.addLine("X: " + angX);
        telemetry.addLine("Y: " + angY);
    }
}
