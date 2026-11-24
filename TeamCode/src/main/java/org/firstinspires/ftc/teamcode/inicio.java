package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
@Autonomous(name = "Início")
public class inicio extends LinearOpMode {
    GoBildaPinpointDriver odometria;
    public void calculos(double potenciaX, double potenciaY , double IMU, double giro, final double Alvo){
        odometria.initialize();
        odometria.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        Pose2D pose2D = odometria.getPosition();
        final double alvoY = pose2D.getY(DistanceUnit.MM);
        final double alvoX = pose2D.getX(DistanceUnit.MM);
        double angX = pose2D.getX(DistanceUnit.MM);
        double angY = pose2D.getY(DistanceUnit.MM);
        double kP =
                0.001;
        double vx =
                potenciaX;
        double vy =
                potenciaY;
        double angulo =
                IMU;
        double omega =
                giro;
        double erro =
                Alvo - angulo;
        double Proporcional =
                kP * erro;
        if (potenciaX == 0 && potenciaY == 0){
            double erro_atualpodX = alvoX - angX;
            double erro_atualpodY = alvoY - angY;

            double potX;
            double potY;
        }
        double vxRot = vx * Math.cos(angulo) + vy * Math.sin(angulo); //Calcula o valor vetorial rotacional no eixo X
        double vyRot = -vx * Math.sin(angulo) + vy * Math.cos(angulo); //Calcula o valor vetorial rotacional no eixo Y

        //Traduz as contas para o motores
        double m1 = vxRot + omega + Proporcional; //Alterações apenas de movimentos laterais e giro
        //Conta entre parenteses = Sen e Cos de 60
        double m2 = -0.5 * vxRot + (Math.sqrt(3) / 2.0) * vyRot + omega + Proporcional;
        double m3 = -0.5 * vxRot - (Math.sqrt(3) / 2.0) * vyRot + omega + Proporcional;

        //Define o máximo de velocidade dos motores como 1.0

        double maxMotor = Math.max(Math.abs(m1), Math.max(Math.abs(m2), Math.abs(m3)));
        if (maxMotor > 0.8) {
            m1 /= maxMotor;
            m2 /= maxMotor;
            m3 /= maxMotor;
        }
        double max_velocity;
        max_velocity = 1600;
        m1 *= max_velocity;
        m2 *= max_velocity;
        m3 *= max_velocity;

        /*
        motor1.setVelocity(m1);
        motor2.setVelocity(m2);
        motor3.setVelocity(m3);
         */

    }
    public void andar(int AlvoX, int Alvoy, double potY, double potX, double giro){
        odometria.initialize();
        odometria.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0));
while(opModeIsActive()){
        Pose2D pose2D = odometria.getPosition();
        double podY;
        double podX;
        podY = (int) pose2D.getY(DistanceUnit.MM);
        double giroscopio;
        double Y = Integer.compare(Alvoy,(int) podY);
        Y *= potY;
        podX = (int) pose2D.getX(DistanceUnit.MM);
        double X = Integer.compare(AlvoX,(int) podX);
        X *= potX;
        giroscopio =
                pose2D.getHeading(AngleUnit.RADIANS);
        calculos(X, Y, giroscopio, giro, giroscopio);
}
//fazer uma função parecida com o integer.compare so que com números double.
    }
    public void shoot(long tempoSeg){
//        intake.setPower(0.8);
//        shooter.setPower(0.7);
        sleep((tempoSeg * 1000));
//        intake.setPower(0);
//        shooter.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
