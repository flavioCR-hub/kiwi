package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class inicio /* extends OpMode */{
//
//
//    public void init(){
//
//    }
//
//    public void loop(){
//
//    }
    GoBildaPinpointDriver odometria;
    double alvoY;
    double alvoX;
    public double[] calculos(double potenciaX, double potenciaY , double IMU, double giro, final double Alvo, double X, double Y){
        odometria.initialize();
        odometria.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        Pose2D pose2D = odometria.getPosition();
        alvoY = pose2D.getY(DistanceUnit.CM);
        alvoX = pose2D.getX(DistanceUnit.CM);
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
        return new double[]{m1, m2, m3};
    }
}
