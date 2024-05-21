#include <stdio.h>
#include <webots/supervisor.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <math.h>

#define TIME_STEP 256
#define QtddSensoresProx 8
#define QtddLeds 10
  
int main(int argc, char **argv) {
    int i = 0;
    char texto[256];
    double LeituraSensorProx[QtddSensoresProx];
    double AceleradorDireito = 1.0, AceleradorEsquerdo = 1.0;
    for (i = 0; i < 256; i++) texto[i] = '0';

    wb_robot_init();

    WbDeviceTag MotorEsquerdo, MotorDireito;

    MotorEsquerdo = wb_robot_get_device("left wheel motor");
    MotorDireito = wb_robot_get_device("right wheel motor");

    wb_motor_set_position(MotorEsquerdo, INFINITY);
    wb_motor_set_position(MotorDireito, INFINITY);

    wb_motor_set_velocity(MotorEsquerdo, 0);
    wb_motor_set_velocity(MotorDireito, 0);

    WbDeviceTag SensorProx[8];
    SensorProx[0] = wb_robot_get_device("ps0");
    SensorProx[1] = wb_robot_get_device("ps1");
    SensorProx[2] = wb_robot_get_device("ps2");
    SensorProx[3] = wb_robot_get_device("ps3");
    SensorProx[4] = wb_robot_get_device("ps4");
    SensorProx[5] = wb_robot_get_device("ps5");
    SensorProx[6] = wb_robot_get_device("ps6");
    SensorProx[7] = wb_robot_get_device("ps7");

    wb_distance_sensor_enable(SensorProx[0],TIME_STEP);

    wb_distance_sensor_enable(SensorProx[1],TIME_STEP);

    wb_distance_sensor_enable(SensorProx[2],TIME_STEP);

    wb_distance_sensor_enable(SensorProx[3],TIME_STEP);

    wb_distance_sensor_enable(SensorProx[4],TIME_STEP);

    wb_distance_sensor_enable(SensorProx[5],TIME_STEP);

    wb_distance_sensor_enable(SensorProx[6],TIME_STEP);

    wb_distance_sensor_enable(SensorProx[7],TIME_STEP);

    WbDeviceTag Leds[10];
    Leds[0] = wb_robot_get_device("led0");
    Leds[1] = wb_robot_get_device("led1");
    Leds[2] = wb_robot_get_device("led2");
    Leds[3] = wb_robot_get_device("led3");
    Leds[4] = wb_robot_get_device("led4");
    Leds[5] = wb_robot_get_device("led5");
    Leds[6] = wb_robot_get_device("led6");
    Leds[7] = wb_robot_get_device("led7");
    Leds[8] = wb_robot_get_device("led8");
    Leds[9] = wb_robot_get_device("led9");

    for (i = 0; i < 10; i++) {
        wb_led_set(Leds[i], 0);
    }
    
    WbNodeRef constCaixa = wb_supervisor_node_get_from_def("CAIXA_LEVE");
    const double *posicao_caixa_inicial = wb_supervisor_node_get_position(constCaixa);
    double vector[3] = {posicao_caixa_inicial[0], posicao_caixa_inicial[1], posicao_caixa_inicial[2]};
    
    while (wb_robot_step(TIME_STEP) != -1) {
        for (i = 0; i < 256; i++) texto[i] = 0;

        for (i = 0; i < QtddSensoresProx; i++) {
            LeituraSensorProx[i] = wb_distance_sensor_get_value(SensorProx[i]) - 60;
            sprintf(texto, "%s|%d: %5.2f  ", texto, i, LeituraSensorProx[i]);
        }

        printf("%s\n", texto);

        const double *posicao_caixa_atual = wb_supervisor_node_get_position(constCaixa);

        if (fabs(vector[0] - posicao_caixa_atual[0]) > 0.001 ||
            fabs(vector[1] - posicao_caixa_atual[1]) > 0.001 ||
            fabs(vector[2] - posicao_caixa_atual[2]) > 0.001) {
            
            AceleradorDireito = 0;
            AceleradorEsquerdo = 0;
            wb_motor_set_velocity(MotorEsquerdo, 0);
            wb_motor_set_velocity(MotorDireito, 0);

            for (i = 0; i < QtddLeds; i++) {
                wb_led_set(Leds[i], 1);
            }
            printf("ACHOU  A CAIXA\n");
            break;
        }

        if (LeituraSensorProx[0] > 10) {
            AceleradorDireito = -1;
            AceleradorEsquerdo = 1;
        } else {
            AceleradorDireito = 1.0;
            AceleradorEsquerdo = 1.0;
        }

        wb_motor_set_velocity(MotorEsquerdo, 6.28 * AceleradorEsquerdo);
        wb_motor_set_velocity(MotorDireito, 6.28 * AceleradorDireito);
    }

    wb_robot_cleanup();

    return 0;
}
