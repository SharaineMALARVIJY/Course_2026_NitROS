# Composants

- ESC : Tamiya ESC TBLE-04S
- batteries : T04S
- moteur prop : Mabuchimotor RS540SH Ref:BD109829
- dynamixel : AX-12A
- capteur IR : Sharp GP2Y0A41SK0F
- accéléoromètre/gyromètre : Bosh BNO055
- Fourche optique : TT Electronics OPB815L, OPB815WZ Series
- STM32 : Nucleo 432 KC
- LIDAR : SLAMTEC RPLIDAR A2M12

## dynamixel

Paramètres à configurer dans :

- teleop.launch life : set cmd_dir_node's **'baudrate'** to **1000000**
- l'EEPROM du dynamixel par le logiciel Dynamixel Wizard (branchement : dynamixel <-> U2D2 <-> PC)

        id = 1
        baudrate = 1000000 # (bolide 1 : 115200)

Remarque : si le moteur génère des parasites dans le TTL -> diminuer le baudrate