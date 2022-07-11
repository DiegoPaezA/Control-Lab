# Low-cost control laboratory kit - Control(LAB)

<p align="justify">Control(Lab) is developed following the [TCLab](https://is.gd/1HAwDJ) idea: a printed circuit board (PCB) that allows connection with the Arduino processing unit. The board has three transistors that work as heaters and three temperature sensors to follow the temperature variations in each heater. The operation of the plant is based on the control of the electrical current that flows through the transistors using the pulse width modulation (PWM) generated from the system's processing unit. The control action generates a variation in the temperature of the plant heaters limited by software to 100°C. This variation is measured by the temperature sensors and registered in the Arduino, thus establishing a closed control loop.</p>

<p align="justify">To develop an affordable and easy-to-build control lab kit, low-cost and easily accessible elements were selected. The LM35 temperature sensor replaced the TMP36 temperature sensor used in the TCLAB reference plant; the new sensor fulfills the necessary operating and functionality ranges. The sensor is linear and generates a voltage output proportional to the temperature variation (T° C = 10mV). Also, it has calibrated precision of 1°C, its measurement range is -55 ° C to 150 ° C, and it has a guaranteed precision of 0.5°C at 25°C ambient temperature. Finally, the configuration of three TIP31C NPN transistors (BJTs) as heaters allows implementing controllers of the single-input-single-output (SISO) type or multiple-input multiple-output (MIMO) with a low-cost and accessible device. Figure 1 shows the Control kit (Lab).</p>

<p align="justify">This repository presents different examples focused on testing the operation of the different ESP32 functionalities using FreeRTOS. The examples have been validated using an ESP32 DEVKIT DOIT board and the documentation of each of the examples is being done to address the theoretical concepts of each of the topics addressed.</p>

![Control Laboratory Kit - Control(Lab)](control-lab-pcb\control-lab_2.png)

## References

Detailed description of the Control(Lab) can be find on the on the Article: [Comparative Study of PID, DMC, and Fuzzy PD+I Controllers in a Control Laboratory Kit]()


The documentation will be published on my personal web page <https://www.mrdpaez.com>
