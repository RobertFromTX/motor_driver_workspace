# Motor Driver Projects Workspace

Welcome to my motor drivers workspace! This is a collection of my projects relating to the control of a brushed DC motor (the 130 hobbyist motors you can buy on 
[Amazon](https://www.amazon.com/EUDAX-Strong-Magnetic-Electric-Control/dp/B078MR3H45/ref=sr_1_6?dib=eyJ2IjoiMSJ9.Mi12UooWtkp3PWu2Z6tXafIic6M0ilyJrYsl1jk9aac-xZxBIBXfPDNZZhVbgp8u-3IMtRl0HF8KxVue0eU04tiCkusHrIS3g0qXWnSmOvxhppvvnclhw-HVnOTGHb-ilcCqeCso6MpGV5Z4JQBvUn1u8E7UYdM6qoMjjDWm7a2lRa5se4qkOD_7oMvxKQtrgYltrTegXI0lGJZI4lkUEodLs8D83AW1akrHUWJ6jtWwhv7hgMwpcgbftKdDYS9Pcs-4tzcQW-fxjXw2MgJIZm8Cc2-TARSGuAwsssESM_0.WBXoqtZVDjbgOsXRFmrOjTV-M3FtOEI8N8Tqyi750Vc&dib_tag=se&keywords=130+motor&qid=1747979369&sr=8-6)). All the code is written for the
[STM32 NUCLEO-F303K8 development board](https://estore.st.com/en/products/evaluation-tools/product-evaluation-tools/mcu-mpu-eval-tools/stm32-mcu-mpu-eval-tools/stm32-nucleo-boards/nucleo-f303k8.html).

## Project Descriptions
### Motor Encoder Programmer
Project made due to the need of programming the 
[AS5600](https://www.amazon.com/Alinan-Magnetic-Precision-Induction-Measurement/dp/B09QYC916Q/ref=sr_1_4?crid=1SV0JS444KX5K&dib=eyJ2IjoiMSJ9.YZizyEOTJGdbWMdPyBu7o8_cRRzTCm89DuGdPAFj5Ki68YsJLYF_Hpv_lazIelp2fKh9iyoTalWdotBEC8uFNMCL_RoMzuClUFsxKW2oNV9L4Qzm9mGPkHGN_SCfMfPP_3I_tEYE9PBRf3rOb7ZHZsk7C5xpygdguLrTKK-QwHpA-zZ8IN97YgFH73l6gyWbUty9vB7XpHKspo5bnsJKcUl5iWjP4svVtcf7bC2noSo.JE2igM9cCutbMAtuO9a89v-P-EsS8AmG-5HIm1QTDoQ&dib_tag=se&keywords=as5600+motor+encoder&qid=1747979707&sprefix=as5600+motor+encode%2Caps%2C179&sr=8-4) 
motor encoder chips. I2C is used to communicate with the registers on the AS5600 chips, and all the code is based off of the 
[datasheet](https://files.seeedstudio.com/wiki/Grove-12-bit-Magnetic-Rotary-Position-Sensor-AS5600/res/Magnetic%20Rotary%20Position%20Sensor%20AS5600%20Datasheet.pdf). 
The project can be used to read all of the registers on the chip, but I mainly use it to enable the the analog output on the chip between 0 
and 3.3V depending on the angle the encoder records. 

### Motor Position Reader
This module reads the positions of two motors, and stores their angles in two arrays, motor1_pos and motor2_pos. These arrays store the last 3 angles recorded so that digital PID controllers can be implemented for them. A timer (TIM2) generates an trigger output (TRGO) signal every second, which gets the ADC on the stm32 to sample and convert the analog signal from the motor encoder output (which are connected to pins A0 and A1) into a digital value. The ADC works with the DMA so that fewer clock cycles are spent retrieving the position.
