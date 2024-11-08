	/******************************************************************************
	* @file           : main.c
	* @brief          : Main program body
	* (c) EE2028 Teaching Team
	******************************************************************************/


	/* Includes ------------------------------------------------------------------*/


	//Consolidated code for assignment
	/* Includes ------------------------------------------------------------------*/
	#include "main.h"
	#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
	#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"
	#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.h"
	#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h"
	#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"
	#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h"
	#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.h"

	#include "stm32l4xx_hal_conf.h"
	#include <math.h>
	#include "stdio.h"
	#define IMU_FREQ 9
	#define PULSE_DELAY 30000
	#define LINE_DELAY 5000     // Delay between ASCII art lines
	static void MX_GPIO_Init(void);
	int last_press_time_0;  // Time of the last button press
	int last_press_time_1;  // Time of the last button press
	int last_press_time_2;  // Time of the last button press

	int mode = 0;  // Tracks the mode of the system
	float gyro_data[3];
	float temp_data;
	float pres_data;
	float humid_data;
	float accel_data;
	float mag_data;
	float mag_offset_X;
	float mag_offset_Y;
	float mag_offset_Z;
	float gyro_offset_X;
	float gyro_offset_Y;
	float gyro_offset_Z;
	float accel_offset_X;
	float accel_offset_Y;
	float accel_offset_Z;
	double ax, ay, az;
	double gx, gy, gz;
	int blink_freq = 2; //Hz
	int ghost_capture_count = 0;


	///UART
	static void UART1_Init(void);
	UART_HandleTypeDef huart1;

	extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)

	float GET_TEMP(){
		float temp_data;
		temp_data = BSP_TSENSOR_ReadTemp();			// read temperature sensor
		return temp_data;
	}

	float GET_HUMID(){
		float humidity_data;
		humidity_data = BSP_HSENSOR_ReadHumidity();
		return humidity_data;
	}

	float GET_PRES(){
		float pressure_data;
		pressure_data = BSP_PSENSOR_ReadPressure();
		return pressure_data;
	}

	float GET_MAG(){
		float magneto_data[3];
		int16_t magneto_data_i16[3] = { 0 };
		BSP_MAGNETO_GetXYZ(magneto_data_i16);
		magneto_data[0] = (float)magneto_data_i16[0] - mag_offset_X;
		magneto_data[1] = (float)magneto_data_i16[1] - mag_offset_Y;
		magneto_data[2] = (float)magneto_data_i16[2] - mag_offset_Z;
		float mag_result = sqrt(magneto_data[0]*magneto_data[0]+magneto_data[1]*magneto_data[1]+magneto_data[2]*magneto_data[2]);
		return mag_result;
	}
	void GET_MAG_DATA(float* mag_x, float* mag_y, float* mag_z, float* mag_magnitude) {
	    int16_t magneto_data_i16[3] = { 0 };
	    BSP_MAGNETO_GetXYZ(magneto_data_i16);

	    // Apply the offsets to get calibrated magnetic data
	    *mag_x = (float)magneto_data_i16[0] - mag_offset_X;
	    *mag_y = (float)magneto_data_i16[1] - mag_offset_Y;
	    *mag_z = (float)magneto_data_i16[2] - mag_offset_Z;

	    // Calculate the magnitude of the magnetic field (magnitude of the 3D vector)
	    *mag_magnitude = sqrt((*mag_x) * (*mag_x) + (*mag_y) * (*mag_y) + (*mag_z) * (*mag_z));
	}


	void MAG_CALIBRATE(){
		char message1[] = "Calibrating...\n";       // Fixed message
		char message_print1[32];        // UART transmit buffer. See the comment in the line above.
		sprintf(message_print1, "%s\r", message1);
		HAL_UART_Transmit(&huart1, (uint8_t*)message_print1, strlen(message_print1),0xFFFF); //Sending in normal mode

		int16_t magneto_data_i16[3] = { 0 };
		BSP_MAGNETO_GetXYZ(magneto_data_i16);
		mag_offset_X = (float)magneto_data_i16[0];
		mag_offset_Y = (float)magneto_data_i16[1];
		mag_offset_Z = (float)magneto_data_i16[2];
		char message_print2[128];        // UART transmit buffer. See the comment in the line above.
		sprintf(message_print2, "Before Calibration: Mag X : %.2f; Mag Y : %.2f; Mag Z : %.2f\r\n", mag_offset_X, mag_offset_Y, mag_offset_Z);
		HAL_UART_Transmit(&huart1, (uint8_t*)message_print2, strlen(message_print2),0xFFFF); //Sending in normal mode

		float magneto_data[3];
		BSP_MAGNETO_GetXYZ(magneto_data_i16);
		magneto_data[0] = (float)magneto_data_i16[0] - mag_offset_X;
		magneto_data[1] = (float)magneto_data_i16[1] - mag_offset_Y;
		magneto_data[2] = (float)magneto_data_i16[2] - mag_offset_Z;
		char message_print3[128];        // UART transmit buffer. See the comment in the line above.
		sprintf(message_print3, "After Calibration: Mag X : %.2f; Mag Y : %.2f; Mag Z : %.2f\r\n", magneto_data[0], magneto_data[1], magneto_data[2]);
		HAL_UART_Transmit(&huart1, (uint8_t*)message_print3, strlen(message_print3),0xFFFF); //Sending in normal mode
	}
	void detect_ghost_direction(float mag_x, float mag_y, float mag_z) {
	    char direction[64] = "";           // To hold horizontal direction
	    char vertical_direction[32] = "";  // To hold vertical direction

	    // Thresholds for detecting direction
	    const float THRESHOLD_X = 300.0;
	    const float THRESHOLD_Y = 400.0;
	    const float THRESHOLD_Z = 1000.0;

	    // Determine horizontal direction (East/West/North/South)
	    if (fabs(mag_x) > THRESHOLD_X) {
	        if (mag_x > 0) {
	            strcat(direction, "East");
	        } else {
	            strcat(direction, "West");
	        }
	    }

	    if (fabs(mag_y) > THRESHOLD_Y) {
	        if (mag_y > 0) {
	            if (strlen(direction) > 0) {
	                strcat(direction, " North");
	            } else {
	                strcat(direction, "North");
	            }
	        } else {
	            if (strlen(direction) > 0) {
	                strcat(direction, " South");
	            } else {
	                strcat(direction, "South");
	            }
	        }
	    }

	    // Determine vertical direction (Above/Below)
	    if (fabs(mag_z) > THRESHOLD_Z) {
	        if (mag_z > 0) {
	            strcpy(vertical_direction, "Below");
	        } else {
	            strcpy(vertical_direction, "Above");
	        }
	    }

	    // Create the message conditionally based on whether direction or vertical_direction is not empty
	    char message[128] = "";
	    if (strlen(direction) > 0 && strlen(vertical_direction) > 0) {
	        // Both horizontal and vertical directions detected
	        sprintf(message, "Horizontal: %s Vertical: %s\r\n", direction, vertical_direction);
	    } else if (strlen(direction) > 0) {
	        // Only horizontal direction detected
	        sprintf(message, "Horizontal: %s\r\n", direction);
	    } else if (strlen(vertical_direction) > 0) {
	        // Only vertical direction detected
	        sprintf(message, "Vertical: %s\r\n", vertical_direction);
	    }

	    // If there's a valid message, print and transmit it
	    if (strlen(message) > 0) {
	        HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
	    }
	}




	void IMU_CALIBRATE(){
		BSP_GYRO_GetXYZ(gyro_data);
		gyro_data[0] = gyro_data[0];
		gyro_data[1] = gyro_data[1];
		gyro_data[2] = gyro_data[2];
		gyro_offset_X = gyro_data[0];
		gyro_offset_Y = gyro_data[1];
		gyro_offset_Z = gyro_data[2];
		int16_t accel_data_i16[3] = { 0 };
		BSP_ACCELERO_AccGetXYZ(accel_data_i16);		// read accelerometer
		accel_offset_X = (float)accel_data_i16[0] * (9.8/1000.0f);
		accel_offset_Y = (float)accel_data_i16[1] * (9.8/1000.0f);
		accel_offset_Z = (float)accel_data_i16[2] * (9.8/1000.0f);
	}

	float GET_ACC(){
		float accel_data[3];
		int16_t accel_data_i16[3] = { 0 };			// array to store the x, y and z readings.
		BSP_ACCELERO_AccGetXYZ(accel_data_i16);		// read accelerometer
		accel_data[2] = (float)accel_data_i16[2] * (9.8/1000.0f);
		return accel_data[2];
	}

	void GhostBustingModeStartupBeep() {
	    // Start time of the beep sequence
	    int start_time = HAL_GetTick();
	    int beep_duration = 100; // Duration for each beep in milliseconds

	    while (HAL_GetTick() - start_time < beep_duration * 2) {
	        int current_time = HAL_GetTick();

	        // Toggle at intervals to create the fast beep effect
	        if ((current_time - start_time) % beep_duration < beep_duration / 2) {
	            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);  // Turn on buzzer
	        } else {
	            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); // Turn off buzzer
	        }
	    }
	}

	const char* abort_ascii[] = {
	        "               AAA                    BBBBBBBBBBBBBBBBB             OOOOOOOOO          RRRRRRRRRRRRRRRRR        TTTTTTTTTTTTTTTTTTTTTTT",
	        "              A:::A                   B::::::::::::::::B          OO:::::::::OO        R::::::::::::::::R       T:::::::::::::::::::::T",
	        "             A:::::A                  B::::::BBBBBB:::::B       OO:::::::::::::OO      R::::::RRRRRR:::::R      T:::::::::::::::::::::T",
	        "            A:::::::A                 BB:::::B     B:::::B     O:::::::OOO:::::::O     RR:::::R     R:::::R     T:::::TT:::::::TT:::::T",
	        "           A:::::::::A                  B::::B     B:::::B     O::::::O   O::::::O       R::::R     R:::::R     TTTTTT  T:::::T  TTTTTT",
	        "          A:::::A:::::A                 B::::B     B:::::B     O:::::O     O:::::O       R::::R     R:::::R             T:::::T        ",
	        "         A:::::A A:::::A                B::::BBBBBB:::::B      O:::::O     O:::::O       R::::RRRRRR:::::R              T:::::T        ",
	        "        A:::::A   A:::::A               B:::::::::::::BB       O:::::O     O:::::O       R:::::::::::::RR               T:::::T        ",
	        "       A:::::A     A:::::A              B::::BBBBBB:::::B      O:::::O     O:::::O       R::::RRRRRR:::::R              T:::::T        ",
	        "      A:::::AAAAAAAAA:::::A             B::::B     B:::::B     O:::::O     O:::::O       R::::R     R:::::R             T:::::T        ",
	        "     A:::::::::::::::::::::A            B::::B     B:::::B     O:::::O     O:::::O       R::::R     R:::::R             T:::::T        ",
	        "    A:::::AAAAAAAAAAAAA:::::A           B::::B     B:::::B     O::::::O   O::::::O       R::::R     R:::::R             T:::::T        ",
	        "   A:::::A             A:::::A        BB:::::BBBBBB::::::B     O:::::::OOO:::::::O     RR:::::R     R:::::R           TT:::::::TT      ",
	        "  A:::::A               A:::::A       B:::::::::::::::::B       OO:::::::::::::OO      R::::::R     R:::::R           T:::::::::T      ",
	        " A:::::A                 A:::::A      B::::::::::::::::B          OO:::::::::OO        R::::::R     R:::::R           T:::::::::T      ",
	        "AAAAAAA                   AAAAAAA     BBBBBBBBBBBBBBBBB             OOOOOOOOO          RRRRRRRR     RRRRRRR           TTTTTTTTTTT      "
	    };


	const char* countdown_ascii_3[] = {
	    "                                  333333333333333   ",
	    "                                 3:::::::::::::::33 ",
	    "                                 3::::::33333::::::3",
	    "                                 3333333     3:::::3",
	    "                                             3:::::3",
	    "                                             3:::::3",
	    "                                     33333333:::::3 ",
	    "                                     3:::::::::::3  ",
	    "                                     33333333:::::3 ",
	    "                                             3:::::3",
	    "                                             3:::::3",
	    "                                             3:::::3",
	    "                                 3333333     3:::::3",
	    "                                 3::::::33333::::::3",
	    "                                 3:::::::::::::::33 ",
	    "                                  333333333333333   "
	};

	const char* countdown_ascii_2[] = {
	    "                                 222222222222222    ",
	    "                                2:::::::::::::::22  ",
	    "                                2::::::222222:::::2 ",
	    "                                2222222     2:::::2 ",
	    "                                            2:::::2 ",
	    "                                            2:::::2 ",
	    "                                         2222::::2  ",
	    "                                    22222::::::22   ",
	    "                                 22::::::::222      ",
	    "                                2:::::22222         ",
	    "                                2:::::2             ",
	    "                                2:::::2             ",
	    "                                2:::::2       222222",
	    "                                2::::::2222222:::::2",
	    "                                2::::::::::::::::::2",
	    "                                22222222222222222222"
	};

	const char* countdown_ascii_1[] = {
	    "                                      1111111       ",
	    "                                     1::::::1       ",
	    "                                    1:::::::1       ",
	    "                                    111:::::1       ",
	    "                                       1::::1       ",
	    "                                       1::::1       ",
	    "                                       1::::1       ",
	    "                                       1::::l       ",
	    "                                       1::::l       ",
	    "                                       1::::l       ",
	    "                                       1::::l       ",
	    "                                       1::::l       ",
	    "                                     111::::::111   ",
	    "                                     1::::::::::1   ",
	    "                                     1::::::::::1   ",
	    "                                     111111111111   "
	};

	const char* explosion_ascii[] = {
	    "                               ________________                    ",
	    "                          ____/ (  (    )   )  \\___                ",
	    "                         /( (  (  )   _    ))  )   )\\              ",
	    "                       ((     (   )(    )  )   (   )  )            ",
	    "                     ((/  ( _(   )   (   _) ) (  () )  )           ",
	    "                    ( (  ( (_)   ((    (   )  .((_ ) .  )_         ",
	    "                   ( (  )    (      (  )    )   ) . ) (   )        ",
	    "                  (  (   (  (   ) (  _  ( _) ).  ) . ) ) ( )       ",
	    "                  ( (  ( (  (  )     (_  )  ) )  _)   ) _( ( )     ",
	    "                  ((  (   )(    (     _    )   _) _(_ (  (_ )      ",
	    "                   (_((__(_(__(( ( ( |  ) ) ) )_))__))_)___)       ",
	    "                   ((__)        \\\\||lll|l||///          \\_))       ",
	    "                            (   /(/ (  )  ) )\\   )                 ",
	    "                          (    ( ( ( | | ) ) )\\   )               ",
	    "                           (   /(| / ( )) ) ) )) )                 ",
	    "                         (     ( ((((_(|)_)))))     )              ",
	    "                          (      \\(|(|)|/     )                ",
	    "                        (        |(()||        )             ",
	    "                          (     //|/l|)|\\\\ \\     )              ",
	    "                        (/ / //  /|//||||\\\\  \\ \\  \\ _)            ",
	    "        !!! BOOM !!!                SYSTEM FAILURE                "
	};

	void Self_Destruct_Sequence(void) {
	    char message[] = "GHOST IS HERE! ABORT ABORT ABORT!\r\n";
		HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);

	    // Countdown from 3 to 1 using ASCII art
	    const char** countdown_ascii[] = {countdown_ascii_3, countdown_ascii_2, countdown_ascii_1};

	    // Display the "ABORT" ASCII art
	        for (int line = 0; line < sizeof(abort_ascii) / sizeof(abort_ascii[0]); line++) {
	            HAL_UART_Transmit(&huart1, (uint8_t*)abort_ascii[line], strlen(abort_ascii[line]), HAL_MAX_DELAY);
	            HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
	        }

	        // Add spacing after "ABORT" ASCII art
	        for (int i = 0; i < 5; i++) {
	            HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
	        }

	    for (int i = 0; i < 3; i++) {
	        for (int line = 0; line < 16; line++) {
	            HAL_UART_Transmit(&huart1, (uint8_t*)countdown_ascii[i][line], strlen(countdown_ascii[i][line]), HAL_MAX_DELAY);
	            HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
	        }

	        // Add extra newlines for spacing between numbers
	        for (int gap = 0; gap < 3; gap++) {
	            HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
	        }

	        // Rapid buzzer pulses for each countdown number
	        for (int j = 0; j < 10; j++) {
	            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3); // Toggle buzzer
	            for (volatile int k = 0; k < PULSE_DELAY; k++);  // Delay without HAL_Delay
	        }
	    }

	    // Clear screen for explosion art
	    for (int i = 0; i < 10; i++) {
	        HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
	    }

	    // Display ASCII explosion art
	    for (int line = 0; line < sizeof(explosion_ascii) / sizeof(explosion_ascii[0]); line++) {
	        HAL_UART_Transmit(&huart1, (uint8_t*)explosion_ascii[line], strlen(explosion_ascii[line]), HAL_MAX_DELAY);
	        HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
	    }

	    // Final message before shutting down UART
	    char final_message[] = "SELF-DESTRUCT INITIATED! COMMUNICATION TERMINATED.\n";
	    if (HAL_UART_Transmit(&huart1, (uint8_t*)final_message, strlen(final_message), HAL_MAX_DELAY) != HAL_OK) {
	    }
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

	    // Disable UART to simulate communication cut-off
	    HAL_UART_DeInit(&huart1);


	    // Infinite loop to halt the system after self-destruct
	    while (1);
	}
	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
		if(GPIO_Pin == BUTTON_EXTI13_Pin){
			// Get the current system time (in milliseconds)
			int current_time = HAL_GetTick();
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);  // Ensure buzzer is off initially
			// Check if the second press occurred within 1 second in Normal Mode
			if (mode == 0) {
				if ((current_time - last_press_time_0) <= 1000) {
					char message1[] = "\t Entering Ghost_Busting_Mode... \r\n";       // Fixed message
					char message_print1[64];        // UART transmit buffer. See the comment in the line above.
					sprintf(message_print1, "%s", message1);
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print1, strlen(message_print1),0xFFFF); //Sending in normal mode

					MAG_CALIBRATE();
					mode = 1;
					GhostBustingModeStartupBeep();
//					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
//					BSP_LED_Off(LED2);
				}
				last_press_time_0 = HAL_GetTick();
			}
			// Check for the first press in Ghost Busting Mode
			else if (mode == 1) {
				if ((current_time - last_press_time_1) <= 1000){
					char message2[] = "\t Entering Normal_Mode...\r\n";       // Fixed message
					char message_print2[32];        // UART transmit buffer. See the comment in the line above.
					sprintf(message_print2, "%s", message2);
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print2, strlen(message_print2),0xFFFF); //Sending in normal mode

					mode = 0;
					char message_print3[32];        // UART transmit buffer. See the comment in the line above.
					sprintf(message_print3, "Total ghosts captured: %d\r\n", ghost_capture_count);
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print3, strlen(message_print3),0xFFFF); //Sending in normal mode

					ghost_capture_count = 0;
				}
				else{
					char message4[] = "Ghost Captured! \r\n";       // Fixed message
					char message_print4[32];        // UART transmit buffer. See the comment in the line above.
					sprintf(message_print4, "%s", message4);
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print4, strlen(message_print4),0xFFFF); //Sending in normal mode
					mode = 2;
					IMU_CALIBRATE();
					BSP_LED_Off(LED2);
					ghost_capture_count++;
				}
				last_press_time_1 = HAL_GetTick();
			}
			else if (mode == 2){
				if ((current_time - last_press_time_1) <= 1000){
					char message5[] = "\t Entering Normal_Mode...\r\n";       // Fixed message
					char message_print5[32];        // UART transmit buffer. See the comment in the line above.
					sprintf(message_print5, "%s", message5);
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print5, strlen(message_print5),0xFFFF); //Sending in normal mode
					mode = 0;
					char message_print6[32];        // UART transmit buffer. See the comment in the line above.
					sprintf(message_print6, "Total ghosts captured: %d\r\n", ghost_capture_count);
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print6, strlen(message_print6),0xFFFF); //Sending in normal mode
					ghost_capture_count = 0;
				}
				else{
					char message7[] = "\t Entering Ghost_Busting_Mode... \r\n";       // Fixed message
					char message_print7[32];        // UART transmit buffer. See the comment in the line above.
					sprintf(message_print7, "%s", message7);
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print7, strlen(message_print7),0xFFFF); //Sending in normal mode
					mode = 1;
				}
				last_press_time_1 = HAL_GetTick();

			}
			// Update the last press time

		}
	}

	//  UART REFERENCE
	//	char message1[] = "Entering Normal_Mode...\r\n";       // Fixed message
	//	char message_print1[32];        // UART transmit buffer. See the comment in the line above.
	//	sprintf(message_print1, "%s", message1);
	//	HAL_UART_Transmit(&huart1, (uint8_t*)message_print1, strlen(message_print1),0xFFFF); //Sending in normal mode

	int main(void)
	{
		initialise_monitor_handles(); // for semi-hosting support (printf)
		/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
		HAL_Init();
		/* UART initialization  */
		UART1_Init();
		//HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_3);

		MX_GPIO_Init();

		/* Peripheral initializations using BSP functions */
		BSP_ACCELERO_Init();
		BSP_TSENSOR_Init();
		BSP_LED_Init(LED2);
		BSP_HSENSOR_Init();
		BSP_MAGNETO_Init();
		BSP_GYRO_Init();
		BSP_PSENSOR_Init();

		/* Get Start Tick*/
		int tickstart;
		tickstart = HAL_GetTick();
		int tickstart1;
		tickstart1 = HAL_GetTick();
		//Startup message
		char message1[] = "\t Entering Normal_Mode...\r\n";       // Fixed message
		char message_print1[32];        // UART transmit buffer. See the comment in the line above.
		sprintf(message_print1, "%s", message1);
		HAL_UART_Transmit(&huart1, (uint8_t*)message_print1, strlen(message_print1),0xFFFF); //Sending in normal mode
		HAL_GPIO_EXTI_Callback(GPIOC);

		while(1){
			while (mode == 0){
				int t1 = HAL_GetTick();

				if((t1 - tickstart)>= 1000){
					BSP_GYRO_GetXYZ(gyro_data);
					gyro_data[0] = gyro_data[0]/1000;
					gyro_data[1] = gyro_data[1]/1000;
					gyro_data[2] = gyro_data[2]/1000;
					temp_data = GET_TEMP();
					pres_data = GET_PRES();
					humid_data = GET_HUMID();
					accel_data = GET_ACC();
					mag_data = GET_MAG();
					//UART MSG
					char message_print2[128];        // UART transmit buffer. See the comment in the line above.
					sprintf(message_print2, "T: %.2f °C, P: %.2f hPa, H: %.2f %%rH, A(z-axis): %.2f m/s^2, Gx : %.2f dps; Gy : %.2f dps; Gz : %.2f dps, M: %.2f gauss\r\n", temp_data, pres_data, humid_data, accel_data, gyro_data[0], gyro_data[1], gyro_data[2], mag_data);
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print2, strlen(message_print2),0xFFFF); //Sending in normal mode
					tickstart = HAL_GetTick();
			//reference use of hal uart transmit
			//               char message1[] = "Welcome to EE2028 !!!\r\n";       // Fixed message
			//               // Be careful about the buffer size used. Here, we assume that seconds_count does not exceed 6 decimal digits
			//               char message_print[32];        // UART transmit buffer. See the comment in the line above.
			//               sprintf(message_print, "%d: %s", seconds_count, message1);
			//               HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF); //Sending in normal mode
				}
			}

			while (mode == 1) {
			    int t2 = HAL_GetTick();

			    // Check infrared sensor state for imminent danger
			    GPIO_PinState sensor_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
			    if (sensor_state == GPIO_PIN_RESET) {
			        char infrared_message[] = "Infrared sensor detected a close object! Initiating self-destruct!\r\n";
			        HAL_UART_Transmit(&huart1, (uint8_t*)infrared_message, strlen(infrared_message), 0xFFFF);
			        Self_Destruct_Sequence(); // Trigger self-destruct if the infrared sensor detects an object
			    }

			    // Proceed with the regular ghost activity checks
			    if ((t2 - tickstart) >= 1000) {
			        float temp_data_check = GET_TEMP();
			        if (temp_data_check < temp_data - 2) {
			            char message_print3[128];
			            sprintf(message_print3, "Phantasm manifestation detected! Sudden temperature drop detected! Latest reading: %.2f °C. Possible ghost nearby!\r\n", temp_data_check);
			            HAL_UART_Transmit(&huart1, (uint8_t*)message_print3, strlen(message_print3), 0xFFFF);
						// Call to detect ghost direction based on magnetometer readings
    					float mag_x, mag_y, mag_z, mag_magnitude;
    					GET_MAG_DATA(&mag_x, &mag_y, &mag_z, &mag_magnitude);
    					detect_ghost_direction(mag_x, mag_y, mag_z);
			        }
			        float humid_data_check = GET_HUMID();
			        if ((humid_data_check < humid_data - 5) || (humid_data_check > humid_data + 5)) {
			            char message_print4[128];
			            sprintf(message_print4, "Phantasm manifestation detected! Atmospheric change detected! Latest reading: %.2f %%rH. Possible ghostly presence!\r\n", humid_data_check);
			            HAL_UART_Transmit(&huart1, (uint8_t*)message_print4, strlen(message_print4), 0xFFFF);
						float mag_x, mag_y, mag_z, mag_magnitude;
    					GET_MAG_DATA(&mag_x, &mag_y, &mag_z, &mag_magnitude);
    					detect_ghost_direction(mag_x, mag_y, mag_z);
			        }
			        float pres_data_check = GET_PRES();
			        if ((pres_data_check < pres_data - 5) || (pres_data_check > pres_data + 5)) {
			            char message_print5[128];
			            sprintf(message_print5, "Spectre activity detected! Pressure anomaly detected! Latest reading: %.2f hPa. Ghost activity suspected.\r\n", pres_data_check);
			            HAL_UART_Transmit(&huart1, (uint8_t*)message_print5, strlen(message_print5), 0xFFFF);
						float mag_x, mag_y, mag_z, mag_magnitude;
    					GET_MAG_DATA(&mag_x, &mag_y, &mag_z, &mag_magnitude);
    					detect_ghost_direction(mag_x, mag_y, mag_z);
			        }
			        float accel_data_check = GET_ACC();
			        float gyro_data_check[3];
			        BSP_GYRO_GetXYZ(gyro_data_check);
			        gyro_data_check[0] = gyro_data_check[0] / 1000;
			        gyro_data_check[1] = gyro_data_check[1] / 1000;
			        gyro_data_check[2] = gyro_data_check[2] / 1000;
			        if ((accel_data_check < accel_data - 0.5) || (accel_data_check > accel_data + 0.5) ||
			            (gyro_data_check[0] < gyro_data[0] - 5) || (gyro_data_check[0] > gyro_data[0] + 5) ||
			            (gyro_data_check[1] < gyro_data[1] - 5) || (gyro_data_check[1] > gyro_data[1] + 5) ||
			            (gyro_data_check[2] < gyro_data[2] - 5) || (gyro_data_check[2] > gyro_data[2] + 5))
						{
			            char message_print6[256];
			            sprintf(message_print6, "Poltergeist detected! Device orientation compromised! Latest reading: A: %.2f m/s^2 / Gx : %.2f dps; Gy : %.2f dps; Gz : %.2f dps. Possible ghost interaction!\r\n", accel_data_check, gyro_data_check[0], gyro_data_check[1], gyro_data_check[2]);
			            HAL_UART_Transmit(&huart1, (uint8_t*)message_print6, strlen(message_print6), 0xFFFF);
						float mag_x, mag_y, mag_z, mag_magnitude;
    					GET_MAG_DATA(&mag_x, &mag_y, &mag_z, &mag_magnitude);
    					detect_ghost_direction(mag_x, mag_y, mag_z);
			        }

			        // Check for magnetic field anomalies
			        float mag_data_check = GET_MAG();
			                if(mag_data_check < 300){
			                  blink_freq = 2;
			                }
			                else{
			                  if(mag_data_check > 5000){
			                    blink_freq = 16;
			                  }
			                  else{
			                    blink_freq = 8;
			                  }
			                  char message7[] = "Ghost detected in! Prepare to bust!\r\n";
			                  char message_print7[64];  // UART transmit buffer for the fixed message
			                  sprintf(message_print7, "%s", message7);
			                  HAL_UART_Transmit(&huart1, (uint8_t*)message_print7, strlen(message_print7), 0xFFFF);  // Send the "Ghost detected" message

			                  // Get detailed magnetometer data to detect direction
			                  float mag_x, mag_y, mag_z, mag_magnitude;
			                  GET_MAG_DATA(&mag_x, &mag_y, &mag_z, &mag_magnitude);

			                  // Transmit the latest magnetometer readings
			                  char message_magneto[128];  // UART transmit buffer for the magnetometer readings
			                  sprintf(message_magneto, "Magnetometer readings: Mag X: %.2f, Mag Y: %.2f, Mag Z: %.2f, Magnitude: %.2f\r\n", mag_x, mag_y, mag_z, mag_magnitude);
			                  HAL_UART_Transmit(&huart1, (uint8_t*)message_magneto, strlen(message_magneto), 0xFFFF);

			                  // Detect direction based on magnetometer data
			                  detect_ghost_direction(mag_x, mag_y, mag_z);


			                }



			        // Self-destruct if ghost activity is extreme
			        if (mag_data_check > 5000 || pres_data_check > (pres_data + 5) || temp_data_check < (temp_data - 5)) {
			            char extreme_message[] = "EXTREME GHOST ACTIVITY DETECTED! INITIATING SELF-DESTRUCT!\r\n";
			            HAL_UART_Transmit(&huart1, (uint8_t*)extreme_message, strlen(extreme_message), 0xFFFF);
			            // Initiating Self-Destruct Sequence
			            Self_Destruct_Sequence();
			        }

			        tickstart = HAL_GetTick();
			    }

			    // LED and buzzer toggling
			    if ((t2 - tickstart1) >= (1000 / blink_freq)) {
			        BSP_LED_Toggle(LED2);
			        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);  // Toggle buzzer
			        tickstart1 = HAL_GetTick();
			    }
			}

			while (mode == 2){
				int t3 = HAL_GetTick();
	//
	//			if((t3 - tickstart)>= 1000){
				if((t3 - tickstart)>= 1/IMU_FREQ){
					int16_t accel_data_i16[3] = { 0 };
					float accel_data[3];
					BSP_ACCELERO_AccGetXYZ(accel_data_i16);		// read accelerometer
					accel_data[0] = ((float)accel_data_i16[0] - accel_offset_X) * (9.8/1000.0f);
					accel_data[1] = ((float)accel_data_i16[1] - accel_offset_Y) * (9.8/1000.0f);
					accel_data[2] = ((float)accel_data_i16[2] - accel_offset_X) * (9.8/1000.0f);
					BSP_GYRO_GetXYZ(gyro_data);
					gyro_data[0] = (gyro_data[0] - gyro_offset_X)/1000;
					gyro_data[1] = (gyro_data[1] - gyro_offset_Y)/1000;
					gyro_data[2] = (gyro_data[2] - gyro_offset_Z)/1000;

					// angles based on accelerometer
					ay = atan2(accel_data[0], sqrt( pow(accel_data[1], 2) + pow(accel_data[2], 2))) * 180 / M_PI;
					ax = atan2(accel_data[1], sqrt( pow(accel_data[0], 2) + pow(accel_data[2], 2))) * 180 / M_PI;

					gx = gx + gyro_data[0]/IMU_FREQ;
					gy = gy - gyro_data[1]/IMU_FREQ;
					gz = gz + gyro_data[2]/IMU_FREQ;
					// complementary filter
					// tau = DT*(A)/(1-A)
					// = 0.48sec
					gx = gx * 0.96 + ax * 0.04;
					gy = gy * 0.96 + ay * 0.04;
		//			gx = gx * 0.85 + ax * 0.15;
		//			gy = gy * 0.85 + ay * 0.15;

					//UART MSG
					char message_print[128];        // UART transmit buffer. See the comment in the line above.
					sprintf(message_print, "%.4f, %.4f, %.4f\r\n", gx, gy, gz);
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF); //Sending in normal mode
					tickstart = HAL_GetTick();
				}
			}
		}
	}

	static void MX_GPIO_Init(void)
	{
		__HAL_RCC_GPIOC_CLK_ENABLE();	// Enable AHB2 Bus for GPIOC
		__HAL_RCC_GPIOA_CLK_ENABLE(); // Enable GPIOA for buzzer

		GPIO_InitTypeDef GPIO_InitStruct = {0};

		// Configuration of BUTTON_EXTI13_Pin (GPIO-C Pin-13) as AF,
		GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
		// Configure buzzer on PA3
		GPIO_InitStruct.Pin = GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		//infrared sensor PB2:)
		GPIO_InitStruct.Pin = GPIO_PIN_2;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		// Enable NVIC EXTI line 13
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	}

	static void UART1_Init(void)
	{
			/* Pin configuration for UART. BSP_COM_Init() can do this automatically */
			__HAL_RCC_GPIOB_CLK_ENABLE();
			GPIO_InitTypeDef GPIO_InitStruct = {0};
			GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
			GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; 	//ownself discover
			GPIO_InitStruct.Pull = GPIO_NOPULL;			//ownself dicsover
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

			/* Configuring UART1 */
			huart1.Instance = USART1;
			huart1.Init.BaudRate = 115200;
			huart1.Init.WordLength = UART_WORDLENGTH_8B;
			huart1.Init.StopBits = UART_STOPBITS_1;
			huart1.Init.Parity = UART_PARITY_NONE;
			huart1.Init.Mode = UART_MODE_TX_RX;
			huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
			huart1.Init.OverSampling = UART_OVERSAMPLING_16;
			huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
			huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
			if (HAL_UART_Init(&huart1) != HAL_OK)
			{
			while(1);
			}

	}







