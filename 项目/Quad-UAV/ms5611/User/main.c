/**
 ******************************************************************************
 * @file    main.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/02/28
 * @brief   Ö÷º¯Êý
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "bsp_systick.h"
#include "bsp_usart.h"
#include "bsp_spi.h"
#include "bsp_mpu6000.h"
#include "bsp_tim.h"
#include "bsp_exti.h"
#include "filter.h"
#include "bsp_ms5611.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

static int ms5611_crc4(uint16_t *n_prom)
{
    int16_t cnt;
	uint16_t n_rem;
	uint16_t crc_read;
	uint8_t n_bit;

	n_rem = 0x00;

	/* save the read crc */
	crc_read = n_prom[7];

	/* remove CRC byte */
	n_prom[7] = (0xFF00 & (n_prom[7]));

	for (cnt = 0; cnt < 16; cnt++) {
		/* uneven bytes */
		if (cnt & 1) {
			n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);

		} else {
			n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);
		}

		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & 0x8000) {
				n_rem = (n_rem << 1) ^ 0x3000;

			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	/* final 4 bit remainder is CRC value */
	n_rem = (0x000F & (n_rem >> 12));
	n_prom[7] = crc_read;

	/* return true if CRCs match */
	return (0x000F & crc_read) == (n_rem ^ 0x00);
}

unsigned char crc;
int main(void)
{
    ms5611param_t param_s;
    SystickInit(180);
    USARTInit();
    spi_init();
    mpu6000_init();
    timer_tbinit();
    exti_init();
    ms5611_init(&param_s);
		uint16_t n_prom[8];
	for(uint8_t i=0;i<8;i++)
	{
		n_prom[i]=param_s.prom_data.prom[i];
	}
		
		crc = ms5611_crc4(n_prom);
    while (1)
    {
        ms5611_GetData(&param_s);
        ms5611_CalculateAltitude(&param_s);
        //printf("%f,", param_s.temp);
        
        delay_ms(10);
    }
}
