

#ifndef  __CTRL_DEBUG_H__
#define  __CTRL_DEBUG_H__



#define  DEBUG_VERSION				  (1U)
#define  RELEASE_VERSION			  (0u)

#define  DEBUG_CAN_BAUDRATE_CFG       (0U)
#define  DEBUG_CAN_ID_STORED          (0U)  /*!< If 0, means ID was defined by marco, else , read ID from flash. */

#define  DEBUG_UART_ECHO_MODE_ON      (0U)  /*!< If 1, to start a function that to send back what received.  */
#define  DEBUG_TIMER_CLK_OUTPUT		  (1U)  /*!< If 1, a plus controlled by TIMER will be outputted by GPIO pin. */
#define  DEBUG_MOTOR_RUN              (0U)  /*!< If 1, step motor will run whatever direction , speed.  */
#define  DEBUG_CAN_COM                (0U)  /*!<  */
#define  DEBUG_CAN_BASIC_MODE		  (0U)
#define  FUNCTION_BOOT                (0U)
#define  DEBUG_INDEPENDENT_TEST       (1U)  /*!< If 1, Motor, valve, motor origin detector, object detector will test by manual. */
#define	 DEBUG_PROTCOL_CAN_RX_FRAME	  (0U)
#define  DEBUG_STATE_PRINT            (1U)

#if (DEBUG_VERSION == RELEASE_VERSION)
# error "Error, please make sure a specific VERSION."
#endif

#endif /* __CTRL_DEBUG_H__ */
