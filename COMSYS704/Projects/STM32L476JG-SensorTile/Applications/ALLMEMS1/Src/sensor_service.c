/**
  ******************************************************************************
  * @file    ble_interface.c

  ******************************************************************************
  */
#include <stdio.h>
#include "main.h"
#include "ble_interface.h"
#include "bluenrg_utils.h"
#include "bluenrg_l2cap_aci.h"
#include "uuid_ble_service.h"


/* Exported variables ---------------------------------------------------------*/
int connected = FALSE;
uint8_t set_connectable = TRUE;

volatile uint32_t FeatureMask;

/* Imported Variables -------------------------------------------------------------*/
extern uint32_t ConnectionBleStatus;


extern uint32_t FirstConnectionConfig;

extern TIM_HandleTypeDef    TimCCHandle;
//extern TIM_HandleTypeDef    TimEnvHandle;
//extern TIM_HandleTypeDef    TimAudioDataHandle;



extern uint8_t bdaddr[6];
extern uint8_t NodeName[8];

extern uint32_t uhCCR4_Val;

/* Private variables ------------------------------------------------------------*/


static uint16_t HWServW2STHandle;

static uint16_t AccGyroMagCharHandle;
//static uint16_t AccEventCharHandle;



/* Code for BlueVoice integration - End Section */

//static uint16_t ConfigServW2STHandle;
//static uint16_t ConfigCharHandle;
//
//static uint16_t ConsoleW2STHandle;
//static uint16_t TermCharHandle;
//static uint16_t StdErrCharHandle;





static uint16_t connection_handle = 0;

/* Private functions ------------------------------------------------------------*/
static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);
static void GAP_DisconnectionComplete_CB(void);

/************************************/
/* Hardware Characteristics Service */
/************************************/
static void AccGyroMag_AttributeModified_CB(uint8_t *att_data);




/* Private define ------------------------------------------------------------*/

#ifdef ACC_BLUENRG_CONGESTION
#define ACI_GATT_UPDATE_CHAR_VALUE safe_aci_gatt_update_char_value
static int32_t breath;


/* @brief  Update the value of a characteristic avoiding (for a short time) to
 *         send the next updates if an error in the previous sending has
 *         occurred.
 * @param  servHandle The handle of the service
 * @param  charHandle The handle of the characteristic
 * @param  charValOffset The offset of the characteristic
 * @param  charValueLen The length of the characteristic
 * @param  charValue The pointer to the characteristic
 * @retval tBleStatus Status
 */
tBleStatus safe_aci_gatt_update_char_value(uint16_t servHandle, 
				      uint16_t charHandle,
				      uint8_t charValOffset,
				      uint8_t charValueLen,   
				      const uint8_t *charValue)
{
  tBleStatus ret = BLE_STATUS_INSUFFICIENT_RESOURCES;
  
  if (breath > 0) {
    breath--;
  } else {
    ret = aci_gatt_update_char_value(servHandle,charHandle,charValOffset,charValueLen,charValue);
    
    if (ret != BLE_STATUS_SUCCESS){
      breath = ACC_BLUENRG_CONGESTION_SKIP;
    }
  }
  
  return (ret);
}

#else /* ACC_BLUENRG_CONGESTION */
#define ACI_GATT_UPDATE_CHAR_VALUE aci_gatt_update_char_value
#endif /* ACC_BLUENRG_CONGESTION */



/**
 * @brief  Add the HW Features service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_HW_SW_ServW2ST_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];
  
  /* Environmental */
  uint8_t max_attr_records= 4;

  COPY_HW_SENS_W2ST_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE,
                          1+3*max_attr_records,
                          &HWServW2STHandle);

  if (ret != BLE_STATUS_SUCCESS) {
	  XPRINTF("Error Creating Service\r\n");
    goto fail;
  }
  

  COPY_ACC_GYRO_MAG_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+3*3*2,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &AccGyroMagCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
	  XPRINTF("Error Creating Characterstic\r\n");
    goto fail;
  }

//  COPY_ACC_EVENT_W2ST_CHAR_UUID(uuid);
//  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+3, //2+2,
//                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
//                           ATTR_PERMISSION_NONE,
//                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
//                           16, 1, &AccEventCharHandle);
//
//  if (ret != BLE_STATUS_SUCCESS) {
//    goto fail;
//  }

  return BLE_STATUS_SUCCESS;

fail:
  //ALLMEMS1_PRINTF("Error while adding HW's Characteristcs service.\n");
  return BLE_STATUS_ERROR;
}

/**
 * @brief  Update acceleration/Gryoscope and Magneto characteristics value
 * @param  BSP_MOTION_SENSOR_Axes_t Acc Structure containing acceleration value in mg
 * @param  BSP_MOTION_SENSOR_Axes_t Gyro Structure containing Gyroscope value
 * @param  BSP_MOTION_SENSOR_Axes_t Mag Structure containing magneto value
 * @retval tBleStatus      Status
 */
tBleStatus AccGyroMag_Update(BSP_MOTION_SENSOR_Axes_t *Acc,BSP_MOTION_SENSOR_Axes_t *Gyro,BSP_MOTION_SENSOR_Axes_t *Mag)
{  
  tBleStatus ret;

  uint8_t buff[2+3*3*2];

  STORE_LE_16(buff   ,(HAL_GetTick()>>3));

  STORE_LE_16(buff+2 ,Acc->x);
  STORE_LE_16(buff+4 ,Acc->y);
  STORE_LE_16(buff+6 ,Acc->z);

//  Gyro->x/=100;
//  Gyro->y/=100;
//  Gyro->z/=100;

  STORE_LE_16(buff+8 ,Gyro->x*10);
  STORE_LE_16(buff+10,Gyro->y*10);
  STORE_LE_16(buff+12,Gyro->z);

  /* Apply Magneto calibration */
//  x = Mag->x;// - MAG_Offset.x;
//  y = Mag->y;// - MAG_Offset.y;
//  z = Mag->z;// - MAG_Offset.z;

  STORE_LE_16(buff+14,Mag->x);
  STORE_LE_16(buff+16,Mag->y);
  STORE_LE_16(buff+18,Mag->z);

  ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServW2STHandle, AccGyroMagCharHandle, 0, 2+3*3*2, buff);

  if (ret != BLE_STATUS_SUCCESS){
//    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
//      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Acc/Gyro/Mag Char\r\n");
//      Stderr_Update(BufferToWrite,BytesToWrite);
//    } else {
      XPRINTF("Error Updating Acc/Gyro/Mag Char\r\n");
//    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;	
}



/**
 * @brief  Puts the device in connectable mode.
 * @param  None 
 * @retval None
 */
void setConnectable(void)
{  
  char local_name[8] = {AD_TYPE_COMPLETE_LOCAL_NAME,NodeName[1],NodeName[2],NodeName[3],NodeName[4],NodeName[5],NodeName[6],NodeName[7]};
  uint8_t manuf_data[25];
  
//  uint8_t manuf_data[26] = {
//    2,0x0A,0x00 /* 0 dBm */, // Trasmission Power
//    //8,0x09,NAME_BLUEMS, // Complete Name
//    8,0x09,NodeName[1],NodeName[2],NodeName[3],NodeName[4],NodeName[5],NodeName[6],NodeName[7], // Complete Name
//    13,0xFF,0x01/*SKD version */,
//    0x02,
//    0x00 /* AudioSync+AudioData */,
//    0xE0 /* ACC + Gyro + Mag + Environmental + Battery Info */,
//    0x00 /*  Hardware Events + MotionFX + SD Card Logging */,
//    0x00, /*  */
//    0x00, /* BLE MAC start */
//    0x00,
//    0x00,
//    0x00,
//    0x00,
//    0x00, /* BLE MAC stop */
//  };
  
  /* Filling Manufacter Advertise data */
  manuf_data[0 ] = 8U;
  manuf_data[1 ] = 0x09U;
  manuf_data[2 ] = NodeName[1];/* Complete Name */
  manuf_data[3 ] = NodeName[2];
  manuf_data[4 ] = NodeName[3];
  manuf_data[5 ] = NodeName[4];
  manuf_data[6 ] = NodeName[5];
  manuf_data[7 ] = NodeName[6];
  manuf_data[8 ] = NodeName[7];           
  manuf_data[9 ] = 15U;
  manuf_data[10] = 0xFFU;
  manuf_data[11] = 0x30U;/* STM Manufacter AD */
  manuf_data[12] = 0x00U;
#ifdef BLE_MANAGER_SDKV2
  manuf_data[13] = 0x02U;
#else /* BLE_MANAGER_SDKV2 */
  manuf_data[13] = 0x01U;
#endif /* BLE_MANAGER_SDKV2 */
  manuf_data[14] = 0x02U; /* Board Type */
  manuf_data[15] = 0x08U; /* Firmware ID */
  manuf_data[16] = 0x00U;
  manuf_data[17] = 0x00U;
  manuf_data[18] = 0x00U;
  
  /* BLE MAC */
  manuf_data[19] = bdaddr[5];
  manuf_data[20] = bdaddr[4];
  manuf_data[21] = bdaddr[3];
  manuf_data[22] = bdaddr[2];
  manuf_data[23] = bdaddr[1];
  manuf_data[24] = bdaddr[0];

  



  /* disable scan response */
  hci_le_set_scan_resp_data(0,NULL);
  aci_gap_set_discoverable(ADV_IND, 0, 0,
                           RANDOM_ADDR,
                           NO_WHITE_LIST_USE,
                           sizeof(local_name), local_name, 0, NULL, 0, 0);

  /* Send Advertising data */
  aci_gap_update_adv_data(25, manuf_data);
}

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  uint8_t addr[6] Address of peer device
 * @param  uint16_t handle Connection handle
 * @retval None
 */
static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{  
  connected = TRUE;
  connection_handle = handle;

#ifdef ALLMEMS1_DEBUG_CONNECTION
  ALLMEMS1_PRINTF("\r\n>>>>>>CONNECTED %x:%x:%x:%x:%x:%x\r\n\r\n",addr[5],addr[4],addr[3],addr[2],addr[1],addr[0]);
#endif /* ALLMEMS1_DEBUG_CONNECTION */

  ConnectionBleStatus=0;
  


}

/**
 * @brief  This function is called when the peer device get disconnected.
 * @param  None 
 * @retval None
 */
static void GAP_DisconnectionComplete_CB(void)
{
  connected = FALSE;

//#ifdef ALLMEMS1_DEBUG_CONNECTION
  XPRINTF("<<<<<<DISCONNECTED\r\n");
//#endif /* ALLMEMS1_DEBUG_CONNECTION */



  /* Make the device connectable again. */
  set_connectable = TRUE;

  ConnectionBleStatus=0;
  

  
  /************************/
  /* Stops all the Timers */
  /************************/
  

  
  /* Stop Timer For Acc/Gyro/Mag */
  if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_4) != HAL_OK){
    /* Stopping Error */
    Error_Handler();
  }
  
}

///**
// * @brief  This function is called when there is a Bluetooth Read request
// * @param  uint16_t handle Handle of the attribute
// * @retval None
// */
void Read_Request_CB(uint16_t handle)
{
  //EXIT:
  if(connection_handle != 0)
    aci_gatt_allow_read(connection_handle);
}

/**
 * @brief  This function is called when there is a change on the gatt attribute
 * With this function it's possible to understand if one application 
 * is subscribed or not to the one service
 * @param uint16_t att_handle Handle of the attribute
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval None
 */
void Attribute_Modified_CB(uint16_t attr_handle, uint8_t * att_data, uint8_t data_length)
{


    	if(attr_handle == AccGyroMagCharHandle + 2) {
      AccGyroMag_AttributeModified_CB(att_data);
    	}


}


/**
 * @brief  This function is called when there is a change on the gatt attribute for Acc,Gyro and Mag
 * With this function it's possible to understand if one application 
 * is subscribed or not to the Acc,Gyro and Mag service
 * @param uint8_t *att_data attribute data
 * @retval None
 */
static void AccGyroMag_AttributeModified_CB(uint8_t *att_data)
{
  if (att_data[0] == 01) {
    W2ST_ON_CONNECTION(W2ST_CONNECT_ACC_GYRO_MAG);

    /* Start the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_4) != HAL_OK){
      /* Starting Error */
      Error_Handler();
    }

    /* Set the new Capture compare value */
    {
      uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
      /* Set the Capture Compare Register value */
      __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_4, (uhCapture + uhCCR4_Val));
    }
  } else if (att_data[0] == 0) {
    W2ST_OFF_CONNECTION(W2ST_CONNECT_ACC_GYRO_MAG);

    /* Stop the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_4) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }      
  }

    XPRINTF("--->Acc/Gyro/Mag=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ACC_GYRO_MAG) ? " ON\r\n" : " OFF\r\n\n");

}





/**
 * @brief  This function is called whenever there is an ACI event to be processed.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  void *pckt Pointer to the ACI packet
 * @retval None
 */
void HCI_Event_CB(void *pckt)
{
  hci_uart_pckt *hci_pckt = pckt;
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;
  
  XPRINTF("HCI_Event_CB\r\n\r\n");

  if(hci_pckt->type != HCI_EVENT_PKT) {
    return;
  }
  
  switch(event_pckt->evt){
    
  case EVT_DISCONN_COMPLETE:
    {
      GAP_DisconnectionComplete_CB();
    }
    break;
  case EVT_LE_META_EVENT:
    {
      evt_le_meta_event *evt = (void *)event_pckt->data;
      
      switch(evt->subevent){
      case EVT_LE_CONN_COMPLETE:
        {
          evt_le_connection_complete *cc = (void *)evt->data;
          GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
        }
        break;
      }
    }
    break;
  case EVT_VENDOR:
    {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;
      switch(blue_evt->ecode){
      case EVT_BLUE_GATT_READ_PERMIT_REQ:
        {
          evt_gatt_read_permit_req *pr = (void*)blue_evt->data; 
          Read_Request_CB(pr->attr_handle);                    
        }
        break;
      case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
        {
          evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
          Attribute_Modified_CB(evt->attr_handle, evt->att_data,evt->data_length);
        }
        break;
      }
    }
    break;
  }
}

