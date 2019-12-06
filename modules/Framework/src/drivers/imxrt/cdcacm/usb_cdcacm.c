/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file usb_cdcacm.cpp
 *
 * Driver for the imxrt USB CDCACM VCOM driver.
 *
 */

#include <dp_config.h>
#include <dp_log.h>
#include <board_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/ioctl.h>
#include <arch/board/board.h>
#include <chip.h>

#include "imxrt_periphclks.h"

#include "usb_cdcacm.h"
#include "usb_device_cdc_acm.h"
#include "usb_device_descriptor.h"
#include "usb_phy.h"
#include "usb_register.h"
#include "circular_buf.h"

struct usb_cdcacm_dev_s {
    bool initialised;
    usb_cdc_vcom_struct_t s_cdcVcom;
    //struct circular_buffer xmit;
    struct circular_buffer recv;
};

static usb_status_t USB_DeviceCdcVcomCallback(class_handle_t handle, uint32_t event, void *param);
static usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param);
static int USB_OTG1_IRQHandler(int irq, void *context, void *arg);
static uint16_t VCP_DataRx (uint8_t* Buf, uint32_t Len);
static uint16_t VCP_DataTx(uint8_t data);

static bool usb_cdcacm_init(void);
static void test(void);

int usb_cdcacm_register(const char *path, struct usb_cdcacm_dev_s *dev);
int usb_cdcacm_unregister(const char *path, struct usb_cdcacm_dev_s *dev);

static int	usb_cdcacm_open(struct file *filep);
static int	usb_cdcacm_close(struct file *filep);
static ssize_t	usb_cdcacm_read(struct file *filep, char *buffer, size_t buflen);
static ssize_t	usb_cdcacm_write(struct file *filep, const char *buffer, size_t buflen);
static int	usb_cdcacm_ioctl(struct file *filep, int cmd, unsigned long arg);

const struct file_operations usb_cdcacm_fops = {
    open	: usb_cdcacm_open,
    close	: usb_cdcacm_close,
    read	: usb_cdcacm_read,
    write	: usb_cdcacm_write,
    seek	: NULL,
    ioctl	: usb_cdcacm_ioctl,
    poll	: NULL,
};
#define SEND_BUFF
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) USB_LINK_USB_GLOBAL static uint8_t s_currRecvBuf[DATA_BUFF_SIZE];
#ifdef SEND_BUFF
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) USB_LINK_USB_GLOBAL static uint8_t s_currSendBuf[DATA_BUFF_SIZE];
#endif

static int
usb_cdcacm_open(struct file *filep)
{
    //DP_INFO("%s::%d", __FUNCTION__, __LINE__);
    return 0;
}

static int
usb_cdcacm_close(struct file *filep)
{
    //DP_INFO("%s::%d", __FUNCTION__, __LINE__);
    return 0;
}

static ssize_t
usb_cdcacm_read(struct file *filep, char *buffer, size_t buflen)
{
    struct inode *inode = filep->f_inode;
    struct usb_cdcacm_dev_s *dev = inode->i_private;

    return read_circular_buf(&dev->recv, (uint8_t *)buffer, buflen);
}

static ssize_t
usb_cdcacm_write(struct file *filep, const char *buffer, size_t buflen)
{
    struct inode *inode = filep->f_inode;
    struct usb_cdcacm_dev_s *dev = inode->i_private;

    //DP_INFO("write len is %d", buflen);
#ifdef SEND_BUFF
    // 如果传入的buffer不在dtcm空间，这里需要手动拷贝到dtcm空间里，才能保证数据正确性！
    memcpy(s_currSendBuf, buffer, buflen);
    USB_DeviceCdcAcmSend(dev->s_cdcVcom.cdcAcmHandle, USB_CDC_VCOM_BULK_IN_ENDPOINT, s_currSendBuf, buflen);
#else
    USB_DeviceCdcAcmSend(dev->s_cdcVcom.cdcAcmHandle, USB_CDC_VCOM_BULK_IN_ENDPOINT, (uint8_t *)buffer, buflen);
#endif
    return buflen;
    //return write_circular_buf(&dev->xmit, (uint8_t *)buffer, buflen);
}

static int
usb_cdcacm_ioctl(struct file *filep, int cmd, unsigned long arg)
{
    struct inode *inode = filep->f_inode;
    struct usb_cdcacm_dev_s *dev = inode->i_private;
    int ret = 0;
    int count;
    
    switch (cmd)
    {
        case FIONREAD:
            count = get_data_count(&dev->recv);
            *(int *)arg = count;
            ret = 0;
            break;
        case FIONSPACE:
            *(int *)arg = 512;
            ret = 0;
            break;
        default:
            ret = -1;
    }

    return ret;
}

/* Data structure of virtual com device */
struct usb_cdcacm_dev_s g_cdcacm_dev;

#define USB_CDCACM_IRQ_ATTACH(irq, handler)                          \
   do {                                                      \
        const int res = irq_attach(irq, handler, NULL);          \
        (void)res;                                         \
        assert(res >= 0);                                  \
        up_enable_irq(irq);                                \
    } while(0)

extern usb_device_endpoint_struct_t g_UsbDeviceCdcVcomDicEndpoints[];
extern usb_device_class_struct_t g_UsbDeviceCdcVcomConfig;

/* Line codinig of cdc device */
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_lineCoding[LINE_CODING_SIZE] = {
   /* E.g. 0x00,0xC2,0x01,0x00 : 0x0001C200 is 115200 bits per second */
   (LINE_CODING_DTERATE >> 0U) & 0x000000FFU,
   (LINE_CODING_DTERATE >> 8U) & 0x000000FFU,
   (LINE_CODING_DTERATE >> 16U) & 0x000000FFU,
   (LINE_CODING_DTERATE >> 24U) & 0x000000FFU,
   LINE_CODING_CHARFORMAT,
   LINE_CODING_PARITYTYPE,
   LINE_CODING_DATABITS};

/* Abstract state of cdc device */
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_abstractState[COMM_FEATURE_DATA_SIZE] = {(STATUS_ABSTRACT_STATE >> 0U) & 0x00FFU,
                                                         (STATUS_ABSTRACT_STATE >> 8U) & 0x00FFU};

/* Country code of cdc device */
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_countryCode[COMM_FEATURE_DATA_SIZE] = {(COUNTRY_SETTING >> 0U) & 0x00FFU,
                                                       (COUNTRY_SETTING >> 8U) & 0x00FFU};

/* CDC ACM information */
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static usb_cdc_acm_info_t s_usbCdcAcmInfo;

//USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) uint8_t USB_USART_RX_BUF[DATA_BUFF_SIZE];

static uint32_t s_recvSize = 0;
static uint32_t s_sendSize = 0;

/* USB device class information */
static usb_device_class_config_struct_t s_cdcAcmConfig[1] = {{
    USB_DeviceCdcVcomCallback, 0, &g_UsbDeviceCdcVcomConfig,
}};

/* USB device class configuraion information */
static usb_device_class_config_list_struct_t s_cdcAcmConfigList = {
    s_cdcAcmConfig, USB_DeviceCallback, 1,
};

uint16_t USB_USART_RX_STA=0;       				//接收状态标记

static uint16_t VCP_DataRx (uint8_t* Buf, uint32_t Len)
{
    uint32_t i;
    //DP_INFO("VCP_DataRx Buf is 0x%02x 0x%02x 0x%02x 0x%02x", Buf[0], Buf[1], Buf[2], Buf[3]);
    //DP_INFO("VCP_DataRx len is %d", Len);
#if 0
	for(i=0;i<Len;i++)
	{  
	    if((USB_USART_RX_STA&0x8000)==0)		//等待Main回显完毕
        {   
    	    USB_USART_RX_BUF[USB_USART_RX_STA&0x3FFF] = Buf[i];
            USB_USART_RX_STA++;
        }
	}

    USB_USART_RX_STA |= 0x8000;
#else
    write_circular_buf(&g_cdcacm_dev.recv, Buf, Len);
#endif
	return kStatus_USB_Success;
}

static uint16_t VCP_DataTx(uint8_t data)
{  
    uint8_t senddata=0;
    
    senddata=data;
    USB_DeviceCdcAcmSend(g_cdcacm_dev.s_cdcVcom.cdcAcmHandle, USB_CDC_VCOM_BULK_IN_ENDPOINT, &senddata, 1);
	return kStatus_USB_Success;
} 

/*!
 * @brief CDC class specific callback function.
 *
 * This function handles the CDC class specific requests.
 *
 * @param handle          The CDC ACM class handle.
 * @param event           The CDC ACM class event type.
 * @param param           The parameter of the class specific request.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceCdcVcomCallback(class_handle_t handle, uint32_t event, void *param)
{
    uint32_t len;
    uint8_t *uartBitmap;
    usb_device_cdc_acm_request_param_struct_t *acmReqParam;
    usb_device_endpoint_callback_message_struct_t *epCbParam;
    usb_status_t error = kStatus_USB_Error;
    usb_cdc_acm_info_t *acmInfo = &s_usbCdcAcmInfo;
    acmReqParam = (usb_device_cdc_acm_request_param_struct_t *)param;
    epCbParam = (usb_device_endpoint_callback_message_struct_t *)param;
    //DP_INFO("USB_DeviceCdcVcomCallback :: event %d", event);
    switch (event)
    {
        case kUSB_DeviceCdcEventSendResponse:
        {
            if ((epCbParam->length != 0) && (!(epCbParam->length % g_UsbDeviceCdcVcomDicEndpoints[0].maxPacketSize)))
            {
                /* If the last packet is the size of endpoint, then send also zero-ended packet,
                 ** meaning that we want to inform the host that we do not have any additional
                 ** data, so it can flush the output.
                 */
                error = USB_DeviceCdcAcmSend(handle, USB_CDC_VCOM_BULK_IN_ENDPOINT, NULL, 0);
            }
            else if ((1 == g_cdcacm_dev.s_cdcVcom.attach) && (1 == g_cdcacm_dev.s_cdcVcom.startTransactions))
            {
                if ((epCbParam->buffer != NULL) || ((epCbParam->buffer == NULL) && (epCbParam->length == 0)))
                {
                    /* User: add your own code for send complete event */
                    /* Schedule buffer for next receive event */
                    error = USB_DeviceCdcAcmRecv(handle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_currRecvBuf,
                                                 g_UsbDeviceCdcVcomDicEndpoints[0].maxPacketSize);
                }
            }
            else
            {
            }
        }
        break;
        case kUSB_DeviceCdcEventRecvResponse:
        {
            if ((1 == g_cdcacm_dev.s_cdcVcom.attach) && (1 == g_cdcacm_dev.s_cdcVcom.startTransactions))
            {
                s_recvSize = epCbParam->length;
                VCP_DataRx(s_currRecvBuf,s_recvSize);
                s_recvSize = 0;
                if (!s_recvSize)
                {
                    /* Schedule buffer for next receive event */
                    error = USB_DeviceCdcAcmRecv(handle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_currRecvBuf,
                                                 g_UsbDeviceCdcVcomDicEndpoints[0].maxPacketSize);
                }
                
            }
        }
        break;
        case kUSB_DeviceCdcEventSerialStateNotif:
            ((usb_device_cdc_acm_struct_t *)handle)->hasSentState = 0;
            error = kStatus_USB_Success;
            break;
        case kUSB_DeviceCdcEventSendEncapsulatedCommand:
            break;
        case kUSB_DeviceCdcEventGetEncapsulatedResponse:
            break;
        case kUSB_DeviceCdcEventSetCommFeature:
            if (USB_DEVICE_CDC_FEATURE_ABSTRACT_STATE == acmReqParam->setupValue)
            {
                if (1 == acmReqParam->isSetup)
                {
                    *(acmReqParam->buffer) = s_abstractState;
                }
                else
                {
                    *(acmReqParam->length) = 0;
                }
            }
            else if (USB_DEVICE_CDC_FEATURE_COUNTRY_SETTING == acmReqParam->setupValue)
            {
                if (1 == acmReqParam->isSetup)
                {
                    *(acmReqParam->buffer) = s_countryCode;
                }
                else
                {
                    *(acmReqParam->length) = 0;
                }
            }
            else
            {
            }
            error = kStatus_USB_Success;
            break;
        case kUSB_DeviceCdcEventGetCommFeature:
            if (USB_DEVICE_CDC_FEATURE_ABSTRACT_STATE == acmReqParam->setupValue)
            {
                *(acmReqParam->buffer) = s_abstractState;
                *(acmReqParam->length) = COMM_FEATURE_DATA_SIZE;
            }
            else if (USB_DEVICE_CDC_FEATURE_COUNTRY_SETTING == acmReqParam->setupValue)
            {
                *(acmReqParam->buffer) = s_countryCode;
                *(acmReqParam->length) = COMM_FEATURE_DATA_SIZE;
            }
            else
            {
            }
            error = kStatus_USB_Success;
            break;
        case kUSB_DeviceCdcEventClearCommFeature:
            break;
        case kUSB_DeviceCdcEventGetLineCoding:
            *(acmReqParam->buffer) = s_lineCoding;
            *(acmReqParam->length) = LINE_CODING_SIZE;
            error = kStatus_USB_Success;
            break;
        case kUSB_DeviceCdcEventSetLineCoding:
        {
            if (1 == acmReqParam->isSetup)
            {
                *(acmReqParam->buffer) = s_lineCoding;
            }
            else
            {
                *(acmReqParam->length) = 0;
            }
        }
            error = kStatus_USB_Success;
            break;
        case kUSB_DeviceCdcEventSetControlLineState:
        {
            
            s_usbCdcAcmInfo.dteStatus = acmReqParam->setupValue;
            /* activate/deactivate Tx carrier */
            if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_CARRIER_ACTIVATION)
            {
                acmInfo->uartState |= USB_DEVICE_CDC_UART_STATE_TX_CARRIER;
            }
            else
            {
                acmInfo->uartState &= (uint16_t)~USB_DEVICE_CDC_UART_STATE_TX_CARRIER;
            }

            /* activate carrier and DTE */
            /***********************修改过代码********************/
            acmInfo->dteStatus |= USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE;
            acmInfo->uartState |= USB_DEVICE_CDC_UART_STATE_RX_CARRIER;
            /*****************************************************/
//            if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE)
//            {
//                acmInfo->uartState |= USB_DEVICE_CDC_UART_STATE_RX_CARRIER;
//            }
//            else
//            {

//                acmInfo->uartState &= (uint16_t)~USB_DEVICE_CDC_UART_STATE_RX_CARRIER;
//            }

            /* Indicates to DCE if DTE is present or not */
            acmInfo->dtePresent = (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE) ? true : false;

            /* Initialize the serial state buffer */
            acmInfo->serialStateBuf[0] = NOTIF_REQUEST_TYPE;                /* bmRequestType */
            acmInfo->serialStateBuf[1] = USB_DEVICE_CDC_NOTIF_SERIAL_STATE; /* bNotification */
            acmInfo->serialStateBuf[2] = 0x00;                              /* wValue */
            acmInfo->serialStateBuf[3] = 0x00;
            acmInfo->serialStateBuf[4] = 0x00; /* wIndex */
            acmInfo->serialStateBuf[5] = 0x00;
            acmInfo->serialStateBuf[6] = UART_BITMAP_SIZE; /* wLength */
            acmInfo->serialStateBuf[7] = 0x00;
            /* Notifiy to host the line state */
            acmInfo->serialStateBuf[4] = acmReqParam->interfaceIndex;
            /* Lower byte of UART BITMAP */
            uartBitmap = (uint8_t *)&acmInfo->serialStateBuf[NOTIF_PACKET_SIZE + UART_BITMAP_SIZE - 2];
            uartBitmap[0] = acmInfo->uartState & 0xFFu;
            uartBitmap[1] = (acmInfo->uartState >> 8) & 0xFFu;
            len = (uint32_t)(NOTIF_PACKET_SIZE + UART_BITMAP_SIZE);
            if (0 == ((usb_device_cdc_acm_struct_t *)handle)->hasSentState)
            {
                error = USB_DeviceCdcAcmSend(handle, USB_CDC_VCOM_INTERRUPT_IN_ENDPOINT, acmInfo->serialStateBuf, len);
                if (kStatus_USB_Success != error)
                {
                    usb_echo("kUSB_DeviceCdcEventSetControlLineState error!");
                }
                ((usb_device_cdc_acm_struct_t *)handle)->hasSentState = 1;
            }

            /* Update status */
            if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_CARRIER_ACTIVATION)
            {
                /*  To do: CARRIER_ACTIVATED */
            }
            else
            {
                /* To do: CARRIER_DEACTIVATED */
            }
            if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE)
            {
                /* DTE_ACTIVATED */
                if (1 == g_cdcacm_dev.s_cdcVcom.attach)
                {
                    g_cdcacm_dev.s_cdcVcom.startTransactions = 1;
                }
            }
            else
            {
                /* DTE_DEACTIVATED */
                if (1 == g_cdcacm_dev.s_cdcVcom.attach)
                {
                    g_cdcacm_dev.s_cdcVcom.startTransactions = 0;
                }
            }
        }
        break;
        case kUSB_DeviceCdcEventSendBreak:
            break;
        default:
            break;
    }

    return error;
}

/*!
 * @brief USB device callback function.
 *
 * This function handles the usb device specific requests.
 *
 * @param handle          The USB device handle.
 * @param event           The USB device event type.
 * @param param           The parameter of the device specific request.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_Error;
    uint16_t *temp16 = (uint16_t *)param;
    uint8_t *temp8 = (uint8_t *)param;
    //DP_INFO("USB_DeviceCallback :: event %d", event);

    switch (event)
    {
        case kUSB_DeviceEventBusReset:
        {
            g_cdcacm_dev.s_cdcVcom.attach = 0;
            /* Get USB speed to configure the device, including max packet size and interval of the endpoints. */
            if (kStatus_USB_Success == USB_DeviceClassGetSpeed(CONTROLLER_ID, &g_cdcacm_dev.s_cdcVcom.speed))
            {
                USB_DeviceSetSpeed(handle, g_cdcacm_dev.s_cdcVcom.speed);
            }
        }
        break;
        case kUSB_DeviceEventSetConfiguration:
            if (param)
            {
                g_cdcacm_dev.s_cdcVcom.attach = 1;
                g_cdcacm_dev.s_cdcVcom.currentConfiguration = *temp8;
                if (USB_CDC_VCOM_CONFIGURE_INDEX == (*temp8))
                {
                    /* Schedule buffer for receive */
                    USB_DeviceCdcAcmRecv(g_cdcacm_dev.s_cdcVcom.cdcAcmHandle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_currRecvBuf,
                                         g_UsbDeviceCdcVcomDicEndpoints[0].maxPacketSize);
                }
            }
            break;
        case kUSB_DeviceEventSetInterface:
            if (g_cdcacm_dev.s_cdcVcom.attach)
            {
                uint8_t interface = (uint8_t)((*temp16 & 0xFF00U) >> 0x08U);
                uint8_t alternateSetting = (uint8_t)(*temp16 & 0x00FFU);
                if (interface < USB_CDC_VCOM_INTERFACE_COUNT)
                {
                    g_cdcacm_dev.s_cdcVcom.currentInterfaceAlternateSetting[interface] = alternateSetting;
                }
            }
            break;
        case kUSB_DeviceEventGetConfiguration:
            break;
        case kUSB_DeviceEventGetInterface:
            break;
        case kUSB_DeviceEventGetDeviceDescriptor:
            if (param)
            {
                error = USB_DeviceGetDeviceDescriptor(handle, (usb_device_get_device_descriptor_struct_t *)param);
            }
            break;
        case kUSB_DeviceEventGetConfigurationDescriptor:
            if (param)
            {
                error = USB_DeviceGetConfigurationDescriptor(handle,
                                                             (usb_device_get_configuration_descriptor_struct_t *)param);
            }
            break;
        case kUSB_DeviceEventGetStringDescriptor:
            if (param)
            {
                /* Get device string descriptor request */
                error = USB_DeviceGetStringDescriptor(handle, (usb_device_get_string_descriptor_struct_t *)param);
            }
            break;
        default:
            break;
    }

    return error;
}

static int USB_OTG1_IRQHandler(int irq, void *context, void *arg)
{
   USB_DeviceEhciIsrFunction(g_cdcacm_dev.s_cdcVcom.deviceHandle);
   return 0;
}

static bool usb_cdcacm_init(void)
{
    // CLOCK_EnableUsbhs0PhyPllClock
    //DP_INFO("CLOCK_EnableUsbhs0PhyPllClock");
    USBPHY1->CTRL &= ~USBPHY_CTRL_SFTRST_MASK;         /* release PHY from reset */
    USBPHY1->CTRL &= ~USBPHY_CTRL_CLKGATE_MASK;

    USBPHY1->PWD  = 0;
    USBPHY1->CTRL |= USBPHY_CTRL_ENAUTOCLR_PHY_PWD_MASK |
                        USBPHY_CTRL_ENAUTOCLR_CLKGATE_MASK |
                        USBPHY_CTRL_ENUTMILEVEL2_MASK |
                        USBPHY_CTRL_ENUTMILEVEL3_MASK;

    // CLOCK_EnableUsbhs0Clock
    //DP_INFO("CLOCK_EnableUsbhs0Clock");
    imxrt_clockall_usboh3();
    USB1->USBCMD |= USBHS_USBCMD_RST_MASK;

    usb_phy_config_struct_t phyConfig = {
        BOARD_USB_PHY_D_CAL, BOARD_USB_PHY_TXCAL45DP, BOARD_USB_PHY_TXCAL45DM,
    };

    USB_EhciPhyInit(CONTROLLER_ID, BOARD_XTAL0_CLK_HZ, &phyConfig);

    g_cdcacm_dev.s_cdcVcom.speed = USB_SPEED_FULL;
    g_cdcacm_dev.s_cdcVcom.attach = 0;
    g_cdcacm_dev.s_cdcVcom.cdcAcmHandle = (class_handle_t)NULL;
    g_cdcacm_dev.s_cdcVcom.deviceHandle = NULL;

    if (kStatus_USB_Success != USB_DeviceClassInit(CONTROLLER_ID, &s_cdcAcmConfigList, &g_cdcacm_dev.s_cdcVcom.deviceHandle)) {
        DP_ERR("USB device init failed\n");
        return false;
    } else {
        g_cdcacm_dev.s_cdcVcom.cdcAcmHandle = s_cdcAcmConfigList.config->classHandle;
    }

    USB_CDCACM_IRQ_ATTACH(IMXRT_IRQ_USBOTG1, USB_OTG1_IRQHandler);

    if(USB_DeviceRun(g_cdcacm_dev.s_cdcVcom.deviceHandle) != kStatus_USB_Success) {
        DP_ERR("USB_DeviceRun failed\n");
        return false;
    }
    
    return true;
}

static void test(void)
{
    uint32_t len;
    uint8_t rx[32] = {0};
    for(;;) {
#if 0       
        if(USB_USART_RX_STA&0x8000)
        {					   
        	len=USB_USART_RX_STA&0x3FFF;//得到此次接收到的数据长度
        	DP_INFO("len:%d\n",len);
            USB_DeviceCdcAcmSend(g_cdcacm_dev.s_cdcVcom.cdcAcmHandle, USB_CDC_VCOM_BULK_IN_ENDPOINT, USB_USART_RX_BUF,len);
        	USB_USART_RX_STA=0;
        }
#else
        len = get_data_count(&g_cdcacm_dev.recv);
        if(len > 0) {
            memset(rx, 0, sizeof(rx));
            uint32_t data_cnts = read_circular_buf(&g_cdcacm_dev.recv, rx, len);
            USB_DeviceCdcAcmSend(g_cdcacm_dev.s_cdcVcom.cdcAcmHandle, USB_CDC_VCOM_BULK_IN_ENDPOINT, rx, data_cnts);
        }
#endif
        sleep(1);
    }
}

__EXPORT int usb_cdcacm_main(int argc, char *argv[]);
__EXPORT int sercon_main(int argc, char *argv[]);
__EXPORT int serdis_main(int argc, char *argv[]);

int usb_cdcacm_register(const char *path, struct usb_cdcacm_dev_s *dev)
{
    int ret;
    ret = register_driver(path, &usb_cdcacm_fops, 0666, (void *)dev);
    if(ret < 0) {
        DP_ERR("Failed to register driver [%s]\n", path);
        return -1;
    }

    init_circular_buf(&g_cdcacm_dev.recv, 4*1024);
    //init_circular_buf(&g_cdcacm_dev.xmit, 1*1024);

    usb_cdcacm_init();

    dev->initialised = true;
    
    return ret;
}

int usb_cdcacm_unregister(const char *path, struct usb_cdcacm_dev_s *dev)
{
    return 0;
}

int usb_cdcacm_main(int argc, char *argv[])
{
    if(!g_cdcacm_dev.initialised) {
        usb_cdcacm_register(CDCACM_DEVICE_PATH, &g_cdcacm_dev);
    }

    if (argc > 1) {
		if (!strcmp(argv[1], "test")) {
			test();
		}
	}

    return 0;
}

int sercon_main(int argc, char *argv[])
{
    if(!g_cdcacm_dev.initialised) {
        usb_cdcacm_register(CDCACM_DEVICE_PATH, &g_cdcacm_dev);
    }

    return 0;
}

int serdis_main(int argc, char *argv[])
{
    if(g_cdcacm_dev.initialised) {
        usb_cdcacm_unregister(CDCACM_DEVICE_PATH, &g_cdcacm_dev);
    }
    
    return 0;
}
