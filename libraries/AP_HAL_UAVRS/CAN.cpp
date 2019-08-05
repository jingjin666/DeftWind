/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 *
 * With modifications for Ardupilot CAN driver
 * Copyright (C) 2017 Eugene Shamaev
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/system.h>

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

#if HAL_WITH_UAVCAN

#include <cassert>
#include <cstring>
#include "CAN.h"

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <unistd.h>
#include <drivers/drv_hrt.h>

#include <platforms/dp_micro_hal.h>
#include <arch/board/board.h>
#include "imxrt_periphclks.h"

#include "Scheduler.h"

/*
 * FOR INVESTIGATION:
 * AP_HAL::micros64() was called for monotonic time counter
 * pavel-kirienko: This will work as long as we don't need to synchronize the autopilot's own clock with an external
 * time base, e.g. a GNSS time provided by an external GNSS receiver. Libuavcan's STM32 driver supports automatic time
 * synchronization only if it has a dedicated hardware timer to work with.
 */

extern const AP_HAL::HAL& hal;

#include <AP_UAVCAN/AP_UAVCAN.h>

using namespace UAVRS;

extern "C" {
    static int can1_irq(int irq, void *context, void *arg);
#if CAN_NUM_IFACES > 1
    static int can2_irq(int irq, void *context, void *arg);
#endif
} //extern "C"

#define CAN_IRQ_ATTACH(irq, handler)                          \
   do {                                                      \
        const int res = irq_attach(irq, handler, NULL);          \
        (void)res;                                         \
        assert(res >= 0);                                  \
        up_enable_irq(irq);                                \
    } while(0)

#define RX_BUFFER_NUM (8)       //消息缓冲序号
#define TX_BUFFER_NUM (9)       //消息缓冲序号

int8_t FLEXCAN_SendMsg(CAN_Type *base, flexcan_frame_t* msg)
{
    int8_t ret = -1;
    
    /* Assertion. */
    assert(msg != NULL);
    
    if(FLEXCAN_TransferSendBlocking(base, TX_BUFFER_NUM, msg) == kStatus_Success) {
        ret = 0;//发送数据，阻塞传输
    } else {
        ret = -1;
    }
        
    return ret;
}

uint64_t clock::getUtcUSecFromCanInterrupt()
{
    return AP_HAL::micros64();
}

uavcan::MonotonicTime clock::getMonotonic()
{
    return uavcan::MonotonicTime::fromUSec(AP_HAL::micros64());
}

BusEvent::BusEvent(UAVRSCANManager& can_driver) :
    _signal(0)
{
    sem_init(&_wait_semaphore, 0, 0);
}

BusEvent::~BusEvent()
{
}

bool BusEvent::wait(uavcan::MonotonicDuration duration)
{
    struct hrt_call wait_call;

    irqstate_t irs = dp_enter_critical_section();
    if (_signal) {
        _signal = 0;
        dp_leave_critical_section(irs);
        return true;
    }

    sem_init(&_wait_semaphore, 0, 0);
    dp_leave_critical_section(irs);

    hrt_call_after(&wait_call, duration.toUSec(), (hrt_callout) signalFromCallOut, this);
    sem_wait(&_wait_semaphore);

    hrt_cancel(&wait_call);

    irs = dp_enter_critical_section();
    if (_signal) {
        _signal = 0;
        dp_leave_critical_section(irs);

        return true;
    }
    dp_leave_critical_section(irs);

    return false;
}

void BusEvent::signalFromCallOut(BusEvent *sem)
{
    sem_post(&sem->_wait_semaphore);
}

void BusEvent::signalFromInterrupt()
{
    _signal++;
    sem_post(&_wait_semaphore);
}

static void handleTxInterrupt(uint8_t iface_index)
{
    if (iface_index < CAN_NUM_IFACES) {
        uint64_t utc_usec = clock::getUtcUSecFromCanInterrupt();

        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
            if (hal.can_mgr[i] != nullptr) {
                UAVRSCAN* iface = ((UAVRSCANManager*) hal.can_mgr[i])->getIface_out_to_in(iface_index);
                if (iface != nullptr) {
                    iface->handleTxInterrupt(utc_usec);
                }
            }
        }
    }
}

static void handleRxInterrupt(uint8_t iface_index, uint8_t fifo_index)
{
    if (iface_index < CAN_NUM_IFACES) {
        uint64_t utc_usec = clock::getUtcUSecFromCanInterrupt();

        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
            if (hal.can_mgr[i] != nullptr) {
                UAVRSCAN* iface = ((UAVRSCANManager*) hal.can_mgr[i])->getIface_out_to_in(iface_index);
                if (iface != nullptr) {
                    iface->handleRxInterrupt(fifo_index, utc_usec);
                }
            }
        }
    }
}

int UAVRSCAN::computeTimings(const uint32_t target_bitrate, Timings& out_timings)
{
    return 0;
}

int16_t UAVRSCAN::send(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline, uavcan::CanIOFlags flags)
{
    int16_t ret;

    if (frame.isErrorFrame() || frame.dlc > 8) {
        return -ErrUnsupportedFrame;
    }

    CriticalSectionLocker lock;

    flexcan_frame_t msg;

    if (frame.isExtended()) {
        msg.format = kFLEXCAN_FrameFormatExtend;    //拓展格式
        msg.id = FLEXCAN_ID_EXT(frame.id);
    } else {
        msg.format = kFLEXCAN_FrameFormatStandard;  //标准格式
        msg.id = FLEXCAN_ID_STD(frame.id);
    }

    if (frame.isRemoteTransmissionRequest()) {
        msg.type = kFLEXCAN_FrameTypeRemote;
    } else {
        msg.type = kFLEXCAN_FrameTypeData;
    }

    msg.length = frame.dlc;
    
    msg.dataByte0 = frame.data[0];
    msg.dataByte1 = frame.data[1];
    msg.dataByte2 = frame.data[2];
    msg.dataByte3 = frame.data[3];
    msg.dataByte4 = frame.data[4];
    msg.dataByte5 = frame.data[5];
    msg.dataByte6 = frame.data[6];
    msg.dataByte7 = frame.data[7];

    if((ret = FLEXCAN_SendMsg(can_, &msg)) == -1) {
        //printf("Send msg failed, frame.id is 0x%08x\n", frame.id);
    } else {
        //printf("Send msg ok, frame.id is 0x%08x\n", frame.id);
    }

    TxItem& txi = pending_tx_[0];
    txi.deadline = tx_deadline;
    txi.frame = frame;
    txi.loopback = (flags & uavcan::CanIOFlagLoopback) != 0;
    txi.abort_on_error = (flags & uavcan::CanIOFlagAbortOnError) != 0;
    txi.pending = true;

    // 此处必须返回1，否则会一直重发！！！
    return 1;
}

int16_t UAVRSCAN::receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic,
                        uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags)
{
    out_ts_monotonic = clock::getMonotonic(); // High precision is not required for monotonic timestamps
    uint64_t utc_usec = 0;
    {
        CriticalSectionLocker lock;
        if (rx_queue_.available() == 0) {
            return 0;
        }

        CanRxItem frm;
        rx_queue_.pop(frm);
        out_frame = frm.frame;
        utc_usec = frm.utc_usec;
        out_flags = frm.flags;
    }
    out_ts_utc = uavcan::UtcTime::fromUSec(utc_usec);
    return 1;
}

int16_t UAVRSCAN::configureFilters(const uavcan::CanFilterConfig* filter_configs, uint16_t num_configs)
{
    return 0;
}

bool UAVRSCAN::waitMsrINakBitStateChange(bool target_state)
{
#if 0
    const unsigned Timeout = 1000;
    for (unsigned wait_ack = 0; wait_ack < Timeout; wait_ack++) {
        const bool state = (can_->MSR & bxcan::MSR_INAK) != 0;
        if (state == target_state) {
            return true;
        }
        hal.scheduler->delay_microseconds(1000);
    }
    return false;
#endif
return true;
}

int UAVRSCAN::init(const uint32_t bitrate, const OperatingMode mode)
{
    /*
     * Object state - CAN interrupts are disabled, so it's safe to modify it now
     */
    rx_queue_.clear();
    error_cnt_ = 0;
    served_aborts_cnt_ = 0;
    uavcan::fill_n(pending_tx_, NumTxMailboxes, TxItem());
    peak_tx_mailbox_index_ = 0;
    had_activity_ = false;

    uint32_t mcrTemp;
    uint32_t can_rxid = 0;                               //不过滤ID
    
    uint32_t canclk = 20000000;                          //20Mhz
    flexcan_rx_mb_config_t mb_config;
    flexcan_config_t can_config;

    FLEXCAN_GetDefaultConfig(&can_config);               //先配置为默认值 
    can_config.baudRate = bitrate;                       //波特率为500Kbit
    can_config.enableLoopBack = mode;                    //设置模式
    FLEXCAN_Init(can_, &can_config, canclk);             //初始化CAN

    FLEXCAN_SetRxMbGlobalMask(can_, FLEXCAN_RX_MB_EXT_MASK(can_rxid, 0, 0));

    FLEXCAN_EnableMbInterrupts(can_, 1<<RX_BUFFER_NUM);  //使能RX消息缓冲中断

    /* Setup Rx Message Buffer. */
    mb_config.format = kFLEXCAN_FrameFormatStandard;      //标准帧
    mb_config.type = kFLEXCAN_FrameTypeData;              //数据帧
    mb_config.id = FLEXCAN_ID_EXT(can_rxid);              //接收的ID
    FLEXCAN_SetRxMbConfig(can_, RX_BUFFER_NUM, &mb_config, true);
    
    /* Setup Tx Message Buffer. */
    FLEXCAN_SetTxMbConfig(can_, TX_BUFFER_NUM, true);
    return 0;
}

void UAVRSCAN::handleTxMailboxInterrupt(uint8_t mailbox_index, bool txok, const uint64_t utc_usec)
{
    if (mailbox_index < NumTxMailboxes) {

        had_activity_ = had_activity_ || txok;

        TxItem& txi = pending_tx_[mailbox_index];

        if (txi.loopback && txok && txi.pending) {
            CanRxItem frm;
            frm.frame = txi.frame;
            frm.flags = uavcan::CanIOFlagLoopback;
            frm.utc_usec = utc_usec;
            rx_queue_.push(frm);
        }

        txi.pending = false;
    }
}

void UAVRSCAN::handleTxInterrupt(const uint64_t utc_usec)
{
#if 0
    // TXOK == false means that there was a hardware failure
    if (can_->TSR & bxcan::TSR_RQCP0) {
        const bool txok = can_->TSR & bxcan::TSR_TXOK0;
        can_->TSR = bxcan::TSR_RQCP0;
        handleTxMailboxInterrupt(0, txok, utc_usec);
    }
    if (can_->TSR & bxcan::TSR_RQCP1) {
        const bool txok = can_->TSR & bxcan::TSR_TXOK1;
        can_->TSR = bxcan::TSR_RQCP1;
        handleTxMailboxInterrupt(1, txok, utc_usec);
    }
    if (can_->TSR & bxcan::TSR_RQCP2) {
        const bool txok = can_->TSR & bxcan::TSR_TXOK2;
        can_->TSR = bxcan::TSR_RQCP2;
        handleTxMailboxInterrupt(2, txok, utc_usec);
    }
#endif
    if(update_event_ != nullptr) {
        update_event_->signalFromInterrupt();
    }

    pollErrorFlagsFromISR();
}

void UAVRSCAN::handleRxInterrupt(uint8_t fifo_index, uint64_t utc_usec)
{
    uavcan::CanFrame frame;
    flexcan_frame_t rx_frame;

    /*
     * Read the frame contents
     */
    if (FLEXCAN_GetMbStatusFlags(can_, 1<<RX_BUFFER_NUM))       //判断CAN的RX信息缓冲是否收到数据
    {
        FLEXCAN_ClearMbStatusFlags(can_, 1<<RX_BUFFER_NUM);     //清除中断标志位
        FLEXCAN_ReadRxMb(can_, RX_BUFFER_NUM, &rx_frame);       //读取数据
    }

    frame.id = rx_frame.id;
    
    if(rx_frame.format == kFLEXCAN_FrameFormatExtend)
        frame.id |= uavcan::CanFrame::FlagEFF;
    if(rx_frame.type == kFLEXCAN_FrameTypeRemote)
        frame.id |= uavcan::CanFrame::FlagRTR;
    //printf("frame.id is 0x%08x\n", frame.id);
    
    frame.dlc = rx_frame.length;
    frame.data[0] = rx_frame.dataByte0;
    frame.data[1] = rx_frame.dataByte1;
    frame.data[2] = rx_frame.dataByte2;
    frame.data[3] = rx_frame.dataByte3;
    frame.data[4] = rx_frame.dataByte4;
    frame.data[5] = rx_frame.dataByte5;
    frame.data[6] = rx_frame.dataByte6;
    frame.data[7] = rx_frame.dataByte7;

    /*
     * Store with timeout into the FIFO buffer and signal update event
     */
    CanRxItem frm;
    frm.frame = frame;
    frm.flags = 0;
    frm.utc_usec = utc_usec;
    rx_queue_.push(frm);

    had_activity_ = true;
    if(update_event_ != nullptr) {
        update_event_->signalFromInterrupt();
    }

    pollErrorFlagsFromISR();  
}

void UAVRSCAN::pollErrorFlagsFromISR()
{
#if 0
    const uint8_t lec = uint8_t((can_->ESR & bxcan::ESR_LEC_MASK) >> bxcan::ESR_LEC_SHIFT);
    if (lec != 0) {
        can_->ESR = 0;
        error_cnt_++;

        // Serving abort requests
        for (int i = 0; i < NumTxMailboxes; i++) { // Dear compiler, may I suggest you to unroll this loop please.
            TxItem& txi = pending_tx_[i];
            if (txi.pending && txi.abort_on_error) {
                can_->TSR = TSR_ABRQx[i];
                txi.pending = false;
                served_aborts_cnt_++;
            }
        }
    }
#endif    
}

void UAVRSCAN::discardTimedOutTxMailboxes(uavcan::MonotonicTime current_time)
{
#if 0
    CriticalSectionLocker lock;
    for (int i = 0; i < NumTxMailboxes; i++) {
        TxItem& txi = pending_tx_[i];
        if (txi.pending && txi.deadline < current_time) {
            can_->TSR = TSR_ABRQx[i];  // Goodnight sweet transmission
            txi.pending = false;
            error_cnt_++;
        }
    }
#endif
}

bool UAVRSCAN::canAcceptNewTxFrame(const uavcan::CanFrame& frame) const
{
#if 0
    /*
     * We can accept more frames only if the following conditions are satisfied:
     *  - There is at least one TX mailbox free (obvious enough);
     *  - The priority of the new frame is higher than priority of all TX mailboxes.
     */
    {
        static const uint32_t TME = bxcan::TSR_TME0 | bxcan::TSR_TME1 | bxcan::TSR_TME2;
        const uint32_t tme = can_->TSR & TME;

        if (tme == TME) {   // All TX mailboxes are free (as in freedom).
            return true;
        }

        if (tme == 0) {     // All TX mailboxes are busy transmitting.
            return false;
        }
    }

    /*
     * The second condition requires a critical section.
     */
    CriticalSectionLocker lock;

    for (int mbx = 0; mbx < NumTxMailboxes; mbx++) {
        if (pending_tx_[mbx].pending && !frame.priorityHigherThan(pending_tx_[mbx].frame)) {
            return false; // There's a mailbox whose priority is higher or equal the priority of the new frame.
        }
    }

    return true; // This new frame will be added to a free TX mailbox in the next @ref send().
#endif
return true;
}

bool UAVRSCAN::isRxBufferEmpty() const
{
    CriticalSectionLocker lock;
    return rx_queue_.available() == 0;
}

uint64_t UAVRSCAN::getErrorCount() const
{
    CriticalSectionLocker lock;
    return error_cnt_;
    //TODO: + rx_queue_.getOverflowCount();
}

unsigned UAVRSCAN::getRxQueueLength() const
{
    CriticalSectionLocker lock;
    return rx_queue_.available();
}

bool UAVRSCAN::hadActivity()
{
    CriticalSectionLocker lock;
    const bool ret = had_activity_;
    had_activity_ = false;
    return ret;
}

bool UAVRSCAN::begin(uint32_t bitrate)
{
    if (init(bitrate, OperatingMode::NormalMode) == 0) {
        bitrate_ = bitrate;
        initialized_ = true;
    } else {
        initialized_ = false;
    }
    return initialized_;
}

void UAVRSCAN::reset()
{
    if (initialized_ && bitrate_ != 0) {
        init(bitrate_, OperatingMode::NormalMode);
    }
}

bool UAVRSCAN::is_initialized()
{
    return initialized_;
}

int32_t UAVRSCAN::available()
{
    if (initialized_) {
        return getRxQueueLength();
    } else {
        return -1;
    }
}

int32_t UAVRSCAN::tx_pending()
{
    int32_t ret = -1;

    if (initialized_) {
        ret = 0;
        CriticalSectionLocker lock;

        for (int mbx = 0; mbx < NumTxMailboxes; mbx++) {
            if (pending_tx_[mbx].pending) {
                ret++;
            }
        }
    }

    return ret;
}

/*
 * CanDriver
 */

/**
 * CANx register sets
 */
CAN_Type* const Can[2] = { reinterpret_cast<CAN_Type*>(CAN1_BASE), reinterpret_cast<CAN_Type*>(CAN2_BASE) };

UAVRSCANManager::UAVRSCANManager() :
    update_event_(*this), if0_(Can[0], nullptr, 0, CAN_RX_QUEUE_SIZE), if1_(
    Can[1], nullptr, 1, CAN_RX_QUEUE_SIZE), initialized_(false), p_uavcan(nullptr)
{
    uavcan::StaticAssert<(CAN_RX_QUEUE_SIZE <= UAVRSCAN::MaxRxQueueCapacity)>::check();

    for(uint8_t i = 0; i < CAN_NUM_IFACES; i++) {
        _ifaces_out_to_in[i] = UINT8_MAX;
    }
}

uavcan::CanSelectMasks UAVRSCANManager::makeSelectMasks(const uavcan::CanFrame* (&pending_tx)[uavcan::MaxCanIfaces]) const
{
    uavcan::CanSelectMasks msk;

    for (uint8_t i = 0; i < _ifaces_num; i++) {
        if (ifaces[i] != nullptr) {
            if (!ifaces[i]->isRxBufferEmpty()) {
                msk.read |= 1 << i;
            }

            if (pending_tx[i] != nullptr) {
                if (ifaces[i]->canAcceptNewTxFrame(*pending_tx[i])) {
                    msk.write |= 1 << i;
                }
            }
        }
    }

    return msk;
}

bool UAVRSCANManager::hasReadableInterfaces() const
{
    bool ret = false;

    for (uint8_t i = 0; i < _ifaces_num; i++) {
        if (ifaces[i] != nullptr) {
            ret |= !ifaces[i]->isRxBufferEmpty();
        }
    }

    return ret;
}

int16_t UAVRSCANManager::select(uavcan::CanSelectMasks& inout_masks,
                              const uavcan::CanFrame* (&pending_tx)[uavcan::MaxCanIfaces], const uavcan::MonotonicTime blocking_deadline)
{
    const uavcan::CanSelectMasks in_masks = inout_masks;
    const uavcan::MonotonicTime time = clock::getMonotonic();

    for (uint8_t i = 0; i < _ifaces_num; i++) {
        if (ifaces[i] != nullptr) {
            ifaces[i]->discardTimedOutTxMailboxes(time);
            {
                CriticalSectionLocker cs_locker;
                ifaces[i]->pollErrorFlagsFromISR();
            }
        }
    }

    inout_masks = makeSelectMasks(pending_tx); // Check if we already have some of the requested events
    if ((inout_masks.read & in_masks.read) != 0 || (inout_masks.write & in_masks.write) != 0) {
        return 1;
    }

    (void) update_event_.wait(blocking_deadline - time); // Block until timeout expires or any iface updates
    inout_masks = makeSelectMasks(pending_tx); // Return what we got even if none of the requested events are set
    return 1; // Return value doesn't matter as long as it is non-negative
}

void UAVRSCANManager::initOnce(uint8_t can_number)
{
    {
        CriticalSectionLocker lock;
        if (can_number == 0) {
            imxrt_clockall_can1();
            imxrt_clockall_can1_serial();
        }
#if CAN_NUM_IFACES > 1
        if (can_number == 1) {
            imxrt_clockall_can2();
            imxrt_clockall_can2_serial();
        }
#endif
    }

    if (can_number == 0) {
#if defined(GPIO_CAN1_RX) && defined(GPIO_CAN1_TX)
        dp_arch_configgpio(GPIO_CAN1_RX);
        dp_arch_configgpio(GPIO_CAN1_TX);
#else
# error  "Need to define GPIO_CAN1_RX/TX"
#endif
    }
#if CAN_NUM_IFACES > 1
    if (can_number == 1) {
#if defined(GPIO_CAN2_RX) && defined(GPIO_CAN2_TX)
        dp_arch_configgpio(GPIO_CAN2_RX);
        dp_arch_configgpio(GPIO_CAN2_TX);
#else
# error  "Need to define GPIO_CAN2_RX/TX"
#endif // defined(GPIO_CAN2_RX) && defined(GPIO_CAN2_TX)
    }
#endif // CAN_NUM_IFACES > 1

    /*
     * IRQ
     */
    if (can_number == 0) {
        CAN_IRQ_ATTACH(IMXRT_IRQ_CAN1, can1_irq);
    }

#if CAN_NUM_IFACES > 1
    if (can_number == 1) {
        CAN_IRQ_ATTACH(IMXRT_IRQ_CAN2, can2_irq);
    }
#endif // CAN_NUM_IFACES > 1
}

int UAVRSCANManager::init(const uint32_t bitrate, const UAVRSCAN::OperatingMode mode, uint8_t can_number)
{
    int res = -ErrNotImplemented;
    static bool initialized_once[CAN_NUM_IFACES];

    if (can_number < CAN_NUM_IFACES) {
        res = 0;

        if (AP_BoardConfig_CAN::get_can_debug(can_number) >= 2) {
            printf("UAVRSCANManager::init Bitrate %lu mode %d bus %d\n\r", static_cast<unsigned long>(bitrate),
                   static_cast<int>(mode), static_cast<int>(can_number));
        }

        // If this outside physical interface was never inited - do this and add it to in/out conversion tables
        if (!initialized_once[can_number]) {
            initialized_once[can_number] = true;
            _ifaces_num++;
            _ifaces_out_to_in[can_number] = _ifaces_num - 1;

            if (AP_BoardConfig_CAN::get_can_debug(can_number) >= 2) {
                printf("UAVRSCANManager::init First initialization bus %d\n\r", static_cast<int>(can_number));
            }

            initOnce(can_number);
        }

        /*
         * CAN1
         */
        if (can_number == 0) {
            if (AP_BoardConfig_CAN::get_can_debug(0) >= 2) {
                printf("UAVRSCANManager::init Initing iface 0...\n\r");
            }
            ifaces[_ifaces_out_to_in[can_number]] = &if0_;               // This link must be initialized first,
        }

#if CAN_NUM_IFACES > 1
        /*
         * CAN2
         */
        if (can_number == 1) {
            if (AP_BoardConfig_CAN::get_can_debug(1) >= 2) {
                printf("UAVRSCANManager::init Initing iface 1...\n\r");
            }
            ifaces[_ifaces_out_to_in[can_number]] = &if1_;                          // Same thing here.
        }
#endif

        ifaces[_ifaces_out_to_in[can_number]]->set_update_event(&update_event_);
        res = ifaces[_ifaces_out_to_in[can_number]]->init(bitrate, mode);
        if (res < 0) {
            ifaces[_ifaces_out_to_in[can_number]] = nullptr;
            return res;
        }

        if (AP_BoardConfig_CAN::get_can_debug(can_number) >= 2) {
            printf("UAVRSCANManager::init CAN drv init OK, res = %d\n\r", res);
        }
    }

    return res;
}

UAVRSCAN* UAVRSCANManager::getIface(uint8_t iface_index)
{
    if (iface_index < _ifaces_num) {
        return ifaces[iface_index];
    }

    return nullptr;
}

UAVRSCAN* UAVRSCANManager::getIface_out_to_in(uint8_t iface_index)
{
    // Find which internal interface corresponds to required outside physical interface
    if (iface_index < CAN_NUM_IFACES) {
        if (_ifaces_out_to_in[iface_index] != UINT8_MAX) {
            return ifaces[_ifaces_out_to_in[iface_index]];
        }
    }

    return nullptr;
}

bool UAVRSCANManager::hadActivity()
{
    bool ret = false;

    // Go through all interfaces that are present in this manager
    for (uint8_t i = 0; i < _ifaces_num; i++)
    {
        if (ifaces[i] != nullptr) {
            ret |= ifaces[i]->hadActivity();
        }
    }

    return ret;
}

bool UAVRSCANManager::begin(uint32_t bitrate, uint8_t can_number)
{
    // Try to init outside physical interface 'can_number'
    if (init(bitrate, UAVRSCAN::OperatingMode::NormalMode, can_number) >= 0) {
        return true;
    }

    return false;
}

bool UAVRSCANManager::is_initialized()
{
    return initialized_;
}

void UAVRSCANManager::initialized(bool val)
{
    initialized_ = val;
}

AP_UAVCAN *UAVRSCANManager::get_UAVCAN(void)
{
    return p_uavcan;
}

void UAVRSCANManager::set_UAVCAN(AP_UAVCAN *uavcan)
{
    p_uavcan = uavcan;
}

/*
 * Interrupt handlers
 */
extern "C" {
    static int can1_irq(int irq, void *context, void *arg)
    {
        if (irq == IMXRT_IRQ_CAN1) {
            //printf("can1_irq rx \n");
            handleRxInterrupt(0, 0);
        } else {
            printf("can1_irq unhandled");
        }
        return 0;
    }

#if CAN_NUM_IFACES > 1
    static int can2_irq(int irq, void *context, void *arg)
    {
        if (irq == IMXRT_IRQ_CAN2) {
            //printf("can2_irq rx \n");
            handleRxInterrupt(1, 0);
        } else {
            printf("can2_irq unhandled");
        }
        return 0;
    }
#endif

} // extern "C"

#endif
