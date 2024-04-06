#pragma once

#include <FlexCAN_T4.h>
#include <imxrt.h>
#include <inttypes.h>

enum class CanInterface { CAN1, CAN2, CAN3 };

enum CanBaudrate : uint32_t {
  CAN_BAUDRATE_125Kbps = 125000,
  CAN_BAUDRATE_250Kbps = 250000,
  CAN_BAUDRATE_500Kbps = 500000,
  CAN_BAUDRATE_1000Kbps = 1000000,
};

struct CanFilter {
  uint32_t id;
  uint32_t mask;
  bool ide;
};

typedef void (*can_rx_isr_func)(const CAN_message_t &msg);

struct CanBeginInfo {
  CanBaudrate baudrate;

  CanFilter *filters;
  size_t filter_count;

  bool loopback;

  CanBeginInfo()
      : baudrate(CAN_BAUDRATE_1000Kbps), filters(nullptr), filter_count(0),
        loopback(false) {}
};

template <CanInterface INTERFACE> static int __get_can_irq() {
  if constexpr (INTERFACE == CanInterface::CAN1) {
    return IRQ_CAN1;
  } else if constexpr (INTERFACE == CanInterface::CAN2) {
    return IRQ_CAN2;
  } else {
    return IRQ_CAN3;
  }
}

template <CanInterface INTERFACE> struct __RxSoftwareFifo {
  static constexpr size_t FIFO_CAP = 128;
  CAN_message_t m_queue[FIFO_CAP];
  unsigned int m_start;
  unsigned int m_end;

  // has to be called from ISR_CANX
  void enqueue(const CAN_message_t &msg) {
    m_queue[m_end] = msg;
    m_end = (m_end + 1) % FIFO_CAP;
  }

  int dequeue(CAN_message_t &msg) {
    NVIC_DISABLE_IRQ(__get_can_irq<INTERFACE>());
    if (m_start == m_end) {
      NVIC_ENABLE_IRQ(__get_can_irq<INTERFACE>());
      return 0;
    }
    msg = m_queue[m_start];
    m_start = (m_start + 1) % FIFO_CAP;

    NVIC_ENABLE_IRQ(__get_can_irq<INTERFACE>());
    return 1;
  }
};

template <CanInterface INTERFACE>
static __RxSoftwareFifo<INTERFACE> &__getSF() {
  static __RxSoftwareFifo<INTERFACE> s_fifo;
  return s_fifo;
}

template <CanInterface INTERFACE>
static void __rx_enqueue(const CAN_message_t &msg) {
  __RxSoftwareFifo<INTERFACE> &fifo = __getSF<INTERFACE>();
  fifo.enqueue(msg);
}

template <CanInterface INTERFACE> struct CAN {
public:
  void begin(const CanBeginInfo &begin_info) {
    m_flexcan.begin();
    m_flexcan.setMaxMB(64);
    m_flexcan.setBaudRate(static_cast<uint32_t>(begin_info.baudrate));
    m_flexcan.enableFIFO();
    if (begin_info.filter_count > 0) {
      m_flexcan.setFIFOFilter(REJECT_ALL);
    }
    for (size_t i = 0; i < begin_info.filter_count; ++i) {
      m_flexcan.setFIFOManualFilter(i, begin_info.filters[i].id,
                                    begin_info.filters[i].mask,
                                    begin_info.filters[i].ide ? EXT : STD);
    }
    m_flexcan.enableFIFOInterrupt();
    m_flexcan.onReceive(__rx_enqueue<INTERFACE>);
    if (begin_info.loopback) {
      m_flexcan.enableLoopBack();
    }
    m_flexcan.mailboxStatus();
  }

  /**
   * Receives a message from the CAN and writes it to @param msg.
   * @returns 1 iff. a message was received and written, otherwise (if not
   * message was received) returns false. NOTE: This function should not be
   * called from a ISR!
   */
  int recv(CAN_message_t &msg) { return __getSF<INTERFACE>().dequeue(msg); }

  /**
   * Sends @param msg on the CAN.
   * @returns 1 iff. the message was successfully send otherwise returns false.
   * NOTE: Sending is not buffered sending a lot might overflow the TX_QUEUE,
   * which leads to dropped messages!
   */
  int send(const CAN_message_t &msg) { return m_flexcan.write(msg); }

private:
  static constexpr CAN_DEV_TABLE FlexCanModule() {
    if constexpr (INTERFACE == CanInterface::CAN1) {
      return CAN_DEV_TABLE::CAN1;
    } else if constexpr (INTERFACE == CanInterface::CAN2) {
      return CAN_DEV_TABLE::CAN2;
    } else if constexpr (INTERFACE == CanInterface::CAN3) {
      return CAN_DEV_TABLE::CAN3;
    }
  }

  FlexCAN_T4<CAN::FlexCanModule(), RX_SIZE_1024, TX_SIZE_1024> m_flexcan;
};

typedef CAN<CanInterface::CAN1> Can1;
typedef CAN<CanInterface::CAN2> Can2;
typedef CAN<CanInterface::CAN3> Can3;
