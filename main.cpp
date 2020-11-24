/*
 * Copyright (c) 2020 Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtos/Thread.h"
#include "rtos/ThisThread.h"
#include "rtos/EventFlags.h"
#include "rtos/Mail.h"

#include "platform/SharedPtr.h"

#include "drivers/BufferedSerial.h"
#include "drivers/DigitalOut.h"

#include "mbed_trace.h"

#include "BleSerialTestServer.h"

#include <chrono>

#define RX_BUFFER_XOFF_THRESHOLD 128
#define RX_BUFFER_XON_THRESHOLD 196

#define DEFAULT_TIMEOUT 30s
#define READ_WRITE_DELAY 0

#define TEST_ASSERT_NO_TIMEOUT(x) TEST_ASSERT(!(x & osFlagsError))

#define EVENT_FLAGS_BLE_READY           0x0001
#define EVENT_FLAGS_BLE_CONNECTED       0x0002
#define EVENT_FLAGS_BLE_DISCONNECTED    0x0004

using namespace std::chrono;

//static mbed::BufferedSerial pc(STDIO_UART_TX, STDIO_UART_RX, 115200);
static events::EventQueue main_queue;
static rtos::EventFlags flags;
static BleSerialTestServer test_srv(main_queue);
static rtos::Thread ble_thread(osPriorityNormal,
        MBED_CONF_RTOS_THREAD_STACK_SIZE << 2,
        NULL, "ble-thread");

static mbed::DigitalOut led2(LED2, 1);

class TestGapEventHandler : public ble::Gap::EventHandler {

public:

    TestGapEventHandler() { }

    virtual ~TestGapEventHandler() { }

    void onConnectionComplete(const ConnectionCompleteEvent &event) override {
        printf("connected\r\n");
    }

    void onDisconnectionComplete(const DisconnectionCompleteEvent &event) override {
        printf("disconnected\r\n");
        flags.set(EVENT_FLAGS_BLE_DISCONNECTED);
    }


    void onPhyUpdateComplete(ble_error_t status, connection_handle_t connectionHandle,
            phy_t txPhy, phy_t rxPhy) {
        if(status == BLE_ERROR_NONE) {
            printf("phy update complete:\r\n");
            printf("\ttxPhy: %d, rxPhy: %d\r\n", txPhy.value(), rxPhy.value());
        } else {
            printf("phy update error: %d\r\n", status);
        }
    }
};

static void ble_thread_main(void) {

    // Add the TestGapEventHandler
    static TestGapEventHandler test_eh;
    test_srv.gap_eh_chain.addEventHandler(&test_eh);

    // This should never return
    test_srv.start();
}

static void on_ble_ready(void) {
    flags.set(EVENT_FLAGS_BLE_READY);
}

PlatformMutex trace_mtx;
void trace_mtx_get(void) {
    trace_mtx.lock();
}

void trace_mtx_release(void) {
    trace_mtx.unlock();
}

int main()
{
    mbed_trace_init();
    mbed_trace_mutex_wait_function_set(trace_mtx_get);
    mbed_trace_mutex_release_function_set(trace_mtx_release);

    mbed_trace_include_filters_set("btdfu");

    // Setup BleSerialTestServer
    test_srv.on_ready(on_ble_ready);

    // Spin off BLE thread
    ble_thread.start(ble_thread_main);

    flags.wait_all_for(EVENT_FLAGS_BLE_READY, DEFAULT_TIMEOUT);

    main_queue.dispatch_forever();

    return 0;
}

