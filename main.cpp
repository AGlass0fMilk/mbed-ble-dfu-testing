/* mbed Microcontroller Library
 * Copyright (c) 2017-2019 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>

#include "platform/Callback.h"
#include "platform/NonCopyable.h"
#include "platform/mbed_stats.h"

#include "ble/GattServer.h"
#include "BLEProcess.h"

#include "UARTService.h"

#include "rtos/Thread.h"
#include "rtos/ThisThread.h"
#include "rtos/Queue.h"
#include "rtos/EventFlags.h"
#include "rtos/Mutex.h"

#include "events/EventQueue.h"

#include "mbed_trace.h"

#include <memory>
#include <string>

#include "ChainableGattServerEventHandler.h"
#include "ChainableGapEventHandler.h"

#define TRACE_GROUP "main"

using namespace std::chrono;

UARTService uart_service;

static volatile ble::connection_handle_t connection = 0;
mbed::SharedPtr<UARTService::BleSerial> ser;

static rtos::Thread ble_thread;

#define EVENT_FLAGS_CONNECTED 0x0001
static rtos::EventFlags flags;

static PlatformMutex tr_mutex;
void trace_mutex_wait(void) {
    tr_mutex.lock();
}

void trace_mutex_release(void) {
    tr_mutex.unlock();
}

void trace_init(void) {

    mbed_trace_mutex_wait_function_set(trace_mutex_wait);
    mbed_trace_mutex_release_function_set(trace_mutex_release);

    mbed_trace_init();
}

// Works for simple single-connection use cases
class ConnectionManager : public Gap::EventHandler {

public:

    virtual ~ConnectionManager() { }

    void onConnectionComplete(const ble::ConnectionCompleteEvent &event) {
        connection = event.getConnectionHandle();
        flags.set(EVENT_FLAGS_CONNECTED);
    }
};

ChainableGapEventHandler gap_eh;

void ble_main(void) {

    BLE &ble_interface = BLE::Instance();
    events::EventQueue ble_queue;

    BLEProcess ble_process(ble_queue, ble_interface);

    static ChainableGattServerEventHandler eh;
    eh.addEventHandler(&uart_service);
    ble_interface.gattServer().setEventHandler(&eh);

    static ConnectionManager conn_mgr;
    gap_eh.addEventHandler(&conn_mgr);
    gap_eh.addEventHandler(&uart_service);
    // TODO - rework this, ble process currently adds itself and sets Gap handler

    ble_process.on_init(callback(&uart_service, &UARTService::start));

    // TESTING
    //ble_queue.call_every(3s, mbed::callback(test_write));

    // bind the event queue to the ble interface, initialize the interface
    // and start advertising
    ble_process.start();
}

void print_heap_stats(void) {

    mbed_stats_heap_t stats;
    mbed_stats_heap_get(&stats);

    tr_info("heap stats:\r\n"
            "current size: %lu\r\n"
            "max size: %lu\r\n"
            "total size: %lu\r\n"
            "reserved size: %lu\r\n"
            "alloc cnt: %lu\r\n"
            "alloc fail cnt: %lu\r\n"
            "overhead size: %lu\r\n",
            stats.current_size,
            stats.max_size,
            stats.total_size,
            stats.reserved_size,
            stats.alloc_cnt,
            stats.alloc_fail_cnt,
            stats.overhead_size);

}

int main() {

    trace_init();

    ble_thread.start(mbed::callback(ble_main));

    while(true) {

        // Print heap stats before
        print_heap_stats();

        // Wait until we're connected
        flags.wait_any_for(EVENT_FLAGS_CONNECTED, rtos::Kernel::wait_for_u32_forever);

        tr_info("connected, getting BleSerial handle");

        ser = uart_service.get_ble_serial_handle(connection);

        // Print heap stats after
        print_heap_stats();

        while(!ser->is_shutdown()) {
            // Read 10 bytes and echo them back...
            uint8_t c = 0;
            ssize_t result = ser->read(&c, 10); // echo it back
            if(result == -ESHUTDOWN) {
                tr_info("BleSerial shutdown while reading!");
                ser = nullptr;
            } else {
                result = ser->write(&c, result);
                if(result == -ESHUTDOWN) {
                    tr_info("BleSerial shutdown while writing!");
                    ser = nullptr;
                }
            }
        }
    }
}
