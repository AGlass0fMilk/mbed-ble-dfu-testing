/*
 * Copyright (c) 2020 Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 */

#if !MBED_TEST_MODE

#include "rtos/Thread.h"
#include "rtos/ThisThread.h"

#include "BleSerialTestServer.h"

#include "greentea-client/test_env.h"
#include "unity/unity.h"

#include <chrono>

using namespace std::chrono;

static BleSerialTestServer test_srv;

void ble_thread_main(void) {
    // This should never return
    test_srv.start();
}

int main() {

    // Setup BleSerialTestServer

    // Spin off BLE thread
    rtos::Thread ble_thread(osPriorityNormal, MBED_CONF_RTOS_THREAD_STACK_SIZE,
            NULL, "ble-thread");

    ble_thread.start(ble_thread_main);

    GREENTEA_SETUP(60, "ble_serial_test");

    while(true) {

        // TODO receive from each connection
        // TODO transmit to each connection
        // MTU update test?
        // Non-blocking/Blocking tests?
        // Memory leakage test
        // Disconnect while writing/reading
        rtos::ThisThread::sleep_for(1s);

    }

    return 0;
}

//static control_t hello_world_test(const size_t call_count)
//{
//    // send a message to the host runner
//    greentea_send_kv("init", "hello");
//
//    // wait until we get a message back
//    // if this takes too long, the timeout will trigger, so no need to handle this here
//    char _key[20], _value[128];
//    while (1) {
//        greentea_parse_kv(_key, _value, sizeof(_key), sizeof(_value));
//
//        // check if the key equals init, and if the return value is 'world'
//        if (strcmp(_key, "init") == 0) {
//            TEST_ASSERT_EQUAL(0, strcmp(_value, "world"));
//            break;
//        }
//    }
//
//    return CaseNext;
//}

#endif
