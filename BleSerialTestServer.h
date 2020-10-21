/*
 * BleSerialTestServer.h
 *
 *  Created on: Oct 21, 2020
 *      Author: gdbeckstein
 */

#ifndef BLESERIALTESTSERVER_H_
#define BLESERIALTESTSERVER_H_

#include "utest/utest.h"
#include "unity/unity.h"

#include "events/EventQueue.h"

#include "ble/BLE.h"
#include "ble/Gap.h"
#include "gap/ChainableGapEventHandler.h"
#include "gatt/ChainableGattServerEventHandler.h"

#include "UARTService.h"

static const uint16_t MAX_ADVERTISING_PAYLOAD_SIZE = 50;
static const char DEVICE_NAME[] = "TestUART";

using namespace utest::v1;

using namespace ble;

class BleSerialTestServer : public Gap::EventHandler,
                      public GattServer::EventHandler
{

public:

    BleSerialTestServer() :
        _ble_interface(BLE::Instance()),
        _gap(_ble_interface.gap()),
        _gatt_srv(_ble_interface.gattServer()),
        _adv_data_builder(_adv_buffer),
        _adv_handle(ble::LEGACY_ADVERTISING_HANDLE) {

    }

    virtual ~BleSerialTestServer() {

    }

    /**
     * Initialize the ble interface, configure it and start advertising.
     */
    void start() {

        /* Problem with test configuration if we reinitilize... */
        TEST_ASSERT(_ble_interface.hasInitialized());

        /* Setup event handlers */
        gap_eh_chain.addEventHandler(this);
        gatt_srv_eh_chain.addEventHandler(this);
        _gap.setEventHandler(&gap_eh_chain);
        _gatt_srv.setEventHandler(&gatt_srv_eh_chain);

        _ble_interface.onEventsToProcess(
                makeFunctionPointer(this,  &BleSerialTestServer::schedule_ble_events));

        ble_error_t error = _ble_interface.init(
                this, &BleSerialTestServer::on_init_complete);

        TEST_ASSERT_EQUAL(BLE_ERROR_NONE, error);

        _event_queue.dispatch_forever();

        // We should never reach this
        TEST_ASSERT(1);

    }

private:

    void on_init_complete(BLE::InitializationCompleteCallbackContext *event) {

        TEST_ASSERT_EQUAL(BLE_ERROR_NONE, event->error);

        uart_svc.start(_ble_interface);

        _event_queue.call(this, &BleSerialTestServer::start_advertising);

    }

    void start_advertising() {

        ble_error_t error;

        ble::AdvertisingParameters adv_params;

        error = _gap.setAdvertisingParameters(_adv_handle,  adv_params);

        TEST_ASSERT_EQUAL(BLE_ERROR_NONE, error);

        _adv_data_builder.clear();
        _adv_data_builder.setFlags();
        _adv_data_builder.setName(DEVICE_NAME);

        error = _gap.setAdvertisingPayload(
                _adv_handle, _adv_data_builder.getAdvertisingData());

        TEST_ASSERT_EQUAL(BLE_ERROR_NONE, error);

        error = _gap.startAdvertising(_adv_handle);

        TEST_ASSERT_EQUAL(BLE_ERROR_NONE, error);

    }

    void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *event) {
        _event_queue.call(mbed::callback(&event->ble, &BLE::processEvents));
    }

public:

    ChainableGapEventHandler gap_eh_chain;
    ChainableGattServerEventHandler gatt_srv_eh_chain;

    UARTService uart_svc;

private:

    events::EventQueue _event_queue;
    BLE &_ble_interface;
    ble::Gap &_gap;
    ble::GattServer &_gatt_srv;

    uint8_t _adv_buffer[MAX_ADVERTISING_PAYLOAD_SIZE];

    ble::AdvertisingDataBuilder _adv_data_builder;
    ble::advertising_handle_t _adv_handle;

};



#endif /* BLESERIALTESTSERVER_H_ */
