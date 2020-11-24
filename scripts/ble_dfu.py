import argparse
import asyncio
import signal
import re
from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice
from bleak.backends.service import BleakGATTService, BleakGATTCharacteristic, BleakGATTDescriptor
from bleak.uuids import *
from typing import Union
import time

# TODO split this up into modules

# UUIDs for DFUService
DFUServiceBaseUUID = '53880000-65fd-4651-ba8e-91527f06c887'
DFUServiceSlotUUID = '53880001-65fd-4651-ba8e-91527f06c887'
DFUServiceOffsetUUID = '53880002-65fd-4651-ba8e-91527f06c887'
DFUServiceBinaryStreamUUID = '53880003-65fd-4651-ba8e-91527f06c887'
DFUServiceControlUUID = '53880004-65fd-4651-ba8e-91527f06c887'
DFUServiceStatusUUID = '53880005-65fd-4651-ba8e-91527f06c887'
DeviceInformationServiceUUID = 0x180A
FirmwareRevStringUUID = 0x2A26
CharacteristicUserDescriptionDescriptorUUID = 0x2901

# DFU Control Characteristic Bitfields
DFUControlEnableBit = (1 << 0)
DFUControlCommitBit = (1 << 1)
DFUControlDeltaBit = (1 << 2)
DFUControlFlowControlBit = (1 << 7)

# DFU Status Characteristic Bitfields
DFUStatusUnexpectedSequenceIDBit = (1 << 7)

# TODO implement some generic utility scanner activity script that can be used by all BLE scripts...


def chunks(l, n):
    for i in range(0, len(l), n):
        yield l[i:i+n]


def short_bt_sig_uuid_to_long(short: Union[str, int]) -> str:
    """
    Returns the long-format UUID of a Bluetooth SIG-specified short UUID
    :param short: Short-form BT SIG-specified UUID (as a hex string or a raw integer)
    :return: The fully-qualified long-form BT SIG UUID
    """
    hex_form = short
    if type(short) == str:
        pass
    elif type(short) == int:
        hex_form = hex(short)[2:]
    else:
        raise TypeError

    return f'0000{hex_form}-0000-1000-8000-00805f9b34fb'


def get_chunk_n(data, chunksize: int, n: int):
    start = chunksize*n
    end = chunksize*(n+1)
    if len(data) <= start:
        return []
    if len(data) <= end:
        return data[start:]

    return data[start:end]


class DFUService:

    def __init__(self, client: BleakClient, svc: BleakGATTService, dev_info_svc: BleakGATTService = None,
                 mtu: int = 384):  # TODO MTU argument from command line
        self.client = client
        self.svc = svc
        self.dev_info_svc = dev_info_svc
        self.mtu = mtu
        self.slot_char = svc.get_characteristic(DFUServiceSlotUUID)
        self.offset_char = svc.get_characteristic(DFUServiceOffsetUUID)
        self.bds_char = svc.get_characteristic(DFUServiceBinaryStreamUUID)
        self.control_char = svc.get_characteristic(DFUServiceControlUUID)
        self.status_char = svc.get_characteristic(DFUServiceStatusUUID)
        self.fw_rev_char = svc.get_characteristic(short_bt_sig_uuid_to_long(FirmwareRevStringUUID))
        self.fw_rev_desc = None
        self.fw_rev_str = ''
        self.fw_rev_desc_str = ''

        self.sequence_num = 0
        self.sequence_rollover_ctr = 0

        # Asynchronous event states
        self.mutex = asyncio.Lock()
        self.new_status_evt = asyncio.Event()
        self.new_ctrl_evt = asyncio.Event()
        self.status_val = 0
        self.control_val = 0

        self.retransmissions = 0

        self.total_sent = 0
        self.start_time = 0

        # If the Firmware Revision String characteristic does not exist in the server, it must exist in the
        # Gatt Server's Device Information Service. Otherwise it is an error.
        if not self.fw_rev_char and self.dev_info_svc:
            self.fw_rev_char = self.dev_info_svc.get_characteristic(short_bt_sig_uuid_to_long(FirmwareRevStringUUID))

        if not self.fw_rev_char:
            print(f'error: Firmware Revision String Characteristic does not exist on Gatt Server')
            raise AssertionError

        if not self.slot_char or not self.offset_char or not self.bds_char or not self.status_char or not self.control_char:
            print(f'error: mandatory characteristics missing from DFU Service')
            raise AssertionError

    async def get_firmware_revision(self) -> (str, Union[str, None]):
        """
        Gets the firmware revision from the optional Firmware Revision String characteristic of the DFU Service
        or from the Gatt Server's Device Information Service.

        If the Firmware Revision String characteristic exists on the DFU Service, it may have an optional
        Characteristic User Description descriptor (CUDD) that uniquely identifies which embedded device the
        target firmware is executed on. This is required if the Gatt Server has multiple DFU Service instances

        :return: tuple (fw_rev, device_description)
        """
        self.fw_rev_str = await self.client.read_gatt_char(self.fw_rev_char)
        self.fw_rev_desc = self.fw_rev_char.get_descriptor(
            short_bt_sig_uuid_to_long(CharacteristicUserDescriptionDescriptorUUID))
        if self.fw_rev_desc:
            self.fw_rev_desc_str = await self.client.read_gatt_descriptor(self.fw_rev_desc.handle)

        return self.fw_rev_str, self.fw_rev_desc_str

    def handle_status_notification(self, char_handle: int, data: bytearray):
        self.status_val = data[0]
        print(f'Status notification: {hex(self.status_val)}')
        self.new_status_evt.set()

    def handle_control_notification(self, char_handle: int, data: bytearray):
        self.control_val = data[0]
        print(f'Control notification: {hex(self.control_val)}')
        self.new_ctrl_evt.set()

    async def perform_dfu(self, bin_file: str, slot: int = 0):
        self.total_sent = 0
        self.start_time = time.time()

        try:

            # Set the selected slot before enabling DFU
            await self.client.write_gatt_char(self.slot_char, bytearray([slot]), response=True)
            if slot != 0:
                await self.new_status_evt.wait()
                self.new_status_evt.clear()
            print('Slot written...')

            await asyncio.sleep(0.25)

            # Subscribe to notifications on Control and Status characteristics
            await self.client.start_notify(self.control_char, self.handle_control_notification)
            await self.client.start_notify(self.status_char, self.handle_status_notification)
            print('Notifications subscribed...')

            # Enable DFU
            await self.client.write_gatt_char(self.control_char, bytearray([DFUControlEnableBit]), response=True)
            await self.new_status_evt.wait()
            self.new_status_evt.clear()
            print('DFU enabled')

            # Open the binary file and start writing it out
            with open(bin_file, 'rb') as f:
                data = f.read()
                send_complete = False
                while not send_complete:
                    # Check notifications
                    if self.new_status_evt.is_set():  # Check status first (error or retransmission request)
                        self.new_status_evt.clear()
                        if self.status_val & DFUStatusUnexpectedSequenceIDBit:
                            self.retransmissions += 1
                            requested_seq_num = self.status_val & 0x7F
                            # Make sure to handle rollover cases
                            if requested_seq_num > self.sequence_num:
                                self.sequence_rollover_ctr -= 1
                            self.sequence_num = requested_seq_num
                            print(f'Retransmission requested from sequence number: {self.sequence_num}'
                                  f' (retransmissions: {self.retransmissions})')
                        elif self.status_val > 1:
                            print(f'DFU Status Error: {self.status_val}')
                            # TODO raise a real exception :)
                            raise Exception
                    if self.new_ctrl_evt.is_set():  # Check control event
                        self.new_ctrl_evt.clear()
                        while self.control_val & DFUControlFlowControlBit:  # Flow control bit is set, pause transmission
                            print('Flow control - paused')
                            await self.new_ctrl_evt.wait()  # Wait until control is updated again
                            self.new_ctrl_evt.clear()
                        # Check enable bit
                        if not self.control_val & DFUControlEnableBit:
                            print(f'DFU has been aborted by peer')
                            # TODO raise a better exception that just exception :)
                            raise Exception

                    # Send the next packet
                    print(f'Sending packet # {(128*self.sequence_rollover_ctr) + self.sequence_num}'
                          f'(total bytes: {self.total_sent}, time: {time.time() - self.start_time})')
                    packet = bytearray([self.sequence_num])
                    data_chunk = get_chunk_n(data, self.mtu, (128 * self.sequence_rollover_ctr) + self.sequence_num)
                    packet += bytearray(data_chunk)
                    if len(data_chunk) < self.mtu:
                        # TODO what happens if the last send fails? We will miss the retransmission request!
                        send_complete = True
                        if len(data_chunk) == 0:
                            continue
                    try:
                        await asyncio.wait_for(self.client.write_gatt_char(self.bds_char, packet, False),
                                               timeout=0.025)
                        await asyncio.sleep(0.075)
                        self.total_sent += len(data_chunk)
                    except asyncio.exceptions.TimeoutError as e:
                        print(f'Warning - timeout error occured while writing bds char')
                        await asyncio.sleep(0.1)

                    self.sequence_num += 1
                    if self.sequence_num >= 128:
                        self.sequence_rollover_ctr += 1
                        self.sequence_num = 0
            await self.terminate()
        except asyncio.CancelledError as e:
            print("Cancelled!")
        finally:
            await self.client.disconnect()

    async def terminate(self):
        print(f'Stopping notifications...')
        await self.client.stop_notify(self.status_char)
        await self.client.stop_notify(self.control_char)


async def scan_for_name(name) -> BLEDevice:
    """
    Scans for a specifically-named BLE device nearby
    :param name: Name of device to scan for
    :return: BLEDevice handle to found device, None if not found
    """
    # TODO make this use start/stop to end early when target is found
    devices = await BleakScanner.discover(timeout=5.0)
    for device in devices:
        if name == device.name:
            return device

    return None


def looks_like_mac(name: str) -> bool:
    """
    Checks if the given name looks like a MAC address or not
    :param name: Name to check
    :return: True if name looks likele a MAC address, false otherwise
    """
    mac_regex = '^([0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2})$'
    if re.search(mac_regex, name):
        return True
    else:
        return False


async def main():
    parser = argparse.ArgumentParser(description='Perform a wireless firmware update over BLE using '
                                                 'the Mbed DFUService')
    parser.add_argument('device', help='BLE device address OR name to connect to '
                                       '(address in hex format, can be separated by colons)')
    parser.add_argument('update_bin', help='Binary file to update the target device with')
    parser.add_argument('-v', dest='verbose', action='store_true',
                        help='Increase verbosity of output')
    parser.add_argument('-s', '--slot', dest='slot', default=0, type=int,
                        help='Image slot to update (defaults to 0)')
    parser.add_argument('--device-description', dest='device_description',
                        help='If a Gatt server has multiple DFU service instances available, the device description'
                             'uniquely identifies what device within the system the firmware update will be executed'
                             'on. This is a string parameter and must match the '
                             'intended target exactly (case insensitive)!')

    # TODO maybe add a flag to have a delta binary directory where multiple versions can be placed and automatically
    # TODO how to identify binary version? Parse mcuboot header?
    # used by the dfu client based on the target's firmware version.
    """
    parser.add_argument('--delta-binary', dest='delta_bin',
                        help='Delta binary file. The delta binary will be compared byte-for-byte with the provided '
                             'update binary. Any sections that are matching will be skipped when transferring the'
                             'update binary to the target. This can sometimes help shorten DFU times if only small'
                             'portions of the application binary have changed. This file MUST be identical to the'
                             'firmware on the device!')
    """

    args = parser.parse_args()

    # Check if the device argument looks like a MAC address
    addr = None
    name = None
    if looks_like_mac(args.device):
        addr = args.device
    else:
        name = args.device

    if not addr:
        print(f'Scanning for nearby device named {name}...')
        # Scan to find by name
        device = await scan_for_name(name)
        if not device:
            print(f'Could not find device {name} nearby')
            raise AssertionError
        else:
            addr = device.address
            print(f'Found! Address: {addr}')

    print(f'Attempting to connect to device {addr}...')

    # TODO disconnected callback
    client = BleakClient(addr)
    connected = await client.connect()
    if not connected:
        print(f'Failed to connect to device {addr}')
        raise AssertionError

    print(f'Connected to {addr}', f'({name})' if name else '')

    if args.verbose:
        for svc in client.services:
            print(f'Service: {svc.uuid}')
            for char in svc.characteristics:
                print(f'\tCharacteristic: {char.uuid} ({", ".join(char.properties)})')
                for desc in char.descriptors:
                    print(f'\t\tDescriptor: {desc.uuid}')

    # TODO bleak does not currently support multiple services with the same UUID!
    dfu_svc_handle = client.services.get_service(DFUServiceBaseUUID)
    if not dfu_svc_handle:
        print(f'DFU Service not found on server!')
        raise AssertionError

    dev_info_svc = client.services.get_service(DeviceInformationServiceUUID)

    dfu_svc = DFUService(client, dfu_svc_handle, dev_info_svc)

    # Get the firmware version and device string
    fw_rev, dev_str = await dfu_svc.get_firmware_revision()

    print(f'DFU Service found with firmware rev {fw_rev.decode("utf-8")}',
          f'for device {dev_str.decode("utf-8")}' if dev_str != '' else '')

    await dfu_svc.perform_dfu(args.update_bin, args.slot)

    await dfu_svc.terminate()
    await client.disconnect()


async def shutdown(sig, async_loop):
    """Cleanup tasks tied to the service's shutdown."""
    print(f'Received exit signal {sig.name}...')
    print(f'Number of total tasks: {len(asyncio.all_tasks())}')
    tasks = [t for t in asyncio.all_tasks() if t is not
             asyncio.current_task()]

    print(f'Cancelling {len(tasks)} outstanding tasks')
    [task.cancel() for task in tasks]
    await asyncio.gather(*tasks, return_exceptions=True)
    async_loop.stop()


if __name__ == '__main__':
    loop = asyncio.get_event_loop()

    # Gracefully shutdown with signal handlers
    # TODO windows-supported graceful shutdown
    signals = (signal.SIGHUP, signal.SIGTERM, signal.SIGINT)
    for s in signals:
        loop.add_signal_handler(
            s, lambda s=s: asyncio.create_task(shutdown(s, loop))
        )

    try:
        main_task = loop.create_task(main())
        loop.run_forever()

    except Exception as e:
        raise e

    finally:
        loop.close()
