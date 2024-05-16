import argparse
import asyncio
import logging
import time
import fire

from bleak import BleakClient, BleakScanner
from bleak.uuids import normalize_uuid_16, uuid16_dict

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s %(name)s - [%(levelname)s] > %(message)s"
)
logger = logging.getLogger(__name__)
logger.level = logging.DEBUG

name_prefix = "C26_b108"


def _filter_device(device, advertisement_data):
    if not device.name:
        return False
    result = device.name.startswith(name_prefix)
    if result:
        logger.debug(f"found device: {device}, {advertisement_data}")
    return result


async def _read_etag(client):
    host_epoch = int(round(time.time()))

    # read current time
    value = await client.read_gatt_char(normalize_uuid_16(0xFFF1))
    epoch = int.from_bytes(value, byteorder="little", signed=False)

    # read time zone
    value = await client.read_gatt_char(normalize_uuid_16(0xFFF2))
    tz_min = int.from_bytes(value, byteorder="little", signed=True)
    logger.info(
        f"# host ts: {host_epoch}, etag ts: {epoch}, diff ({epoch - host_epoch})s, tz: {tz_min // 60}h"
    )

    # battery
    value = await client.read_gatt_char(normalize_uuid_16(0xFFF3))
    battery = int.from_bytes(value, byteorder="little", signed=False)

    # temperature
    value = await client.read_gatt_char(normalize_uuid_16(0xFFF4))
    temp = int.from_bytes(value, byteorder="little", signed=True)
    logger.info(f"# battery: {battery}mV, temperature: {temp}°C")

    # RTC collaborate
    value = await client.read_gatt_char(normalize_uuid_16(0xFFF5))
    rtc = int.from_bytes(value, byteorder="little", signed=False)
    logger.info(f"# rtc: {rtc}")


async def _set_time(client):
    epoch = int(round(time.time()))
    logger.info(f"setting time: {epoch}")

    # set current time
    await client.write_gatt_char(
        normalize_uuid_16(0xFFF1), epoch.to_bytes(4, byteorder="little")
    )


async def _change_mode(client, mode: int):
    # change mode
    await _do_cmd(client, 2, mode)


EPD_CMD_CLR = 1
EPD_CMD_MODE = 2
EPD_CMD_BUF = 3
EPD_CMD_BUF_CONT = 4
EPD_CMD_LUT = 5
EPD_CMD_RST = 6
EPD_CMD_BW = 7
EPD_CMD_RED = 8
EPD_CMD_DP = 9
EPD_CMD_FILL = 10
EPD_CMD_BUF_PUT = 11
EPD_CMD_BUF_GET = 12
EPD_CMD_SNV_WRITE = 13
EPD_CMD_SNV_READ = 14
EPD_CMD_SAVE_CFG = 15


async def _do_cmd(client, cmd, payload=None):
    data = [cmd]

    if cmd in [EPD_CMD_MODE, EPD_CMD_DP]:
        data.append(payload)
    elif cmd in [EPD_CMD_CLR, EPD_CMD_RST, EPD_CMD_BW, EPD_CMD_RED]:
        # no need payload
        pass
    elif cmd == EPD_CMD_BUF:
        chunk_size = 60 
        for i in range(0, len(payload), chunk_size):
            chunk = payload[i : i + chunk_size]
            cmd = cmd if i == 0 else EPD_CMD_BUF_CONT
            logger.debug(f"sending chunk={i+len(chunk)} of data={len(payload)}")
            await client.write_gatt_char(
                normalize_uuid_16(0xFFFE), bytes([cmd] + chunk)
            )
        return
    else:
        raise Exception(f"unsupported cmd: {cmd}, payload: {payload}")

    logger.debug(f"do cmd: {data}")
    await client.write_gatt_char(normalize_uuid_16(0xFFFE), bytes(data))


async def _upload_image_raw_data(client, bw_data, red_data):
    await _do_cmd(client, EPD_CMD_RST)
    time.sleep(2)

    if bw_data:
        await _do_cmd(client, EPD_CMD_BUF, bw_data)
        await _do_cmd(client, EPD_CMD_BW)
    if red_data:
        await _do_cmd(client, EPD_CMD_BUF, red_data)
        await _do_cmd(client, EPD_CMD_RED)
    # display with lut 0
    await _do_cmd(client, EPD_CMD_DP, 0)

    time.sleep(15)


# 296x128
test_bw_data = [0] * (128 * 296 // 8 // 4) + [255] * (128 * 296 // 8 // 4 * 3)
test_red_data = [0] * (128 * 296 // 8 // 4 * 3) + [255] * (128 * 296 // 8 // 4)


async def run_ble_client(timeout=30):
    logger.info("starting scan...")

    device = await BleakScanner.find_device_by_filter(
        filterfunc=_filter_device,
        timeout=timeout,
    )
    if device is None:
        logger.error("could not find device")
        raise Exception("device not found")

    logger.info("connecting to device...")

    async with BleakClient(device) as client:
        logger.info("connected")

        # await _read_etag(client)
        # await _set_time(client)
        # await _change_mode(client, 1)
        await _upload_image_raw_data(client, test_bw_data, test_red_data)

        logger.info("disconnection...")
        await client.disconnect()

    logger.info("disconnected")


if __name__ == "__main__":
    fire.Fire()
