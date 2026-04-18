import asyncio
from bleak import BleakClient, BleakScanner

DEVICE_NAME = "Rocket-FC"
CHAR_UUID = "abcdefab-1234-1234-1234-abcdefabcdef"

async def send_command(client: BleakClient, cmd: str):
    await client.write_gatt_char(CHAR_UUID, cmd.encode())
    print(f"Sent: {cmd}")

async def main():
    print(f"Scanning for '{DEVICE_NAME}'...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)
    if device is None:
        print("Device not found. Is it powered on and advertising?")
        return

    print(f"Found {device.name} ({device.address}), connecting...")
    async with BleakClient(device) as client:
        print("Connected! Type a command (CALIBRATE / RESET) or 'q' to quit.")
        while True:
            cmd = input("> ").strip().upper()
            if cmd == "Q":
                break
            if cmd in ("CALIBRATE", "RESET"):
                await send_command(client, cmd)
            else:
                print("Unknown command. Try CALIBRATE or RESET.")

asyncio.run(main())
