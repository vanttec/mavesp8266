import ipaddress
import argparse
import asyncio
import tqdm
from subprocess import STDOUT, check_output
import requests
import aiohttp

from find_esp import scan_network

semaphore = asyncio.Semaphore(100)


def create_blink_url(address: ipaddress.ip_address):
    return f"http://{address}/blink"


async def send_blink_request(
    async_session: aiohttp.ClientSession, address: ipaddress.ip_address
):
    async with semaphore:
        async with async_session.get(
            url=create_blink_url(address), timeout=0.5
        ) as resp:
            return None


async def blink(hosts):
    async with aiohttp.ClientSession() as async_session:
        tasks = [
            asyncio.create_task(
                send_blink_request(async_session=async_session, address=address)
            )
            for address in hosts
        ]
        await asyncio.gather(*tasks)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("ip_network")
    args = parser.parse_args()

    network = ipaddress.ip_network(args.ip_network)
    print(f"Scanning {network} with {len(list(network))} hosts.")
    scan_result = asyncio.run(scan_network(network))
    print(f"Found {len(scan_result)} hosts")
    print([str(ip) for ip in scan_result.keys()])

    asyncio.run(blink(hosts=scan_result.keys()))
