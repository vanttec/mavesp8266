import ipaddress
import argparse
import asyncio
import aiohttp

def create_url(address: ipaddress.ip_address):
    return f'http://{address}/info.json'

semaphore = asyncio.Semaphore(200)
async def check_host(async_session: aiohttp.ClientSession, address: ipaddress.ip_address):
    async with semaphore:
        try:
            async with async_session.get(url=create_url(address), timeout=0.5) as resp:
                return (resp.status, await resp.json())
        except (aiohttp.ClientError, asyncio.TimeoutError):
            return None

async def scan_network(network: ipaddress.ip_network):
    async with aiohttp.ClientSession() as async_session:
        tasks = [
            asyncio.create_task(coro=check_host(
                async_session=async_session,
                address=address
            )) for address in network
        ]
        host_list = dict(zip(list(network), await asyncio.gather(*tasks, return_exceptions=False)))
        # Only get addresses with 200 status code response
        filtered_list = dict(
            filter(lambda item: item[1] is not None and item[1][0] == 200, host_list.items())
        )
        return filtered_list

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('ip_network')
    args = parser.parse_args()
    network = ipaddress.ip_network(args.ip_network)
    print(f"Scanning {network} with {len(list(network))} hosts.")
    scan_result = asyncio.run(scan_network(network))
    print(f"Found {len(scan_result)} hosts")
    print(scan_result)
