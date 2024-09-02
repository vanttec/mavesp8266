import ipaddress
import argparse
import asyncio
import tqdm
from subprocess import STDOUT, check_output

from find_esp import scan_network

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("ip_network")
    parser.add_argument("-e", "--env", default="esp07")
    args = parser.parse_args()

    network = ipaddress.ip_network(args.ip_network)
    print(f"Scanning {network} with {len(list(network))} hosts.")
    scan_result = asyncio.run(scan_network(network))
    print(f"Found {len(scan_result)} hosts")
    print([str(ip) for ip in scan_result.keys()])

    for address in tqdm.tqdm(scan_result.keys()):
        cmd = [
            "pio",
            "run",
            "-d",
            "..",
            "-e",
            args.env,
            "--target",
            "upload",
            "--upload-port",
            str(address),
        ]
        print(f"Running... {' '.join(cmd)}")
        output = check_output(cmd, stderr=STDOUT, timeout=60)
