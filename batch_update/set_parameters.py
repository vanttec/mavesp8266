import ipaddress
import argparse
import asyncio
import tqdm
from subprocess import STDOUT, check_output
import requests

from find_esp import scan_network


class KeyValuePairAction(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        result = {}
        for item in values:
            key, value = item.split("=")
            result[key] = value
        setattr(namespace, self.dest, result)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("ip_network")
    parser.add_argument(
        "parameters",
        nargs="+",
        action=KeyValuePairAction,
        help="Key-Value pairs in the format key=value"
    )

    args = parser.parse_args()

    network = ipaddress.ip_network(args.ip_network)
    print(f"Scanning {network} with {len(list(network))} hosts.")
    scan_result = asyncio.run(scan_network(network))
    print(f"Found {len(scan_result)} hosts")
    print([str(ip) for ip in scan_result.keys()])

    print(args.parameters)

    for address in tqdm.tqdm(scan_result.keys()):
        # TODO using sync requests instead of async, so what?
        url = f'http://{str(address)}/setparameters'
        requests.post(url, data=args.parameters)
