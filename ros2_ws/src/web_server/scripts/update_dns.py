#!/usr/bin/env python3
import os
import sys
import socket
import subprocess
import boto3
from botocore.exceptions import BotoCoreError, ClientError

def get_local_ip():
    """
    Finds the primary active local IP address of the machine by establishing
    a dummy UDP socket connection to an external address (Google's Public DNS).
    This doesn't send any traffic but forces the OS to choose the correct interface.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # 8.8.8.8 is used as a dummy endpoint to trace local interface routing
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    except Exception as e:
        print(f"[-] Failed to determine local IP dynamically: {e}", file=sys.stderr)
        # Attempt fallback to hostname-based resolution
        try:
            ip = socket.gethostbyname(socket.gethostname())
        except Exception:
            ip = '127.0.0.1'
    finally:
        s.close()
    return ip

def bootstrap_self_signed(domain_name):
    """
    Generates temporary self-signed SSL certificates if none exist.
    This prevents Nginx from failing to start up before Certbot runs.
    """
    cert_dir = f"/etc/letsencrypt/live/{domain_name}"
    cert_file = f"{cert_dir}/fullchain.pem"
    key_file = f"{cert_dir}/privkey.pem"

    if not os.path.exists(cert_file) or not os.path.exists(key_file):
        print(f"[*] SSL certificates not found at {cert_dir}. Generating temporary self-signed certificates to bootstrap Nginx...")
        os.makedirs(cert_dir, exist_ok=True)
        try:
            subprocess.run([
                'openssl', 'req', '-x509', '-nodes', '-days', '365',
                '-newkey', 'rsa:2048',
                '-keyout', key_file,
                '-out', cert_file,
                '-subj', f"/CN={domain_name}"
            ], check=True)
            print("[+] Temporary self-signed certificates generated successfully.")
        except Exception as e:
            print(f"[-] Failed to generate self-signed certificates: {e}", file=sys.stderr)
    else:
        print(f"[+] Existing certificates found at {cert_dir}. Skipping self-signed bootstrap.")

def main():
    hosted_zone_id = os.environ.get('HOSTED_ZONE_ID')
    domain_name = os.environ.get('DOMAIN_NAME')

    if not domain_name:
        print("[-] Error: DOMAIN_NAME environment variable is not set.", file=sys.stderr)
        sys.exit(1)

    if not hosted_zone_id:
        print("[-] Error: HOSTED_ZONE_ID environment variable is not set.", file=sys.stderr)
        sys.exit(1)

    print(f"[*] Detecting local IP address...")
    local_ip = get_local_ip()
    print(f"[+] Active local IP detected: {local_ip}")

    if local_ip == '127.0.0.1':
        print("[-] Warning: Resolved to localhost loopback IP. Skipping Route 53 update to avoid routing issues.", file=sys.stderr)
        sys.exit(1)

    print(f"[*] Updating Route 53 record: {domain_name} -> {local_ip}")
    try:
        client = boto3.client('route53')
        response = client.change_resource_record_sets(
            HostedZoneId=hosted_zone_id,
            ChangeBatch={
                'Comment': 'Auto-updating local IP for ReloBot',
                'Changes': [
                    {
                        'Action': 'UPSERT',
                        'ResourceRecordSet': {
                            'Name': domain_name,
                            'Type': 'A',
                            'TTL': 300,
                            'ResourceRecords': [{'Value': local_ip}]
                        }
                    }
                ]
            }
        )
        change_info = response.get('ChangeInfo', {})
        print(f"[+] Route 53 update requested successfully. Status: {change_info.get('Status')}")
    except (BotoCoreError, ClientError) as e:
        print(f"[-] AWS Route 53 API Error: {e}", file=sys.stderr)
        # We don't exit immediately on DNS update error, so we can still try to bootstrap SSL if needed.

    # Bootstrap self-signed certificate if needed so Nginx starts successfully
    bootstrap_self_signed(domain_name)

if __name__ == '__main__':
    main()
