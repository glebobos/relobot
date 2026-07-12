#!/usr/bin/env python3
import os
import sys
import json
import argparse
import boto3
from botocore.exceptions import BotoCoreError, ClientError

def get_hosted_zone_id(route53_client, zone_name):
    """
    Looks up the Route 53 hosted zone ID for the specified domain name.
    """
    # Route 53 zones are typically returned with a trailing dot
    search_name = zone_name.strip().lower()
    if not search_name.endswith('.'):
        search_name += '.'

    print(f"[*] Querying Route 53 hosted zones for '{zone_name}'...")
    try:
        paginator = route53_client.get_paginator('list_hosted_zones')
        for page in paginator.paginate():
            for zone in page.get('HostedZones', []):
                if zone.get('Name').lower() == search_name:
                    # Zone ID is usually in format '/hostedzone/Z123456789'
                    zone_id = zone.get('Id').split('/')[-1]
                    print(f"[+] Found Hosted Zone ID: {zone_id}")
                    return zone_id
    except (BotoCoreError, ClientError) as e:
        print(f"[-] Route 53 API Error: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"[-] Error: Could not find hosted zone '{zone_name}' in your Route 53 account.", file=sys.stderr)
    print("[-] Please ensure the hosted zone is active in Route 53 and that you have permissions to view it.", file=sys.stderr)
    sys.exit(1)

def create_iam_policy(iam_client, policy_name, hosted_zone_id):
    """
    Creates a minimal Route 53 change policy restricted to the hosted zone ID.
    """
    policy_document = {
        "Version": "2012-10-17",
        "Statement": [
            {
                "Effect": "Allow",
                "Action": [
                    "route53:ListHostedZones",
                    "route53:GetChange"
                ],
                "Resource": "*"
            },
            {
                "Effect": "Allow",
                "Action": [
                    "route53:ChangeResourceRecordSets",
                    "route53:GetHostedZone",
                    "route53:ListResourceRecordSets"
                ],
                "Resource": f"arn:aws:route53:::hostedzone/{hosted_zone_id}"
            }
        ]
    }

    print(f"[*] Creating IAM Policy '{policy_name}'...")
    try:
        response = iam_client.create_policy(
            PolicyName=policy_name,
            PolicyDocument=json.dumps(policy_document),
            Description=f"Minimal Route 53 access for ReloBot SSL automation on zone {hosted_zone_id}"
        )
        policy_arn = response['Policy']['Arn']
        print(f"[+] Created IAM Policy. ARN: {policy_arn}")
        return policy_arn
    except ClientError as e:
        if e.response['Error']['Code'] == 'EntityAlreadyExists':
            # Retrieve existing policy ARN
            # We assume it already exists; construct the ARN manually based on account ID
            # Or query the policy list. Querying is safer.
            print(f"[*] Policy '{policy_name}' already exists. Retrieving existing policy ARN...")
            try:
                # Find account ID by parsing caller identity
                sts = boto3.client('sts')
                account_id = sts.get_caller_identity()['Account']
                policy_arn = f"arn:aws:iam::{account_id}:policy/{policy_name}"
                print(f"[+] Reusing existing Policy ARN: {policy_arn}")
                return policy_arn
            except Exception as sts_err:
                print(f"[-] Failed to construct Policy ARN: {sts_err}", file=sys.stderr)
                sys.exit(1)
        else:
            print(f"[-] IAM Policy Error: {e}", file=sys.stderr)
            sys.exit(1)

def create_iam_user(iam_client, user_name):
    """
    Creates the IAM user if they do not exist.
    """
    print(f"[*] Creating IAM User '{user_name}'...")
    try:
        iam_client.create_user(UserName=user_name)
        print(f"[+] Created IAM User: {user_name}")
    except ClientError as e:
        if e.response['Error']['Code'] == 'EntityAlreadyExists':
            print(f"[*] IAM User '{user_name}' already exists. Reusing.")
        else:
            print(f"[-] IAM User Error: {e}", file=sys.stderr)
            sys.exit(1)

def attach_user_policy(iam_client, user_name, policy_arn):
    """
    Attaches the policy to the user.
    """
    print(f"[*] Attaching policy '{policy_arn}' to user '{user_name}'...")
    try:
        iam_client.attach_user_policy(UserName=user_name, PolicyArn=policy_arn)
        print("[+] Policy attached successfully.")
    except ClientError as e:
        print(f"[-] Failed to attach policy: {e}", file=sys.stderr)
        sys.exit(1)

def create_access_key(iam_client, user_name):
    """
    Generates a new access key for the IAM user.
    """
    print(f"[*] Generating Access Key for IAM User '{user_name}'...")
    try:
        response = iam_client.create_access_key(UserName=user_name)
        access_key = response['AccessKey']
        print("[+] Generated new AWS Access Key.")
        return access_key['AccessKeyId'], access_key['SecretAccessKey']
    except ClientError as e:
        print(f"[-] Failed to generate access key: {e}", file=sys.stderr)
        sys.exit(1)

def write_env_file(env_path, access_key_id, secret_access_key, hosted_zone_id, zone_name):
    """
    Creates or updates the .env file in the web server workspace.
    """
    # Subdomain defaults to robot.<domain>
    domain_name = f"robot.{zone_name.strip().lower()}"

    content = f"""# AWS Route 53 Credentials and Hosted Zone configurations
# Auto-generated by setup_aws_ssl.py
AWS_ACCESS_KEY_ID={access_key_id}
AWS_SECRET_ACCESS_KEY={secret_access_key}
HOSTED_ZONE_ID={hosted_zone_id}
DOMAIN_NAME={domain_name}
"""
    print(f"[*] Writing environment variables to '{env_path}'...")
    try:
        # Check if file exists, warn user if it does
        if os.path.exists(env_path):
            print(f"[!] Warning: File '{env_path}' already exists. Overwriting with new credentials.")

        with open(env_path, 'w') as f:
            f.write(content)
        print(f"[+] Environment file successfully saved to: {env_path}")
    except Exception as e:
        print(f"[-] Failed to write .env file: {e}", file=sys.stderr)
        sys.exit(1)

def main():
    parser = argparse.ArgumentParser(
        description="Automate Route 53 SSL setup by creating IAM Policy, IAM User, Access Keys, and generating the .env file."
    )
    parser.add_argument("zone_name", help="Route 53 hosted zone name (e.g. 'glebos.click')")
    parser.add_argument("--user-name", default="relobot-dns-updater", help="IAM User name to create/use")
    parser.add_argument("--policy-name", default="relobot-route53-policy", help="IAM Policy name to create/use")
    args = parser.parse_args()

    # Define path to output .env
    workspace_root = "/home/admin/projects/relobot"
    env_path = os.path.join(workspace_root, "ros2_ws", ".env")

    print("[*] Initializing AWS Clients...")
    try:
        route53 = boto3.client('route53')
        iam = boto3.client('iam')
    except Exception as e:
        print(f"[-] AWS Session Initialization Error: {e}", file=sys.stderr)
        print("[-] Please ensure your local AWS CLI environment is configured (`aws configure` or env variables).", file=sys.stderr)
        sys.exit(1)

    # 1. Lookup Hosted Zone ID
    hosted_zone_id = get_hosted_zone_id(route53, args.zone_name)

    # 2. Create IAM Policy
    policy_arn = create_iam_policy(iam, args.policy_name, hosted_zone_id)

    # 3. Create IAM User
    create_iam_user(iam, args.user_name)

    # 4. Attach Policy
    attach_user_policy(iam, args.user_name, policy_arn)

    # 5. Create Access Key
    access_key_id, secret_access_key = create_access_key(iam, args.user_name)

    # 6. Write .env File
    write_env_file(env_path, access_key_id, secret_access_key, hosted_zone_id, args.zone_name)

    print("\n[+] AWS Setup Completed Successfully!")
    print(f"[+] New IAM User '{args.user_name}' is now restricted to Route 53 Zone ID '{hosted_zone_id}'.")
    print(f"[+] You can now run './start_robot.sh up --dev' to boot the stack and request certificates.")

if __name__ == '__main__':
    main()
