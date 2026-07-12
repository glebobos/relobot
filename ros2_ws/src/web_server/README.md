# ReloBot Web Server SSL/TLS Configuration

This directory contains configuration, scripts, and certificates for securing the ReloBot web controller interface using SSL/TLS.

## Architecture

We secure the robot interface using a publicly trusted Let's Encrypt certificate mapped to a custom subdomain (e.g. `robot.glebos.click`). 

1. **`ros2_certbot` Service**: An auxiliary container running the official `certbot/dns-route53` image.
   - On startup, it runs `update_dns.py` to fetch the Raspberry Pi's active local IP address and updates the Route 53 `A` record.
   - It performs the ACME DNS-01 challenge by communicating with AWS Route 53 to temporarily set the verification TXT records.
   - It saves/renews the certificate files and writes them to a shared volume mapped to `src/web_server/certs/`.
2. **`ros2_frontend` Service**: The main Nginx container.
   - It mounts the certificate directory read-only.
   - It terminates SSL on port 443.
   - It reverse-proxies WebSocket (`/rosbridge/` -> `:9090`) and camera streams (`/camera-stream/` -> `:8080`) to ensure all connections are secured, preventing browsers from blocking mixed content.

---

## Setup Guide

### Step 1: Automated AWS Setup
If you have the AWS CLI configured on your machine with administrative permissions, you can use the automated script in `helpers/` to set up the Route 53 IAM user, minimal IAM policy, access keys, and the `.env` file automatically:

```bash
# Install boto3 if you don't have it locally
pip install boto3

# Run the setup script (passing your Route 53 domain name)
./helpers/setup_aws_ssl.py glebos.click
```
This script will look up the zone ID, create a restricted IAM policy, create the user, attach the policy, generate keys, and write them directly into `ros2_ws/.env`.

---

### Step 2: Run the Stack
Start the robot stack using the standard launch script:
```bash
./start_robot.sh up --dev
```
On startup:
- The `ros2_certbot` container will run `update_dns.py` to point `robot.glebos.click` to the local IP of the Pi.
- It will negotiate with Let's Encrypt and place the certificate and private key in `./src/web_server/certs/live/robot.glebos.click/`.
- Nginx will load the certificate, allowing you to access the secure control panel at `https://robot.glebos.click`.
