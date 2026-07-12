#!/bin/sh
set -e

# Ensure DOMAIN_NAME is set
DOMAIN_NAME="${DOMAIN_NAME:-robot.glebos.click}"

# Run the update DNS script
python3 /scripts/update_dns.py

# Check internet connection
if ping -c 1 -W 2 8.8.8.8 >/dev/null 2>&1; then
  echo '[*] Internet connection detected. Proceeding with SSL automation...'
  if [ ! -f "/etc/letsencrypt/renewal/${DOMAIN_NAME}.conf" ]; then
    echo '[*] Real certificate not found. Deleting bootstrap self-signed certs...'
    rm -rf "/etc/letsencrypt/live/${DOMAIN_NAME}" "/etc/letsencrypt/archive/${DOMAIN_NAME}"
    certbot certonly --dns-route53 -d "${DOMAIN_NAME}" --email "admin@${DOMAIN_NAME}" --agree-tos --non-interactive
    python3 /scripts/reload_nginx.py
  else
    echo '[*] Real certificate already exists. Checking for renewal...'
    certbot renew --post-hook 'python3 /scripts/reload_nginx.py'
  fi
else
  echo '[!] No internet connection detected. Skipping Let’s Encrypt verification. Serving interface using self-signed or cached certificates.'
  if [ -d "/etc/letsencrypt/live/${DOMAIN_NAME}" ]; then
    python3 /scripts/reload_nginx.py
  fi
fi
