#!/usr/bin/env python3
import socket
import http.client
import sys

class UnixHTTPConnection(http.client.HTTPConnection):
    """
    Custom HTTPConnection that communicates over a Unix domain socket
    instead of a TCP socket. Used to talk to the Docker daemon.
    """
    def __init__(self, unix_socket_path):
        super().__init__('localhost')
        self.unix_socket_path = unix_socket_path

    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.connect(self.unix_socket_path)

def reload_nginx(container_name="ros2_frontend"):
    print(f"[*] Sending SIGHUP reload signal to container '{container_name}' via Docker socket...")
    try:
        conn = UnixHTTPConnection('/var/run/docker.sock')
        # Docker Engine API: POST /containers/{id}/kill?signal=HUP
        conn.request('POST', f'/containers/{container_name}/kill?signal=HUP')
        resp = conn.getresponse()
        
        if resp.status in (200, 204):
            print(f"[+] Nginx configuration reloaded successfully inside container '{container_name}'.")
        else:
            body = resp.read().decode('utf-8', errors='ignore')
            print(f"[-] Docker API returned status {resp.status}: {body}", file=sys.stderr)
            sys.exit(1)
    except Exception as e:
        print(f"[-] Failed to communicate with Docker socket: {e}", file=sys.stderr)
        print("[-] Note: Ensure '/var/run/docker.sock' is mounted to the container.", file=sys.stderr)
        sys.exit(1)

if __name__ == '__main__':
    reload_nginx()
