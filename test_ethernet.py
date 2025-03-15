import socket
import argparse
import os

def send_udp_packet(ip: str, port: int):
    data = bytes({0x69, 0x69, 0x69, 0x69, 0x69, 0x69, 0x69, 0x69})
    
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(data, (ip, port))
        print(f"Sent {data.hex()} to {ip}:{port}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Send a UDP packet with 8 bytes.")
    parser.add_argument("ip", type=str, help="Destination IP address")
    parser.add_argument("port", type=int, help="Destination port number")
    
    args = parser.parse_args()
    send_udp_packet(args.ip, args.port)