#!/usr/bin/env python3
import asyncio
import socket
import sys
from aiohttp import web

# Configuration
# SITL is started with --out udpin:127.0.0.1:14556
# This means SITL is listening on 14556.
SITL_UDP_IP = '127.0.0.1'
SITL_UDP_PORT = 14556

# We provide a WebSocket server on 14550 for web-based GCS
WS_HOST = '127.0.0.1'  # Explicitly bind to localhost to avoid some OS resolution issues
WS_PORT = 14550

class MavlinkWSBridge:
    def __init__(self):
        self.ws_clients = set()
        self.udp_sock = None
        self.sitl_addr = (SITL_UDP_IP, SITL_UDP_PORT)
        self.last_sitl_rx_addr = None

    async def udp_receiver(self):
        """Receive MAVLink from SITL and broadcast to all WS clients."""
        print(f"[UDP] Listening for SITL responses on bridge's local port...")
        
        # Create a non-blocking UDP socket
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.setblocking(False)
        # We don't bind to a specific port, just let the OS pick one
        # and we send to SITL_UDP_PORT first to 'connect'.
        
        # Poke SITL to establish the connection
        # Sending a dummy byte or just waiting for the first WS message to trigger it.
        # Most GCS send heartbeats, so that will trigger it.
        
        loop = asyncio.get_running_loop()
        while True:
            try:
                # Using 4096 as buffer size for MAVLink
                data, addr = await loop.sock_recvfrom(self.udp_sock, 4096)
                self.last_sitl_rx_addr = addr
                
                if self.ws_clients:
                    # Broadcast binary MAVLink to all connected WebSockets
                    # We wrap it in a list to avoid 'set changed size during iteration'
                    for ws in list(self.ws_clients):
                        try:
                            if not ws.closed:
                                await ws.send_bytes(data)
                        except Exception as e:
                            print(f"[WS] Broadcast error: {e}")
                            self.ws_clients.remove(ws)
            except Exception as e:
                # If we get a socket error, just log and continue
                if not isinstance(e, asyncio.CancelledError):
                    print(f"[UDP] Recv error: {e}")
                await asyncio.sleep(0.1)

    async def health_check(self, request):
        """HTTP health check to verify the server is reachable."""
        return web.Response(text="MAVLink WebSocket Bridge is RUNNING\n")

    async def ws_handler(self, request):
        """Handle incoming WebSocket connections from Web GCS."""
        # Check if it's a websocket request
        if request.headers.get('Upgrade', '').lower() != 'websocket':
            return await self.health_check(request)

        ws = web.WebSocketResponse()
        await ws.prepare(request)
        
        client_info = request.remote
        print(f"[WS] Client connected from {client_info}")
        self.ws_clients.add(ws)
        
        # Proactively poke SITL with a MAVLink heartbeat to start the stream.
        # This is a standard MAVLink v1 heartbeat packet (SysID 255, CompID 190).
        heartbeat = b'\xfe\t\x00\xff\xbe\x00\x00\x00\x00\x00\x01\x01\x03\x01\x03\x1f\x1c'
        if self.udp_sock:
            self.udp_sock.sendto(heartbeat, self.sitl_addr)
            print(f"[UDP] Sent heartbeat poke to SITL at {self.sitl_addr}")
        
        try:
            async for msg in ws:
                if msg.type == web.WSMsgType.BINARY:
                    # Forward binary MAVLink from WS to SITL
                    if self.udp_sock:
                        try:
                            self.udp_sock.sendto(msg.data, self.sitl_addr)
                        except Exception as e:
                            print(f"[UDP] Send error: {e}")
                elif msg.type == web.WSMsgType.TEXT:
                    # Some web clients might send JSON-wrapped MAVLink or commands
                    print(f"[WS] Received text (unexpected): {msg.data[:50]}...")
                elif msg.type == web.WSMsgType.ERROR:
                    print(f"[WS] Connection error from {client_info}: {ws.exception()}")
        except Exception as e:
            print(f"[WS] Handler error: {e}")
        finally:
            if ws in self.ws_clients:
                self.ws_clients.remove(ws)
            print(f"[WS] Client {client_info} disconnected")
        
        return ws

async def main():
    bridge = MavlinkWSBridge()
    
    app = web.Application()
    # Accept any path
    app.router.add_get('/', bridge.ws_handler)
    app.router.add_get('/ws', bridge.ws_handler)
    app.router.add_get('/mavlink', bridge.ws_handler)
    app.router.add_get('/health', bridge.health_check)
    
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, WS_HOST, WS_PORT)
    
    print(f"==================================================")
    print(f"  MAVLink WebSocket Bridge")
    print(f"  UDP (SITL)  : {SITL_UDP_IP}:{SITL_UDP_PORT}")
    print(f"  WebSocket   : ws://{WS_HOST}:{WS_PORT}")
    print(f"==================================================")
    
    # Run both tasks concurrently
    await asyncio.gather(
        site.start(),
        bridge.udp_receiver()
    )

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[Bridge] Shutting down.")
    except Exception as e:
        print(f"[Bridge] Fatal error: {e}")
        sys.exit(1)
