import argparse
import asyncio
import websockets
import json

clients = set()

async def handler(websocket, path):
    clients.add(websocket)
    print(f"New client connected: {websocket.remote_address}")
    try:
        async for message in websocket:
            print(f"Message from {websocket.remote_address}: {message}")
            if message:
                try:
                    data = json.loads(message)
                    for client in clients:
                        if client != websocket:
                            await client.send(json.dumps(data))
                            print(f"Sent message to {client.remote_address}")
                except json.JSONDecodeError as e:
                    print(f"Failed to decode JSON message: {e}")
            else:
                print("Received empty message from client")
    except websockets.exceptions.ConnectionClosed as e:
        print(f"Client {websocket.remote_address} disconnected with exception {e}")
    finally:
        clients.remove(websocket)
        print(f"Client disconnected: {websocket.remote_address}")

def main():
    parser = argparse.ArgumentParser(description="WebSocket Signaling Server")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Host to bind the server")
    parser.add_argument("--port", type=int, default=9000, help="Port to bind the server")
    args = parser.parse_args()

    print(f"Starting signaling server on {args.host}:{args.port}")
    asyncio.get_event_loop().run_until_complete(
        websockets.serve(handler, args.host, args.port)
    )
    asyncio.get_event_loop().run_forever()

if __name__ == "__main__":
    main()
