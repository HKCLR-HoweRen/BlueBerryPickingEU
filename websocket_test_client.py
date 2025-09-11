import asyncio
import websockets
from websockets.exceptions import ConnectionClosedError

async def main():
    try:
        async with websockets.connect("ws://localhost:8080") as websocket:
            print("Connected to server")  # 新增：打印连接成功信息
            for i in range(3):
                message = f"Hello World {i}"
                print(f"Sending: {message}")
                await websocket.send(message)
                msg = await websocket.recv()
                print(f"Received: {msg}")
    except ConnectionClosedError as e:
        print(f"Connection closed: {e}")
    except Exception as e:
        print(f"Error: {e}")

asyncio.run(main())