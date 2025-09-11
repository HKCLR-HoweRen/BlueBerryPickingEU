import asyncio
import websockets
from websockets.legacy.server import WebSocketServerProtocol

async def ws_handle(websocket: WebSocketServerProtocol, path: str = None):
    print(f"Client connected to path: {path}")  # 新增：打印客户端连接信息
    try:
        async for message in websocket:
            print(f"Received message: {message}")  # 新增：打印接收到的消息
            await websocket.send(message)
            print(f"Sent echo: {message}")  # 新增：打印发送的回显消息
    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected")  # 新增：打印客户端断开信息
    finally:
        print("Connection handler finished")  # 新增：打印处理结束信息

async def main():
    async with websockets.serve(ws_handle, "localhost", 8080):
        await asyncio.Future()  # run forever

asyncio.run(main())