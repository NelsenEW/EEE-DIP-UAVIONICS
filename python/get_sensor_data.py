import websocket

ws = websocket.WebSocket()
ws.connect("http://192.168.1.1/sensor")

result = ws.recv()
print(result)
ws.close()