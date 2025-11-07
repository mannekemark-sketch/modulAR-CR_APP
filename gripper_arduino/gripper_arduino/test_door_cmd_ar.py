# http_to_ros.py
from fastapi import FastAPI, Request
import uvicorn
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

app = FastAPI()
data_queue = []

# --- ROS2 setup ---
class SimplePublisher(Node):
    def __init__(self):
        super().__init__('http_bridge')
        self.pub = self.create_publisher(String, '/http_data', 10)
        self.create_timer(0.1, self.publish_from_queue)

    def publish_from_queue(self):
        if data_queue:
            msg = String()
            msg.data = data_queue.pop(0)
            self.pub.publish(msg)
            self.get_logger().info(f'Published: {msg.data}')

def ros_thread():
    rclpy.init()
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

threading.Thread(target=ros_thread, daemon=True).start()

# --- HTTP endpoint ---
@app.post("/data")
async def receive_data(req: Request):
    body = await req.json()
    # Expect something like {"value": "hello world"}
    val = str(body.get("value"))
    data_queue.append(val)
    return {"status": "ok", "received": val}

# Run HTTP server

def main():
    uvicorn.run(app, host="0.0.0.0", port=8000)
if __name__ == "__main__":
    main()
print ("lol")