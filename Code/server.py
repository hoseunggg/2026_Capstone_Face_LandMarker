from flask import Flask, request
from flask_cors import CORS
import requests

app = Flask(__name__)
CORS(app)

ESP32_URL = "http://192.168.173.55/set"  # 여기를 ESP32 IP로 변경

@app.route("/")
def home():
    return "Mac relay server OK"

@app.route("/set")
def set_value():
    x = request.args.get("x", "none")
    y = request.args.get("y", "none")
    yaw = request.args.get("yaw", "0")
    pitch = request.args.get("pitch", "0")
    roll = request.args.get("roll", "0")

    print(f"Mac 수신: x={x}, y={y}, yaw={yaw}, pitch={pitch}, roll={roll}", flush=True)

    try:
        res = requests.get(
            ESP32_URL,
            params={
                "x": x,
                "y": y,
                "yaw": yaw,
                "pitch": pitch,
                "roll": roll
            },
            timeout=1.0
        )

        print(f"ESP32 전송 OK: {res.status_code} {res.text}", flush=True)

        return {
            "status": "ok",
            "sent_to_esp32": True,
            "esp32_status": res.status_code
        }

    except Exception as e:
        print("ESP32 전송 실패:", e, flush=True)

        return {
            "status": "error",
            "sent_to_esp32": False,
            "error": str(e)
        }, 500

if __name__ == "__main__":
    app.run(
        host="0.0.0.0",
        port=8443,
        debug=True,
        ssl_context=("cert.pem", "key.pem")
    )
