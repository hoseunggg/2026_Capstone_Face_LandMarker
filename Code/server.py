import os

from flask import Flask, request
from flask_cors import CORS
import requests

app = Flask(__name__)
CORS(app)

ESP32_URL = os.getenv("ESP32_URL", "http://192.168.252.39/set")
REQUEST_TIMEOUT_SECONDS = float(os.getenv("ESP32_TIMEOUT", "1.0"))
FORWARDED_PARAMS = ("x", "y", "yaw", "pitch", "roll", "command")


@app.route("/")
def home():
    return "Mac relay server OK"


@app.route("/set")
def set_value():
    payload = build_payload()
    log_received_payload(payload)

    try:
        response = forward_to_esp32(payload)
    except requests.RequestException as exc:
        print("ESP32 전송 실패:", exc, flush=True)
        return {
            "status": "error",
            "sent_to_esp32": False,
            "error": str(exc),
        }, 500

    print(f"ESP32 전송 OK: {response.status_code} {response.text}", flush=True)
    return {
        "status": "ok",
        "sent_to_esp32": True,
        "esp32_status": response.status_code,
    }


def build_payload():
    return {
        "x": request.args.get("x", "none"),
        "y": request.args.get("y", "none"),
        "yaw": request.args.get("yaw", "0"),
        "pitch": request.args.get("pitch", "0"),
        "roll": request.args.get("roll", "0"),
        "command": request.args.get("command", ""),
    }


def log_received_payload(payload):
    values = ", ".join(f"{key}={payload[key]}" for key in FORWARDED_PARAMS)
    print(f"Mac 수신: {values}", flush=True)


def forward_to_esp32(payload):
    return requests.get(
        ESP32_URL,
        params=payload,
        timeout=REQUEST_TIMEOUT_SECONDS,
    )


if __name__ == "__main__":
    app.run(
        host="0.0.0.0",
        port=8443,
        debug=True,
        ssl_context=("cert.pem", "key.pem")
    )
