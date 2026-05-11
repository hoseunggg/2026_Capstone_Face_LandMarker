from flask import Flask, request
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

@app.route("/")
def home():
    return "Mac test server OK"

@app.route("/set")
def set_value():
    x = request.args.get("x", "none")
    y = request.args.get("y", "none")

    print(f"신호 받음: x={x}, y={y}", flush=True)

    return {
        "status": "ok",
        "x": x,
        "y": y
    }

if __name__ == "__main__":
    app.run(
        host="0.0.0.0",
        port=8443,
        debug=True,
        ssl_context=("cert.pem", "key.pem")
    )