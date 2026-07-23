#!/usr/bin/env python3

import atexit

from flask import Flask, jsonify, render_template

from modules.mavlink import mavlink


app = Flask(__name__)


@app.route("/")
def index():
    return render_template("mavlink_dashboard.html")


@app.route("/api/mavlink/status")
def mavlink_status():
    return jsonify(mavlink.snapshot())


def start_modules() -> None:
    mavlink.start()


def stop_modules() -> None:
    mavlink.stop()


if __name__ == "__main__":
    start_modules()
    atexit.register(stop_modules)

    app.run(
        host="0.0.0.0",
        port=5001,
        debug=False,
        threaded=True,
        use_reloader=False,
    )
