#!/usr/bin/env python3

import atexit

from flask import Flask, jsonify, render_template

from modules.gnss import gnss
from modules.mavlink import mavlink


app = Flask(__name__)


@app.route("/")
def flight_dashboard():
    return render_template(
        "mavlink_dashboard.html",
        active_tab="flight",
    )


@app.route("/gnss")
def gnss_dashboard():
    return render_template(
        "gnss_dashboard.html",
        active_tab="gnss",
    )


@app.route("/api/mavlink/status")
def mavlink_status():
    return jsonify(mavlink.snapshot())


@app.route("/api/gnss/status")
def gnss_status():
    return jsonify(gnss.snapshot())


# Temporary compatibility route for the existing GNSS JavaScript.
@app.route("/api/status")
def legacy_gnss_status():
    return jsonify(gnss.snapshot())


def start_modules() -> None:
    mavlink.start()
    gnss.start()


def stop_modules() -> None:
    gnss.stop()
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
