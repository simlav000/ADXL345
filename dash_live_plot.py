import socket, threading
import dash
from dash import dcc, html
from dash.dependencies import Output, Input
from collections import deque



# Buffers for plotting
buffer_len = 500
x_vals = deque(maxlen=buffer_len)
y_vals = deque(maxlen=buffer_len)
z_vals = deque(maxlen=buffer_len)

def tcp_listener():
    HOST = "127.0.0.1"  # now points to Pi via SSH tunnel
    PORT = 9000
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))

        while True:
            try:
                data = s.recv(1024).decode()
                for line in data.strip().split("\n"):
                    x, y, z = map(float, line.strip().split(","))
                    x_vals.append(x)
                    y_vals.append(y)
                    z_vals.append(z)

            except (KeyboardInterrupt, BrokenPipeError):
                print("Job's done")
                break

# Start listener in the background
threading.Thread(target=tcp_listener, daemon=True).start()

# Dash setup for live plotting on web-interface.
app = dash.Dash(__name__)
app.layout = html.Div([
    html.H2("Live Accelerometer Data"),
    dcc.Graph(id="live-plot"),
    dcc.Interval(id="interval", interval=100, n_intervals=0)
])


@app.callback(Output("live-plot", "figure"), [Input("interval", "n_intervals")])
def update_graph(_):
    if x_vals and y_vals and z_vals:
        print(f"Latest: X={x_vals[-1]:.2f}, Y={y_vals[-1]:.2f}, Z={z_vals[-1]:.2f}")
    else:
        print("Buffers still empty or awaiting data...")
    return {
        "data": [
            {"y": list(x_vals), "type": "line", "name": "X"},
            {"y": list(y_vals), "type": "line", "name": "Y"},
            {"y": list(z_vals), "type": "line", "name": "Z"},
        ],
        "layout": {"yaxis": {"range": [-1, 1]}, "xaxis": {"visible": False}}
    }

if __name__ == "__main__":
    app.run(debug=False)
