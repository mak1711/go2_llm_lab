# Go2 Lab — LLM Commands, MCP, Nav2 and RViz

## Step 0 — Obtain the package and install dependencies

```bash

mkdir -p ~/ros2_ws/src

cd ~/ros2_ws/src

git clone https://github.com/mak1711/go2_llm_lab.git

```

```bash

cd ~/ros2_ws/src/go2_llm_lab

```

Install the extra lab dependencies once:

```bash

bash install_dependencies.sh

```

The environment check verifies both ROS packages and Python imports, including Unitree's generated message types. If `unitree_api` cannot be imported, the existing Unitree message workspace must be corrected before proceeding; seeing robot topics alone does not guarantee the Python import works.

Build your workspace from its root:

```bash

cd ~/ros2_ws

colcon build

```

## Step 1 — Create your first high-level movement command

Unitree's sport interface accepts `unitree_api/msg/Request` messages on `/api/sport/request`. API ID **1008** is `Move`: the JSON parameters `x`, `y`, and `z` specify forward (m/s), lateral (m/s), and yaw (rad/s) velocity. API ID **1003** is `StopMove`.

Write a short Python example that turns those messages into one high-level `walk_forward(seconds)` function. Create the file now:

```bash

nano ~/ros2_ws/src/go2_llm_lab/go2_llm_lab/walk_forward_demo.py

```

```python

#!/usr/bin/env python3

"""First example: one high-level command built from ROS 2 messages."""

import json

import os

import time

import rclpy

from rclpy.node import Node

from unitree_api.msg import Request

DRY_RUN = os.environ.get("GO2_DRY_RUN", "1") == "1"

class Robot(Node):

    def __init__(self):

        super().__init__("first_go2_command")

        self.publisher = self.create_publisher(Request, "/api/sport/request", 10)

    def send(self, api_id, values=""):

        msg = Request()

        msg.header.identity.api_id = api_id

        msg.header.identity.id = time.monotonic_ns()

        msg.parameter = values

        if not DRY_RUN:

            self.publisher.publish(msg)

    def walk_forward(self, seconds=1.0):

        """Move forward at 0.2 m/s, then request a stop."""

        print(f"walk_forward(seconds={seconds}) | dry_run={DRY_RUN}")

        if DRY_RUN:

            return

        if self.publisher.get_subscription_count() == 0:

            raise RuntimeError("No subscriber on /api/sport/request")

        try:

            finish = time.monotonic() + min(float(seconds), 2.0)

            while time.monotonic() < finish:

                self.send(1008, json.dumps({"x": 0.2, "y": 0.0, "z": 0.0}))

                time.sleep(0.05)

        finally:

            for _ in range(3):

                self.send(1003)

                time.sleep(0.05)

if __name__ == "__main__":

    rclpy.init()

    node = Robot()

    try:

        if not DRY_RUN:

            deadline = time.monotonic() + 5.0

            while node.publisher.get_subscription_count() == 0 and time.monotonic() < deadline:

                rclpy.spin_once(node, timeout_sec=0.1)

        node.walk_forward(1.0)

    finally:

        node.destroy_node()

        rclpy.shutdown()

```

```bash

python3 ~/ros2_ws/src/go2_llm_lab/go2_llm_lab/walk_forward_demo.py

```

## Step 2 — Create the movement module used by the LLM

The first example has one fixed motion. Now create a reusable class with `move(vx, vy, yaw_rate, seconds)` and `stop()`. Later the MCP server will call these methods; the LLM never needs to construct ROS messages itself.

Create `robot_moves.py`:

```bash

nano ~/ros2_ws/src/go2_llm_lab/go2_llm_lab/robot_moves.py

```

```python

"""High-level motion for the Unitree Go2 (sport API).

This is plain robot code, with nothing LLM-specific in it. It sends the same

requests as Unitree's go2_sport_client example:

  Move     (api 1008)  parameter '{"x": vx, "y": vy, "z": vyaw}'

  StopMove (api 1003)

on the topic /api/sport/request.

Test it on its own (robot standing, remote in hand):

    ros2 run go2_llm_lab walk_test            # 0.3 m/s forward for 2 s

Set GO2_DRY_RUN=1 to print what would be sent without sending anything.

"""

import json

import os

import sys

import time

import rclpy

from rclpy.node import Node

from unitree_api.msg import Request

from unitree_go.msg import SportModeState

MOVE = 1008

STOPMOVE = 1003

RATE_HZ = 20.0           # how often Move is re-sent while walking

MAX_VX = 0.5             # m/s forward/backward

MAX_VY = 0.3             # m/s sideways

MAX_VYAW = 1.0           # rad/s turning

MAX_DURATION_S = 5.0

DRY_RUN = os.environ.get("GO2_DRY_RUN") == "1"

def clamp(value, limit):

    return max(-limit, min(limit, value))

class Go2Mover(Node):

    def __init__(self):

        super().__init__("go2_mover")

        self.pub = self.create_publisher(Request, "/api/sport/request", 10)

        self.state = None

        self.create_subscription(SportModeState, "/lf/sportmodestate", self._on_state, 10)

    def _on_state(self, msg):

        self.state = msg

    def _send(self, api_id, params=None):

        req = Request()

        req.header.identity.id = time.monotonic_ns()   # a new id for every request

        req.header.identity.api_id = api_id

        if params is not None:

            req.parameter = json.dumps(params)

        self.pub.publish(req)

    def connected(self):

        """True if the robot's sport service is subscribed to our topic."""

        return self.pub.get_subscription_count() > 0

    def move(self, vx, vy, vyaw, duration_s):

        """Walk with the given velocity for duration_s seconds, then stop.

        vx: forward m/s (+ forward), vy: sideways m/s (+ left), vyaw: rad/s (+ left turn).

        Blocks until the move is finished. Returns a short text describing what happened.

        """

        vx, vy, vyaw = clamp(vx, MAX_VX), clamp(vy, MAX_VY), clamp(vyaw, MAX_VYAW)

        duration_s = max(0.1, min(MAX_DURATION_S, duration_s))

        what = f"vx={vx:.2f} vy={vy:.2f} vyaw={vyaw:.2f} for {duration_s:.1f} s"

        if DRY_RUN:

            return f"[dry run] would move {what}"

        if not self.connected():

            raise RuntimeError("Robot not connected: nothing is subscribed to /api/sport/request. "

                               "Check the network and that ~/unitree_ros2/setup.sh was sourced.")

        try:

            end = time.monotonic() + duration_s

            while time.monotonic() < end:

                self._send(MOVE, {"x": vx, "y": vy, "z": vyaw})

                time.sleep(1.0 / RATE_HZ)

        finally:

            self.stop()          # always stop, even if something went wrong

        return f"Moved {what}, then stopped."

    def stop(self):

        if DRY_RUN:

            return "[dry run] would send StopMove"

        for _ in range(3):

            self._send(STOPMOVE)

            time.sleep(0.05)

        return "StopMove sent."

def main():

    """Standalone test: walk forward at 0.3 m/s for 2 s."""

    rclpy.init()

    mover = Go2Mover()

    # Give DDS up to 5 s to connect to the robot.

    end = time.monotonic() + 5.0

    while not mover.connected() and time.monotonic() < end:

        rclpy.spin_once(mover, timeout_sec=0.1)

    print(f"Robot connected: {mover.connected()}")

    try:

        print(mover.move(0.3, 0.0, 0.0, 2.0))

    except RuntimeError as err:

        print(err)

        return 1

    except KeyboardInterrupt:

        print(mover.stop())

    finally:

        mover.destroy_node()

        rclpy.shutdown()

    return 0

if __name__ == "__main__":

    sys.exit(main())

```

Build the workspace and test the registered `walk_test` entry point:

```bash

cd ~/ros2_ws

colcon build

ros2 run go2_llm_lab walk_test

```

## Step 3 — Turn one movement function into an MCP tool

MCP makes selected Python functions available to an LLM application. **The robot behavior is already defined in `Go2Mover`; MCP only exposes a named interface to it.**

Create the file now:

```bash

nano ~/ros2_ws/src/go2_llm_lab/go2_llm_lab/mcp_server.py

```

```python

"""An MCP server exposing one high-level Go2 command."""

import rclpy

from mcp.server.fastmcp import FastMCP

from go2_llm_lab.robot_moves import Go2Mover

from go2_llm_lab.ros_utils import spin_in_background

mcp = FastMCP("go2-lab")

mover = None

@mcp.tool()

def walk_forward(seconds: float = 2.0) -> str:

    """Walk forward at 0.3 m/s for seconds (at most 5), then stop."""

    return mover.move(0.3, 0.0, 0.0, seconds)

def main():

    global mover

    rclpy.init()

    mover = Go2Mover()

    spin_in_background(mover)

    mcp.run()   # The Gemini agent connects over MCP stdio.

if __name__ == "__main__":

    main()

```

Look at these pieces of the implementation:

1. `mcp = FastMCP("go2-lab")` constructs the MCP server.

2. `@mcp.tool()` **registers the function immediately below it as an MCP tool**. The LLM sees its name (`walk_forward`), the docstring (what it does), and typed parameters (`seconds: float`, default `2.0`).

3. `return mover.move(0.3, 0.0, 0.0, seconds)` connects the tool to the actual ROS command. The three velocities mean forward motion with no sideways translation or turning.

4. `main()` creates `Go2Mover`, lets ROS callbacks run in a background executor using the provided helper, then starts the MCP server using stdio. **Do not add `print()` statements to the MCP server's stdout**, because stdio carries MCP messages.

You do **not** need to run the MCP server in a separate terminal. The Gemini client in the next step starts it automatically.

## Step 4 — Connect Gemini to MCP and test one tool

Create a Gemini API key through [Google AI Studio](https://aistudio.google.com/app/apikey). In the terminal where you will run the LLM:

```bash

read -rsp "Gemini API key: " GEMINI_API_KEY; echo

export GEMINI_API_KEY

```

Create `agent.py`:

```bash

nano ~/ros2_ws/src/go2_llm_lab/go2_llm_lab/agent.py

```

```python

"""The LLM agent: a chat loop that connects Gemini to the robot's MCP tools.

    export GEMINI_API_KEY=...        # free key: https://aistudio.google.com/app/apikey

    ros2 run go2_llm_lab llm_agent

For each message you type:

  1. send the conversation + the tool list to Gemini,

  2. if Gemini answers with tool calls, run them through the MCP server and

     send the results back to Gemini (repeat),

  3. when Gemini answers with plain text, print it.

"""

import asyncio

import json

import os

import sys

import httpx

from mcp import ClientSession, StdioServerParameters

from mcp.client.stdio import stdio_client

GEMINI_URL = "https://generativelanguage.googleapis.com/v1beta/openai/chat/completions"

MODEL = os.environ.get("GO2_LLM_MODEL", "gemini-3.1-flash-lite")

API_KEY = os.environ.get("GEMINI_API_KEY", "")

SYSTEM_PROMPT = """You control a real Unitree Go2 robot dog through the tools you are given.

Rules:

- Only move the robot with the tools. Never claim it moved unless a tool said so.

- Short moves ("move forward", "turn left") use the walk/turn tools.
  
- For a destination with x/y coordinates, use navigate_to.

- For a destination given by a saved name (such as chair or sofa), use
  go_to_place; use list_places to check available names when needed.
  
- If neither valid coordinates nor a saved place is given, ask for details.
  Never invent or guess coordinates or saved place names.

- If navigate_to is not available yet, say navigation tools will be added later.

- If navigate_to is not among available tools yet, say that Nav2 integration comes later in this lab.

- Do one movement at a time. Do not start a walk while a Nav2 goal is running; call stop first.

- If the person says stop, call stop immediately.

- If a tool returns an error, tell the person what it said.

- Answer in one or two short sentences."""

def to_gemini_tools(mcp_tools):

    """Convert MCP tool descriptions to the function-calling format Gemini expects."""

    return [{

        "type": "function",

        "function": {

            "name": t.name,

            "description": t.description or "",

            "parameters": t.inputSchema or {"type": "object", "properties": {}},

        },

    } for t in mcp_tools]

def trim_history(messages, keep_turns=6):

    """Keep the system prompt and the last few user turns. Cuts only right before a

    user message, so a tool call is never separated from its result."""

    user_idx = [i for i, m in enumerate(messages) if m.get("role") == "user"]

    if len(user_idx) > keep_turns:

        del messages[1:user_idx[-keep_turns]]

async def ask_gemini(http, messages, tools):

    """One request to Gemini. If Gemini is busy (429/500/503), wait and try again (3 tries)."""

    for attempt in range(3):

        resp = await http.post(GEMINI_URL, headers={"Authorization": f"Bearer {API_KEY}"},

                               json={"model": MODEL, "messages": messages, "tools": tools})

        if resp.status_code in (429, 500, 503) and attempt < 2:

            await asyncio.sleep(2 * (attempt + 1))     # wait 2 s, then 4 s

            continue

        if resp.status_code != 200:

            raise RuntimeError(f"Gemini error {resp.status_code}: {resp.text[:300]}")

        return resp.json()["choices"][0]["message"]

async def handle_message(session, http, messages, tools):

    """Run one user message to completion. Returns Gemini's final text."""

    for _ in range(10):                       # safety limit on tool rounds

        reply = await ask_gemini(http, messages, tools)

        messages.append(reply)

        tool_calls = reply.get("tool_calls") or []

        if not tool_calls:

            return reply.get("content") or ""

        for call in tool_calls:

            name = call["function"]["name"]

            args = json.loads(call["function"].get("arguments") or "{}")

            print(f"  [tool call]   {name}({args})")

            result = await session.call_tool(name, args)

            text = "\n".join(getattr(c, "text", str(c)) for c in result.content)

            print(f"  [tool result] {text}")

            messages.append({"role": "tool", "tool_call_id": call.get("id", ""), "content": text})

    return "(stopped: too many tool calls in a row)"

async def run():

    if not API_KEY:

        print("Set GEMINI_API_KEY first (free key: https://aistudio.google.com/app/apikey)")

        return 1

    # Start the MCP server as a child process; we talk to it over its stdin/stdout.

    server = StdioServerParameters(command=sys.executable,

                                   args=["-m", "go2_llm_lab.mcp_server"],

                                   env=dict(os.environ))

    async with stdio_client(server) as (read, write):

        async with ClientSession(read, write) as session:

            await session.initialize()

            tools = to_gemini_tools((await session.list_tools()).tools)

            print(f"Model: {MODEL}")

            print("Tools: " + ", ".join(t["function"]["name"] for t in tools))

            print("Type a request (e.g. 'walk forward'), or 'quit'.\n")

            messages = [{"role": "system", "content": SYSTEM_PROMPT}]

            async with httpx.AsyncClient(timeout=30.0) as http:

                while True:

                    try:

                        text = (await asyncio.to_thread(input, "> ")).strip()

                    except (EOFError, KeyboardInterrupt):

                        break

                    if text.lower() in ("quit", "exit"):

                        break

                    if not text:

                        continue

                    messages.append({"role": "user", "content": text})

                    try:

                        print(await handle_message(session, http, messages, tools) + "\n")

                    except (RuntimeError, httpx.HTTPError) as err:

                        print(f"(error: {err})\n")

                    trim_history(messages)

    return 0

def main():

    sys.exit(asyncio.run(run()))

if __name__ == "__main__":

    main()

```

The agent starts the MCP server, asks it for its tool definitions, converts them to the Gemini function-call format, sends the user's text to Gemini, executes any chosen tool via MCP, then returns the tool result to the model.

Build and run:

```bash

cd ~/ros2_ws

colcon build

ros2 run go2_llm_lab llm_agent

```

## Step 5 — Add the other high-level tools

There is **no need to change `agent.py`** whenever you add a tool. The agent calls `session.list_tools()` when it starts. You add a normal Python function, register it using `@mcp.tool()`, and restart the agent so it fetches the new list.

Open the existing MCP server:

```bash

nano ~/ros2_ws/src/go2_llm_lab/go2_llm_lab/mcp_server.py

```

Insert the following functions **after `walk_forward()` and before `def main()`**:

```python

@mcp.tool()

def walk_backward(seconds: float = 2.0) -> str:

    """Walk backward at 0.3 m/s for seconds (max 5), then stop."""

    return mover.move(-0.3, 0.0, 0.0, seconds)

@mcp.tool()

def turn_left(seconds: float = 2.0) -> str:

    """Turn left at 0.5 rad/s for seconds (max 5), then stop."""

    return mover.move(0.0, 0.0, 0.5, seconds)

@mcp.tool()

def turn_right(seconds: float = 2.0) -> str:

    """Turn right at 0.5 rad/s for seconds (max 5), then stop."""

    return mover.move(0.0, 0.0, -0.5, seconds)

@mcp.tool()

def stop() -> str:

    """Stop Go2 motion by sending the StopMove sport request."""

    return mover.stop()

```

Rebuild and restart the running agent:

```bash

cd ~/ros2_ws

colcon build

ros2 run go2_llm_lab llm_agent

```

## Step 6 — Use the provided Nav2 and RViz components

The package you cloned already includes **complete working files to review and run**

- `config/nav2_params.yaml` — Nav2 planner, controller, costmaps and velocity-smoothing settings.

- `rviz/nav2.rviz` — the map, planned path, costmaps, and navigation tools in RViz.

- `launch/nav2.launch.py` — starts Nav2, RViz, and optionally the Go2 velocity bridge.

- `go2_llm_lab/cmd_vel_bridge.py` — converts `/cmd_vel` to the Go2 sport API.

- `go2_llm_lab/nav_goal.py` — sends and monitors `NavigateToPose` goals.

Start Nav2 and RViz for **planning-only testing**:

```bash

ros2 launch go2_llm_lab nav2.launch.py bridge:=false

```

In RViz you can see the supplied map and use **Nav2 Goal** to inspect planning. The **Publish Point** tool publishes `/clicked_point` coordinates.

## Step 7 — Extend the existing MCP server with Nav2 tools

You can now add navigation to the **same** MCP server you built in Steps 3 and 5. This is the key extension: you are not replacing the movement tools, and you are not modifying Nav2 itself. You register two more Python functions that call the provided `Nav2Goal` helper.

Open the existing server:

```bash

nano ~/ros2_ws/src/go2_llm_lab/go2_llm_lab/mcp_server.py

```

**7.1 Import and create the navigation helper**

Add this import beside the existing imports:

```python

from go2_llm_lab.nav_goal import Nav2Goal

```

Immediately after the existing `mover = None` line, add:

```python

nav = None

```

Change the existing `main()` function to:

```python

def main():

    global mover, nav

    rclpy.init()

    mover = Go2Mover()

    nav = Nav2Goal()

    spin_in_background(mover, nav)

    mcp.run()

```

This creates a ROS 2 action client for Nav2 **alongside** the existing sport-API publisher. Both receive ROS callbacks from the same helper executor.

**7.2 Register two new functions**

Place these definitions with the other `@mcp.tool()` functions, above `main()`:

```python

@mcp.tool()

def navigate_to(x: float, y: float, yaw_deg: float = 0.0) -> str:

    """Navigate to given x, y coordinates in the map frame (metres).

    yaw_deg is the final heading in degrees. Call navigation_status for progress.

    Only use coordinates explicitly supplied by the user.

    """

    if nav.busy():

        return "Navigation already active; call stop first."

    return nav.send(x, y, yaw_deg)

@mcp.tool()

def navigation_status() -> dict:

    """Report the current Nav2 goal status and remaining distance."""

    return nav.status()

```

The signature of `navigate_to` tells Gemini that it needs `x` and `y` numbers, while `yaw_deg` is optional. The description explains that these coordinates are in the map frame. `navigation_status` needs no arguments and reports the action client's state. `agent.py` automatically exposes both on its next startup because MCP advertises the updated tool list.

**7.3 Prevent direct commands while a Nav2 goal is active**

Insert this helper above the tool functions:

```python

def check_no_goal():

    if nav is not None and nav.busy():

        raise RuntimeError("A Nav2 goal is active. Use stop first.")

```

Add `check_no_goal()` as the **first line inside each of** `walk_forward`, `walk_backward`, `turn_left`, and `turn_right`, immediately before its `return mover.move(...)`. For example:

```python

@mcp.tool()

def walk_forward(seconds: float = 2.0) -> str:

    """Walk forward at 0.3 m/s for seconds (max 5), then stop."""

    check_no_goal()

    return mover.move(0.3, 0.0, 0.0, seconds)

```

Replace the **existing** `stop` tool (do not add a second tool with the same name) with:

```python

@mcp.tool()

def stop() -> str:

    """Cancel any active navigation goal and send StopMove to the Go2."""

    return f"{nav.cancel()} {mover.stop()}"

```

This cancels the active Nav2 goal, if any, and requests that the Go2 stop. It is a software stop command; keep the physical remote/emergency stop available. Avoid simultaneous commands from multiple computers.

Rebuild:

```bash

cd ~/ros2_ws

colcon build

```

Restart the LLM agent. Its tool list should now include the five movement tools **plus** `navigate_to` and `navigation_status` (seven tools total).

## Step 8 — Run the complete system

**Terminal 1: start Nav2 + RViz.** Initially keep the motion bridge disabled:

```bash

ros2 launch go2_llm_lab nav2.launch.py bridge:=false

```

**Terminal 2: start the LLM agent (dry-run for direct movement).** Enter the Gemini key in this terminal if it is not already set:

```bash

read -rsp "Gemini API key: " GEMINI_API_KEY; echo

export GEMINI_API_KEY

GO2_DRY_RUN=1 ros2 run go2_llm_lab llm_agent

```

The outcome is a single LLM that can choose between high-level body-frame sport commands and coordinate-based navigation through Nav2.

## Step 9 — Navigate to named places using RViz coordinates

So far, the LLM can navigate when the user supplies coordinates such as `go to x=1.2 y=-0.4`. We can also give locations meaningful names, such as **chair**, **sofa**, or **door**, so the user can simply say **"go to the sofa"**. Students will choose each point manually in RViz and save it to a small JSON file. The LLM will use MCP to look up its coordinates; it does not need to detect or recognize the object.

### 9.1 — Read coordinates from RViz

With the instructor's map available and RViz open, start this in a separate terminal:

```bash
ros2 topic echo /clicked_point --once
```

In RViz, select **Publish Point** in the toolbar and click a suitable destination on the map. For example, clicking a **clear, navigable location near** the chair, rather than on the chair itself, produces a message similar to:

```yaml
header:
  frame_id: map
point:
  x: 1.20
  y: -0.40
  z: 0.0
```

Record `x` and `y`. The values above are only examples: **use coordinates from your own map**. The clicked point must use the `map` frame, which is the frame expected by `navigate_to`. Choose a final heading (`yaw_deg`) in degrees if you want the Go2 to face a particular direction when it arrives.

### 9.2 — Save locations manually

Create one file at the root of your cloned package:

```bash
nano ~/ros2_ws/src/go2_llm_lab/places.json
```

Enter your own destinations in this format:

```json
{
  "chair": {"x": 1.20, "y": -0.40, "yaw_deg": 0.0},
  "sofa":  {"x": 2.80, "y": 1.10, "yaw_deg": 90.0},
  "door":  {"x": -0.70, "y": 3.50, "yaw_deg": 180.0}
}
```

Replace every example coordinate with a point you selected in RViz. Each name must be unique. These are **navigation destinations near objects**, not object-detection results. Keep the same map and coordinate frame when reusing the saved points; after rebuilding or changing the map, verify the locations again.

### 9.3 — Add the named-place MCP tools

Open the existing MCP server:

```bash
nano ~/ros2_ws/src/go2_llm_lab/go2_llm_lab/mcp_server.py
```

Add the following imports beside the other imports at the top:

```python
import json
from pathlib import Path
```

After the existing `mover = None` and `nav = None` variables, add this helper. It reads the saved locations from the JSON file when a tool is called:

```python
PLACES_FILE = Path.home() / "ros2_ws/src/go2_llm_lab/places.json"


def load_places() -> dict:
    """Read manually saved map-frame destinations."""
    with PLACES_FILE.open("r", encoding="utf-8") as file:
        places = json.load(file)
    if not isinstance(places, dict):
        raise ValueError("places.json must contain an object of named places")
    return places
```

Now add **two new tool functions** alongside the existing `@mcp.tool()` functions, **above `def main()`**. Do not replace the existing movement or navigation functions:

```python
@mcp.tool()
def list_places() -> list[str]:
    """List the names of saved destinations (chair, sofa, door, etc.)."""
    return sorted(load_places().keys())


@mcp.tool()
def go_to_place(name: str) -> str:
    """Navigate to a saved place by name (e.g., chair or sofa).

    Only use a name present in the saved places file; never guess coordinates.
    """
    places = load_places()
    requested = name.strip().casefold()
    match = next((key for key in places if key.casefold() == requested), None)

    if match is None:
        return f"Unknown place '{name}'. Available places: {', '.join(sorted(places))}"
    if nav.busy():
        return "Navigation already active. Call stop first."

    point = places[match]
    if not isinstance(point, dict) or "x" not in point or "y" not in point:
        return f"Saved place '{match}' has invalid coordinates."

    return nav.send(
        float(point["x"]),
        float(point["y"]),
        float(point.get("yaw_deg", 0.0)),
    )
```

`list_places()` lets the LLM discover which place names exist. `go_to_place(name)` looks up the exact saved name (ignoring case) and passes its coordinates to the **same `nav.send()`** method used by `navigate_to(x, y, yaw_deg)`. An unknown name returns an error message instead of guessing a destination. Because the JSON is loaded when each tool runs, you can edit `places.json` without changing the Python functions.


### 9.4 — Build and try named destinations

Rebuild the workspace and restart the LLM agent so that it discovers the two new MCP tools:

```bash
cd ~/ros2_ws
colcon build
ros2 run go2_llm_lab llm_agent
```


