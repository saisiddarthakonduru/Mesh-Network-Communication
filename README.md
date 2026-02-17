📡 ESPNowMeshFull – Multi-Hop ESP-NOW Mesh for ESP32

A full-featured ESP-NOW mesh networking framework for ESP32 using the Arduino core.

This project merges:

🔹 Multi-hop ESP-NOW Mesh (TTL, routing, deduplication, RSSI-based forwarding)

🔹 Reliable messaging with ACK support

🔹 Neighbor discovery system

🔹 Serial terminal command interface

🔹 Debug logging and routing diagnostics

All implemented in a single .ino file: ESPNowMeshFull.ino.

🚀 Features
✅ Mesh Networking

Multi-hop forwarding

TTL-based routing

Loop prevention (path tracking)

Duplicate packet detection

RSSI-based route selection

✅ Reliable Messaging

ACK-based delivery confirmation

Automatic retry mechanism

Configurable timeout and retry count

Success and failure callbacks

✅ Node Discovery

Broadcast discovery requests

Automatic neighbor table maintenance

Role-based identification

✅ Serial Command Interface

Control your mesh from Serial Monitor at 115200 baud.

🛠 Hardware Requirements

ESP32 boards (2 or more)

USB cable

Arduino IDE (with ESP32 board package installed)

📦 Installation
1️⃣ Install ESP32 Board Package

In Arduino IDE:

File → Preferences → Additional Boards Manager URLs


Add:

https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json


Then:

Tools → Board → Boards Manager → Search “ESP32”


Install it.

2️⃣ Upload the Code

Open ESPNowMeshFull.ino

Select your ESP32 board

Select correct COM port

Upload

Open Serial Monitor

Set baud rate to 115200

🧠 How It Works

Each ESP32 node:

Initializes ESP-NOW

Maintains a neighbor table

Routes packets based on:

RSSI strength

TTL value

Previously visited nodes (path tracking)

Automatically forwards packets

This enables true mesh communication without WiFi routers.

💻 Serial Commands

All commands start with /

Command	Description
/d	Trigger discovery
/l	List neighbors
/s <message>	Broadcast message
/t <MAC> <msg>	Send unicast message
/sr <MAC> <msg>	Send reliable message (with ACK)
/r <role>	Set node role
/ttl <value>	Set default TTL
/debug on/off	Enable or disable debug
/ping	Broadcast ping
/status	Show mesh status
/help	Show command list
📡 Example Usage
🔍 Discover Neighbors
/d

📢 Broadcast Message
/s Hello Mesh

🎯 Unicast Message
/t AABBCCDDEEFF Hello Node


(MAC can include colons or not)

🔐 Reliable Send
/sr AABBCCDDEEFF Important Message

🔄 Reliable Messaging Flow

Sender sends message with unique msg_id

Receiver sends ACK|msg_id

Sender clears pending slot

If timeout occurs → message retries

If max retries exceeded → failure callback triggered

🧭 Routing Strategy

Routing priority:

Direct neighbor with strong RSSI

Best available neighbor not in path

Broadcast fallback

Loop prevention via:

Path tracking

Duplicate cache

TTL decrement

⚙ Configuration Constants
#define MESH_MAX_PATH 8
#define MESH_TTL_DEFAULT 4
#define MESH_MAX_PENDING_ACKS 10
#define MESH_DEFAULT_ACK_TIMEOUT 3000
#define MESH_DEFAULT_ACK_RETRIES 3


You can tune these for:

Network size

Performance

Reliability

📊 Debug Mode

Enable:

/debug on


Shows:

Packet forwarding decisions

Duplicate detection

Neighbor updates

ACK handling

RSSI routing decisions

🏗 Project Structure
ESPNowMeshFull.ino
│
├── ESPNowMesh class
│   ├── Routing logic
│   ├── ACK handling
│   ├── Discovery system
│   └── Neighbor management
│
└── SerialTerminal class
    ├── Command parser
    ├── Mesh control
    └── Status display


Single-file design makes it easy to:

Clone

Modify

Experiment

📈 Scalability

Supports:

Multi-hop networks

Dozens of nodes (depending on memory)

Dynamic join/leave

Automatic stale neighbor cleanup

⚠ Limitations

ESP-NOW range limited (~100–250m typical)

Max payload: 64 bytes

Works only on same WiFi channel

No encryption enabled by default

🔮 Future Improvements (Optional Ideas)

Encryption support

OTA firmware updates

Web dashboard

JSON packet support

Battery-powered optimizations

Role-based routing priority

