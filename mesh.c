/* ESPNowMeshFull.ino
 *
 * Full-feature single-file merge of:
 *  - ESPNowMesh (multi-hop ESP-NOW mesh with TTL, path tracking, dedupe, RSSI routing, ACKs, discovery)
 *  - SerialTerminal (serial command interface)
 *
 * Target: ESP32 (Arduino core)
 *
 * Open Serial Monitor at 115200 baud and use commands like:
 *   /d  -> discovery
 *   /l  -> list neighbors
 *   /s  -> broadcast
 *   /t <MAC> <msg> -> unicast
 *   /sr <MAC> <msg> -> reliable send with ACK
 *   /r <role> -> set role
 */

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <stdarg.h>

// -----------------------------
// Config constants
// -----------------------------
#define MESH_MAX_PATH     8
#define MESH_PAYLOAD_LEN  64
#define MESH_CACHE_SIZE   16
#define MESH_TTL_DEFAULT  4
#define DEFAULT_WIFI_CHANNEL 1
#define MESH_MAX_PENDING_ACKS 10
#define MESH_DEFAULT_ACK_TIMEOUT 3000
#define MESH_DEFAULT_ACK_RETRIES 3
#define MESH_DEFAULT_NEIGHBOR_EXPIRY 300000UL
#define MESH_DEFAULT_CLEANUP_INTERVAL 60000UL

// -----------------------------
// ESPNowMesh Class (header + impl merged)
// -----------------------------
class ESPNowMesh {
public:
  typedef void (*MeshCallback)(const char* msg, const uint8_t* sender);
  typedef void (*AckCallback)(uint32_t msg_id, const uint8_t* dest_mac);

  struct MeshPacket {
    uint8_t sender[6];
    uint8_t receiver[6];
    uint8_t ttl;
    uint8_t path_len;
    uint8_t path[MESH_MAX_PATH][6];
    uint32_t msg_id;
    char payload[MESH_PAYLOAD_LEN];
  };

  struct Neighbor {
    uint8_t mac[6];
    String role;
    int rssi;
    unsigned long lastSeen;
  };

  ESPNowMesh();
  void begin(int rssi_threshold = -80, uint8_t wifi_channel = DEFAULT_WIFI_CHANNEL, bool long_range = false);
  void send(const char* msgText, const uint8_t* target_mac = nullptr, uint8_t ttl = MESH_TTL_DEFAULT, uint32_t msg_id = 0);
  void sendReliably(const char* msg, const uint8_t* target_mac, uint8_t ttl = MESH_TTL_DEFAULT);
  void onReceive(MeshCallback cb);
  void loop();
  void enableAutoDiscovery(unsigned long interval);
  void broadcastDiscovery();
  void enableDebug(bool enabled);
  void setUnicastForwarding(bool enabled);
  void setRetryFallback(bool enabled);
  void setFallbackToBroadcast(bool enabled);
  void setMaxRetries(uint8_t retries);
  void setAckTimeout(uint32_t timeout_ms);
  void setAckRetries(uint8_t retries);
  void onSendSuccess(AckCallback cb);
  void onSendFailure(AckCallback cb);
  Neighbor* getNeighbors(uint8_t &count);
  void setRole(const char* role);
  const char* getRole() const;
  void clearDuplicateCache();
  uint32_t generateMsgId();

private:
  // internal fields
  bool retryFallback = false;
  bool fallbackToBroadcast = true;
  uint8_t maxRetries = 2;
  bool useUnicast = true;
  bool debugMode = false;
  String selfRole = "unknown";
  int rssiThreshold;
  uint8_t cacheIndex = 0;
  unsigned long lastDiscovery = 0;
  unsigned long discoveryInterval = 0;
  uint8_t neighborCount = 0;
  uint8_t wifiChannel = DEFAULT_WIFI_CHANNEL;
  uint16_t messageSeq = 0;
  bool longRangeMode = false;

  // ack tracking
  struct PendingAck {
    bool active = false;
    uint32_t msg_id;
    uint8_t dest_mac[6];
    char payload[MESH_PAYLOAD_LEN];
    unsigned long timestamp;
    uint8_t attempts;
    uint8_t ttl;
  } pendingAcks[MESH_MAX_PENDING_ACKS];

  uint32_t ackTimeout = MESH_DEFAULT_ACK_TIMEOUT;
  uint8_t ackRetries = MESH_DEFAULT_ACK_RETRIES;
  AckCallback onSuccessCallback = nullptr;
  AckCallback onFailureCallback = nullptr;

  struct RetryState {
    MeshPacket msg;
    int attempts;
    int lastTriedIndex;
  } retryState;

  struct MsgCache {
    uint8_t sender[6];
    uint32_t msg_id;
    unsigned long timestamp;
  } cache[MESH_CACHE_SIZE];

  Neighbor neighbors[MESH_CACHE_SIZE];
  MeshCallback userCallback = nullptr;

  static ESPNowMesh* instance;
  static void _onRecvStatic(const esp_now_recv_info_t *info, const uint8_t *data, int len);
  static void _onSendStatic(const uint8_t *mac_addr, esp_now_send_status_t status);

  void _onRecv(const uint8_t* mac, const uint8_t* data, int len, int rssi);
  void updateNeighbor(const uint8_t* mac, const char* role, int rssi);
  bool isBroadcast(const uint8_t* mac);
  bool isDuplicate(uint8_t* sender, uint32_t msg_id);
  void addToCache(uint8_t* sender, uint32_t msg_id);
  bool isInPath(const uint8_t* mac, const MeshPacket& msg);
  void debugLog(const char* fmt, ...);
  bool managePeer(const uint8_t* mac, uint8_t channel = 0, bool encrypt = false);
  int findFreePendingSlot();
  void checkPendingAcks();
  void resendPendingMessage(int index);
  void clearPendingAck(int index);
  bool isPendingAckActive(int index);
  uint8_t* findBestRoute(const uint8_t* targetMac, int& bestRssi, const MeshPacket* msg = nullptr);
  void _handleAck(uint32_t ack_id, const uint8_t* sender);

  unsigned long neighborExpiryTime = MESH_DEFAULT_NEIGHBOR_EXPIRY;
  unsigned long neighborCleanupInterval = MESH_DEFAULT_CLEANUP_INTERVAL;
  unsigned long lastNeighborCleanup = 0;
  unsigned long lastCacheCleanup = 0;
  uint8_t loopTaskIndex = 0;
  unsigned long nextTaskCheck = 0;

  void cleanupMessageCache(unsigned long currentTime);
  void cleanupStaleNeighbors(unsigned long currentTime);
  void checkForDiscovery(unsigned long currentTime);
  void logNeighborStatus();
};

// static instance init
ESPNowMesh* ESPNowMesh::instance = nullptr;

ESPNowMesh::ESPNowMesh() {
  rssiThreshold = -80;
  wifiChannel = DEFAULT_WIFI_CHANNEL;
  messageSeq = 0;
  cacheIndex = 0;
  neighborCount = 0;
}

// -----------------------------
// Implementation
// -----------------------------
void ESPNowMesh::begin(int rssi_threshold, uint8_t wifi_channel, bool long_range) {
  rssiThreshold = rssi_threshold;
  wifiChannel = wifi_channel;
  longRangeMode = long_range;

  WiFi.mode(WIFI_STA);
  delay(100);
  WiFi.disconnect();
  delay(100);
  esp_wifi_set_channel(wifiChannel, WIFI_SECOND_CHAN_NONE);
  if (longRangeMode) {
    // Attempt to enable long range if available
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
  } else {
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
  }
  delay(100);

  if (esp_now_init() != ESP_OK) {
    if (debugMode) Serial.println("[MESH] ESP-NOW init failed!");
    return;
  }
  esp_now_register_recv_cb(_onRecvStatic);
  esp_now_register_send_cb(_onSendStatic);

  // add broadcast peer
  esp_now_peer_info_t peer = {};
  memset(peer.peer_addr, 0xFF, 6);
  peer.channel = wifiChannel;
  peer.encrypt = false;
  esp_now_del_peer(peer.peer_addr);
  esp_now_add_peer(&peer);

  instance = this;
  if (debugMode) Serial.println("[MESH] ESPNowMesh started");
}

uint32_t ESPNowMesh::generateMsgId() {
  uint32_t timeComponent = (millis() & 0xFFFF) << 16;
  uint32_t seqComponent = (++messageSeq) & 0xFFFF;
  return timeComponent | seqComponent;
}

void ESPNowMesh::clearDuplicateCache() {
  for (int i=0;i<MESH_CACHE_SIZE;i++){
    cache[i].msg_id = 0;
    memset(cache[i].sender,0,6);
    cache[i].timestamp = 0;
  }
  cacheIndex = 0;
  if (debugMode) debugLog("Message deduplication cache cleared");
}

void ESPNowMesh::send(const char* msgText, const uint8_t* target_mac, uint8_t ttl, uint32_t msg_id) {
  MeshPacket msg;
  WiFi.macAddress(msg.sender);

  if (target_mac) {
    memcpy(msg.receiver, target_mac, 6);
    if (!isBroadcast(target_mac)) managePeer(target_mac, wifiChannel, false);
  } else {
    memset(msg.receiver, 0xFF, 6);
  }

  msg.ttl = ttl;
  msg.path_len = 0;
  if (msg.path_len < MESH_MAX_PATH) {
    memcpy(msg.path[msg.path_len++], msg.sender, 6);
  }

  msg.msg_id = (msg_id != 0) ? msg_id : generateMsgId();

  size_t msgLen = strlen(msgText);
  if (msgLen >= MESH_PAYLOAD_LEN) msgLen = MESH_PAYLOAD_LEN - 1;
  memcpy(msg.payload, msgText, msgLen);
  msg.payload[msgLen] = '\0';

  // Unicast attempt via best route
  if (useUnicast && target_mac && !isBroadcast(msg.receiver)) {
    int bestRSSI = -128;
    uint8_t* bestMac = findBestRoute(target_mac, bestRSSI, &msg);
    if (bestMac && bestRSSI >= rssiThreshold && !isBroadcast(bestMac)) {
      managePeer(bestMac, wifiChannel, false);
      esp_err_t result = esp_now_send(bestMac, (uint8_t*)&msg, sizeof(msg));
      if (result == ESP_OK) {
        if (retryFallback) retryState = {msg, 0, 0};
        return;
      }
    }
  }

  // Broadcast fallback
  uint8_t broadcastAddr[6];
  memset(broadcastAddr, 0xFF, 6);
  managePeer(broadcastAddr, wifiChannel, false);
  esp_now_send(broadcastAddr, (uint8_t*)&msg, sizeof(msg));
}

void ESPNowMesh::sendReliably(const char* msg, const uint8_t* target_mac, uint8_t ttl) {
  if (!target_mac) {
    if (debugMode) debugLog("sendReliably requires a target MAC");
    return;
  }
  int slot = findFreePendingSlot();
  if (slot < 0) {
    if (debugMode) debugLog("No free slots for reliable message tracking");
    return;
  }
  uint32_t id = generateMsgId();
  pendingAcks[slot].active = true;
  pendingAcks[slot].msg_id = id;
  memcpy(pendingAcks[slot].dest_mac, target_mac, 6);
  strlcpy(pendingAcks[slot].payload, msg, MESH_PAYLOAD_LEN);
  pendingAcks[slot].timestamp = millis();
  pendingAcks[slot].attempts = 1;
  pendingAcks[slot].ttl = ttl;
  send(msg, target_mac, ttl, id);
}

void ESPNowMesh::broadcastDiscovery() {
  if (debugMode) Serial.println("[MESH] Broadcasting discovery");
  uint8_t b[6]; memset(b,0xFF,6);
  managePeer(b, wifiChannel, false);
  for (int i=0;i<3;i++) {
    const char discoveryMsg[] = "DISCOVERY_REQ";
    esp_now_send(b, (uint8_t*)discoveryMsg, sizeof(discoveryMsg));
    delay(random(50,150));
  }
  delay(20);
  send("DISCOVERY_REQ", nullptr, 1, 0);
}

void ESPNowMesh::enableAutoDiscovery(unsigned long interval) {
  discoveryInterval = interval;
  lastDiscovery = 0;
}

void ESPNowMesh::onReceive(MeshCallback cb) {
  userCallback = cb;
  if (debugMode) debugLog("Receive callback registered");
}

ESPNowMesh::Neighbor* ESPNowMesh::getNeighbors(uint8_t &count) {
  count = neighborCount;
  return neighbors;
}

void ESPNowMesh::setRole(const char* role) { selfRole = role; }
const char* ESPNowMesh::getRole() const { return selfRole.c_str(); }
void ESPNowMesh::enableDebug(bool enabled) { debugMode = enabled; }
void ESPNowMesh::setUnicastForwarding(bool enabled) { useUnicast = enabled; }
void ESPNowMesh::setRetryFallback(bool enabled) { retryFallback = enabled; }
void ESPNowMesh::setFallbackToBroadcast(bool enabled) { fallbackToBroadcast = enabled; }
void ESPNowMesh::setMaxRetries(uint8_t retries) { maxRetries = retries; }
void ESPNowMesh::setAckTimeout(uint32_t timeout_ms) { ackTimeout = timeout_ms; }
void ESPNowMesh::setAckRetries(uint8_t retries) { ackRetries = retries; }
void ESPNowMesh::onSendSuccess(AckCallback cb) { onSuccessCallback = cb; }
void ESPNowMesh::onSendFailure(AckCallback cb) { onFailureCallback = cb; }

int ESPNowMesh::findFreePendingSlot() {
  for (int i=0;i<MESH_MAX_PENDING_ACKS;i++) if (!pendingAcks[i].active) return i;
  return -1;
}

bool ESPNowMesh::managePeer(const uint8_t* mac, uint8_t channel, bool encrypt) {
  if (!mac) return false;
  static const uint8_t bcast[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  if (memcmp(mac, bcast, 6) == 0) return true;
  if (channel == 0) channel = wifiChannel;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = channel;
  peer.encrypt = encrypt;
#if (ESP_PLATFORM)
  // check if peer exists (ESP-IDF)
  bool exists = esp_now_is_peer_exist(mac);
  if (exists) {
    if (esp_now_mod_peer(&peer) == ESP_OK) return true;
    esp_now_del_peer(mac);
  }
#endif
  return esp_now_add_peer(&peer) == ESP_OK;
}

void ESPNowMesh::_onRecvStatic(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!instance) return;
  // small non-mesh packets (discovery) handling
  if (len < (int)sizeof(MeshPacket) && len > 0) {
    char msg[128] = {0};
    int mlen = (len < 127) ? len : 127;
    memcpy(msg, data, mlen); msg[mlen] = '\0';
    if (strcmp(msg, "DISCOVERY_REQ") == 0) {
      instance->managePeer(info->src_addr, instance->wifiChannel, false);
      char rsp[64];
      snprintf(rsp, sizeof(rsp), "DISCOVERY_RSP|%s|%lu", instance->selfRole.c_str(), millis());
      delay(random(10,50));
      esp_now_send(info->src_addr, (uint8_t*)rsp, strlen(rsp)+1);
      return;
    } else if (strncmp(msg, "DISCOVERY_RSP|", 14) == 0) {
      char *role = strchr(msg, '|');
      if (role) {
        role++;
        char *end = strchr(role, '|'); if (end) *end = '\0';
        instance->updateNeighbor(info->src_addr, role, info->rx_ctrl->rssi);
        instance->managePeer(info->src_addr, instance->wifiChannel, false);
      }
      return;
    }
  }
  instance->_onRecv(info->src_addr, data, len, info->rx_ctrl->rssi);
}

void ESPNowMesh::_onSendStatic(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Optional feedback hook; not used in main logic currently
  if (!instance) return;
  if (instance->debugMode) {
    char b[32];
    if (!mac_addr) strcpy(b, "BROADCAST");
    else sprintf(b, "%02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0],mac_addr[1],mac_addr[2],mac_addr[3],mac_addr[4],mac_addr[5]);
    instance->debugLog("Send callback: %s status=%d", b, (int)status);
  }
}

void ESPNowMesh::_onRecv(const uint8_t* mac, const uint8_t* data, int len, int rssi) {
  if (len != sizeof(MeshPacket)) {
    if (debugMode) Serial.println("[MESH] Invalid packet size");
    return;
  }
  MeshPacket msg;
  memcpy(&msg, data, sizeof(msg));
  msg.payload[MESH_PAYLOAD_LEN-1] = '\0';

  if (debugMode) {
    Serial.printf("[MESH] Received from %02X:%02X:%02X:%02X:%02X:%02X RSSI:%d MSG:%s\n",
      mac[0],mac[1],mac[2],mac[3],mac[4],mac[5], rssi, msg.payload);
  }

  bool isAckMessage = false;
  if (strncmp(msg.payload, "ACK|", 4) == 0) {
    uint32_t ack_id = strtoul(msg.payload+4, nullptr, 10);
    _handleAck(ack_id, mac);
    isAckMessage = true;
  }

  if (isDuplicate((uint8_t*)msg.sender, msg.msg_id)) {
    if (debugMode) Serial.println("[MESH] Duplicate, ignoring");
    return;
  }
  addToCache((uint8_t*)msg.sender, msg.msg_id);

  uint8_t ourMac[6]; WiFi.macAddress(ourMac);
  bool isUnicast = !(msg.receiver[0]==0xFF && msg.receiver[1]==0xFF);
  bool isForUs = isUnicast ? (memcmp(msg.receiver, ourMac, 6) == 0) : true;
  bool isInternal = (strcmp(msg.payload,"DISCOVERY_REQ")==0 ||
                     strncmp(msg.payload,"DISCOVERY_RSP|",14)==0 ||
                     isAckMessage);

  if (isForUs) {
    if (!isInternal && userCallback) userCallback(msg.payload, msg.sender);
    if (strcmp(msg.payload,"DISCOVERY_REQ")==0) {
      char rsp[64];
      snprintf(rsp, sizeof(rsp), "DISCOVERY_RSP|%s|%lu", selfRole.c_str(), millis());
      delay(random(10,50));
      send(rsp, mac, 1, 0);
      return;
    } else if (strncmp(msg.payload,"DISCOVERY_RSP|",14)==0) {
      char buf[64];
      strncpy(buf, msg.payload+14, sizeof(buf)-1); buf[sizeof(buf)-1]='\0';
      char *role = strtok(buf,"|");
      if (role) updateNeighbor(mac, role, rssi);
      return;
    } else if (isUnicast && !isAckMessage) {
      char ackMsg[32];
      snprintf(ackMsg, sizeof(ackMsg),"ACK|%u", msg.msg_id);
      send(ackMsg, msg.sender, MESH_TTL_DEFAULT, 0);
    }
  }

  // Forwarding logic
  if (msg.ttl > 0 && !isInPath(ourMac, msg) && (!isUnicast || !isForUs)) {
    if (msg.path_len < MESH_MAX_PATH) memcpy(msg.path[msg.path_len++], ourMac, 6);
    msg.ttl--;
    bool isCritical = isAckMessage || msg.ttl <= 2;
    if (rssi >= rssiThreshold || isCritical) {
      esp_err_t result = ESP_FAIL;
      if (isUnicast && useUnicast) {
        int bestRSSI;
        int routingThreshold = isCritical ? (rssiThreshold - 15) : rssiThreshold;
        uint8_t* nextHop = findBestRoute(msg.receiver, bestRSSI, &msg);
        if (nextHop && !isBroadcast(nextHop)) {
          if (bestRSSI >= routingThreshold) {
            managePeer(nextHop, wifiChannel, false);
            result = esp_now_send(nextHop, (uint8_t*)&msg, sizeof(msg));
          }
        }
      }
      if (!isUnicast || result != ESP_OK) {
        uint8_t b[6]; memset(b,0xFF,6); managePeer(b, wifiChannel, false);
        result = esp_now_send(nullptr, (uint8_t*)&msg, sizeof(msg));
      }
    } else {
      if (debugMode) Serial.printf("[MESH] Not forwarding - RSSI too low (%d < %d)\n", rssi, rssiThreshold);
    }
  }
}

// duplicate detection with timestamps
bool ESPNowMesh::isDuplicate(uint8_t* sender, uint32_t msg_id) {
  unsigned long now = millis();
  const unsigned long MESSAGE_TTL_MS = 30000UL;
  for (int i=0;i<MESH_CACHE_SIZE;i++) {
    if (cache[i].msg_id == msg_id && memcmp(cache[i].sender, sender, 6) == 0) {
      if ((now - cache[i].timestamp) < MESSAGE_TTL_MS) {
        if (debugMode) debugLog("Duplicate message ID 0x%08X", msg_id);
        return true;
      } else {
        cache[i].timestamp = now;
        if (debugMode) debugLog("Old cache entry refreshed for ID 0x%08X", msg_id);
        return false;
      }
    }
  }
  return false;
}

void ESPNowMesh::addToCache(uint8_t* sender, uint32_t msg_id) {
  memcpy(cache[cacheIndex].sender, sender, 6);
  cache[cacheIndex].msg_id = msg_id;
  cache[cacheIndex].timestamp = millis();
  if (debugMode) debugLog("Added message ID 0x%08X to cache idx %d", msg_id, cacheIndex);
  cacheIndex = (cacheIndex + 1) % MESH_CACHE_SIZE;
}

void ESPNowMesh::updateNeighbor(const uint8_t* mac, const char* role, int rssi) {
  for (uint8_t i=0;i<neighborCount;i++) {
    if (memcmp(mac, neighbors[i].mac, 6)==0) {
      neighbors[i].lastSeen = millis();
      neighbors[i].rssi = rssi;
      neighbors[i].role = role;
      if (debugMode) debugLog("Updated neighbor");
      return;
    }
  }
  if (neighborCount < MESH_CACHE_SIZE) {
    memcpy(neighbors[neighborCount].mac, mac, 6);
    neighbors[neighborCount].role = role;
    neighbors[neighborCount].rssi = rssi;
    neighbors[neighborCount].lastSeen = millis();
    neighborCount++;
    if (debugMode) debugLog("Added neighbor");
  } else {
    if (debugMode) debugLog("Neighbor table full");
  }
}

bool ESPNowMesh::isBroadcast(const uint8_t* mac) {
  static const uint8_t b[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  return memcmp(mac,b,6)==0;
}

void ESPNowMesh::debugLog(const char* fmt, ...) {
  if (!debugMode) return;
  char buf[256];
  va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  Serial.print("[MESH] "); Serial.println(buf);
}

void ESPNowMesh::checkPendingAcks() {
  unsigned long now = millis();
  for (int i=0;i<MESH_MAX_PENDING_ACKS;i++) {
    if (!pendingAcks[i].active) continue;
    unsigned long waited = now - pendingAcks[i].timestamp;
    if (waited > ackTimeout) {
      if (pendingAcks[i].attempts < ackRetries) {
        resendPendingMessage(i);
      } else {
        if (debugMode) debugLog("Message delivery failed ID 0x%08X", pendingAcks[i].msg_id);
        if (onFailureCallback) onFailureCallback(pendingAcks[i].msg_id, pendingAcks[i].dest_mac);
        clearPendingAck(i);
      }
    }
  }
}

void ESPNowMesh::resendPendingMessage(int index) {
  if (!isPendingAckActive(index)) return;
  pendingAcks[index].attempts++;
  pendingAcks[index].timestamp = millis();
  if (debugMode) debugLog("Resending message ID 0x%08X attempt %u", pendingAcks[index].msg_id, pendingAcks[index].attempts);
  send(pendingAcks[index].payload, pendingAcks[index].dest_mac, pendingAcks[index].ttl, pendingAcks[index].msg_id);
}

void ESPNowMesh::clearPendingAck(int index) {
  if (index<0||index>=MESH_MAX_PENDING_ACKS) return;
  pendingAcks[index].active = false;
}

bool ESPNowMesh::isPendingAckActive(int index) {
  if (index<0||index>=MESH_MAX_PENDING_ACKS) return false;
  return pendingAcks[index].active;
}

void ESPNowMesh::_handleAck(uint32_t ack_id, const uint8_t* sender) {
  if (debugMode) debugLog("Handling ACK 0x%08X", ack_id);
  for (int i=0;i<MESH_MAX_PENDING_ACKS;i++) {
    if (pendingAcks[i].active && pendingAcks[i].msg_id == ack_id) {
      if (onSuccessCallback) onSuccessCallback(ack_id, pendingAcks[i].dest_mac);
      clearPendingAck(i);
      return;
    }
  }
  if (debugMode) debugLog("ACK for unknown ID 0x%08X", ack_id);
}

void ESPNowMesh::loop() {
  unsigned long now = millis();
  if (now >= nextTaskCheck) {
    switch (loopTaskIndex) {
      case 0: checkForDiscovery(now); break;
      case 1: cleanupStaleNeighbors(now); break;
      case 2: cleanupMessageCache(now); break;
    }
    loopTaskIndex = (loopTaskIndex + 1) % 3;
    nextTaskCheck = now + 10;
  }
  checkPendingAcks();
}

void ESPNowMesh::checkForDiscovery(unsigned long currentTime) {
  if (discoveryInterval == 0) return;
  if (currentTime - lastDiscovery > discoveryInterval) {
    lastDiscovery = currentTime;
    broadcastDiscovery();
    if (debugMode) logNeighborStatus();
  }
}

void ESPNowMesh::logNeighborStatus() {
  debugLog("Neighbor count: %d", neighborCount);
  for (uint8_t i=0;i<neighborCount;i++){
    debugLog("Neighbor %d: %02X:%02X:%02X:%02X:%02X:%02X RSSI:%d lastSeen:%lu",
      i+1, neighbors[i].mac[0],neighbors[i].mac[1],neighbors[i].mac[2],
      neighbors[i].mac[3],neighbors[i].mac[4],neighbors[i].mac[5],
      neighbors[i].rssi, millis() - neighbors[i].lastSeen);
  }
}

void ESPNowMesh::cleanupStaleNeighbors(unsigned long currentTime) {
  if (currentTime - lastNeighborCleanup <= neighborCleanupInterval) return;
  lastNeighborCleanup = currentTime;
  int removed=0;
  for (uint8_t i=0;i<neighborCount;i++){
    if (currentTime - neighbors[i].lastSeen > neighborExpiryTime) {
      for (uint8_t j=i;j<neighborCount-1;j++) memcpy(&neighbors[j], &neighbors[j+1], sizeof(Neighbor));
      neighborCount--; i--; removed++;
    }
  }
  if (debugMode && removed>0) debugLog("Removed %d stale neighbors", removed);
}

void ESPNowMesh::cleanupMessageCache(unsigned long currentTime) {
  if (currentTime - lastCacheCleanup <= 30000UL) return;
  lastCacheCleanup = currentTime;
  const unsigned long CACHE_EXPIRY_MS = 60000UL;
  int cleaned=0;
  for (int i=0;i<MESH_CACHE_SIZE;i++){
    if (cache[i].msg_id != 0 && ((currentTime > cache[i].timestamp && (currentTime - cache[i].timestamp > CACHE_EXPIRY_MS))
        || (currentTime < cache[i].timestamp && (currentTime + (0xFFFFFFFF - cache[i].timestamp) > CACHE_EXPIRY_MS)))) {
      cache[i].msg_id = 0; memset(cache[i].sender,0,6); cache[i].timestamp = 0; cleaned++;
    }
  }
  if (debugMode && cleaned>0) debugLog("Cleaned %d expired cache entries", cleaned);
}

// Routing: RSSI-based tiered selection
uint8_t* ESPNowMesh::findBestRoute(const uint8_t* targetMac, int& bestRssi, const MeshPacket* msg) {
  bestRssi = -128;
  static uint8_t broadcastAddr[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

  // direct neighbor check
  if (targetMac) {
    for (uint8_t i=0;i<neighborCount;i++) {
      if (memcmp(targetMac, neighbors[i].mac, 6)==0) {
        if (neighbors[i].rssi >= rssiThreshold) {
          bestRssi = neighbors[i].rssi;
          return (uint8_t*)targetMac;
        } else {
          bestRssi = neighbors[i].rssi;
          break;
        }
      }
    }
  }

  // find best neighbor not in path with good signal
  int bestGood = -128; uint8_t* bestGoodNeighbor = nullptr;
  for (uint8_t i=0;i<neighborCount;i++){
    if ((msg && isInPath(neighbors[i].mac, *msg)) || (targetMac && memcmp(targetMac, neighbors[i].mac,6)==0)) continue;
    if (neighbors[i].rssi >= rssiThreshold && neighbors[i].rssi > bestGood) {
      bestGood = neighbors[i].rssi; bestGoodNeighbor = neighbors[i].mac;
    }
  }
  if (bestGoodNeighbor) { bestRssi = bestGood; return bestGoodNeighbor; }

  // fallback to direct target (weak) or any neighbor with best RSSI
  if (targetMac && bestRssi > -128) return (uint8_t*)targetMac;

  int bestPoor = -128; uint8_t* bestPoorNeighbor = nullptr;
  for (uint8_t i=0;i<neighborCount;i++){
    if ((msg && isInPath(neighbors[i].mac, *msg)) || (targetMac && memcmp(targetMac, neighbors[i].mac,6)==0)) continue;
    if (neighbors[i].rssi > bestPoor) { bestPoor = neighbors[i].rssi; bestPoorNeighbor = neighbors[i].mac; }
  }
  if (bestPoorNeighbor) { bestRssi = bestPoor; return bestPoorNeighbor; }

  bestRssi = -100;
  return broadcastAddr;
}

bool ESPNowMesh::isInPath(const uint8_t* mac, const MeshPacket& msg) {
  for (int i=0;i<msg.path_len;i++) if (memcmp(mac, msg.path[i], 6)==0) return true;
  return false;
}

// -----------------------------
// SerialTerminal (merged)
// -----------------------------
class SerialTerminal {
public:
    SerialTerminal(ESPNowMesh& meshInstance, Stream& serialPort = Serial)
        : mesh(meshInstance), serial(serialPort) {
        // defaults
        commandPrefix = '/';
        echoEnabled = true;
        promptEnabled = true;
        prompt = "> ";
        defaultTTL = MESH_TTL_DEFAULT;
    }

    void process() {
        while (serial.available() > 0) {
            char c = serial.read();
            if (echoEnabled) serial.write(c);

            if (c == '\n' || c == '\r') {
                if (commandBuffer.length() > 0) {
                    if (echoEnabled && c == '\r') serial.write('\n');
                    handleCommand(commandBuffer);
                    commandBuffer = "";
                    if (promptEnabled) serial.print(prompt);
                }
            }
            else if (c == 8 || c == 127) {
                if (commandBuffer.length() > 0) {
                    commandBuffer.remove(commandBuffer.length() - 1);
                    if (echoEnabled) { serial.write(8); serial.write(' '); serial.write(8); }
                }
            }
            else {
                commandBuffer += c;
            }
        }
    }

    void setCommandPrefix(char prefix) { commandPrefix = prefix; }
    void enableEcho(bool enable) { echoEnabled = enable; }
    void enablePrompt(bool enable) { promptEnabled = enable; }
    void setPrompt(const String& newPrompt) { prompt = newPrompt; }

private:
    ESPNowMesh& mesh;
    Stream& serial;
    char commandPrefix;
    bool echoEnabled;
    bool promptEnabled;
    String prompt;
    uint8_t defaultTTL;
    String commandBuffer;

    void handleCommand(String cmd) {
        cmd.trim();
        if (cmd.length() == 0) return;

        if (cmd.charAt(0) != commandPrefix) {
            serial.println("Commands must start with " + String(commandPrefix));
            return;
        }

        cmd = cmd.substring(1);
        int spaceIndex = cmd.indexOf(' ');
        String command, args;
        if (spaceIndex != -1) {
            command = cmd.substring(0, spaceIndex);
            args = cmd.substring(spaceIndex + 1);
        } else {
            command = cmd;
            args = "";
        }
        command.toLowerCase();

        if (command == "d" || command == "discovery") {
            cmdDiscovery();
        }
        else if (command == "l" || command == "list") {
            cmdListNeighbors();
        }
        else if (command == "s" || command == "send") {
            cmdBroadcast(args);
        }
        else if (command == "t" || command == "target") {
            cmdUnicast(args);
        }
        else if (command == "sr" || command == "sendreliable") {
            cmdReliable(args);
        }
        else if (command == "r" || command == "role") {
            cmdSetRole(args);
        }
        else if (command == "ttl") {
            cmdSetTTL(args);
        }
        else if (command == "debug") {
            cmdDebug(args);
        }
        else if (command == "ping") {
            cmdPing();
        }
        else if (command == "status") {
            printStatus();
        }
        else if (command == "help" || command == "?") {
            printHelp();
        }
        else {
            serial.println("Unknown command. Type " + String(commandPrefix) + "help or " + String(commandPrefix) + "? for available commands.");
        }
    }

    void cmdDiscovery() {
        mesh.broadcastDiscovery();
        serial.println("Discovery broadcast initiated");
    }

    void cmdListNeighbors() {
        uint8_t count;
        auto* neighbors = mesh.getNeighbors(count);

        serial.println("\n=== NEIGHBOR LIST ===");
        serial.printf("Total count: %d\n", count);

        for (int i = 0; i < count; i++) {
            serial.print("  Node ");
            serial.print(i + 1);
            serial.print(": ");
            printMac(neighbors[i].mac);
            serial.print("\n    Role: ");
            serial.print(neighbors[i].role);
            serial.print("\n    RSSI: ");
            serial.print(neighbors[i].rssi);
            serial.print(" dBm\n    Last seen: ");
            serial.print((millis() - neighbors[i].lastSeen) / 1000);
            serial.println(" seconds ago");
        }
        serial.println("=====================");
    }

    void cmdBroadcast(const String& msg) {
        if (msg.length() == 0) {
            serial.println("Error: Message cannot be empty");
            return;
        }

        mesh.send(msg.c_str(), nullptr, defaultTTL);
        serial.print("Broadcast message sent: ");
        serial.println(msg);
    }

    void cmdUnicast(const String& args) {
        int spacePos = args.indexOf(' ');
        if (spacePos == -1) {
            serial.println("Error: Format should be " + String(commandPrefix) + "t <MAC> <message>");
            return;
        }

        String macStr = args.substring(0, spacePos);
        String msg = args.substring(spacePos + 1);

        if (msg.length() == 0) {
            serial.println("Error: Message cannot be empty");
            return;
        }

        uint8_t targetMac[6];
        if (!macStringToBytes(macStr, targetMac)) {
            serial.println("Error: Invalid MAC address format. Expected 12 hex characters (with or without colons)");
            return;
        }

        serial.print("Sending to MAC: ");
        printMac(targetMac);
        serial.println();

        mesh.send(msg.c_str(), targetMac, defaultTTL);
        serial.print("Unicast message sent: ");
        serial.println(msg);
    }

    void cmdReliable(const String& args) {
        int spacePos = args.indexOf(' ');
        if (spacePos == -1) {
            serial.println("Error: Format should be " + String(commandPrefix) + "sr <MAC> <message>");
            return;
        }

        String macStr = args.substring(0, spacePos);
        String msg = args.substring(spacePos + 1);

        if (msg.length() == 0) {
            serial.println("Error: Message cannot be empty");
            return;
        }

        uint8_t targetMac[6];
        if (!macStringToBytes(macStr, targetMac)) {
            serial.println("Error: Invalid MAC address format. Expected 12 hex characters (with or without colons)");
            return;
        }

        serial.print("Sending reliable message to MAC: ");
        printMac(targetMac);
        serial.println();

        mesh.sendReliably(msg.c_str(), targetMac, defaultTTL);
        serial.print("Reliable message sent: ");
        serial.println(msg);
        serial.println("Waiting for acknowledgment...");
    }

    void cmdSetRole(const String& role) {
        if (role.length() == 0) {
            serial.println("Error: Role cannot be empty");
            return;
        }

        mesh.setRole(role.c_str());
        serial.print("Role set to: ");
        serial.println(role);
    }

    void cmdSetTTL(const String& ttlStr) {
        if (ttlStr.length() == 0) {
            serial.println("Error: TTL value required");
            return;
        }

        int ttl = ttlStr.toInt();
        if (ttl <= 0 || ttl > 10) {
            serial.println("Error: TTL must be between 1 and 10");
            return;
        }

        defaultTTL = ttl;
        serial.print("Default TTL set to: ");
        serial.println(defaultTTL);
    }

    void cmdDebug(const String& mode) {
        if (mode.length() == 0) {
            serial.println("Error: Specify 'on' or 'off'");
            return;
        }

        bool debugOn = mode.equalsIgnoreCase("on");
        mesh.enableDebug(debugOn);
        serial.print("Debug mode set to: ");
        serial.println(debugOn ? "ON" : "OFF");
    }

    void cmdPing() {
        mesh.send("PING", nullptr, defaultTTL);
        serial.println("PING broadcast sent");
    }

    void printHelp() {
        serial.println("\nAvailable commands:");
        serial.println("  /d, /discovery - Trigger discovery");
        serial.println("  /l, /list      - List neighbors");
        serial.println("  /s <message>   - Send broadcast message");
        serial.println("  /t <mac> <msg> - Send unicast message");
        serial.println("  /sr <mac> <msg>- Send reliable message with ACK");
        serial.println("  /r <role>      - Set node role");
        serial.println("  /ttl <value>   - Set default TTL");
        serial.println("  /debug on|off  - Enable/disable debug output");
        serial.println("  /status        - Show current mesh status");
        serial.println("  /ping          - Send ping to all nodes");
        serial.println("  /help or /?    - Show this help message");
    }

    void printStatus() {
        serial.println("\n=== MESH STATUS ===");
        serial.print("MAC Address: ");
        serial.println(WiFi.macAddress());
        serial.print("Role: ");
        serial.println(mesh.getRole());
        serial.print("WiFi Status: ");
        serial.print(WiFi.status());

        switch (WiFi.status()) {
            case WL_CONNECTED: serial.println(" (CONNECTED)"); break;
            case WL_DISCONNECTED: serial.println(" (DISCONNECTED)"); break;
            case WL_IDLE_STATUS: serial.println(" (IDLE)"); break;
            default: serial.println(" (OTHER)");
        }

        serial.print("RSSI: ");
        serial.print(WiFi.RSSI());
        serial.println(" dBm");

        serial.println("Mesh Settings:");
        serial.print("- Default TTL: ");
        serial.println(defaultTTL);

        cmdListNeighbors();
    }

    void printMac(const uint8_t* mac) {
        for (int i = 0; i < 6; i++) {
            serial.printf("%02X", mac[i]);
            if (i < 5) serial.print(":");
        }
    }

    bool macStringToBytes(const String& macStr, uint8_t* targetMac) {
        String processedMac = macStr;
        processedMac.replace(":", "");

        if (processedMac.length() != 12) {
            return false;
        }

        for (int i = 0; i < 6; i++) {
            char byteHex[3] = {processedMac.charAt(i*2), processedMac.charAt(i*2+1), 0};
            targetMac[i] = strtoul(byteHex, nullptr, 16);
        }
        return true;
    }
};

// -----------------------------
// Global instances and callbacks
// -----------------------------
ESPNowMesh mesh;
SerialTerminal terminal(mesh);

void appSendSuccess(uint32_t id, const uint8_t* dest) {
  Serial.printf("APP: Sent success ID 0x%08X to %02X:%02X:%02X:%02X:%02X:%02X\n",
    id, dest[0],dest[1],dest[2],dest[3],dest[4],dest[5]);
}
void appSendFailure(uint32_t id, const uint8_t* dest) {
  Serial.printf("APP: Send failure ID 0x%08X to %02X:%02X:%02X:%02X:%02X:%02X\n",
    id, dest[0],dest[1],dest[2],dest[3],dest[4],dest[5]);
}

void onMeshReceive(const char* msg, const uint8_t* sender) {
  Serial.print("📡 Received: ");
  Serial.println(msg);
  Serial.print("      From: ");
  for (int i=0;i<6;i++){ Serial.printf("%02X", sender[i]); if (i<5) Serial.print(":"); }
  Serial.println();
}

// -----------------------------
// Setup + Loop
// -----------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== ESP-NOW FULL MESH (merged) ===");
  mesh.begin(-80, DEFAULT_WIFI_CHANNEL, false);
  mesh.enableDebug(true);
  mesh.setRole("node");
  mesh.onReceive(onMeshReceive);
  mesh.onSendSuccess(appSendSuccess);
  mesh.onSendFailure(appSendFailure);
  //mesh.enableAutoDiscovery(15000); // auto discovery every 15s
  Serial.println("Type /help for commands.");
  Serial.print("> ");
}

void loop() {
  mesh.loop();
  terminal.process();
}
