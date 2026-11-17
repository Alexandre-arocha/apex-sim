#include "sim/telemetry_client.hpp"

#include <algorithm>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#endif

namespace sim {
namespace {

#ifdef _WIN32
using Handle = SOCKET;
using AddrLen = int;
constexpr Handle kInvalid = INVALID_SOCKET;

void close_handle(Handle h) { closesocket(h); }

struct WinsockSession {
  WinsockSession() {
    WSADATA data;
    ok = WSAStartup(MAKEWORD(2, 2), &data) == 0;
  }
  ~WinsockSession() {
    if (ok) WSACleanup();
  }
  bool ok = false;
};

bool init_network() {
  static const WinsockSession session;
  return session.ok;
}
#else
using Handle = int;
using AddrLen = socklen_t;
constexpr Handle kInvalid = -1;

void close_handle(Handle h) { ::close(h); }
bool init_network() { return true; }
#endif

#ifdef MSG_NOSIGNAL
constexpr int kSendFlags = MSG_NOSIGNAL;  // um servidor que cai nao deve matar o processo
#else
constexpr int kSendFlags = 0;
#endif

}  // namespace

std::array<std::uint8_t, kWireSize> to_wire(const can_frame_t& frame) {
  std::array<std::uint8_t, kWireSize> record{};
  for (std::size_t i = 0; i < 4; ++i) {
    record[i] = static_cast<std::uint8_t>((frame.id >> (8 * i)) & 0xFFu);
  }
  const std::size_t dlc = std::min<std::size_t>(frame.dlc, 8);
  record[4] = static_cast<std::uint8_t>(dlc);
  for (std::size_t i = 0; i < dlc; ++i) record[5 + i] = frame.data[i];
  return record;
}

TelemetryClient::~TelemetryClient() { close(); }

bool TelemetryClient::connect(const std::string& host, std::uint16_t port) {
  close();
  if (!init_network()) return false;

  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo* found = nullptr;
  if (getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &found) != 0) return false;

  Handle connected = kInvalid;
  for (const addrinfo* a = found; a != nullptr; a = a->ai_next) {
    const Handle candidate = ::socket(a->ai_family, a->ai_socktype, a->ai_protocol);
    if (candidate == kInvalid) continue;
    if (::connect(candidate, a->ai_addr, static_cast<AddrLen>(a->ai_addrlen)) == 0) {
      connected = candidate;
      break;
    }
    close_handle(candidate);
  }
  freeaddrinfo(found);
  if (connected == kInvalid) return false;
  handle_ = static_cast<std::intptr_t>(connected);
  return true;
}

bool TelemetryClient::send(const can_frame_t& frame) {
  const auto record = to_wire(frame);
  return send_bytes(record.data(), record.size());
}

bool TelemetryClient::send_bytes(const std::uint8_t* data, std::size_t size) {
  if (handle_ < 0) return false;
  const Handle h = static_cast<Handle>(handle_);
  std::size_t sent = 0;
  while (sent < size) {
#ifdef _WIN32
    const int n = ::send(h, reinterpret_cast<const char*>(data) + sent,
                         static_cast<int>(size - sent), kSendFlags);
#else
    const ssize_t n = ::send(h, data + sent, size - sent, kSendFlags);
#endif
    if (n <= 0) {
      close();
      return false;
    }
    sent += static_cast<std::size_t>(n);
  }
  return true;
}

void TelemetryClient::close() {
  if (handle_ >= 0) close_handle(static_cast<Handle>(handle_));
  handle_ = -1;
}

}  // namespace sim
