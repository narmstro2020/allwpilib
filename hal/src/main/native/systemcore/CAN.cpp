// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/CAN.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <cstdio>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <wpi/DenseMap.h>
#include <wpi/mutex.h>
#include <wpi/timestamp.h>

#include "PortsInternal.h"
#include "hal/Errors.h"
#include "hal/Threads.h"
#include "wpinet/EventLoopRunner.h"
#include "wpinet/uv/Poll.h"
#include "wpinet/uv/Timer.h"

namespace {

static constexpr uint32_t MatchingBitMask = CAN_EFF_MASK | CAN_RTR_FLAG;

static_assert(CAN_RTR_FLAG == HAL_CAN_IS_FRAME_REMOTE);
static_assert(CAN_EFF_FLAG == HAL_CAN_IS_FRAME_11BIT);

uint32_t MapMessageIdToSocketCan(uint32_t id) {
  // Message and RTR map directly
  uint32_t toRet = id & MatchingBitMask;

  // Reverse the 11 bit flag
  if ((id & HAL_CAN_IS_FRAME_11BIT) == 0) {
    toRet |= CAN_EFF_FLAG;
  }

  return toRet;
}

uint32_t MapSocketCanToMessageId(uint32_t id) {
  // Message and RTR map directly
  uint32_t toRet = id & MatchingBitMask;

  // Reverse the 11 bit flag
  if ((id & CAN_EFF_FLAG) == 0) {
    toRet |= HAL_CAN_IS_FRAME_11BIT;
  }

  return toRet;
}

struct FrameStore {
  canfd_frame frame;
  uint64_t timestamp{0};
};

struct SocketCanState {
  wpi::EventLoopRunner readLoopRunner;
  wpi::EventLoopRunner writeLoopRunner;
  wpi::mutex writeMutex[hal::kNumCanBuses];
  int socketHandle[hal::kNumCanBuses];
  // ms to count/timer map
  wpi::DenseMap<uint16_t, std::pair<size_t, std::weak_ptr<wpi::uv::Timer>>>
      timers;
  // ms to bus mask/packet
  wpi::DenseMap<uint16_t,
                std::array<std::optional<canfd_frame>, hal::kNumCanBuses>>
      timedFrames;
  // packet to time
  wpi::DenseMap<uint32_t, std::array<uint16_t, hal::kNumCanBuses>> packetToTime;

  wpi::mutex readMutex[hal::kNumCanBuses];
  // TODO(thadhouse) we need a MUCH better way of doing this masking
  wpi::DenseMap<uint32_t, FrameStore> readFrames[hal::kNumCanBuses];

  bool InitializeBuses();

  void TimerCallback(uint16_t time);

  void RemovePeriodic(uint8_t busMask, uint32_t messageId);
  void AddPeriodic(wpi::uv::Loop& loop, uint8_t busMask, uint16_t time,
                   const canfd_frame& frame);
};

}  // namespace

static SocketCanState* canState;

namespace hal::init {
void InitializeCAN() {
  canState = new SocketCanState{};
}
}  // namespace hal::init

bool SocketCanState::InitializeBuses() {
  bool success = true;
  readLoopRunner.ExecSync([this, &success](wpi::uv::Loop& loop) {
    int32_t status = 0;
    HAL_SetCurrentThreadPriority(true, 50, &status);
    if (status != 0) {
      std::printf("Failed to set CAN thread priority\n");
    }

    for (int i = 0; i < hal::kNumCanBuses; i++) {
      std::scoped_lock lock{writeMutex[i]};
      socketHandle[i] = socket(PF_CAN, SOCK_RAW, CAN_RAW);
      if (socketHandle[i] == -1) {
        success = false;
        return;
      }

      ifreq ifr;
      std::snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "can%d", i);

      if (ioctl(socketHandle[i], SIOCGIFINDEX, &ifr) == -1) {
        success = false;
        return;
      }

      sockaddr_can addr;
      std::memset(&addr, 0, sizeof(addr));
      addr.can_family = AF_CAN;
      addr.can_ifindex = ifr.ifr_ifindex;

      if (bind(socketHandle[i], reinterpret_cast<const sockaddr*>(&addr),
               sizeof(addr)) == -1) {
        success = false;
        return;
      }

      std::printf("Successfully bound to can interface %d\n", i);

      auto poll = wpi::uv::Poll::Create(loop, socketHandle[i]);
      if (!poll) {
        success = false;
        return;
      }

      poll->pollEvent.connect(
          [this, fd = socketHandle[i], canIndex = i](int mask) {
            if (mask & UV_READABLE) {
              canfd_frame frame;
              int rVal = read(fd, &frame, sizeof(frame));
              if (rVal <= 0) {
                // TODO(thadhouse) error handling
                return;
              }
              if (frame.can_id & CAN_ERR_FLAG) {
                // Do nothing if this is an error frame
                return;
              }

              uint32_t messageId = MapSocketCanToMessageId(frame.can_id);
              uint64_t timestamp = wpi::Now();
              // Ensure FDF flag is set for the read later.
              if (rVal == CANFD_MTU) {
                frame.flags = CANFD_FDF;
              }

              std::scoped_lock lock{readMutex[canIndex]};
              auto& msg = readFrames[canIndex][messageId];
              msg.frame = frame;
              msg.timestamp = timestamp;
            }
          });

      poll->Start(UV_READABLE);
    }
  });
  return success;
}

void SocketCanState::TimerCallback(uint16_t time) {
  auto& busFrames = timedFrames[time];
  for (size_t i = 0; i < busFrames.size(); i++) {
    const auto& frame = busFrames[i];
    if (!frame.has_value()) {
      continue;
    }
    std::scoped_lock lock{writeMutex[i]};
    int mtu = (frame->flags & CANFD_FDF) ? CANFD_MTU : CAN_MTU;
    send(canState->socketHandle[i], &*frame, mtu, 0);
  }
}

void SocketCanState::RemovePeriodic(uint8_t busId, uint32_t messageId) {
  // Find time, and remove from map
  auto& time = packetToTime[messageId][busId];
  auto storedTime = time;
  time = 0;

  // Its already been removed
  if (storedTime == 0) {
    return;
  }

  // Reset frame
  timedFrames[storedTime][busId].reset();

  auto& timer = timers[storedTime];
  // Stop the timer
  timer.first--;
  if (timer.first == 0) {
    if (auto l = timer.second.lock()) {
      l->Stop();
    }
  }
}

void SocketCanState::AddPeriodic(wpi::uv::Loop& loop, uint8_t busId,
                                 uint16_t time, const canfd_frame& frame) {
  packetToTime[frame.can_id][busId] = time;
  timedFrames[time][busId] = frame;
  auto& timer = timers[time];
  timer.first++;
  if (timer.first == 1) {
    auto newTimer = wpi::uv::Timer::Create(loop);
    newTimer->timeout.connect([this, time] { TimerCallback(time); });
    newTimer->Start(wpi::uv::Timer::Time{time}, wpi::uv::Timer::Time{time});
  }
}

namespace hal {
bool InitializeCanBuses() {
  return canState->InitializeBuses();
}
}  // namespace hal

namespace {}  // namespace

extern "C" {

void HAL_CAN_SendMessage(int32_t busId, uint32_t messageId,
                         const struct HAL_CANMessage* message, int32_t periodMs,
                         int32_t* status) {
  if (busId < 0 || busId >= hal::kNumCanBuses) {
    *status = PARAMETER_OUT_OF_RANGE;
    return;
  }

  messageId = MapMessageIdToSocketCan(messageId);

  // TODO determine on the real roborio is a non periodic send removes any
  // periodic send.
  if (periodMs == HAL_CAN_SEND_PERIOD_STOP_REPEATING) {
    canState->writeLoopRunner.ExecSync([messageId, busId](wpi::uv::Loop&) {
      canState->RemovePeriodic(busId, messageId);
    });

    *status = 0;
    return;
  }

  canfd_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = messageId;
  frame.flags |=
      (message->flags & HAL_CANFlags::HAL_CAN_FD_DATALENGTH) ? CANFD_FDF : 0;
  frame.flags |=
      (message->flags & HAL_CANFlags::HAL_CAN_FD_BITRATESWITCH) ? CANFD_BRS : 0;
  if (message->dataSize) {
    auto size =
        (std::min)(message->dataSize, static_cast<uint8_t>(sizeof(frame.data)));
    std::memcpy(frame.data, message->data, size);
    frame.len = size;
  }

  int mtu = (message->flags & HAL_CANFlags::HAL_CAN_FD_DATALENGTH) ? CANFD_MTU
                                                                   : CAN_MTU;
  {
    std::scoped_lock lock{canState->writeMutex[busId]};
    int result = send(canState->socketHandle[busId], &frame, mtu, 0);
    if (result != mtu) {
      // TODO(thadhouse) better error
      *status = HAL_ERR_CANSessionMux_InvalidBuffer;
      return;
    }
  }

  if (periodMs > 0) {
    canState->writeLoopRunner.ExecAsync(
        [busId, periodMs, frame](wpi::uv::Loop& loop) {
          canState->AddPeriodic(loop, busId, periodMs, frame);
        });
  }
}
void HAL_CAN_ReceiveMessage(int32_t busId, uint32_t messageId,
                            struct HAL_CANReceiveMessage* message,
                            int32_t* status) {
  if (busId < 0 || busId >= hal::kNumCanBuses) {
    message->message.dataSize = 0;
    message->timeStamp = 0;
    *status = PARAMETER_OUT_OF_RANGE;
    return;
  }

  std::scoped_lock lock{canState->readMutex[busId]};

  auto& msg = canState->readFrames[busId][messageId];
  if (msg.timestamp == 0) {
    message->message.dataSize = 0;
    message->timeStamp = 0;
    *status = HAL_ERR_CANSessionMux_MessageNotFound;
    return;
  }

  message->message.flags = HAL_CANFlags::HAL_CAN_NO_FLAGS;
  message->message.flags |= (msg.frame.flags & CANFD_FDF)
                                ? HAL_CANFlags::HAL_CAN_FD_DATALENGTH
                                : HAL_CANFlags::HAL_CAN_NO_FLAGS;
  message->message.flags |= (msg.frame.flags & CANFD_BRS)
                                ? HAL_CANFlags::HAL_CAN_FD_BITRATESWITCH
                                : HAL_CANFlags::HAL_CAN_NO_FLAGS;

  message->message.dataSize = msg.frame.len;
  if (msg.frame.len > 0) {
    std::memcpy(message->message.data, msg.frame.data, msg.frame.len);
  }

  message->timeStamp = msg.timestamp;
  *status = 0;
  msg.timestamp = 0;
  return;
}
HAL_CANStreamHandle HAL_CAN_OpenStreamSession(int32_t busId, uint32_t messageId,
                                              uint32_t messageIDMask,
                                              uint32_t maxMessages,
                                              int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}
void HAL_CAN_CloseStreamSession(HAL_CANStreamHandle sessionHandle) {}
void HAL_CAN_ReadStreamSession(HAL_CANStreamHandle sessionHandle,
                               struct HAL_CANStreamMessage* messages,
                               uint32_t messagesToRead, uint32_t* messagesRead,
                               int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return;
}
void HAL_CAN_GetCANStatus(int32_t busId, float* percentBusUtilization,
                          uint32_t* busOffCount, uint32_t* txFullCount,
                          uint32_t* receiveErrorCount,
                          uint32_t* transmitErrorCount, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return;
}
}  // extern "C"
