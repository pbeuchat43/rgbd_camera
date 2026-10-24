// Copyright (c) 2026 Paul N. Beuchat. MIT License.
#pragma once
#include "rgbd_camera/provider.hpp"
#include <sys/file.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <stdexcept>

namespace rgbd_camera {
// Per-user session coordination; SDK exclusivity remains necessary across users
// and software that does not use this lock.
class DeviceLock {
 public:
  DeviceLock() = default;
  DeviceLock(const DeviceLock&) = delete;
  DeviceLock& operator=(const DeviceLock&) = delete;
  ~DeviceLock() { close(); }
  void acquire(const std::string& serial) {
    const auto directory = "/tmp/rgbd-camera-" + std::to_string(getuid());
    if (::mkdir(directory.c_str(),0700) != 0 && errno != EEXIST)
      throw std::runtime_error("cannot create camera lock directory");
    struct stat info{};
    if (lstat(directory.c_str(),&info) != 0 || !S_ISDIR(info.st_mode) || info.st_uid != getuid() || (info.st_mode & 077) != 0)
      throw std::runtime_error("unsafe camera lock directory");
    const auto path = directory + "/" + sha256(serial) + ".lock";
    fd_ = ::open(path.c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
    if (fd_ < 0) throw std::runtime_error("cannot open camera ownership lock");
    if (flock(fd_,LOCK_EX|LOCK_NB) != 0) {
      close(); throw std::runtime_error("camera already owned by another rgbd_camera session");
    }
  }
  void close() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }
 private:
  int fd_ = -1;
};
}  // namespace rgbd_camera
