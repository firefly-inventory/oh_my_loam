#pragma once

#include <iomanip>
#include <memory>
#include <mutex>

// format timestamp
#define FMT_TIMESTAMP(timestamp) std::fixed << std::setprecision(3) << timestamp

#define DISALLOW_COPY_AND_ASSIGN(classname) \
  classname(const classname &) = delete;    \
  classname &operator=(const classname &) = delete;

// adapted form baidu apollo cyber/common/macros.h
#define DECLARE_SINGLETON(classname)                                           \
 public:                                                                       \
  static classname *Instance() {                                               \
    static std::unique_ptr<classname> instance{nullptr};                       \
    if (!instance) {                                                           \
      static std::once_flag flag;                                              \
      std::call_once(flag,                                                     \
                     [&] { instance.reset(new (std::nothrow) classname()); }); \
    }                                                                          \
    return instance.get();                                                     \
  }                                                                            \
                                                                               \
 private:                                                                      \
  classname() = default;                                                       \
  DISALLOW_COPY_AND_ASSIGN(classname)
