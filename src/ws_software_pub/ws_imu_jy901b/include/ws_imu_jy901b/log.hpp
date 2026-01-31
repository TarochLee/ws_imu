#pragma once
#include <glog/logging.h>

// 日志宏
#define AINFO  LOG(INFO)
#define AWARN  LOG(WARNING)
#define AERROR LOG(ERROR)
#define AFATAL LOG(FATAL)

// Debug：默认用 VLOG(1)，通过 --v=1 或环境变量 GLOG_v=1 开启
#define ADEBUG VLOG(1)

// 节流：每 N 次一次（glog 原生）
#define AINFO_EVERY_N(n)  LOG_EVERY_N(INFO, (n))
#define AWARN_EVERY_N(n)  LOG_EVERY_N(WARNING, (n))
#define AERROR_EVERY_N(n) LOG_EVERY_N(ERROR, (n))
