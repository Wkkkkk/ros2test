//
// Created by zhihui on 1/15/20.
//

#ifndef CPP_INTRA_COMMON_H
#define CPP_INTRA_COMMON_H

#include <string>
#include <thread>

/**
 * A small convenience function for converting a thread ID to a string
 **/
inline std::string string_thread_id() {
    auto hashed = std::hash<std::thread::id>()(std::this_thread::get_id());
    return std::to_string(hashed);
}

#endif //CPP_INTRA_COMMON_H
