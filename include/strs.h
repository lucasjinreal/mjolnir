/*
 * Copyright (c) 2020 Fagang Jin.
 *
 * This file is part of thor
 * (see manaai.cn).
 *
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */
//
// Created by jintian on 7/14/17.
//

#ifndef _T_STRING_H
#define _T_STRING_H

#include <cmath>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>

using namespace std;

namespace thor {
namespace str_util {

inline void SplitString(const std::string &s, std::vector<std::string> &v,
                        const std::string &c) {
  std::string::size_type pos1, pos2;
  pos2 = s.find(c);

  if (pos2 > s.size()) {
    // indicates delimiter not in this string
    // return this string as vector
    v.push_back(s);
  } else {
    pos1 = 0;
    while (std::string::npos != pos2) {
      v.push_back(s.substr(pos1, pos2 - pos1));

      pos1 = pos2 + c.size();
      pos2 = s.find(c, pos1);
    }
    if (pos1 != s.length()) {
      v.push_back(s.substr(pos1));
    }
  }
}

inline string join_str(const std::string &c, vector<string> s_v) {
  string result_s = "";
  for (auto it = s_v.begin(); it < s_v.end() - 1; ++it) {
    result_s += (*it + c);
  }
  result_s += s_v.back();
  return result_s;
}

inline void StripString(vector<string> s_v, const string &c) {
  for (auto it = s_v.begin(); it < s_v.end(); it++) {
    if (*it == c) {
      s_v.erase(it);
    }
  }
}

inline bool Replace(std::string &str, const std::string &from,
                    const std::string &to) {
  size_t start_pos = str.find(from);
  if (start_pos == std::string::npos)
    return false;
  str.replace(start_pos, from.length(), to);
  return true;
}
inline void ReplaceAll(std::string &str, const std::string &from,
                       const std::string &to) {
  if (from.empty())
    return;
  size_t start_pos = 0;
  while ((start_pos = str.find(from, start_pos)) != std::string::npos) {
    str.replace(start_pos, from.length(), to);
    start_pos += to.length();
  }
}

inline bool endswith(const std::string &ori, const std::string &pat) {
  return ori.size() >= pat.size() &&
         0 == ori.compare(ori.size() - pat.size(), pat.size(), pat);
}
inline bool startswith(const std::string &ori, const std::string &pat) {
  return ori.size() >= pat.size() && 0 == ori.compare(0, pat.size(), pat);
}

inline int split(const std::string &str, char ch,
                 std::vector<std::string> *result) {
  std::stringstream ss(str);
  std::string segment;
  int count = 0;
  while (std::getline(ss, segment, ch)) {
    result->push_back(segment);
    ++count;
  }
  return count;
}
inline void ltrim(std::string *str) {
  if (!str) {
    return;
  }
  str->erase(str->begin(), std::find_if(str->begin(), str->end(), [](int ch) {
               return !std::isspace(ch);
             }));
}
inline std::string ltrim(std::string str) {
  ltrim(&str);
  return str;
}

inline void rtrim(std::string *str) {
  if (!str) {
    return;
  }
  str->erase(std::find_if(str->rbegin(), str->rend(),
                          [](int ch) { return !std::isspace(ch); })
                 .base(),
             str->end());
}
inline std::string rtrim(std::string str) {
  rtrim(&str);
  return str;
}
void trim(std::string *str);
inline std::string trim(std::string str) {
  trim(&str);
  return str;
}

} // namespace str_util
} // namespace thor

#endif //_T_STRING_H
