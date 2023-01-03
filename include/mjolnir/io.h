
#ifndef _T_OS_H
#define _T_OS_H

#ifdef _MSC_VER
#else
#include <glob.h>
#endif
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <cassert>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _WIN32
#include <io.h>
typedef int mode_t;

/// @Note If STRICT_UGO_PERMISSIONS is not defined, then setting Read for any
///       of User, Group, or Other will set Read for User and setting Write
///       will set Write for User.  Otherwise, Read and Write for Group and
///       Other are ignored.
///
/// @Note For the POSIX modes that do not have a Windows equivalent, the modes
///       defined here use the POSIX values left shifted 16 bits.

static const mode_t S_ISUID = 0x08000000;        ///< does nothing
static const mode_t S_ISGID = 0x04000000;        ///< does nothing
static const mode_t S_ISVTX = 0x02000000;        ///< does nothing
static const mode_t S_IRUSR = mode_t(_S_IREAD);  ///< read by user
static const mode_t S_IWUSR = mode_t(_S_IWRITE); ///< write by user
static const mode_t S_IXUSR = 0x00400000;        ///< does nothing
#ifndef STRICT_UGO_PERMISSIONS
static const mode_t S_IRGRP = mode_t(_S_IREAD);  ///< read by *USER*
static const mode_t S_IWGRP = mode_t(_S_IWRITE); ///< write by *USER*
static const mode_t S_IXGRP = 0x00080000;        ///< does nothing
static const mode_t S_IROTH = mode_t(_S_IREAD);  ///< read by *USER*
static const mode_t S_IWOTH = mode_t(_S_IWRITE); ///< write by *USER*
static const mode_t S_IXOTH = 0x00010000;        ///< does nothing
#else
static const mode_t S_IRGRP = 0x00200000; ///< does nothing
static const mode_t S_IWGRP = 0x00100000; ///< does nothing
static const mode_t S_IXGRP = 0x00080000; ///< does nothing
static const mode_t S_IROTH = 0x00040000; ///< does nothing
static const mode_t S_IWOTH = 0x00020000; ///< does nothing
static const mode_t S_IXOTH = 0x00010000; ///< does nothing
#endif
#endif

using std::string;
using std::vector;

namespace mjolnir {
namespace os {

bool exists(string path);
bool isdir(string path);
bool isfile(string path);

vector<string> list_files(string path, bool full_path);
vector<string> list_files_recurse(string path, bool full_path);
vector<string> list_dirs(string path, bool full_path);
vector<string> list_all(string path, bool full_path);

string join(string path, string filename);
string parent_path(string path);
string filename(string path);
int do_mkdir(string path, mode_t mode);
int makedirs(string path, mode_t mode);
string abs_path(const string &path);
string suffix(string path);

std::string GetAbsolutePath(const std::string &prefix,
                            const std::string &relative_path);
inline std::string GetFileName(const std::string &path) {
  std::string filename;
  std::string::size_type loc = path.rfind('/');
  if (loc == std::string::npos) {
    filename = path;
  } else {
    filename = path.substr(loc + 1);
  }
  return filename;
}

std::vector<std::string> glob(const std::string &pattern);

} // namespace os

namespace str_util {

template <typename... Args>
std::string string_format(const std::string &format, Args... args) {
  int size_s = std::snprintf(nullptr, 0, format.c_str(), args...) +
               1; // Extra space for '\0'
  if (size_s <= 0) {
    throw std::runtime_error("Error during formatting.");
  }
  auto size = static_cast<size_t>(size_s);
  std::unique_ptr<char[]> buf(new char[size]);
  std::snprintf(buf.get(), size, format.c_str(), args...);
  return std::string(buf.get(),
                     buf.get() + size - 1); // We don't want the '\0' inside
}

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

} // namespace mjolnir

#define CHECK_NULL(param) (if (param == NULL) cout << "got null.\n";)
#endif //_OS_H
