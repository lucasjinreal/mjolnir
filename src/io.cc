#include "io.h"
#include <dirent.h>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <valarray>
#include <vector>

struct stat info;
typedef struct stat Stat;

using std::cerr;
using std::stringstream;
using std::vector;

namespace mjolnir {
namespace os {

bool exists(std::string p) {
  // if path exists, return true else return false
  assert(!p.empty());
  if (stat(&p[0u], &info) != 0) {
    // path not exist
    return false;
  } else {
    // this is a file
    return true;
  }
}

bool isdir(std::string path) {
  // judge is directory or file, if dir return true, if file return false
  assert(!path.empty());
  if (stat(&path[0u], &info) != 0) {
    // path not exist
    return false;
  } else if (info.st_mode & S_IFDIR) {
    // this is a directory
    return true;
  } else {
    return false;
  }
}

bool isfile(std::string path) {
  // return a path is file or not
  assert(!path.empty());
  if (stat(&path[0u], &info) != 0) {
    // path not exist
    return false;
  } else if (info.st_mode & S_IFDIR) {
    // this is a directory
    return false;
  } else {
    return true;
  }
}

// updated for some filesystem can not
// different dir and file

vector<std::string> list_files(std::string path, bool full_path) {
  assert(!path.empty());
  DIR *dp;
  struct dirent *dirP;
  struct stat file_info;

  vector<std::string> files;
  if ((dp = opendir(&path[0u])) == nullptr) {
    cerr << "dir not exist....";
  }
  while ((dirP = readdir(dp)) != nullptr) {
    // in xfs file system, d_type always be NULL
    if (!dirP->d_type) {
      std::string f_name = path + std::string("/") + std::string(dirP->d_name);
      lstat(f_name.c_str(), &file_info);
      if (S_ISDIR(file_info.st_mode)) {
        continue;
      } else {
        if (full_path) {
          files.push_back(path + std::string("/") + std::string(dirP->d_name));
        } else {
          files.emplace_back(std::string(dirP->d_name));
        }
      }
    } else {
      if (dirP->d_type == DT_REG) {
        if (full_path) {
          files.push_back(path + std::string("/") + std::string(dirP->d_name));
        } else {
          files.emplace_back(std::string(dirP->d_name));
        }
      }
    }
  }

  closedir(dp);
  return files;
}

vector<std::string> list_files_recurse(std::string path, bool full_path) {
  // return all files recursive
  vector<std::string> all_files = list_files(path, full_path);
  vector<std::string> all_dirs = list_dirs(path, full_path);
  for (auto d : all_dirs) {
    vector<std::string> n_res = list_files_recurse(d, full_path);
    all_files.insert(all_files.end(), n_res.begin(), n_res.end());
  }
  return all_files;
}

vector<std::string> list_dirs(std::string path, bool full_path) {
  assert(!path.empty());

#ifdef DEBUG
  cout << path << endl;
#endif // DEBUG

  DIR *dp;
  struct dirent *dirP;
  vector<std::string> files;
  if ((dp = opendir(&path[0u])) == NULL) {
    std::cout << "dir not exist." << std::endl;
  }

  while ((dirP = readdir(dp)) != NULL) {
    if (dirP->d_type == DT_DIR) {
      if (full_path) {
        files.push_back(path + std::string("/") + std::string(dirP->d_name));
      } else {
        files.push_back(std::string(dirP->d_name));
      }
#ifdef DEBUG
      cout << path << endl;
#endif // DEBUG
    }
  }

  closedir(dp);
  return files;
}

/**
 * list all files contains file and folders
 * @param path
 * @return
 */
vector<std::string> list_all(std::string path, bool full_path) {
  assert(!path.empty());
  DIR *dp;
  struct dirent *dirP;
  vector<std::string> files;
  if ((dp = opendir(&path[0u])) == NULL) {
    std::cout << "dir not exist." << std::endl;
  }

  while ((dirP = readdir(dp)) != NULL) {
    if (dirP->d_type == DT_REG) {
      if (full_path) {
        files.push_back(path + std::string("/") + std::string(dirP->d_name));
      } else {
        files.push_back(std::string(dirP->d_name));
      }
    } else if (dirP->d_type == DT_DIR) {
      if (full_path) {
        files.push_back(path + std::string("/") + std::string(dirP->d_name));
      } else {
        files.push_back(std::string(dirP->d_name));
      }
    }
  }

  closedir(dp);
  return files;
}

std::string join(std::string path, std::string filename) {
  assert(!path.empty());
  // path maybe /home/jin/doc1 or /home/jin/doc1/
  // make sure drop the last '/'
  std::string path_string = std::string(path);
  std::string joiner = "/";
#ifdef __WIN32
  joiner = "\\";
#endif

#ifdef _UNIX
  // unix machine
  joiner = "/";
#endif

#ifdef __APPLE__
  joiner = "/";
#endif
  vector<std::string> split_r;

  mjolnir::str_util::SplitString(path_string, split_r, joiner);
  mjolnir::str_util::StripString(split_r, "");
  std::string path_s = mjolnir::str_util::join_str(joiner, split_r);
  std::string joined_path = path_string + joiner + filename;
  return joined_path;
}

std::string parent_path(std::string path) {
  assert(!path.empty());
  // get dir name of a path
  std::string joiner = "/";
#ifdef __WIN32
  joiner = "\\";
#endif

#ifdef __unix__
  // unix machine
  joiner = "/";
#endif

#ifdef __APPLE__
  joiner = "/";
#endif
  vector<std::string> split_r;
  mjolnir::str_util::SplitString(path, split_r, joiner);
  vector<std::string> split_drop_file_name(split_r.begin(), split_r.end() - 1);
  std::string dir = mjolnir::str_util::join_str(joiner, split_drop_file_name);
  return dir;
}

/**
 * this method will return the filename of a path
 * @param path
 * @return
 */
std::string filename(std::string path) {
  assert(!path.empty());
  std::string path_str = path;
  if (exists(path)) {
    if (isdir(path)) {
      // if dir, return directly
      return path_str;
    } else {
      std::string joiner("/");
      std::string joiner_win("\\");

      std::size_t found = path_str.find(joiner);
      std::size_t found_win = path_str.find(joiner_win);

      if (found != std::string::npos) {
        // this is a path from unix
        vector<std::string> split_r;
        mjolnir::str_util::SplitString(path_str, split_r, joiner);
        return split_r[split_r.size() - 1];
      } else if (found_win != std::string::npos) {
        // this is path from windows
        vector<std::string> split_r;
        mjolnir::str_util::SplitString(path_str, split_r, joiner_win);
        return split_r[split_r.size() - 1];
      } else {
        // this is exactly a file
        return path_str;
      }
    }
  } else {
    std::cout << "can not found path or file: " + path_str << std::endl;
    return "";
  }
}

/**
 * this method will create a folder
 * @param path
 */
int do_mkdir(std::string path, mode_t mode) {
  assert(!path.empty());
  Stat st;
  int status = 0;
  if (stat(&path[0u], &st) != 0) {
    /* Directory does not exist. EEXIST for race condition */
    if (mkdir(&path[0u], mode) != 0 && errno != EEXIST)
      status = -1;
  } else if (!S_ISDIR(st.st_mode)) {
    errno = ENOTDIR;
    status = -1;
  }
  return status;
}

/**
 * this method will create a folder recursive
 * @param path
 */
int makedirs(std::string path, mode_t mode) {
  struct stat st;
  for (std::string::iterator iter = path.begin(); iter != path.end();) {
    std::string::iterator newIter = std::find(iter, path.end(), '/');
    std::string newPath = "./" + std::string(path.begin(), newIter);

    if (stat(newPath.c_str(), &st) != 0) {
      if (mkdir(newPath.c_str(), mode) != 0 && errno != EEXIST) {
        std::cout << "cannot create folder [" << newPath
                  << "] : " << strerror(errno) << std::endl;
        return -1;
      }
    } else if (!S_ISDIR(st.st_mode)) {
      errno = ENOTDIR;
      std::cout << "path [" << newPath << "] not a dir " << std::endl;
      return -1;
    } else {
      std::cout << "path [" << newPath << "] already exists " << std::endl;
    }
    iter = newIter;
    if (newIter != path.end())
      ++iter;
  }
  return 0;
}

/**
 * Get the sufix of a file, if it is a file
 * other than return itself
 * @param path
 * @return
 */
std::string suffix(std::string path) {
  assert(!path.empty());
  std::string p = std::string(path);

  vector<std::string> split_result;
  mjolnir::str_util::SplitString(path, split_result, ".");

  if (split_result.size() > 1) {
    return split_result.back();
  } else {
    // indicates this is path
    return p;
  }
}

std::string GetAbsolutePath(const std::string &prefix,
                            const std::string &relative_path) {
  if (relative_path.empty()) {
    return prefix;
  }
  // If prefix is empty or relative_path is already absolute.
  if (prefix.empty() || relative_path[0] == '/') {
    return relative_path;
  }

  if (prefix.back() == '/') {
    return prefix + relative_path;
  }
  return prefix + "/" + relative_path;
}

std::vector<std::string> glob(const std::string &pattern) {
  // glob struct resides on the stack
  glob_t glob_result;
  memset(&glob_result, 0, sizeof(glob_result));

  // do the glob operation
  int return_value = glob(pattern.c_str(), GLOB_TILDE, NULL, &glob_result);
  if (return_value != 0) {
    globfree(&glob_result);
    stringstream ss;
    ss << "glob() failed with return_value " << return_value << std::endl;
    throw std::runtime_error(ss.str());
  }
  // collect all the filenames into a std::list<std::std::string>
  vector<std::string> filenames;
  for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
    filenames.push_back(std::string(glob_result.gl_pathv[i]));
  }
  // cleanup
  globfree(&glob_result);
  // done
  return filenames;
}

} // namespace os
} // namespace mjolnir
