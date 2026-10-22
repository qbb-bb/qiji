#pragma once

#include <string>

namespace danmucamera
{
// 各类标定共用的标定板设定；检测与求解实现留在具体算法目录。
struct BoardSpec
{
  std::string type{"chessboard"};
  int columns{7};
  int rows{6};
  double spacing{0.06};
  double marker_length{0.04};
  int dictionary_id{0};
};
}  // namespace danmucamera
