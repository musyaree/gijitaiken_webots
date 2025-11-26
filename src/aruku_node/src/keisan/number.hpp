#ifndef KEISAN__NUMBER_HPP_
#define KEISAN__NUMBER_HPP_

#include <cmath>

namespace keisan
{

// HANYA PERTAHANKAN VERSI INI
// Fungsi ini menskalakan `value` secara proporsional dari rentang [0, from_max] ke [0, to_max]
template<typename T>
inline T scale(const T & value, const T & from_max, const T & to_max)
{
  return value * to_max / from_max;
}

template<typename T>
inline T wrap(T value, const T & min_value, const T & max_value)
{
  T range = max_value - min_value;
  while (value < min_value) {
    value += range;
  }
  while (value > max_value) {
    value -= range;
  }
  return value;
}

}  // namespace keisan

#endif  // KEISAN__NUMBER_HPP_