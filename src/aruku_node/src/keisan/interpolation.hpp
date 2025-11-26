#ifndef KEISAN__INTERPOLATION_HPP_
#define KEISAN__INTERPOLATION_HPP_

namespace keisan
{

template<typename T>
T smooth(const T & current, const T & target, const double & ratio)
{
  return current * ratio + target * (1.0 - ratio);
}

} // namespace keisan

#endif // KEISAN__INTERPOLATION_HPP_