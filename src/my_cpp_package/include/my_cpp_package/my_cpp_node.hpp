#ifndef MY_CPP_PACKAGE_MY_CPP_NODE_HPP_
#define MY_CPP_PACKAGE_MY_CPP_NODE_HPP_

namespace my_cpp_package {

int add(int lhs, int rhs);

inline int subtract(int lhs, int rhs) {
  return lhs - rhs;
}

}  // namespace my_cpp_package

#endif  // MY_CPP_PACKAGE_MY_CPP_NODE_HPP_
