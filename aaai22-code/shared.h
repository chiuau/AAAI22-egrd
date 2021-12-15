#ifndef FOCTL_SHARED_H
#define FOCTL_SHARED_H

#include <iostream>
#include <vector>
#include <set>
#include <memory>
#include <mutex>
#include <random>
#include <cassert>
#include <string_view>
#include <source_location>
#include <utility>
#include <type_traits>


// #define NDEBUG

#define SHOW_ANSWER

#define IS_SHOW_COMP_TREE false

/* --------------------------------------------------------------------------------------------------
 * A singleton of global variables.
 * -------------------------------------------------------------------------------------------------- */

class Shared final {
public:

  ~Shared() = default;

  Shared(const Shared &) = delete;
  Shared(Shared &&) = delete;
  Shared &operator=(const Shared &) = delete;
  Shared &operator=(Shared &&) = delete;

  static void init(std::random_device::result_type rand_seed,
                   bool is_show_rand_seed);

  static Shared& getInstance() {
    return *Shared::instance;
  }

  [[nodiscard]]
  std::random_device::result_type getRandSeed() const {
    return rand_seed;
  }

  std::mt19937& getRng() {
    return rng;
  }

private:
  Shared() = default;

  void setRandSeed(std::random_device::result_type rand_seed);

private:

  static std::unique_ptr<Shared> instance;
  static std::once_flag flag;

  std::random_device::result_type rand_seed = 0;
  std::mt19937 rng;
};

/* --------------------------------------------------------------------------------------------------
 * ___line___() - print the line number for debugging
 *
 * Usage: ___line___()
 *        __line__("Here")
 *
 * See: https://stackoverflow.com/questions/597078/file-line-and-function-usage-in-c
 * -------------------------------------------------------------------------------------------------- */

void __line__(const std::string& message = "", const std::source_location& location = std::source_location::current());


/* --------------------------------------------------------------------------------------------------
 * remove_front() - remove some elements from the front of a vector and store the result in a new vector.
 *
 * Usage: auto v2 = remove_front(v1, 3);
 *
 * -------------------------------------------------------------------------------------------------- */

template<typename T>
std::vector<T> remove_front(const std::vector<T>& vs, int n = 1) {
  assert(n >= 1);
  std::vector<T> result;
  for(int i=n; i<vs.size(); i++) {
    result.push_back(vs[i]);
  }
  return result;
}

/* --------------------------------------------------------------------------------------------------
 * __pp__() - print something for debugging
 *
 * Usage: __pp__(v)
 *
 * -------------------------------------------------------------------------------------------------- */

template<typename T>
void __pp_impl__(T v) {
  std::cout << v;
}

template<typename T>
void __pp_impl__(const std::vector<T>& vs) {
  std::cout << "[";
  bool isFirst = true;
  for(auto& v : vs) {
    if (isFirst) {
      isFirst = false;
    } else {
      std::cout << ",";
    }
    __pp_impl__(v);
  }
  std::cout << "]";
}

void __pp_impl__(const std::vector<bool>& vs);

template<typename T>
void __pp_impl__(const std::set<T>& vs) {
  std::cout << "{";
  bool isFirst = true;
  for(auto& v : vs) {
    if (isFirst) {
      isFirst = false;
    } else {
      std::cout << ",";
    }
    __pp_impl__(v);
  }
  std::cout << "}";
}

template<typename T1, typename T2>
void __pp_impl__(const std::pair<T1, T2>& p) {
  std::cout << "(";
  __pp_impl__(p.first);
  std::cout << ",";
  __pp_impl__(p.second);
  std::cout << ")";
}


void __pp__();

template<typename T, typename... TS>
void __pp__(T&& v, TS... vs) {
  __pp_impl__(std::forward<T>(v));
  __pp__(vs...);
}



//* See https://stackoverflow.com/questions/40626433/c-how-to-specialize-a-template-using-vectort
//template<typename T>
//struct is_vector {
//  static constexpr bool value = false;
//};
//
//template<template<typename...> class C, typename U>
//struct is_vector<C<U>> {
//  static constexpr bool value = std::is_same<C<U>,std::vector<U>>::value;
//};



/* --------------------------------------------------------------------------------------------------
 * type_name(v) - Show the type information of a variable v
 *
 * Usage: std::cout << type_name<decltype(v)>() << std::endl;
 *
 * See: https://stackoverflow.com/questions/81870/is-it-possible-to-print-a-variables-type-in-standard-c
 * -------------------------------------------------------------------------------------------------- */

template <class T>
constexpr std::string_view type_name() {
  using namespace std;
#ifdef __clang__
  string_view p = __PRETTY_FUNCTION__;
    return string_view(p.data() + 34, p.size() - 34 - 1);
#elif defined(__GNUC__)
  string_view p = __PRETTY_FUNCTION__;
#  if __cplusplus < 201402
  return string_view(p.data() + 36, p.size() - 36 - 1);
#  else
  return string_view(p.data() + 49, p.find(';', 49) - 49);
#  endif
#elif defined(_MSC_VER)
  string_view p = __FUNCSIG__;
    return string_view(p.data() + 84, p.size() - 84 - 7);
#endif
}

/* --------------------------------------------------------------------------------------------------
 * unique_cast(v) - Cast a unique_ptr to a different type
 *
 * Usage: std::unique_ptr<MyClass> obj = unique_cast<MyClass>(std::make_unique<MyOtherClass>());
 *
 * See: https://stackoverflow.com/questions/11002641/dynamic-casting-for-unique-ptr
 * -------------------------------------------------------------------------------------------------- */

template <class destinationT, typename sourceT>
std::unique_ptr<destinationT> unique_cast(std::unique_ptr<sourceT>&& source)
{
  if (!source)
    return std::unique_ptr<destinationT>();

  destinationT* dest_ptr = dynamic_cast<destinationT*>(source.get());
  if(dest_ptr) {
    source.release();
    return std::unique_ptr<destinationT>(dest_ptr);
  } else {
    return std::unique_ptr<destinationT>();
  }
}


/* --------------------------------------------------------------------------------------------------
 * A hash function for integer vectors.
 *
 * See: https://stackoverflow.com/questions/20511347/a-good-hash-function-for-a-vector/20511429
 * See: https://en.cppreference.com/w/cpp/utility/hash
 * See: https://en.cppreference.com/w/cpp/container/vector_bool/hash
 * See: https://www.boost.org/doc/libs/1_76_0/doc/html/hash/reference.html#header.boost.container_hash.hash_hpp
 * -------------------------------------------------------------------------------------------------- */

namespace std
{
  template<> struct hash<std::vector<int>> {
    std::size_t operator()(std::vector<int> const& vec) const {
      std::size_t seed = vec.size();
      for(auto& i : vec) {
        seed ^= static_cast<uint32_t>(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      }
      return seed;
    }
  };
}




#endif //FOCTL_SHARED_H
