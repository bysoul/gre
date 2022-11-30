#ifndef __LIPP_PROB_BASE_H__
#define __LIPP_PROB_BASE_H__

#include <limits>
#include <cmath>
#include <cstdlib>
#include <algorithm>

namespace lipp_prob {

/*template <class T>
class LinearModelInterface{
    virtual inline int predict(T key) const=0;
    virtual inline double predict_double(T key) const=0;
    virtual inline void clear()=0;
};*/
// Linear regression model
template <class T>
class LinearModel
{
public:
    long double a = 0; // slope
    long double b = 0; // intercept

    LinearModel() = default;
    LinearModel(long double a, long double b) : a(a), b(b) {}
    explicit LinearModel(const LinearModel &other) : a(other.a), b(other.b) {}

    inline int predict(T key) const
    {
        return std::floor(a * static_cast<long double>(key) + b);
    }

    inline double predict_double(T key) const
    {
        return a * static_cast<long double>(key) + b;
    }
    inline void clear(){
      a=b=0;
    }
    inline void train_two(long double mid1_key,long double mid2_key,long double mid1_target,long double mid2_target){
      a = (mid2_target - mid1_target) / (mid2_key - mid1_key);
      b = mid1_target - a * mid1_key;
    }
};

template <class T>
class MultiLinearModel
{
public:
    long double mid =0;
    long double a1 = 0; // slope
    long double b1 = 0; // intercept
    long double a2 = 0; // slope
    long double b2 = 0; // intercept

    MultiLinearModel() = default;
    MultiLinearModel(T mid,long double a1, long double b1,long double a2, long double b2) :mid(mid), a1(a1), b1(b1),a2(a2), b2(b2) {}
    explicit MultiLinearModel(const MultiLinearModel &other) : mid(other.mid),a1(other.a1), b1(other.b1),a2(other.a2),b2(other.b2) {}

    inline int predict(T key) const
    {
      if(static_cast<long double>(key)<mid){
        return std::floor(a1 * static_cast<long double>(key) + b1);
      }
      return std::floor(a2 * static_cast<long double>(key) + b2);
    }

    inline double predict_double(T key) const
    {
      if(static_cast<long double>(key)<mid) {
        return a1 * static_cast<long double>(key) + b1;
      }
      return a2 * static_cast<long double>(key) + b2;
    }
    inline void clear(){
      a1=b1=a2=b2=mid=0;
    }
    inline void train_two(long double mid1_key,long double mid2_key,long double mid1_target,long double mid2_target){
      a1= a2= (mid2_target - mid1_target) / (mid2_key - mid1_key);
      b1= b2= mid1_target - a1 * mid1_key;
      mid=(mid1_key+mid2_key)/2;
    }
};

}

#endif
