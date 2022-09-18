/**
 * This file is part of RMPCPP
 *
 * Copyright (C) 2020 Michael Pantic <mpantic at ethz dot ch>
 *
 * RMPCPP is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RMPCPP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RMPCPP. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef RMPCPP_UTIL_POLICY_VECTOR_RANGE_H_
#define RMPCPP_UTIL_POLICY_VECTOR_RANGE_H_
#include <rmpcpp/core/geometry_base.h>

namespace rmpcpp {

/**
 * Range iterator for multi-dimensional vectors.
 * By supplying a start vector, an end vector and and increment vector,
 * this returns an iterator that goes steps through the resulting regular grid.
 *
 * @tparam Vector Inherited from Eigen::Vectord. Defines size.
 */
template <class Vector>
class VectorRange {
 public:
  /**
   * Internal Subclass that defines the
   * actual iterator that is being returned.
   * Note that the current position is no the actual numeric value
   * of that position, but an integer cardinal number that corresponds
   * to the current id of the "grid" location.
   *
   * Example:
   *  start is [-1, -1]
   *  end is [1, 1,]
   *  inc is [0.5, 0.5]
   *
   *  so a tick value of [1,2] corresponds to the number
   *  start + [1 *0.5, 2*0.5] = -0.5, 0.0
   *
   *  This is done to avoid rounding errors by stacking many float numbers
   *  and to have a clear upfront number of elements that the iterator
   *  is going through.
   */
  class ConstVectorRangeIterator {
   public:
    typedef ConstVectorRangeIterator self_type;
    typedef Vector value_type;
    typedef std::forward_iterator_tag iterator_category;

    ConstVectorRangeIterator(const VectorRange<Vector>* parent, Vector current)
        : current_(current), parent_(parent) {}

    self_type operator++() {
      self_type i = *this;
      current_ = parent_->getNext(current_);
      return i;
    }

    self_type operator++(int junk) {
      current_ = parent_->getNext(current_);
      return *this;
    }

    const value_type operator*() { return parent_->getValue(current_); }

    bool operator==(const self_type& rhs) {
      return current_ == rhs.current_ && parent_ == rhs.parent_;
    }

    bool operator!=(const self_type& rhs) {
      return current_ != rhs.current_ || parent_ != rhs.parent_;
    }

   private:
    Vector current_;
    const VectorRange<Vector>* parent_;
  };

  /**
   * Constructor that fully parametrizes the iterator.
   * @param start Start position (sort of "left upper corner" of a grid)
   * @param end  End position (sort of the "right lower corner" of a grid)
   * @param inc  Increments for each dimension.
   */
  VectorRange(const Vector& start, const Vector& end, const Vector& inc)
      : start_range_(start), end_range_(end), inc_(inc) {
    // calculates how many individual increments in each dimension happen to reach
    // the end
    ticks_ = Vector(((end_range_ - start_range_).array() / inc_.array()).floor());
    ticks_ = (ticks_.array() < 0).select(0, ticks_);          // set all elements smaller 0 to 0.
    ticks_ = (!ticks_.array().isFinite()).select(0, ticks_);  // set all elements NAN to 0.

    ticks_plus_one_ = ticks_;

    // Artificial end position, as the convention for iterators is to return
    // last+1  as their end.
    ticks_plus_one_[0] += 1;
  }

  /**
   * Calculates the length of the iterator.
   *  Assumes that the END of the range is included too.
   *  todo(mpantic): Logic is a bit convoluted here, maybe there's a nicer way.
   */
  int length() {
    // return multiplicative digital root :-) (Querprodukt)
    int product = 1;
    for (int i = 0; i < ticks_.size(); ++i) {
      product *= (ticks_[i] == 0.0 ? 1.0 : ticks_[i] + 1.0);  // use 1 for values that have a zero.
    }
    return product;
  };

  /**
   * Returns the first iterator element.
   * Corresponds to all ticks in the vector to be zero.
   */
  ConstVectorRangeIterator begin() const { return ConstVectorRangeIterator(this, Vector::Zero()); }

  /**
   * Returns the end marker (last element + 1)
   */
  ConstVectorRangeIterator end() const { return ConstVectorRangeIterator(this, ticks_plus_one_); }

 private:
  /**
   * Converts the current position of the loop (ticks)
   * to the value this represents.
   */
  Vector getValue(Vector& ticks) const {
    return start_range_ + (ticks.array() * inc_.array()).matrix();
  }

  /**
   *  Returns the next tick vector based on a current position.
   */
  Vector getNext(Vector& current_ticks) const {
    Vector next(current_ticks);
    next[0]++;

    uint i = 0;
    while (next[i] > ticks_[i]) {
      next[i] = 0;
      i++;
      if (i < current_ticks.rows()) {
        next[i]++;
      } else {
        return ticks_plus_one_;
      }
    }
    return next;
  }

  const Vector start_range_, end_range_, inc_;
  Vector ticks_;  ///< integer ticks to increment through
  Vector ticks_plus_one_;
};

}  // namespace rmpcpp

#endif  // RMPCPP_UTIL_POLICY_VECTOR_RANGE_H_
