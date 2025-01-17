// Map implementation -*- C++ -*-

// Copyright (C) 2001, 2002, 2003, 2004, 2005, 2006, 2007, 2008
// Free Software Foundation, Inc.
//
// This file is part of the GNU ISO C++ Library.  This library is free
// software; you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the
// Free Software Foundation; either version 2, or (at your option)
// any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License along
// with this library; see the file COPYING.  If not, write to the Free
// Software Foundation, 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301,
// USA.

// As a special exception, you may use this file as part of a free software
// library without restriction.  Specifically, if other files instantiate
// templates or use macros or inline functions from this file, or you compile
// this file and link it with other files to produce an executable, this
// file does not by itself cause the resulting executable to be covered by
// the GNU General Public License.  This exception does not however
// invalidate any other reasons why the executable file might be covered by
// the GNU General Public License.

/*
 *
 * Copyright (c) 1994
 * Hewlett-Packard Company
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies and
 * that both that copyright notice and this permission notice appear
 * in supporting documentation.  Hewlett-Packard Company makes no
 * representations about the suitability of this software for any
 * purpose.  It is provided "as is" without express or implied warranty.
 *
 *
 * Copyright (c) 1996,1997
 * Silicon Graphics Computer Systems, Inc.
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies and
 * that both that copyright notice and this permission notice appear
 * in supporting documentation.  Silicon Graphics makes no
 * representations about the suitability of this software for any
 * purpose.  It is provided "as is" without express or implied warranty.
 */

/** @file stl_map.h
 *  This is an internal header file, included by other library headers.
 *  You should not attempt to use it directly.
 */

#ifndef _STL_MAP_H
#define _STL_MAP_H 1

#include <bits/functexcept.h>
#include <bits/concept_check.h>

_GLIBCXX_BEGIN_NESTED_NAMESPACE(std, _GLIBCXX_STD_D)

  /**
   *  @brief A standard container made up of (key,value) pairs, which can be
   *  retrieved based on a key, in logarithmic time.
   *
   *  @ingroup Containers
   *  @ingroup Assoc_containers
   *
   *  Meets the requirements of a <a href="tables.html#65">container</a>, a
   *  <a href="tables.html#66">reversible container</a>, and an
   *  <a href="tables.html#69">associative container</a> (using unique keys).
   *  For a @c map<Key,T> the key_type is Key, the mapped_type is T, and the
   *  value_type is std::pair<const Key,T>.
   *
   *  Maps support bidirectional iterators.
   *
   *  The private tree data is declared exactly the same way for map and
   *  multimap; the distinction is made entirely in how the tree functions are
   *  called (*_unique versus *_equal, same as the standard).
  */
  template <typename _Key, typename _Tp, typename _Compare = std::less<_Key>,
            typename _Alloc = std::allocator<std::pair<const _Key, _Tp> > >
    class map
    {
    public:
      typedef _Key                                          key_type;
      typedef _Tp                                           mapped_type;
      typedef std::pair<const _Key, _Tp>                    value_type;
      typedef _Compare                                      key_compare;
      typedef _Alloc                                        allocator_type;

    private:
      // concept requirements
      typedef typename _Alloc::value_type                   _Alloc_value_type;
      __glibcxx_class_requires(_Tp, _SGIAssignableConcept)
      __glibcxx_class_requires4(_Compare, bool, _Key, _Key,
				_BinaryFunctionConcept)
      __glibcxx_class_requires2(value_type, _Alloc_value_type, _SameTypeConcept)

    public:
      class value_compare
      : public std::binary_function<value_type, value_type, bool>
      {
	friend class map<_Key, _Tp, _Compare, _Alloc>;
      protected:
	_Compare comp;

	value_compare(_Compare __c)
	: comp(__c) { }

      public:
	bool operator()(const value_type& __x, const value_type& __y) const
	{ return comp(__x.first, __y.first); }
      };

    private:
      /// This turns a red-black tree into a [multi]map. 
      typedef typename _Alloc::template rebind<value_type>::other 
        _Pair_alloc_type;

      typedef _Rb_tree<key_type, value_type, _Select1st<value_type>,
		       key_compare, _Pair_alloc_type> _Rep_type;

      /// The actual tree structure.
      _Rep_type _M_t;

    public:
      // many of these are specified differently in ISO, but the following are
      // "functionally equivalent"
      typedef typename _Pair_alloc_type::pointer         pointer;
      typedef typename _Pair_alloc_type::const_pointer   const_pointer;
      typedef typename _Pair_alloc_type::reference       reference;
      typedef typename _Pair_alloc_type::const_reference const_reference;
      typedef typename _Rep_type::iterator               iterator;
      typedef typename _Rep_type::const_iterator         const_iterator;
      typedef typename _Rep_type::size_type              size_type;
      typedef typename _Rep_type::difference_type        difference_type;
      typedef typename _Rep_type::reverse_iterator       reverse_iterator;
      typedef typename _Rep_type::const_reverse_iterator const_reverse_iterator;

      // [23.3.1.1] construct/copy/destroy
      // (get_allocator() is normally listed in this section, but seems to have
      // been accidentally omitted in the printed standard)
      /**
       *  @brief  Default constructor creates no elements.
       */
      map()
      : _M_t() { }

      /**
       *  @brief  Creates a %map with no elements.
       *  @param  comp  A comparison object.
       *  @param  a  An allocator object.
       */
      explicit
      map(const _Compare& __comp,
	  const allocator_type& __a = allocator_type())
      : _M_t(__comp, __a) { }

      /**
       *  @brief  %Map copy constructor.
       *  @param  x  A %map of identical element and allocator types.
       *
       *  The newly-created %map uses a copy of the allocation object
       *  used by @a x.
       */
      map(const map& __x)
      : _M_t(__x._M_t) { }

#ifdef __GXX_EXPERIMENTAL_CXX0X__
      /**
       *  @brief  %Map move constructor.
       *  @param  x  A %map of identical element and allocator types.
       *
       *  The newly-created %map contains the exact contents of @a x.
       *  The contents of @a x are a valid, but unspecified %map.
       */
      map(map&& __x)
      : _M_t(std::forward<_Rep_type>(__x._M_t)) { }
#endif

      /**
       *  @brief  Builds a %map from a range.
       *  @param  first  An input iterator.
       *  @param  last  An input iterator.
       *
       *  Create a %map consisting of copies of the elements from [first,last).
       *  This is linear in N if the range is already sorted, and NlogN
       *  otherwise (where N is distance(first,last)).
       */
      template<typename _InputIterator>
        map(_InputIterator __first, _InputIterator __last)
	: _M_t()
        { _M_t._M_insert_unique(__first, __last); }

      /**
       *  @brief  Builds a %map from a range.
       *  @param  first  An input iterator.
       *  @param  last  An input iterator.
       *  @param  comp  A comparison functor.
       *  @param  a  An allocator object.
       *
       *  Create a %map consisting of copies of the elements from [first,last).
       *  This is linear in N if the range is already sorted, and NlogN
       *  otherwise (where N is distance(first,last)).
       */
      template<typename _InputIterator>
        map(_InputIterator __first, _InputIterator __last,
	    const _Compare& __comp,
	    const allocator_type& __a = allocator_type())
	: _M_t(__comp, __a)
        { _M_t._M_insert_unique(__first, __last); }

      // FIXME There is no dtor declared, but we should have something
      // generated by Doxygen.  I don't know what tags to add to this
      // paragraph to make that happen:
      /**
       *  The dtor only erases the elements, and note that if the elements
       *  themselves are pointers, the pointed-to memory is not touched in any
       *  way.  Managing the pointer is the user's responsibility.
       */

      /**
       *  @brief  %Map assignment operator.
       *  @param  x  A %map of identical element and allocator types.
       *
       *  All the elements of @a x are copied, but unlike the copy constructor,
       *  the allocator object is not copied.
       */
      map&
      operator=(const map& __x)
      {
	_M_t = __x._M_t;
	return *this;
      }

#ifdef __GXX_EXPERIMENTAL_CXX0X__
      /**
       *  @brief  %Map move assignment operator.
       *  @param  x  A %map of identical element and allocator types.
       *
       *  The contents of @a x are moved into this map (without copying).
       *  @a x is a valid, but unspecified %map.
       */
      map&
      operator=(map&& __x)
      {
	// NB: DR 675.
	this->clear();
	this->swap(__x); 
	return *this;
      }
#endif

      /// Get a copy of the memory allocation object.
      allocator_type
      get_allocator() const
      { return _M_t.get_allocator(); }

      // iterators
      /**
       *  Returns a read/write iterator that points to the first pair in the
       *  %map.
       *  Iteration is done in ascending order according to the keys.
       */
      iterator
      begin()
      { return _M_t.begin(); }

      /**
       *  Returns a read-only (constant) iterator that points to the first pair
       *  in the %map.  Iteration is done in ascending order according to the
       *  keys.
       */
      const_iterator
      begin() const
      { return _M_t.begin(); }

      /**
       *  Returns a read/write iterator that points one past the last
       *  pair in the %map.  Iteration is done in ascending order
       *  according to the keys.
       */
      iterator
      end()
      { return _M_t.end(); }

      /**
       *  Returns a read-only (constant) iterator that points one past the last
       *  pair in the %map.  Iteration is done in ascending order according to
       *  the keys.
       */
      const_iterator
      end() const
      { return _M_t.end(); }

      /**
       *  Returns a read/write reverse iterator that points to the last pair in
       *  the %map.  Iteration is done in descending order according to the
       *  keys.
       */
      reverse_iterator
      rbegin()
      { return _M_t.rbegin(); }

      /**
       *  Returns a read-only (constant) reverse iterator that points to the
       *  last pair in the %map.  Iteration is done in descending order
       *  according to the keys.
       */
      const_reverse_iterator
      rbegin() const
      { return _M_t.rbegin(); }

      /**
       *  Returns a read/write reverse iterator that points to one before the
       *  first pair in the %map.  Iteration is done in descending order
       *  according to the keys.
       */
      reverse_iterator
      rend()
      { return _M_t.rend(); }

      /**
       *  Returns a read-only (constant) reverse iterator that points to one
       *  before the first pair in the %map.  Iteration is done in descending
       *  order according to the keys.
       */
      const_reverse_iterator
      rend() const
      { return _M_t.rend(); }

#ifdef __GXX_EXPERIMENTAL_CXX0X__
      /**
       *  Returns a read-only (constant) iterator that points to the first pair
       *  in the %map.  Iteration is done in ascending order according to the
       *  keys.
       */
      const_iterator
      cbegin() const
      { return _M_t.begin(); }

      /**
       *  Returns a read-only (constant) iterator that points one past the last
       *  pair in the %map.  Iteration is done in ascending order according to
       *  the keys.
       */
      const_iterator
      cend() const
      { return _M_t.end(); }

      /**
       *  Returns a read-only (constant) reverse iterator that points to the
       *  last pair in the %map.  Iteration is done in descending order
       *  according to the keys.
       */
      const_reverse_iterator
      crbegin() const
      { return _M_t.rbegin(); }

      /**
       *  Returns a read-only (constant) reverse iterator that points to one
       *  before the first pair in the %map.  Iteration is done in descending
       *  order according to the keys.
       */
      const_reverse_iterator
      crend() const
      { return _M_t.rend(); }
#endif

      // capacity
      /** Returns true if the %map is empty.  (Thus begin() would equal
       *  end().)
      */
      bool
      empty() const
      { return _M_t.empty(); }

      /** Returns the size of the %map.  */
      size_type
      size() const
      { return _M_t.size(); }

      /** Returns the maximum size of the %map.  */
      size_type
      max_size() const
      { return _M_t.max_size(); }

      // [23.3.1.2] element access
      /**
       *  @brief  Subscript ( @c [] ) access to %map data.
       *  @param  k  The key for which data should be retrieved.
       *  @return  A reference to the data of the (key,data) %pair.
       *
       *  Allows for easy lookup with the subscript ( @c [] )
       *  operator.  Returns data associated with the key specified in
       *  subscript.  If the key does not exist, a pair with that key
       *  is created using default values, which is then returned.
       *
       *  Lookup requires logarithmic time.
       */
      mapped_type&
      operator[](const key_type& __k)
      {
	// concept requirements
	__glibcxx_function_requires(_DefaultConstructibleConcept<mapped_type>)

	iterator __i = lower_bound(__k);
	// __i->first is greater than or equivalent to __k.
	if (__i == end() || key_comp()(__k, (*__i).first))
          __i = insert(__i, value_type(__k, mapped_type()));
	return (*__i).second;
      }

      // _GLIBCXX_RESOLVE_LIB_DEFECTS
      // DR 464. Suggestion for new member functions in standard containers.
      /**
       *  @brief  Access to %map data.
       *  @param  k  The key for which data should be retrieved.
       *  @return  A reference to the data whose key is equivalent to @a k, if
       *           such a data is present in the %map.
       *  @throw  std::out_of_range  If no such data is present.
       */
      mapped_type&
      at(const key_type& __k)
      {
	iterator __i = lower_bound(__k);
	if (__i == end() || key_comp()(__k, (*__i).first))
	  __throw_out_of_range(__N("map::at"));
	return (*__i).second;
      }

      const mapped_type&
      at(const key_type& __k) const
      {
	const_iterator __i = lower_bound(__k);
	if (__i == end() || key_comp()(__k, (*__i).first))
	  __throw_out_of_range(__N("map::at"));
	return (*__i).second;
      }

      // modifiers
      /**
       *  @brief Attempts to insert a std::pair into the %map.

       *  @param  x  Pair to be inserted (see std::make_pair for easy creation 
       *	     of pairs).

       *  @return  A pair, of which the first element is an iterator that 
       *           points to the possibly inserted pair, and the second is 
       *           a bool that is true if the pair was actually inserted.
       *
       *  This function attempts to insert a (key, value) %pair into the %map.
       *  A %map relies on unique keys and thus a %pair is only inserted if its
       *  first element (the key) is not already present in the %map.
       *
       *  Insertion requires logarithmic time.
       */
      std::pair<iterator, bool>
      insert(const value_type& __x)
      { return _M_t._M_insert_unique(__x); }

      /**
       *  @brief Attempts to insert a std::pair into the %map.
       *  @param  position  An iterator that serves as a hint as to where the
       *                    pair should be inserted.
       *  @param  x  Pair to be inserted (see std::make_pair for easy creation
       *             of pairs).
       *  @return  An iterator that points to the element with key of @a x (may
       *           or may not be the %pair passed in).
       *

       *  This function is not concerned about whether the insertion
       *  took place, and thus does not return a boolean like the
       *  single-argument insert() does.  Note that the first
       *  parameter is only a hint and can potentially improve the
       *  performance of the insertion process.  A bad hint would
       *  cause no gains in efficiency.
       *
       *  See
       *  http://gcc.gnu.org/onlinedocs/libstdc++/23_containers/howto.html#4
       *  for more on "hinting".
       *
       *  Insertion requires logarithmic time (if the hint is not taken).
       */
      iterator
      insert(iterator __position, const value_type& __x)
      { return _M_t._M_insert_unique_(__position, __x); }

      /**
       *  @brief Template function that attempts to insert a range of elements.
       *  @param  first  Iterator pointing to the start of the range to be
       *                 inserted.
       *  @param  last  Iterator pointing to the end of the range.
       *
       *  Complexity similar to that of the range constructor.
       */
      template<typename _InputIterator>
        void
        insert(_InputIterator __first, _InputIterator __last)
        { _M_t._M_insert_unique(__first, __last); }

      /**
       *  @brief Erases an element from a %map.
       *  @param  position  An iterator pointing to the element to be erased.
       *
       *  This function erases an element, pointed to by the given
       *  iterator, from a %map.  Note that this function only erases
       *  the element, and that if the element is itself a pointer,
       *  the pointed-to memory is not touched in any way.  Managing
       *  the pointer is the user's responsibility.
       */
      void
      erase(iterator __position)
      { _M_t.erase(__position); }

      /**
       *  @brief Erases elements according to the provided key.
       *  @param  x  Key of element to be erased.
       *  @return  The number of elements erased.
       *
       *  This function erases all the elements located by the given key from
       *  a %map.
       *  Note that this function only erases the element, and that if
       *  the element is itself a pointer, the pointed-to memory is not touched
       *  in any way.  Managing the pointer is the user's responsibility.
       */
      size_type
      erase(const key_type& __x)
      { return _M_t.erase(__x); }

      /**
       *  @brief Erases a [first,last) range of elements from a %map.
       *  @param  first  Iterator pointing to the start of the range to be
       *                 erased.
       *  @param  last  Iterator pointing to the end of the range to be erased.
       *
       *  This function erases a sequence of elements from a %map.
       *  Note that this function only erases the element, and that if
       *  the element is itself a pointer, the pointed-to memory is not touched
       *  in any way.  Managing the pointer is the user's responsibility.
       */
      void
      erase(iterator __first, iterator __last)
      { _M_t.erase(__first, __last); }

      /**
       *  @brief  Swaps data with another %map.
       *  @param  x  A %map of the same element and allocator types.
       *
       *  This exchanges the elements between two maps in constant
       *  time.  (It is only swapping a pointer, an integer, and an
       *  instance of the @c Compare type (which itself is often
       *  stateless and empty), so it should be quite fast.)  Note
       *  that the global std::swap() function is specialized such
       *  that std::swap(m1,m2) will feed to this function.
       */
      void
#ifdef __GXX_EXPERIMENTAL_CXX0X__
      swap(map&& __x)
#else
      swap(map& __x)
#endif
      { _M_t.swap(__x._M_t); }

      /**
       *  Erases all elements in a %map.  Note that this function only
       *  erases the elements, and that if the elements themselves are
       *  pointers, the pointed-to memory is not touched in any way.
       *  Managing the pointer is the user's responsibility.
       */
      void
      clear()
      { _M_t.clear(); }

      // observers
      /**
       *  Returns the key comparison object out of which the %map was
       *  constructed.
       */
      key_compare
      key_comp() const
      { return _M_t.key_comp(); }

      /**
       *  Returns a value comparison object, built from the key comparison
       *  object out of which the %map was constructed.
       */
      value_compare
      value_comp() const
      { return value_compare(_M_t.key_comp()); }

      // [23.3.1.3] map operations
      /**
       *  @brief Tries to locate an element in a %map.
       *  @param  x  Key of (key, value) %pair to be located.
       *  @return  Iterator pointing to sought-after element, or end() if not
       *           found.
       *
       *  This function takes a key and tries to locate the element with which
       *  the key matches.  If successful the function returns an iterator
       *  pointing to the sought after %pair.  If unsuccessful it returns the
       *  past-the-end ( @c end() ) iterator.
       */
      iterator
      find(const key_type& __x)
      { return _M_t.find(__x); }

      /**
       *  @brief Tries to locate an element in a %map.
       *  @param  x  Key of (key, value) %pair to be located.
       *  @return  Read-only (constant) iterator pointing to sought-after
       *           element, or end() if not found.
       *
       *  This function takes a key and tries to locate the element with which
       *  the key matches.  If successful the function returns a constant
       *  iterator pointing to the sought after %pair. If unsuccessful it
       *  returns the past-the-end ( @c end() ) iterator.
       */
      const_iterator
      find(const key_type& __x) const
      { return _M_t.find(__x); }

      /**
       *  @brief  Finds the number of elements with given key.
       *  @param  x  Key of (key, value) pairs to be located.
       *  @return  Number of elements with specified key.
       *
       *  This function only makes sense for multimaps; for map the result will
       *  either be 0 (not present) or 1 (present).
       */
      size_type
      count(const key_type& __x) const
      { return _M_t.find(__x) == _M_t.end() ? 0 : 1; }

      /**
       *  @brief Finds the beginning of a subsequence matching given key.
       *  @param  x  Key of (key, value) pair to be located.
       *  @return  Iterator pointing to first element equal to or greater
       *           than key, or end().
       *
       *  This function returns the first element of a subsequence of elements
       *  that matches the given key.  If unsuccessful it returns an iterator
       *  pointing to the first element that has a greater value than given key
       *  or end() if no such element exists.
       */
      iterator
      lower_bound(const key_type& __x)
      { return _M_t.lower_bound(__x); }

      /**
       *  @brief Finds the beginning of a subsequence matching given key.
       *  @param  x  Key of (key, value) pair to be located.
       *  @return  Read-only (constant) iterator pointing to first element
       *           equal to or greater than key, or end().
       *
       *  This function returns the first element of a subsequence of elements
       *  that matches the given key.  If unsuccessful it returns an iterator
       *  pointing to the first element that has a greater value than given key
       *  or end() if no such element exists.
       */
      const_iterator
      lower_bound(const key_type& __x) const
      { return _M_t.lower_bound(__x); }

      /**
       *  @brief Finds the end of a subsequence matching given key.
       *  @param  x  Key of (key, value) pair to be located.
       *  @return Iterator pointing to the first element
       *          greater than key, or end().
       */
      iterator
      upper_bound(const key_type& __x)
      { return _M_t.upper_bound(__x); }

      /**
       *  @brief Finds the end of a subsequence matching given key.
       *  @param  x  Key of (key, value) pair to be located.
       *  @return  Read-only (constant) iterator pointing to first iterator
       *           greater than key, or end().
       */
      const_iterator
      upper_bound(const key_type& __x) const
      { return _M_t.upper_bound(__x); }

      /**
       *  @brief Finds a subsequence matching given key.
       *  @param  x  Key of (key, value) pairs to be located.
       *  @return  Pair of iterators that possibly points to the subsequence
       *           matching given key.
       *
       *  This function is equivalent to
       *  @code
       *    std::make_pair(c.lower_bound(val),
       *                   c.upper_bound(val))
       *  @endcode
       *  (but is faster than making the calls separately).
       *
       *  This function probably only makes sense for multimaps.
       */
      std::pair<iterator, iterator>
      equal_range(const key_type& __x)
      { return _M_t.equal_range(__x); }

      /**
       *  @brief Finds a subsequence matching given key.
       *  @param  x  Key of (key, value) pairs to be located.
       *  @return  Pair of read-only (constant) iterators that possibly points
       *           to the subsequence matching given key.
       *
       *  This function is equivalent to
       *  @code
       *    std::make_pair(c.lower_bound(val),
       *                   c.upper_bound(val))
       *  @endcode
       *  (but is faster than making the calls separately).
       *
       *  This function probably only makes sense for multimaps.
       */
      std::pair<const_iterator, const_iterator>
      equal_range(const key_type& __x) const
      { return _M_t.equal_range(__x); }

      template<typename _K1, typename _T1, typename _C1, typename _A1>
        friend bool
        operator==(const map<_K1, _T1, _C1, _A1>&,
		   const map<_K1, _T1, _C1, _A1>&);

      template<typename _K1, typename _T1, typename _C1, typename _A1>
        friend bool
        operator<(const map<_K1, _T1, _C1, _A1>&,
		  const map<_K1, _T1, _C1, _A1>&);
    };

  /**
   *  @brief  Map equality comparison.
   *  @param  x  A %map.
   *  @param  y  A %map of the same type as @a x.
   *  @return  True iff the size and elements of the maps are equal.
   *
   *  This is an equivalence relation.  It is linear in the size of the
   *  maps.  Maps are considered equivalent if their sizes are equal,
   *  and if corresponding elements compare equal.
  */
  template<typename _Key, typename _Tp, typename _Compare, typename _Alloc>
    inline bool
    operator==(const map<_Key, _Tp, _Compare, _Alloc>& __x,
               const map<_Key, _Tp, _Compare, _Alloc>& __y)
    { return __x._M_t == __y._M_t; }

  /**
   *  @brief  Map ordering relation.
   *  @param  x  A %map.
   *  @param  y  A %map of the same type as @a x.
   *  @return  True iff @a x is lexicographically less than @a y.
   *
   *  This is a total ordering relation.  It is linear in the size of the
   *  maps.  The elements must be comparable with @c <.
   *
   *  See std::lexicographical_compare() for how the determination is made.
  */
  template<typename _Key, typename _Tp, typename _Compare, typename _Alloc>
    inline bool
    operator<(const map<_Key, _Tp, _Compare, _Alloc>& __x,
              const map<_Key, _Tp, _Compare, _Alloc>& __y)
    { return __x._M_t < __y._M_t; }

  /// Based on operator==
  template<typename _Key, typename _Tp, typename _Compare, typename _Alloc>
    inline bool
    operator!=(const map<_Key, _Tp, _Compare, _Alloc>& __x,
               const map<_Key, _Tp, _Compare, _Alloc>& __y)
    { return !(__x == __y); }

  /// Based on operator<
  template<typename _Key, typename _Tp, typename _Compare, typename _Alloc>
    inline bool
    operator>(const map<_Key, _Tp, _Compare, _Alloc>& __x,
              const map<_Key, _Tp, _Compare, _Alloc>& __y)
    { return __y < __x; }

  /// Based on operator<
  template<typename _Key, typename _Tp, typename _Compare, typename _Alloc>
    inline bool
    operator<=(const map<_Key, _Tp, _Compare, _Alloc>& __x,
               const map<_Key, _Tp, _Compare, _Alloc>& __y)
    { return !(__y < __x); }

  /// Based on operator<
  template<typename _Key, typename _Tp, typename _Compare, typename _Alloc>
    inline bool
    operator>=(const map<_Key, _Tp, _Compare, _Alloc>& __x,
               const map<_Key, _Tp, _Compare, _Alloc>& __y)
    { return !(__x < __y); }

  /// See std::map::swap().
  template<typename _Key, typename _Tp, typename _Compare, typename _Alloc>
    inline void
    swap(map<_Key, _Tp, _Compare, _Alloc>& __x,
	 map<_Key, _Tp, _Compare, _Alloc>& __y)
    { __x.swap(__y); }

#ifdef __GXX_EXPERIMENTAL_CXX0X__
  template<typename _Key, typename _Tp, typename _Compare, typename _Alloc>
    inline void
    swap(map<_Key, _Tp, _Compare, _Alloc>&& __x,
	 map<_Key, _Tp, _Compare, _Alloc>& __y)
    { __x.swap(__y); }

  template<typename _Key, typename _Tp, typename _Compare, typename _Alloc>
    inline void
    swap(map<_Key, _Tp, _Compare, _Alloc>& __x,
	 map<_Key, _Tp, _Compare, _Alloc>&& __y)
    { __x.swap(__y); }
#endif

_GLIBCXX_END_NESTED_NAMESPACE

#endif /* _STL_MAP_H */
