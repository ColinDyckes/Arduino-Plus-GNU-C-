// TR1 functional_hash.h header -*- C++ -*-

// Copyright (C) 2007 Free Software Foundation, Inc.
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

/** @file tr1/functional_hash.h
 *  This is an internal header file, included by other library headers.
 *  You should not attempt to use it directly.
 */

#ifndef _GLIBCXX_TR1_FUNCTIONAL_HASH_H
#define _GLIBCXX_TR1_FUNCTIONAL_HASH_H 1

#pragma GCC system_header

#if defined(_GLIBCXX_INCLUDE_AS_CXX0X)
#  error TR1 header cannot be included from C++0x header
#endif

#if defined(_GLIBCXX_INCLUDE_AS_TR1)
#  include <tr1_impl/functional_hash.h>
#else
#  define _GLIBCXX_INCLUDE_AS_TR1
#  define _GLIBCXX_BEGIN_NAMESPACE_TR1 namespace tr1 {
#  define _GLIBCXX_END_NAMESPACE_TR1 }
#  define _GLIBCXX_TR1 tr1::
#  include <tr1_impl/functional_hash.h>
#  undef _GLIBCXX_TR1
#  undef _GLIBCXX_END_NAMESPACE_TR1
#  undef _GLIBCXX_BEGIN_NAMESPACE_TR1
#  undef _GLIBCXX_INCLUDE_AS_TR1
#endif

#endif // _GLIBCXX_TR1_FUNCTIONAL_HASH_H
