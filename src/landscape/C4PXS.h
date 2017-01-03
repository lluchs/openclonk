/*
 * OpenClonk, http://www.openclonk.org
 *
 * Copyright (c) 1998-2000, Matthes Bender
 * Copyright (c) 2001-2009, RedWolf Design GmbH, http://www.clonk.de/
 * Copyright (c) 2013-2016, The OpenClonk Team and contributors
 *
 * Distributed under the terms of the ISC license; see accompanying file
 * "COPYING" for details.
 *
 * "Clonk" is a registered trademark of Matthes Bender, used with permission.
 * See accompanying file "TRADEMARK" for details.
 *
 * To redistribute this file separately, substitute the full license texts
 * for the above references.
 */

/* Pixel Sprite system for tiny bits of moving material */

#ifndef INC_C4PXS
#define INC_C4PXS

#include "landscape/C4Material.h"

class C4PXS
{
public:
	C4PXS(): Mat(MNone), x(Fix0), y(Fix0), xdir(Fix0), ydir(Fix0) {}
	friend class C4PXSSystem;
protected:
	int32_t Mat;
	C4Real x,y,xdir,ydir;
protected:
	void Execute();
	void Deactivate();
};

const size_t PXSMax = 10000;

template<typename T, size_t N>
class SparseArray
{
	T data[N];
	static constexpr size_t maskN = (N + 63) / 64;
	uint64_t mask[maskN] = {0};

public:
	T* New()
	{
		for (size_t i = 0; i < maskN; i++)
		{
			uint64_t m = ~mask[i];
			if (m)
			{
				size_t j = __builtin_ffsll(m) - 1;
				size_t idx = i*64 + j;
				if (idx >= N) return nullptr;
				mask[i] |= 1 << j;
				return &data[idx];
			}
		}
		return nullptr;
	}

	void Delete(T *el)
	{
		size_t idx = el - data;
		assert(idx < N);
		size_t i = idx / 64, j = idx % 64;
		mask[i] &= ~(1 << j);
	}

	template<typename Ti, typename SA = SparseArray>
	class Iterator : public std::iterator<std::forward_iterator_tag, Ti>
	{
		SA *array;
		size_t el, pos; // el: current element in data, pos: current bit position in cur
		uint64_t cur;
	public:
		Iterator(SA *array) : array(array), el(0), pos(64), cur(0) { }

		Iterator& operator++()
		{
			while (!cur && el < N)
			{
				el += 64 - pos;
				pos = 0;
				cur = array->mask[el / 64];
			}
			size_t inc = __builtin_ffsll(cur) - 1;
			el += inc;
			pos += inc + 1;
			cur >>= inc + 1;
			if (el >= N)
			{
				array = nullptr;
				el = 0;
			}
			return *this;
		}

		bool operator==(Iterator other) { return array == other.array && el == other.el; }
		bool operator!=(Iterator other) { return !(*this == other); }
		Ti* operator*() const { return &array->data[el]; }
	};

	Iterator<T> begin() { return Iterator<T>(this); }
	Iterator<T> end() { return Iterator<T>(nullptr); }
	Iterator<const T, const SparseArray> begin() const { return Iterator<const T, const SparseArray>(this); }
	Iterator<const T, const SparseArray> end() const { return Iterator<const T, const SparseArray>(nullptr); }
};

class C4PXSSystem
{
public:
	C4PXSSystem();
	~C4PXSSystem();
public:
	int32_t Count;
protected:
	SparseArray<C4PXS, PXSMax> PXS;
	size_t PXSLast; // highest index + 1 set in PXS to speed up execution with few PXS
	size_t PXSFirstFree; // start index to search for a free slot
public:
	void Default();
	void Clear();
	void Execute();
	void Draw(C4TargetFacet &cgo);
	void Synchronize();
	void SyncClearance();
	void Cast(int32_t mat, int32_t num, int32_t tx, int32_t ty, int32_t level);
	bool Create(int32_t mat, C4Real ix, C4Real iy, C4Real ixdir=Fix0, C4Real iydir=Fix0);
	bool Load(C4Group &hGroup);
	bool Save(C4Group &hGroup);
	int32_t GetCount() const { return Count; } // count all PXS
	int32_t GetCount(int32_t mat) const; // count PXS of given material
	int32_t GetCount(int32_t mat, int32_t x, int32_t y, int32_t wdt, int32_t hgt) const; // count PXS of given material in given area. mat==-1 for all materials.
	void Delete(C4PXS *pxp);
protected:
	C4PXS *New();
};

extern C4PXSSystem PXS;
#endif
