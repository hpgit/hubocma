#ifndef __triad_
#define __triad_

#define _THREE_ 3

/*
	_triad Template Class

	8/30/2005
	junggon@gmail.com

	template <class TYPE> class _triad
	method : 
		_triad()
		_triad(TYPE a0, TYPE a1, TYPE a2)
		_triad(const _triad<TYPE> &trd)
		~_triad()
		int size() const;
		const TYPE *get_base() const;
		TYPE *get_base();
		const TYPE &operator [] (int i) const;
		TYPE &operator [] (int i);
		_triad &operator = (const _triad<TYPE> &trd);
		bool operator == (const _triad<TYPE> &trd);
		bool operator != (const _triad<TYPE> &trd);
		int find(const TYPE &value) const;
*/

template <class TYPE> class _triad
{
private :
	TYPE base[_THREE_];
public :
	_triad() {}

	_triad(TYPE a0, TYPE a1, TYPE a2) 
	{ 
		base[0] = a0; base[1] = a1; base[2] = a2; 
	}
	
	_triad(const _triad<TYPE> &trd)
	{
		base[0] = trd.base[0];
		base[1] = trd.base[1];
		base[2] = trd.base[2];
	}
	
	~_triad() {}
	
	int size() const { return _THREE_; }

	const TYPE *get_base() const { return base; }
	
	TYPE *get_base() { return base; }
	
	const TYPE &operator [] (int i) const { return base[i]; }
	
	TYPE &operator [] (int i) { return base[i]; }
	
	_triad &operator = (const _triad<TYPE> &trd)
	{
		base[0] = trd.base[0];
		base[1] = trd.base[1];
		base[2] = trd.base[2];
		return *this;
	}

	bool operator == (const _triad<TYPE> &trd)
	{
		if ( base[0] != trd.base[0] ) return false;
		if ( base[1] != trd.base[1] ) return false;
		if ( base[2] != trd.base[2] ) return false;
		return true;
	}

	bool operator != (const _triad<TYPE> &trd)
	{
		return !(*this == trd);
	}

	int find(const TYPE &value) const
	{
		if ( base[0] == value ) return 0;
		if ( base[1] == value ) return 1;
		if ( base[2] == value ) return 2;
		return -1;
	}
};

#endif