#ifndef __hexad_
#define __hexad_

#define _SIX_ 6

/*
	_hexad Template Class

	8/30/2005
	junggon@gmail.com

	template <class TYPE> class _hexad
	method : 
		_hexad()
		_hexad(const _hexad<TYPE> &qud)
		~_hexad()
		int size() const;
		const TYPE *get_base() const;
		TYPE *get_base();
		const TYPE &operator [] (int i) const;
		TYPE &operator [] (int i);
		_hexad &operator = (const _hexad<TYPE> &qud);
		int find(const TYPE &value) const;
*/

template <class TYPE> class _hexad
{
private :
	TYPE base[_SIX_];
public :
	_hexad() {}
	
	_hexad(const _hexad<TYPE> &qud)
	{
		for ( int i = 0; i < _SIX_; i++ ) base[i] = qud.base[i];	
	}
	
	~_hexad() {}
	
	int size() const { return _SIX_; }

	const TYPE *get_base() const { return base; }
	
	TYPE *get_base() { return base; }
	
	const TYPE &operator [] (int i) const { return base[i]; }
	
	TYPE &operator [] (int i) { return base[i]; }
	
	_hexad &operator = (const _hexad<TYPE> &qud)
	{
		for ( int i = 0; i < _SIX_; i++ ) base[i] = qud.base[i];
		return *this;
	}

	int find(const TYPE &value) const
	{
		for ( int i = 0; i < _SIX_; i++ ) if ( base[i] == value ) return i;
		return -1;
	}
};

#endif