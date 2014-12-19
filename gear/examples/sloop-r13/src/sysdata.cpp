#include <fstream>
#include <iomanip>
#include <vector>
#include <vector>
#include <algorithm>
#include "sysdata.h"

using namespace std;

static vector<double> _data_double;
static vector<int> _data_int;
static vector<bool> _data_bool;

bool GSysData::set_data_double(vector<double> &data_double_) 
{
	if ( data_double_.size() != m_ptr_data_double.size() ) return false;
	vector<double>::iterator iter_double;
	vector<double *>::iterator iter_pdouble;
	for (iter_pdouble = m_ptr_data_double.begin(), iter_double = data_double_.begin(); iter_pdouble != m_ptr_data_double.end(); iter_pdouble++, iter_double++) {
		*(*iter_pdouble) = *iter_double;
	}
	return true;
}

bool GSysData::set_data_int(vector<int> &data_int_)
{
	if ( data_int_.size() != m_ptr_data_int.size() ) return false;
	vector<int>::iterator iter_int;
	vector<int *>::iterator iter_pint;
	for (iter_pint = m_ptr_data_int.begin(), iter_int = data_int_.begin(); iter_pint != m_ptr_data_int.end(); iter_pint++, iter_int++) {
		*(*iter_pint) = *iter_int;
	}
	return true;
}

bool GSysData::set_data_bool(vector<bool> &data_bool_)
{
	if ( data_bool_.size() != m_ptr_data_bool.size() ) return false;
	vector<bool>::iterator iter_bool;
	vector<bool *>::iterator iter_pbool;
	for (iter_pbool = m_ptr_data_bool.begin(), iter_bool = data_bool_.begin(); iter_pbool != m_ptr_data_bool.end(); iter_pbool++, iter_bool++) {
		*(*iter_pbool) = *iter_bool;
	}
	return true;
}

vector<double> GSysData::get_data_double()
{
	vector<double> data_double(m_ptr_data_double.size());
	vector<double>::iterator iter_double;
	vector<double *>::iterator iter_pdouble;
	for (iter_pdouble = m_ptr_data_double.begin(), iter_double = data_double.begin(); iter_pdouble != m_ptr_data_double.end(); iter_pdouble++, iter_double++) {
		*iter_double = *(*iter_pdouble);
	}
	return data_double;
}

vector<int> GSysData::get_data_int()
{
	vector<int> data_int(m_ptr_data_int.size());
	vector<int>::iterator iter_int;
	vector<int *>::iterator iter_pint;
	for (iter_pint = m_ptr_data_int.begin(), iter_int = data_int.begin(); iter_pint != m_ptr_data_int.end(); iter_pint++, iter_int++) {
		*iter_int = *(*iter_pint);
	}
	return data_int;
}

vector<bool> GSysData::get_data_bool()
{
	vector<bool> data_bool(m_ptr_data_bool.size());
	vector<bool>::iterator iter_bool;
	vector<bool *>::iterator iter_pbool;
	for (iter_pbool = m_ptr_data_bool.begin(), iter_bool = data_bool.begin(); iter_pbool != m_ptr_data_bool.end(); iter_pbool++, iter_bool++) {
		*iter_bool = *(*iter_pbool);
	}
	return data_bool;
}

void GSysData::set_filepath_for_writing(const char *fnamew_)
{
	m_filepath_w = string(fnamew_);
	m_filepath_log = m_filepath_w; m_filepath_log.append(".log");
}

void GSysData::set_filepath_for_reading(const char *fnamer_) 
{ 
	m_filepath_r = string(fnamer_); 
}

void GSysData::edit_num_data_set(const char *file_path_, int n_)
{
	ofstream fout;
	fout.open(file_path_, ios_base::binary | ios_base::in | ios_base::ate);
	fout.seekp(0, ios_base::beg);
	fout.write((char *)&n_, sizeof(int));
	fout.seekp(0, ios_base::end);
	fout.close();
}

void GSysData::get_time_sequence(vector<double> &ts_)
{
	ts_ = vector<double>(m_list_time.begin(), m_list_time.end());
}

void GSysData::get_time_sequence(vector<double> &ts_, const char *file_path_)
{
	ifstream fin;
	int n, nd, ni, nb;

	fin.open(file_path_, ios::binary);

	if ( !fin.is_open() ) return;

	fin.read((char *)&n, sizeof(int));
	fin.read((char *)&nd, sizeof(int));
	fin.read((char *)&ni, sizeof(int));
	fin.read((char *)&nb, sizeof(int));

	if ( nd != m_ptr_data_double.size() || ni != m_ptr_data_int.size() || nb != m_ptr_data_bool.size() ) return;

	double t;
	int skip = int(m_ptr_data_double.size()) * sizeof(double)
				+ int(m_ptr_data_int.size()) * sizeof(int)
				+ int(m_ptr_data_bool.size()) * sizeof(bool);	// size of data without time

	ts_.clear();

	while (1) {
		if ( fin.eof() ) break;
		fin.read((char *)&t, sizeof(double));
		ts_.push_back(t);
		fin.seekg(skip, ios_base::cur);
	}

	fin.close();

	ts_.pop_back(); // why??
}

//bool GSysData::_write_data(ofstream *fout_, vector<double *> &ptr_data_)
//{
//	vector<double *>::iterator iter_p;
//
//	for (iter_p = ptr_data_.begin(); iter_p != ptr_data_.end(); iter_p++) {
//		fout_->write((char *)(*iter_p), sizeof(double));
//	}
//
//	return true;
//}
//
//bool GSysData::_write_data(ofstream *fout_, vector<int *> &ptr_data_)
//{
//	vector<int *>::iterator iter_p;
//
//	for (iter_p = ptr_data_.begin(); iter_p != ptr_data_.end(); iter_p++) {
//		fout_->write((char *)(*iter_p), sizeof(int));
//	}
//
//	return true;
//}
//
//bool GSysData::_write_data(ofstream *fout_, vector<bool *> &ptr_data_)
//{
//	vector<bool *>::iterator iter_p;
//
//	for (iter_p = ptr_data_.begin(); iter_p != ptr_data_.end(); iter_p++) {
//		fout_->write((char *)(*iter_p), sizeof(bool));
//	}
//
//	return true;
//}

//bool GSysData::_read_data(ifstream *fin_, vector<double *> &ptr_data_)
//{
//	vector<double *>::iterator iter_p;
//
//	for (iter_p = ptr_data_.begin(); iter_p != ptr_data_.end(); iter_p++) {
//		fin_->read((char *)(*iter_p), sizeof(double));
//	}
//
//	return true;
//}
//
//bool GSysData::_read_data(ifstream *fin_, vector<int *> &ptr_data_)
//{
//	vector<int *>::iterator iter_p;
//
//	for (iter_p = ptr_data_.begin(); iter_p != ptr_data_.end(); iter_p++) {
//		fin_->read((char *)(*iter_p), sizeof(int));
//	}
//
//	return true;
//}
//
//bool GSysData::_read_data(ifstream *fin_, vector<bool *> &ptr_data_)
//{
//	vector<bool *>::iterator iter_p;
//
//	for (iter_p = ptr_data_.begin(); iter_p != ptr_data_.end(); iter_p++) {
//		fin_->read((char *)(*iter_p), sizeof(bool));
//	}
//
//	return true;
//}

bool GSysData::_write_data(ofstream *fout_, vector<double> &data_)
{
	int i;
	vector<double>::iterator iter_p;

	for (i=0, iter_p = data_.begin(); iter_p != data_.end(); iter_p++, i++) {
		m_ptr_data_double_buffer[i] = *iter_p;
	}
	fout_->write((char *)m_ptr_data_double_buffer, m_ptr_data_double.size()*sizeof(double));

	return true;
}

bool GSysData::_write_data(ofstream *fout_, vector<int> &data_)
{
	int i;
	vector<int>::iterator iter_p;

	for (i=0, iter_p = data_.begin(); iter_p != data_.end(); iter_p++, i++) {
		m_ptr_data_int_buffer[i] = *iter_p;
	}
	fout_->write((char *)m_ptr_data_int_buffer, m_ptr_data_int.size()*sizeof(int));

	return true;
}

bool GSysData::_write_data(ofstream *fout_, vector<bool> &data_)	
{
	int i;
	vector<bool>::iterator iter_p;

	for (i=0, iter_p = data_.begin(); iter_p != data_.end(); iter_p++, i++) {
		m_ptr_data_bool_buffer[i] = *iter_p;
	}
	fout_->write((char *)m_ptr_data_bool_buffer, m_ptr_data_bool.size()*sizeof(bool));

	return true;
}

bool GSysData::_read_data(ifstream *fin_, vector<double> &data_)
{
	int i;
	vector<double>::iterator iter_p;

	fin_->read((char *)m_ptr_data_double_buffer, m_ptr_data_double.size()*sizeof(double));
	for (i=0, iter_p = data_.begin(); iter_p != data_.end(); iter_p++, i++) {
		*iter_p = m_ptr_data_double_buffer[i];
	}

	return true;
}

bool GSysData::_read_data(ifstream *fin_, vector<int> &data_)
{
	int i;
	vector<int>::iterator iter_p;

	fin_->read((char *)m_ptr_data_int_buffer, m_ptr_data_int.size()*sizeof(int));
	for (i=0, iter_p = data_.begin(); iter_p != data_.end(); iter_p++, i++) {
		*iter_p = m_ptr_data_int_buffer[i];
	}

	return true;
}

bool GSysData::_read_data(ifstream *fin_, vector<bool> &data_)		
{
	int i;
	vector<bool>::iterator iter_p;

	fin_->read((char *)m_ptr_data_bool_buffer, m_ptr_data_bool.size()*sizeof(bool));
	for (i=0, iter_p = data_.begin(); iter_p != data_.end(); iter_p++, i++) {
		*iter_p = m_ptr_data_bool_buffer[i];
	}

	return true;
}

bool GSysData::load_data_on_memory_from_file()
{
	ifstream fin;
	int n, nd, ni, nb;

	fin.open(m_filepath_r.c_str(), ios::binary);

	if ( !fin.is_open() ) return false;

	fin.read((char *)&n, sizeof(int));
	fin.read((char *)&nd, sizeof(int));
	fin.read((char *)&ni, sizeof(int));
	fin.read((char *)&nb, sizeof(int));

	if ( n < 0 || nd != m_ptr_data_double.size() || ni != m_ptr_data_int.size() || nb != m_ptr_data_bool.size() ) return false;

	m_list_time.resize(n);
	m_list_data_double.resize(n);
	m_list_data_int.resize(n);
	m_list_data_bool.resize(n);

	list<double>::iterator iter_time = m_list_time.begin();
	list< vector<double> >::iterator iter_data_double = m_list_data_double.begin();
	list< vector<int> >::iterator iter_data_int = m_list_data_int.begin();
	list< vector<bool> >::iterator iter_data_bool = m_list_data_bool.begin();

	for (int i=0; i<n; i++) {
		(*iter_data_double).resize(nd);
		(*iter_data_int).resize(ni);
		(*iter_data_bool).resize(nb);

		fin.read((char *)(&(*iter_time)), sizeof(double));
		_read_data(&fin, *iter_data_double);
		_read_data(&fin, *iter_data_int);
		_read_data(&fin, *iter_data_bool);

		iter_time++;
		iter_data_double++;
		iter_data_int++;
		iter_data_bool++;
	}

	fin.close();
	
	return true;
}

void GSysData::save_data_on_memory_into_file()
{
	ofstream fout;
	int n, nd, ni, nb;

	n = int(m_list_time.size());
	nd = int(m_ptr_data_double.size());
	ni = int(m_ptr_data_int.size());
	nb = int(m_ptr_data_bool.size());

	fout.open(m_filepath_w.c_str(), ios::binary);

	fout.write((char *)&n, sizeof(int));
	fout.write((char *)&nd, sizeof(int));
	fout.write((char *)&ni, sizeof(int));
	fout.write((char *)&nb, sizeof(int));

	list<double>::iterator iter_time = m_list_time.begin();
	list< vector<double> >::iterator iter_data_double = m_list_data_double.begin();
	list< vector<int> >::iterator iter_data_int = m_list_data_int.begin();
	list< vector<bool> >::iterator iter_data_bool = m_list_data_bool.begin();

	for (int i=0; i<n; i++) {
		fout.write((char *)(&(*iter_time)), sizeof(double));
		_write_data(&fout, *iter_data_double);
		_write_data(&fout, *iter_data_int);
		_write_data(&fout, *iter_data_bool);

		iter_time++;
		iter_data_double++;
		iter_data_int++;
		iter_data_bool++;
	}

	fout.close();
}

void GSysData::save_data_on_memory_into_file_app(const char *file_path_, bool b_skip_first_)
{
	vector<double> ts_prev;
	get_time_sequence(ts_prev, file_path_);
	double tf = 0;
	if ( ts_prev.size() > 0 ) { tf = ts_prev[ts_prev.size()-1]; }
	double h = 0;
	if ( ts_prev.size() > 1 ) { h = ts_prev[1] - ts_prev[0]; }
	if ( b_skip_first_ ) { h = 0; }

	ofstream fout;
	fout.open(file_path_, ios::binary | ios::app);

	list<double>::iterator iter_time = m_list_time.begin();
	list< vector<double> >::iterator iter_data_double = m_list_data_double.begin();
	list< vector<int> >::iterator iter_data_int = m_list_data_int.begin();
	list< vector<bool> >::iterator iter_data_bool = m_list_data_bool.begin();

	double t;
	int n = int(m_list_time.size());

	for (int i=0; i<n; i++) {

		if ( i==0 && b_skip_first_ ) {
			;
		} else {
			t = *iter_time + tf + h;

			fout.write((char *)(&t), sizeof(double));
			_write_data(&fout, *iter_data_double);
			_write_data(&fout, *iter_data_int);
			_write_data(&fout, *iter_data_bool);
		}

		iter_time++;
		iter_data_double++;
		iter_data_int++;
		iter_data_bool++;
	}

	if ( b_skip_first_ ) {
		edit_num_data_set(file_path_, ts_prev.size()+n-1);
	} else {
		edit_num_data_set(file_path_, ts_prev.size()+n);
	}

	fout.close();
}

bool GSysData::save_data_on_memory_into_file_rng(const char *file_path_, int idx1_, int idx2_, bool b_zero_start_time_)
{
	ofstream fout;
	int n0, n, nd, ni, nb;
	double t;

	n0 = int(m_list_time.size());

	if ( file_path_ == "" || idx1_ < 0 || idx1_ >= n0 || idx2_ < 0 || idx2_ >= n0 || idx1_ > idx2_ ) return false;

	n = idx2_-idx1_+1;
	nd = int(m_ptr_data_double.size());
	ni = int(m_ptr_data_int.size());
	nb = int(m_ptr_data_bool.size());

	fout.open(file_path_, ios::binary);

	fout.write((char *)&n, sizeof(int));
	fout.write((char *)&nd, sizeof(int));
	fout.write((char *)&ni, sizeof(int));
	fout.write((char *)&nb, sizeof(int));

	list<double>::iterator iter_time = m_list_time.begin();
	list< vector<double> >::iterator iter_data_double = m_list_data_double.begin();
	list< vector<int> >::iterator iter_data_int = m_list_data_int.begin();
	list< vector<bool> >::iterator iter_data_bool = m_list_data_bool.begin();

	double t_shift = 0;

	for (int i=0; i<n0; i++) {

		if ( i >= idx1_ && i <= idx2_ ) {
			
			t = *iter_time;

			if ( b_zero_start_time_ ) {
				if ( i == idx1_ ) {
					t_shift = *iter_time;
				}
				t -= t_shift;
			}

			fout.write((char *)(&t), sizeof(double));
			_write_data(&fout, *iter_data_double);
			_write_data(&fout, *iter_data_int);
			_write_data(&fout, *iter_data_bool);
		}

		iter_time++;
		iter_data_double++;
		iter_data_int++;
		iter_data_bool++;
	}

	fout.close();

	return true;
}

void GSysData::write_data_into_memory(double t_)
{
	int nd, ni, nb;

	nd = int(m_ptr_data_double.size());
	ni = int(m_ptr_data_int.size());
	nb = int(m_ptr_data_bool.size());

	_data_double.resize(nd);
	_data_int.resize(ni);
	_data_bool.resize(nb);

	for (int i=0; i<nd; i++) {
		_data_double[i] = *(m_ptr_data_double[i]);
	}
	for (int i=0; i<ni; i++) {
		_data_int[i] = *(m_ptr_data_int[i]);
	}
	for (int i=0; i<nb; i++) {
		_data_bool[i] = *(m_ptr_data_bool[i]);
	}

	m_list_time.push_back(t_);
	m_list_data_double.push_back(_data_double);
	m_list_data_int.push_back(_data_int);
	m_list_data_bool.push_back(_data_bool);
}

bool GSysData::read_data_from_memory(double &t_, int idx_)
{
	int nd, ni, nb;

	nd = int(m_ptr_data_double.size());
	ni = int(m_ptr_data_int.size());
	nb = int(m_ptr_data_bool.size());

	list<double>::iterator iter_time = m_list_time.begin();
	list< vector<double> >::iterator iter_data_double = m_list_data_double.begin();
	list< vector<int> >::iterator iter_data_int = m_list_data_int.begin();
	list< vector<bool> >::iterator iter_data_bool = m_list_data_bool.begin();

	for (int i=0; i<idx_; i++) {
		iter_time++;
		iter_data_double++;
		iter_data_int++;
		iter_data_bool++;
	}

	t_ = *iter_time;
	for (int i=0; i<nd; i++) {
		*m_ptr_data_double[i] = (*iter_data_double)[i];
	}
	for (int i=0; i<ni; i++) {
		*m_ptr_data_int[i] = (*iter_data_int)[i];
	}
	for (int i=0; i<nb; i++) {
		*m_ptr_data_bool[i] = (*iter_data_bool)[i];
	}

	return true;
}

void GSysData::clear_data_on_memory()
{
	m_list_time.clear();
	m_list_data_double.clear();
	m_list_data_int.clear();
	m_list_data_bool.clear();
}

void GSysData::truncate_data_on_memory(int idx_)
{
	list<double>::iterator iter_time = m_list_time.begin();
	list< vector<double> >::iterator iter_data_double = m_list_data_double.begin();
	list< vector<int> >::iterator iter_data_int = m_list_data_int.begin();
	list< vector<bool> >::iterator iter_data_bool = m_list_data_bool.begin();

	for (int i=0; i<idx_; i++) {
		iter_time++;
		iter_data_double++;
		iter_data_int++;
		iter_data_bool++;
	}

	m_list_time.erase(iter_time, m_list_time.end());
	m_list_data_double.erase(iter_data_double, m_list_data_double.end());
	m_list_data_int.erase(iter_data_int, m_list_data_int.end());
	m_list_data_bool.erase(iter_data_bool, m_list_data_bool.end());
}

double GSysData::get_time_from_data_on_memory(int idx_)
{
	list<double>::iterator iter_time = m_list_time.begin();
	for (int i=0; i<idx_; i++) {
		iter_time++;
	}
	return *iter_time;
}

