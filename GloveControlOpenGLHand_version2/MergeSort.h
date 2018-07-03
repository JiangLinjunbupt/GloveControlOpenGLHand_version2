#pragma once

#include<iostream>
#include<Windows.h>
using namespace std;

class MergeSort           //¹é²¢ÅÅÐò
{
public:
	MergeSort(int len) :length(len)
	{
		this->tmp = new float[len];
		this->index = new int[len];
		this->tmp_index = new int[len];
	}
	~MergeSort() {
		delete[] tmp;
		delete[] index;
		delete[] tmp_index;
		tmp = NULL;
		index = NULL;
		tmp_index = NULL;
	}
	bool merge_sort(float *a, int a_length);
	int *index = NULL;
private:
	int length;
	float *tmp = NULL;
	int *tmp_index = NULL;
	void sort(float *a, int first, int last);
	void merge(float *a, int first, int mid, int last);
};