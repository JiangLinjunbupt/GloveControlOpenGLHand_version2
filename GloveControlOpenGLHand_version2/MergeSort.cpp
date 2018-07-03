#include"MergeSort.h"


void MergeSort::sort(float *a, int first, int last)
{
	if (first < last)
	{
		int mid = (first + last) / 2;
		this->sort(a, first, mid);
		this->sort(a, mid + 1, last);
		this->merge(a, first, mid, last);
	}
}

void MergeSort::merge(float *a, int first, int mid, int last)
{
	int i = first; int j = mid + 1;

	int m = mid; int n = last;

	int k = 0;
	int index_k = 0;
	while (i <= m && j <= n)
	{
		if (a[i] < a[j])
		{
			this->tmp[k++] = a[i];
			this->tmp_index[index_k++] = this->index[i];
			i++;
		}
		else
		{
			this->tmp[k++] = a[j];
			this->tmp_index[index_k++] = this->index[j];
			j++;
		}
	}

	while (i <= m)
	{
		this->tmp[k++] = a[i];
		this->tmp_index[index_k++] = this->index[i];
		i++;
	}
	while (j <= n)
	{
		this->tmp[k++] = a[j];
		this->tmp_index[index_k++] = this->index[j];
		j++;
	}

	for (i = 0; i < k; i++)
	{
		a[first + i] = this->tmp[i];
		this->index[first + i] = this->tmp_index[i];
	}
}

bool MergeSort::merge_sort(float *a, int a_length)
{
	if (this->tmp == NULL)
		return false;
	for (int i = 0; i < a_length; i++)
	{
		this->index[i] = i;
	}
	this->sort(a, 0, a_length - 1);
	return true;
}