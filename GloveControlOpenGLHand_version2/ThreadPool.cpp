#include "ThreadPool.h"
#include<iostream>
using namespace std;
ThreadPool::ThreadPool() :_maxQueueSize(0), _running(false) { }
ThreadPool::~ThreadPool()
{
	if (_running)
	{
		stop();
	}
}
void ThreadPool::start(int numThreads)
{
	if (!_threads.empty())
	{
		cout << "vector<thread> _threads不为空" << endl;
		exit(EXIT_FAILURE);
	}
	_running = true;
	_threads.reserve(numThreads); //reserver函数用来给vector预分配存储区大小，即capacity的值
	for (int i = 0; i < numThreads; i++)
	{
		_threads.push_back(thread(&ThreadPool::runInThread, this));//将this->runInThread()作为线程函数
	}
}
void ThreadPool::stop()
{
	{
		unique_lock<mutex> lock(_mutex); //获取互斥量_mutex的所有权，并调用_mutex.lock()对其加锁
		_running = false;
		_notEmpty.notify_all();
	}                                 //执行完大括号内容，lock对象出作用域，析构，调用_mutex.unlock()解锁
	for (size_t i = 0; i < _threads.size(); i++)
	{
		_threads[i].join();
	}
}
void ThreadPool::run(const Task& f)
{
	if (_threads.empty()) { f(); }
	else
	{
		unique_lock<mutex> lock(_mutex);
		while (isFull())
		{
			_notFull.wait(lock);
		}
		if (isFull())
		{
			cout << "!isFull()" << endl;
			exit(EXIT_FAILURE);
		}
		_queue.push_back(f);
		_notEmpty.notify_one();
		//cout << "添加任务到线程池" << endl;
	}
}
ThreadPool::Task ThreadPool::take()
{
	unique_lock<mutex> lock(_mutex);
	while (_queue.empty() && _running)  //stop()后，_running变为false
	{
		_notEmpty.wait(lock);  //阻塞，解锁，当被另外某个线程调用 notify_* 唤醒后，上锁，解除阻塞，再次判断循环条件
	}
	Task task;
	if (!_queue.empty())
	{
		task = _queue.front();  //返回首元素的引用
		_queue.pop_front();//删除双端队列中最前一个元素
		if (_maxQueueSize > 0)
		{
			_notFull.notify_one();
		}
		//cout << "从线程池取任务" << endl;
	}
	return task;
}
bool ThreadPool::isFull()
{
	return _maxQueueSize > 0 && (int)_queue.size() >= _maxQueueSize;
}
void ThreadPool::runInThread()
{
	try
	{
		while (_running)
		{
			Task task = take();
			if (task)
			{
				task();
				_mutex.lock();
				_fulfillTaskNum++;
				if (_fulfillTaskNum == _submitTaskNum)
				{
					_allTaskDone.notify_one();
				}
				_mutex.unlock();
			}
		}
	}
	catch (const exception& ex)
	{
		fprintf(stderr, "reason: %s\n", ex.what());
		abort();
	}
	catch (...)
	{
		fprintf(stderr, "exception caught in ThreadPool \n");
	}
}


int ThreadPool::get_maxQueueSize()
{
	return _maxQueueSize;
}

void ThreadPool::ensureTaskCompleted(int submitTaskNum)
{
	unique_lock<mutex> lck(_mutex);
	_submitTaskNum = submitTaskNum;
	_allTaskDone.wait(lck, [this, &submitTaskNum] {return this->_fulfillTaskNum == submitTaskNum; });
	_submitTaskNum = -1;//置为-1，则在下次调用该函数前，runInThread()中不会唤醒_allTaskDone
	_fulfillTaskNum = 0;
}