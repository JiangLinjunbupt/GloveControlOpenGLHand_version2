#pragma once
#include<thread>
#include<mutex>
#include<functional>  //C++ 11引入了函数对象标准库<functional>
#include<string>
#include<condition_variable>  //条件变量
#include<deque>
#include<vector>
#include<memory>            // For std::shared_ptr<>
using namespace std;

class nocopyable//类nocopyable不可拷贝，继承该类的子类也不可拷贝
{
private:
	nocopyable(const nocopyable& x) = delete; //C++11使用delete关键字显式指示编译器不生成函数的默认版本,同时也禁止重载该函数
	nocopyable& operator=(const nocopyable& x) = delete;
public:
	nocopyable() = default;//default显式地指示编译器生成该函数的默认版本
	~nocopyable() = default;
};

class ThreadPool :public nocopyable
{
public:
	typedef std::function<void()> Task;//可以用普通函数、lambda表达式、bind表达式、函数对象赋值
	explicit ThreadPool();
	~ThreadPool();

	void start(int numThraeds);//设置线程数，创建numThreads个线程
	void stop();//线程池结束
	void run(const Task& f); //添加任务f至任务队列
	void setMaxQueueSize(int maxSize) { _maxQueueSize = maxSize; }
private:
	bool isFull();//任务队列是否已满
	void runInThread(); //线程的入口函数，在内部调用take()取任务
	Task take();//从任务队列中取队首任务

	mutex _mutex;
	condition_variable _notEmpty;
	condition_variable _notFull;
	vector<thread> _threads;
	deque<Task> _queue;  //双端队列deque,能高效插入删除容器的头部元素
	int _maxQueueSize; //任务队列可存放最大任务数
	bool _running;

	condition_variable _allTaskDone;
	int _submitTaskNum = -1;  //提交给线程池的任务数
	int _fulfillTaskNum = 0;//线程池已完成的任务数量
public:
	void ensureTaskCompleted(int submitTaskNum);
	int get_maxQueueSize();
};

