#ifndef MAZE_SINGLETON_H
#define MAZE_SINGLETON_H

template <typename T>
class Singleton
{
public:
	static T* Instance()
	{
		static T instance{};
		return &instance;
	}

	Singleton(const Singleton&) = delete;

	Singleton& operator=(Singleton) = delete;

protected:
	Singleton()
	{
	}
};

#endif //MAZE_SINGLETON_H
