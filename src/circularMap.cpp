#include <circularMap.hpp>


circularMap::circularMap()
{
    this->size = DEFAULT_SIZE;
}

circularMap::circularMap(int size)
{
    this->size = size;
}

iterator circularMap::begin() 
{
    return map_.begin(); 
}

iterator circularMap::end() 
{
    return map_.end();
}

void circularMap::put(K k, V v)
{ 
	map_.insert(std::make_pair(k,v));
	deque_.push_back(k);
	_ensure();  
}

void circularMap::_ensure() 
{ 
	if (deque_.size() > this->size) 
	{ 
		map_.erase(deque_.front()); 
		deque_.pop_front();
	}
}

